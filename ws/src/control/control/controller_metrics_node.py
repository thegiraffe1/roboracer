"""Independent estimated-state evaluator, shared by MAP and MPCC; never drives."""
from dataclasses import fields
import json
import math
import os
import signal
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from ament_index_python.packages import get_package_share_directory
from ackermann_msgs.msg import AckermannDriveStamped
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from perception.tracks import track_dir
from .mpcc_model import Config
from .mpcc_track import Track, footprint_points, FreeMap
from .controller_metrics import LapTimer, RunLog, manifest, summarize


class ControllerMetrics(Node):
    def __init__(self):
        super().__init__('controller_metrics_node')
        p = lambda name, default: self.declare_parameter(name, default).value
        defaults = Config()
        self.config = Config(**{f.name: p(f.name, getattr(defaults, f.name)) for f in fields(Config)})
        controller = p('controller', 'mpcc')
        directory = track_dir(p('track', ''))
        centerline = p('centerline_path', '') or os.path.join(directory, 'centerline.csv')
        map_yaml = p('map_yaml', '') or os.path.join(directory, 'map.yaml')
        self.track = Track.load(centerline, self.config)
        self.free_map = FreeMap(map_yaml)
        self.laps = LapTimer(self.track.length)
        self.previous = self.last_time = None
        self.odom = self.command = None
        paths = [centerline, map_yaml, os.path.join(directory, 'raceline.csv')]
        from pathlib import Path as FilePath
        paths.extend(str(path) for path in FilePath(__file__).parent.glob('mpcc_*.py'))
        paths.extend(str(FilePath(__file__).parent/name) for name in (
            'map_node.py', 'map_guidance.py', 'map_lut.py', 'input_health.py',
            'controller_metrics.py', 'controller_metrics_node.py'))
        share = FilePath(get_package_share_directory('control'))
        paths.extend(str(share/'config'/name) for name in (
            'map_model.json', 'autodrive_map_lut.npz', 'vehicle.yaml', 'mpcc.yaml'))
        # Include map pixels, not just its YAML metadata, in the run fingerprint.
        import yaml
        with open(map_yaml) as stream:
            paths.append(os.path.join(os.path.dirname(map_yaml), yaml.safe_load(stream)['image']))
        metadata = dict(localizer=p('localizer', 'unknown'), use_sim_time=self.get_parameter('use_sim_time').value,
                        controller_max_speed=p('controller_max_speed', 0.))
        self.lap_max_speed = p('lap_max_speed', 15.)
        if not math.isfinite(self.lap_max_speed) or self.lap_max_speed <= 0:
            raise ValueError('lap_max_speed must be finite and positive')
        self.log = RunLog(p('log_root', os.path.expanduser('~/ws/log/mpcc')), manifest(
            controller, self.config, paths, metadata))
        self.create_subscription(PoseStamped, p('pose_topic', '/localized_pose'), self.on_pose, 10)
        self.create_subscription(Odometry, p('odom_topic', '/odom'), self.on_odom, qos_profile_sensor_data)
        self.create_subscription(AckermannDriveStamped, '/drive', self.on_command, 10)
        if controller == 'mpcc':
            self.create_subscription(DiagnosticArray, '/mpcc/diagnostics', self.on_solver, 10)
        self.create_subscription(String, '/autodrive/roboracer_1/sim_reset', self.on_reset, 10)
        self.get_logger().info(f'Controller metrics: {self.log.directory}')

    def on_odom(self, msg):
        self.odom = msg

    def on_command(self, msg):
        self.command = msg

    def on_reset(self, msg):
        self.previous = self.last_time = None
        self.laps.reset()
        self.log.record(dict(kind='reset', monotonic_s=time.monotonic(), reason=msg.data))

    def on_solver(self, msg):
        for status in msg.status:
            if status.name == 'mpcc':
                self.log.record(dict(kind='solver', monotonic_s=time.monotonic(),
                                     values={v.key: json.loads(v.value) for v in status.values}))

    def on_pose(self, msg):
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        ros_now = self.get_clock().now().nanoseconds*1e-9
        q, pos = msg.pose.orientation, msg.pose.position
        if (msg.header.frame_id != 'map' or not 0 <= ros_now-stamp <= self.config.max_input_age or
                stamp <= 0 or not np.isfinite([pos.x, pos.y, q.x, q.y, q.z, q.w]).all() or
                abs(np.linalg.norm([q.x, q.y, q.z, q.w])-1) > .05):
            self.laps.reset()
            return
        # At most 20 Hz, but do not count repeated stamps or reverse time as laps.
        if self.last_time is not None:
            if stamp <= self.last_time:
                if stamp < self.last_time:
                    self.on_reset(String(data='pose clock reversal'))
                return
            if stamp-self.last_time < .045:
                return
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        try:
            theta, contour, lag = self.track.project([pos.x, pos.y], yaw, self.previous)
            r, _, n, _, right, left = self.track.sample(theta)
            x = np.array([[pos.x, pos.y, yaw, 0., 0., theta]])
            corners = footprint_points(x, self.config, 1.)[0]
            offsets = (corners-r) @ n
            violation = max(0., float(offsets.max()-left+self.config.clearance),
                            float(-right+self.config.clearance-offsets.min()))
            lap = self.laps.update(theta, stamp, self.lap_max_speed)
            self.previous, self.last_time = theta, stamp
            odom_stamp = (self.odom.header.stamp.sec+self.odom.header.stamp.nanosec*1e-9
                          if self.odom is not None else None)
            fresh_odom = odom_stamp is not None and 0 <= ros_now-odom_stamp <= self.config.max_input_age
            speed = self.odom.twist.twist.linear.x if fresh_odom else None
            yaw_rate = self.odom.twist.twist.angular.z if fresh_odom else None
            command_stamp = (self.command.header.stamp.sec+self.command.header.stamp.nanosec*1e-9
                             if self.command is not None else None)
            fresh_command = command_stamp is not None and 0 <= ros_now-command_stamp <= self.config.max_input_age
            body = footprint_points(x, self.config, self.free_map.resolution*.5)
            min_clearance = float(np.min(self.free_map.clearance(body)))
            def finite(value):
                return float(value) if value is not None and math.isfinite(value) else None
            self.log.record(dict(kind='state', ros_s=stamp, monotonic_s=time.monotonic(),
                x_m=pos.x, y_m=pos.y, yaw_rad=yaw, speed_mps=finite(speed),
                yaw_rate_radps=finite(yaw_rate), projected_progress_m=theta, contour_m=contour, lag_m=lag,
                boundary_violation_m=violation, lap_time_s=lap,
                min_clearance_m=min_clearance,
                map_violation_m=max(0., self.config.clearance-min_clearance),
                command_speed_mps=finite(self.command.drive.speed) if fresh_command else None,
                command_steering_rad=finite(self.command.drive.steering_angle) if fresh_command else None,
                dropped_rows=self.log.dropped))
        except ValueError as exc:
            self.log.record(dict(kind='invalid_projection', reason=str(exc), monotonic_s=time.monotonic()))
            self.laps.reset()

    def close(self):
        self.log.close()
        summary = summarize(self.log.directory)
        (self.log.directory/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    previous_term = signal.getsignal(signal.SIGTERM)
    def interrupted(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    node = None
    try:
        node = ControllerMetrics()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        if node is not None:
            node.close()
            node.destroy_node()
        rclpy.try_shutdown()
        signal.signal(signal.SIGTERM, previous_term)
