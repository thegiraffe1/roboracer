"""ROS 2 MPCC supervisor. Only this process publishes commands; worker solves."""
from dataclasses import fields
import json
import math
import os
import signal
import time

import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from ackermann_msgs.msg import AckermannDriveStamped
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String

from perception.tracks import track_dir
from .input_health import InputHealth
from .mpcc_model import Config
from .mpcc_track import Track
from .mpcc_worker import Worker
from .mpcc_diagnostics import ControlFault, fault_summary, validate_state_values


def stamp_seconds(stamp):
    return stamp.sec+stamp.nanosec*1e-9


class MPCC(Node):
    def __init__(self, worker_factory=Worker):
        super().__init__('mpcc_node')
        p = lambda name, default: self.declare_parameter(name, default).value
        defaults = Config()
        values = {f.name: p(f.name, Parameter.Type.DOUBLE if f.name in
                           ('wheelbase', 'max_steering_angle') else getattr(defaults, f.name))
                  for f in fields(Config)}
        self.config = Config(**values)
        c = self.config
        track_name = p('track', '')
        directory = track_dir(track_name) if track_name else ''
        centerline = p('centerline_path', '') or os.path.join(directory, 'centerline.csv')
        map_yaml = p('map_yaml', '') or os.path.join(directory, 'map.yaml')
        self.track = Track.load(centerline, c)
        self.worker = worker_factory(c, centerline, map_yaml)
        self.health = InputHealth()
        self.pose = self.odom = None
        self.pose_received = self.odom_received = None
        self.pose_advanced = self.odom_advanced = None
        self.previous = None
        self.last_xy = self.last_stamp = None
        self.last_steer = 0.
        self.next_request = 0.
        self.failed = True
        self.permanent_fault = ''
        self.last_result = {}
        self.last_warning_time = None
        self.publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.diagnostics = self.create_publisher(DiagnosticArray, '/mpcc/diagnostics', 10)
        self.path_publisher = self.create_publisher(Path, '/mpcc/predicted_path', 1)
        self.visualize = p('publish_prediction', False)
        self.create_subscription(PoseStamped, p('pose_topic', '/localized_pose'), self.on_pose, 1)
        self.create_subscription(Odometry, p('odom_topic', '/odom'), self.on_odom, qos_profile_sensor_data)
        reset_topic = p('sim_reset_topic', '/autodrive/roboracer_1/sim_reset')
        if reset_topic:
            self.create_subscription(String, reset_topic, lambda msg: self.reset(), 10)
        # Poll deadlines independently of /clock. A stopped ROS clock must not
        # suppress the safety timer or permit repeated old measurements.
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(min(.005, c.dt/5), self.tick, clock=self.steady_clock)
        self.get_logger().info(f'MPCC acados SQP-RTI: {centerline}; waiting for worker and localized state')

    def on_pose(self, message):
        now = time.monotonic()
        if self.pose is None or stamp_seconds(message.header.stamp) > stamp_seconds(self.pose.header.stamp):
            self.pose_advanced = now
        self.pose, self.pose_received = message, now

    def on_odom(self, message):
        now = time.monotonic()
        if self.odom is None or stamp_seconds(message.header.stamp) > stamp_seconds(self.odom.header.stamp):
            self.odom_advanced = now
        self.odom, self.odom_received = message, now

    def reset(self):
        self.worker.invalidate()
        self.health.reset()
        self.previous = self.last_xy = self.last_stamp = None
        self.pose = self.odom = None
        self.pose_received = self.odom_received = None
        self.pose_advanced = self.odom_advanced = None
        self.failed = True
        # Keep slew reference until stationary; reset does not prove zero steering.

    def publish(self, speed=0., steering=None):
        steering = self.last_steer if steering is None else steering
        if not np.isfinite([speed, steering]).all():
            speed, steering = 0., self.last_steer
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.drive.speed, msg.drive.steering_angle = float(speed), float(steering)
        self.publisher.publish(msg)

    def report(self, result):
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = 'mpcc'
        status.hardware_id = 'autodrive'
        status.level = DiagnosticStatus.OK if result.get('ok') else DiagnosticStatus.ERROR
        status.message = result.get('reason', '') or 'accepted'
        status.values = [KeyValue(key=str(k), value=json.dumps(v, allow_nan=False))
                         for k, v in result.items() if k != 'states']
        msg.status = [status]
        self.diagnostics.publish(msg)

    def stop(self, reason, invalidate=True, details=None):
        if invalidate and not self.failed:
            self.health.fault()
            self.worker.invalidate()
        # Stop before any diagnostic/logging work. Preserve repeated brake commands.
        self.failed = True
        self.publish()
        if (getattr(self, 'last_stop_reason', None) != reason or
                (details and 'publication_time_s' in details)):
            event = dict(ok=False, reason=reason, event='stop')
            if details:
                event['details'] = details
            self.report(event)
            self.last_stop_reason = reason
        # ROS's throttle still performs expensive caller lookup on every call.
        # Gate logging first; stop commands and changed diagnostics stay immediate.
        now = time.monotonic()
        if self.last_warning_time is None or now-self.last_warning_time >= 2.:
            self.last_warning_time = now
            self.get_logger().warning('MPCC stop: '+reason+fault_summary(details))

    def state(self, now, ros_now):
        c = self.config
        yaw_rate = self.odom.twist.twist.angular.z if self.odom is not None else 0.
        fault = self.health.advance(ros_now, now, yaw_rate,
            self.odom_received is not None and now-self.odom_received < c.max_input_age)
        if fault:
            raise ValueError(fault)
        for name, msg, received, advanced in [('pose', self.pose, self.pose_received, self.pose_advanced),
                                            ('odom', self.odom, self.odom_received, self.odom_advanced)]:
            if msg is None:
                raise ValueError('waiting for '+name)
            stamp = stamp_seconds(msg.header.stamp)
            if (stamp <= 0 or not 0 <= ros_now-stamp <= c.max_input_age or
                    now-received > c.max_input_age or advanced is None or now-advanced > c.max_input_age):
                raise ValueError('stale or future '+name)
        ps, od = stamp_seconds(self.pose.header.stamp), stamp_seconds(self.odom.header.stamp)
        if abs(ps-od) > c.max_input_skew:
            raise ControlFault('pose/odometry timestamp skew', pose_odom_skew_s=ps-od,
                               max_input_skew_s=c.max_input_skew,
                               pose_age_s=ros_now-ps, odom_age_s=ros_now-od)
        if self.last_stamp is not None and (ps < self.last_stamp[0] or od < self.last_stamp[1]):
            raise ValueError('input time reversal')
        if self.pose.header.frame_id != 'map':
            raise ControlFault('invalid pose frame', frame=self.pose.header.frame_id, expected='map')
        if self.odom.child_frame_id != 'base_link':
            raise ControlFault('invalid odometry child frame', frame=self.odom.child_frame_id, expected='base_link')
        q, position = self.pose.pose.orientation, self.pose.pose.position
        twist = self.odom.twist.twist
        xy = np.array([position.x, position.y])
        validate_state_values(xy, [q.x, q.y, q.z, q.w],
                              [twist.linear.x, twist.linear.y, twist.angular.z])
        # A measured overshoot of the scaled target is a braking problem, not an
        # invalid localization sample. The solver must explicitly recover it;
        # readings beyond the absolute configured limit still fault immediately.
        if twist.linear.x > c.max_speed+c.feasibility_tolerance:
            raise ValueError('measured speed exceeds absolute MPCC limit')
        if self.last_xy is not None and np.linalg.norm(xy-self.last_xy) > c.max_pose_jump:
            raise ValueError('implausible pose jump')
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        fault = self.health.accept(yaw, (ps, od), now)
        if fault:
            if fault == 'waiting for consistent recovery samples':
                raise ControlFault(fault, recovery_samples=self.health.valid_count,
                                   recovery_required_samples=self.health.recovery_samples,
                                   recovery_elapsed_s=now-self.health.valid_since,
                                   recovery_required_s=self.health.recovery_seconds)
            raise ValueError(fault)
        theta, contour, _ = self.track.project(xy, yaw, self.previous)
        if abs(contour) > c.max_offset:
            raise ValueError('pose outside track acquisition limit')
        # Bounded first-order alignment to newest stamp, using measured body speed
        # and yaw rate. This is an estimate, not a simulator-state input.
        delay = ros_now-ps
        aligned_yaw = yaw+twist.angular.z*delay
        xy += twist.linear.x*delay*np.array([math.cos(yaw), math.sin(yaw)])
        self.previous = theta
        self.last_xy, self.last_stamp = np.array([position.x, position.y]), (ps, od)
        return np.r_[xy, aligned_yaw, twist.linear.x, self.last_steer, theta]

    def tick(self):
        now = time.monotonic()
        input_fault = True
        try:
            event = self.worker.poll(now)
            if self.worker.hung(now) and not self.permanent_fault:
                # Kill without joining in the safety callback. Reinitialization is
                # deliberately latched: restart the node after a genuinely hung solve.
                self.worker.process.terminate()
                self.permanent_fault = 'optimizer hung; restart controller'
            ros_now = self.get_clock().now().nanoseconds*1e-9
            if self.permanent_fault:
                return self.stop(self.permanent_fault)
            if event and (event[0] == 'fault' or
                          (event[0] == 'result' and not event[1].get('ok', False))):
                result = dict(event[1], ok=False)
                self.last_result = result
                self.report(result)
                self.worker.invalidate()
                # A failed candidate invalidates optimizer state, not healthy sensors.
                return self.stop(result.get('reason', 'optimizer failure'), invalidate=False, details={
                    'violated_constraints': result.get('violated_constraints', [])})
            state = self.state(now, ros_now)
            input_fault = False
            if event and event[0] in ('fault', 'result'):
                result = event[1]
                result['ok'] = bool(result.get('ok', False))
                self.last_result = result
                if not all(math.isfinite(result.get(k, float('nan'))) for k in ('speed', 'steering')):
                    raise ValueError('nonfinite optimizer command')
                if (not 0 <= result['speed'] <= self.config.speed_cap or
                        abs(result['steering']) > self.config.max_steering_angle or
                        abs(result['steering']-self.last_steer) > self.config.steering_rate*self.config.dt+1e-6):
                    raise ValueError('optimizer command exceeds actuator limits')
                elapsed = result.get('request_time_s', 0.)+time.monotonic()-now
                if elapsed >= self.config.deadline:
                    raise ControlFault('publication deadline exceeded', publication_time_s=elapsed,
                                       request_time_s=result.get('request_time_s'),
                                       deadline_s=self.config.deadline,
                                       deadline_overrun_s=elapsed-self.config.deadline)
                ros_after = self.get_clock().now().nanoseconds*1e-9
                if any(not 0 <= ros_after-stamp_seconds(msg.header.stamp) <= self.config.max_input_age
                       for msg in (self.pose, self.odom)):
                    input_fault = True
                    raise ValueError('inputs expired before publication')
                self.last_steer = result['steering']
                self.last_stop_reason = None
                self.publish(result['speed'], self.last_steer)
                self.failed = False
                self.report(result)
                if self.visualize:
                    self.publish_path(result['states'])
            if not self.worker.ready:
                return self.stop(self.worker.error or 'optimizer initializing', invalidate=False)
            if self.worker.error:
                return self.stop(self.worker.error)
            if now >= self.next_request and self.worker.pending is None:
                state[4] = self.last_steer
                if self.worker.submit(state, now):
                    self.next_request = now+self.config.dt
            if self.failed:
                self.publish()
        except (ValueError, IndexError, FloatingPointError, OverflowError) as exc:
            # Invalidate even before the first successful command. Rejected input
            # must never leave a pending old-state result eligible for publication.
            waiting = str(exc).startswith('waiting for consistent')
            if not waiting or self.worker.pending is not None:
                self.worker.invalidate()
            if input_fault and not waiting:
                self.health.fault()
            self.stop(str(exc), invalidate=False, details=getattr(exc, 'details', None))

    def publish_path(self, states):
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        for x in states:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x, pose.pose.position.y = float(x[0]), float(x[1])
            pose.pose.orientation.z, pose.pose.orientation.w = math.sin(x[2]/2), math.cos(x[2]/2)
            path.poses.append(pose)
        self.path_publisher.publish(path)

    def close(self):
        for _ in range(10):
            self.publish()
            time.sleep(.05)
        self.worker.close()


def main():
    # Keep the ROS context alive while publishing the shutdown stop burst.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    previous_term = signal.getsignal(signal.SIGTERM)
    def interrupted(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    node = None
    try:
        node = MPCC()
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
