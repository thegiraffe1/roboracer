#!/usr/bin/env python3
"""AutoDRIVE-only ROS 2 MAP2 adapter; external acados installation required."""
import math
import os
import time

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions

from perception.tracks import track_dir
from control.map2 import Reference, sim_steering
from control.map2_vendor.acados_mpc_map2 import AcadosMPCMap2
from control.map2_vendor.mpc_map2_core import build_stage_parameters


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class Map2(Node):
    def __init__(self):
        super().__init__('map2_node')
        p = lambda name, default: self.declare_parameter(name, default).value
        track = p('track', '')
        path = p('raceline_path', '') or os.path.join(track_dir(track), 'raceline.csv')
        pose_topic = p('pose_topic', '/localized_pose')
        odom_topic = p('odom_topic', '/odom')
        self.max_speed = p('max_speed', 3.0)
        speed_scale = p('speed_scale', 0.5)
        self.max_age = p('max_input_age', 0.5)
        self.max_offset = p('max_offset', 0.5)
        self.stop_burst = p('stop_burst', 1.5)
        self.wheelbase = p('wheelbase', Parameter.Type.DOUBLE)
        self.max_steer = p('max_steering_angle', Parameter.Type.DOUBLE)
        self.steer_rate = p('steering_rate', 2.0)  # rad/s, unlike upstream's per-cycle limit
        config = {name: p(name, default) for name, default in {
            'horizon_nodes': 20, 'dt': 0.05, 'v_x_min': 0.1,
            'v_y_max': 0.2, 'psi_dot_max': 4.0,
            'q_n': 100.0, 'q_dphi': 30.0, 'q_v': 10.0,
            'r_vy': 10.0, 'r_psi': 0.1, 'rho': 500.0,
        }.items()}
        margin = p('reference_margin', 0.4)
        positive = [self.max_speed, self.max_age, self.max_offset, self.stop_burst,
                    self.wheelbase, self.max_steer, self.steer_rate, config['dt'], config['v_x_min']]
        if any(v is None or not math.isfinite(v) or v <= 0 for v in positive):
            raise ValueError('geometry, limits and timing must be finite and positive; load vehicle.yaml')
        if config['horizon_nodes'] < 2 or config['v_x_min'] >= self.max_speed:
            raise ValueError('horizon_nodes must be >= 2 and v_x_min < max_speed')
        for name in ('v_y_max', 'psi_dot_max', 'q_n', 'q_dphi', 'q_v', 'r_vy', 'r_psi', 'rho'):
            if not math.isfinite(config[name]) or config[name] < 0:
                raise ValueError(f'{name} must be finite and non-negative')
        config['v_x_max'] = self.max_speed
        self.reference = Reference(np.loadtxt(path, delimiter=';', ndmin=2), speed_scale,
                                   self.max_speed, margin)
        self.config = config
        self.solver = AcadosMPCMap2(config, logger=self.get_logger().info)
        self.pose = self.odom = None
        self.pose_received = self.odom_received = None
        self.last_steer = 0.
        self.last_good = None
        self.failure_since = None
        self.publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.create_subscription(PoseStamped, pose_topic, self.on_pose, 1)
        self.create_subscription(Odometry, odom_topic, self.on_odom, qos_profile_sensor_data)
        self.create_timer(config['dt'], self.control)
        self.get_logger().info(f'MAP2 AutoDRIVE baseline: {path}; analytic steering, GP disabled')

    def on_pose(self, msg):
        self.pose, self.pose_received = msg, time.monotonic()

    def on_odom(self, msg):
        self.odom, self.odom_received = msg, time.monotonic()

    def publish(self, speed=0., steer=0.):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steer)
        self.publisher.publish(msg)

    def fail(self, reason):
        now = time.monotonic()
        if self.failure_since is None:
            self.failure_since = now
            self.reference.previous_s = None
            self.solver._last_x = self.solver._last_u = None
        if now - self.failure_since <= self.stop_burst:
            self.publish()
        self.last_steer = 0.
        self.last_good = None
        self.get_logger().warning(reason, throttle_duration_sec=2.)

    def control(self):
        now = time.monotonic()
        ros_now = self.get_clock().now().nanoseconds * 1e-9
        try:
            for name, msg, received in [('pose', self.pose, self.pose_received),
                                        ('odom', self.odom, self.odom_received)]:
                if msg is None:
                    raise ValueError(f'waiting for {name}')
                age = ros_now - stamp_seconds(msg.header.stamp)
                if stamp_seconds(msg.header.stamp) <= 0 or not 0 <= age <= self.max_age or now - received > self.max_age:
                    raise ValueError(f'stale or future {name}')
            if self.pose.header.frame_id != 'map' or self.odom.child_frame_id != 'base_link':
                raise ValueError('pose must be in map and odometry twist in base_link')
            q = self.pose.pose.orientation
            position = self.pose.pose.position
            values = [position.x, position.y, position.z, q.x, q.y, q.z, q.w]
            if not np.isfinite(values).all() or abs(np.linalg.norm(values[3:]) - 1.) > 0.1:
                raise ValueError('invalid pose or quaternion')
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y**2 + q.z**2))
            twist = self.odom.twist.twist
            if not np.isfinite([twist.linear.x, twist.linear.y, twist.angular.z]).all():
                raise ValueError('non-finite odometry twist')
            x0, s = self.reference.project(position.x, position.y, yaw, self.max_offset)
            profile = self.reference.profile(s)
            params, refs, yaw_refs = build_stage_parameters(
                profile, twist.linear.x, self.config['horizon_nodes'],
                self.config['dt'], self.config['v_x_min'])
            if abs(1 - params[0, 0] * x0[1]) < 0.2:
                raise ValueError('near-singular Frenet state')
            # A zero-speed waypoint means stop; the MPC minimum must not turn it into creep.
            if profile.sample(0.)[0] <= 0.:
                raise ValueError('zero reference speed')
            u, _, states, status = self.solver.solve(x0, params, refs, yaw_refs)
            if status != 0 or not np.isfinite(u).all() or not np.isfinite(states).all():
                raise ValueError(f'MAP2 solve failed: status {status}')
            # Drop late solutions instead of driving on inputs that expired during the solve.
            if time.monotonic() - now > self.config['dt']:
                raise ValueError('MAP2 solve exceeded the control period')
            ros_after = self.get_clock().now().nanoseconds * 1e-9
            if any(not 0 <= ros_after - stamp_seconds(msg.header.stamp) <= self.max_age
                   for msg in (self.pose, self.odom)):
                raise ValueError('inputs expired during the solve')
            steer = sim_steering(u, self.wheelbase, self.max_steer, self.config['v_x_min'])
            delta = self.steer_rate * min(now - self.last_good, self.config['dt']) if self.last_good else self.steer_rate * self.config['dt']
            steer = float(np.clip(steer, self.last_steer - delta, self.last_steer + delta))
            self.publish(np.clip(u[0], 0., self.max_speed), steer)
            self.last_steer, self.last_good, self.failure_since = steer, now, None
        except Exception as exc:
            self.fail(str(exc))


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = Map2()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            for _ in range(10):
                node.publish()
                time.sleep(0.05)
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
