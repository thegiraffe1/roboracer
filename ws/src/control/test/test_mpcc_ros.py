"""ROS transport and safety with a controlled optimizer; no live simulator."""
import math
import time

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from control.mpcc_node import MPCC
from test_mpcc import circle


class FakeWorker:
    def __init__(self, *args):
        self.ready, self.error, self.pending = True, '', None
        self.epoch = 0
        self.failure = None
        self.commands = True

    def invalidate(self):
        self.epoch += 1
        self.pending = None

    def submit(self, state, now):
        self.pending = list(state)
        return True

    def poll(self, now):
        if self.pending is None:
            return None
        self.pending = None
        return 'result', dict(ok=self.failure is None, reason=self.failure or '',
                              speed=.1, steering=.01, states=[])

    def hung(self, now):
        return False

    def close(self):
        pass


def test_mpcc_ros_fresh_state_failure_reset_and_recovery(tmp_path):
    track = circle()
    # Use original equally-spaced coordinates to serialize fixture widths.
    np.savetxt(tmp_path/'centerline.csv', np.column_stack([track.xy, track.width]), delimiter=',')
    rclpy.init(domain_id=64, args=['--ros-args', '-p', 'use_sim_time:=false',
        '-p', 'wheelbase:=0.33', '-p', 'max_steering_angle:=0.524',
        '-p', 'centerline_path:='+str(tmp_path/'centerline.csv'),
        '-p', 'pose_topic:=/mpcc_test_pose', '-p', 'odom_topic:=/mpcc_test_odom'])
    node = MPCC(worker_factory=FakeWorker)
    tester = rclpy.create_node('mpcc_safety_test')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(tester)
    poses = tester.create_publisher(PoseStamped, '/mpcc_test_pose', 10)
    odoms = tester.create_publisher(Odometry, '/mpcc_test_odom', 10)
    drives = []
    tester.create_subscription(AckermannDriveStamped, '/drive', drives.append, 10)

    def spin(duration):
        end = time.monotonic()+duration
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=.002)

    def feed(duration=.45, frame='map', child='base_link', age=0., yaw=math.pi/2, frozen=False):
        end = time.monotonic()+duration
        frozen_stamp = tester.get_clock().now().nanoseconds
        while time.monotonic() < end:
            pose = PoseStamped()
            pose.header.frame_id = frame
            stamp = (frozen_stamp if frozen else tester.get_clock().now().nanoseconds)-int(age*1e9)
            pose.header.stamp.sec, pose.header.stamp.nanosec = divmod(stamp, 10**9)
            pose.pose.position.x = 3.
            pose.pose.orientation.z, pose.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
            odom = Odometry()
            odom.header.stamp = tester.get_clock().now().to_msg()
            odom.child_frame_id = child
            poses.publish(pose)
            odoms.publish(odom)
            spin(.02)

    try:
        spin(.3)
        assert drives and drives[-1].drive.speed == 0
        feed()
        assert drives[-1].drive.speed > 0
        feed(frame='world', duration=.1)
        assert drives[-1].drive.speed == 0
        feed(duration=.1)
        assert drives[-1].drive.speed == 0  # recovery latch
        feed()
        assert drives[-1].drive.speed > 0
        feed(child='odom', duration=.1)
        assert drives[-1].drive.speed == 0
        feed()
        assert drives[-1].drive.speed > 0
        node.worker.failure = 'optimizer deadline exceeded'
        feed(duration=.15)
        assert drives[-1].drive.speed == 0
        node.worker.failure = None
        feed()
        assert drives[-1].drive.speed > 0
        held = drives[-1].drive.steering_angle
        spin(.4)
        assert drives[-1].drive.speed == 0
        assert drives[-1].drive.steering_angle == held
        feed()
        assert drives[-1].drive.speed > 0
        node.reset()
        spin(.03)
        assert drives[-1].drive.speed == 0
        feed(duration=.1)
        assert drives[-1].drive.speed == 0
        feed()
        assert drives[-1].drive.speed > 0
        feed(age=.2, duration=.1)  # skew/time reversal
        assert drives[-1].drive.speed == 0
        feed()
        assert drives[-1].drive.speed > 0
        feed(frozen=True, duration=.45)
        assert drives[-1].drive.speed == 0  # repeated stamps do not refresh the state
    finally:
        executor.remove_node(node)
        executor.remove_node(tester)
        node.destroy_node()
        tester.destroy_node()
        executor.shutdown()
        rclpy.try_shutdown()
