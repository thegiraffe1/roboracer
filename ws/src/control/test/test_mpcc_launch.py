"""End-to-end installed launch + real acados worker on an isolated ROS domain."""
import math
import os
import signal
import subprocess
import time

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from test_mpcc import circle, free_map


@pytest.mark.skipif(not os.environ.get('ACADOS_SOURCE_DIR'), reason='requires pinned acados and built workspace')
def test_installed_launch_real_solver_metrics_and_shutdown(tmp_path):
    track = circle()
    path = tmp_path/'centerline.csv'
    np.savetxt(path, np.column_stack([track.xy, track.width]), delimiter=',')
    free_map(tmp_path)
    env = dict(os.environ, ROS_DOMAIN_ID='67', OPENBLAS_NUM_THREADS='1')
    command = ['ros2', 'launch', 'control', 'mpcc.launch.py', 'use_sim_time:=false',
        'centerline_path:='+str(path), 'map_yaml:='+str(tmp_path/'map.yaml'),
        'pose_topic:=/mpcc_launch_pose', 'odom_topic:=/mpcc_launch_odom',
        'log_root:='+str(tmp_path/'logs')]
    rclpy.init(domain_id=67)
    node = rclpy.create_node('mpcc_launch_test')
    poses = node.create_publisher(PoseStamped, '/mpcc_launch_pose', 10)
    odoms = node.create_publisher(Odometry, '/mpcc_launch_odom', 10)
    drives = []
    node.create_subscription(AckermannDriveStamped, '/drive', drives.append, 10)
    with (tmp_path/'launch.log').open('w') as log:
        process = subprocess.Popen(command, env=env, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        try:
            end = time.monotonic()+45
            while time.monotonic() < end and not any(d.drive.speed > 0 for d in drives):
                assert process.poll() is None, (tmp_path/'launch.log').read_text()
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.header.stamp = node.get_clock().now().to_msg()
                pose.pose.position.x = 3.
                pose.pose.orientation.z = math.sin(math.pi/4)
                pose.pose.orientation.w = math.cos(math.pi/4)
                odom = Odometry()
                odom.header.stamp = pose.header.stamp
                odom.child_frame_id = 'base_link'
                poses.publish(pose)
                odoms.publish(odom)
                rclpy.spin_once(node, timeout_sec=.01)
                time.sleep(.01)
            assert any(d.drive.speed > 0 for d in drives), (tmp_path/'launch.log').read_text()
            os.killpg(process.pid, signal.SIGINT)
            end = time.monotonic()+8
            while process.poll() is None and time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=.02)
            assert process.poll() == 0, (tmp_path/'launch.log').read_text()
            # Drain the final stop burst already in DDS.
            for _ in range(20):
                rclpy.spin_once(node, timeout_sec=.01)
            assert drives[-1].drive.speed == 0
            logs = list((tmp_path/'logs').glob('mpcc-*'))
            assert len(logs) == 1
            assert (logs[0]/'summary.json').exists()
            assert 'Traceback' not in (tmp_path/'launch.log').read_text()
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)
            node.destroy_node()
            rclpy.try_shutdown()
