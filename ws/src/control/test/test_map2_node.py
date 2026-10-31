"""Exercise the actual control callback with ROS messages and a deterministic solver.

No DDS participants are created and no command is sent to a running simulator.
"""
from types import SimpleNamespace
import time

import numpy as np
import pytest

pytest.importorskip('ackermann_msgs')
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from control.map2 import Reference
from control.map2_node import Map2


@pytest.fixture
def controller():
    rows = np.zeros((120, 7))
    t = np.linspace(0, 2 * np.pi, 120, endpoint=False)
    rows[:, 1:3] = np.column_stack([2 * np.cos(t), 2 * np.sin(t)])
    rows[:, 5] = 2.
    node = SimpleNamespace(reference=Reference(rows), max_age=0.5, max_offset=0.5,
                           max_speed=3., wheelbase=0.33, max_steer=0.524,
                           steer_rate=2., last_good=None, last_steer=0.,
                           failure_since=None, stop_burst=1.5,
                           config={'dt': 0.05, 'horizon_nodes': 20, 'v_x_min': 0.1})
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_000_000_000))
    node.get_logger = lambda: SimpleNamespace(warning=lambda *args, **kw: None)
    node.commands = []
    node.publish = lambda speed=0., steer=0.: node.commands.append((speed, steer))
    node.fail = lambda reason: Map2.fail(node, reason)
    node.pose = PoseStamped()
    node.pose.header.frame_id = 'map'
    node.pose.header.stamp.sec = 10
    node.pose.pose.position.x = 2.
    yaw = node.reference.heading[0]
    node.pose.pose.orientation.z = np.sin(yaw / 2)
    node.pose.pose.orientation.w = np.cos(yaw / 2)
    node.odom = Odometry()
    node.odom.header.stamp.sec = 10
    node.odom.child_frame_id = 'base_link'
    node.pose_received = node.odom_received = time.monotonic()
    node.solver = SimpleNamespace(solve=lambda *args: (
        np.array([1., 0., 0.5]), None, np.zeros((21, 3)), 0))
    return node


def test_drives_and_limits_steering(controller):
    Map2.control(controller)
    assert controller.commands[-1] == pytest.approx((1., 0.1))


@pytest.mark.parametrize('failure', ['pose_age', 'odom_age', 'arrival_age', 'frame', 'quaternion',
                                     'odom_frame', 'nan_odom', 'solver_status', 'nan_solver', 'exception'])
def test_failures_stop(controller, failure):
    n = controller
    if failure == 'pose_age':
        n.pose.header.stamp.sec = 9
    elif failure == 'odom_age':
        n.odom.header.stamp.sec = 11
    elif failure == 'arrival_age':
        n.pose_received -= 1.
    elif failure == 'frame':
        n.pose.header.frame_id = 'world'
    elif failure == 'quaternion':
        n.pose.pose.orientation.w = 10.
    elif failure == 'odom_frame':
        n.odom.child_frame_id = 'world'
    elif failure == 'nan_odom':
        n.odom.twist.twist.linear.x = float('nan')
    elif failure == 'solver_status':
        n.solver.solve = lambda *args: (2., None, np.zeros((21, 3)), 1)
    elif failure == 'nan_solver':
        n.solver.solve = lambda *args: (np.array([1., 0., np.nan]), None, np.zeros((21, 3)), 0)
    elif failure == 'exception':
        def broken(*args):
            raise RuntimeError('solver unavailable')
        n.solver.solve = broken
    Map2.control(n)
    assert n.commands[-1] == (0., 0.)
    assert n.last_good is None


def test_stop_burst_expires_and_control_recovers(controller):
    n = controller
    n.pose.header.frame_id = 'world'
    Map2.control(n)
    n.failure_since -= 2.
    Map2.control(n)
    assert len(n.commands) == 1
    n.pose.header.frame_id = 'map'
    Map2.control(n)
    assert n.commands[-1][0] == 1.
    assert n.failure_since is None


def test_late_solution_stops(controller, monkeypatch):
    current = time.monotonic()
    ticks = iter([current, current + 0.06, current + 0.06])
    monkeypatch.setattr('control.map2_node.time.monotonic', lambda: next(ticks))
    Map2.control(controller)
    assert controller.commands[-1] == (0., 0.)


def test_zero_reference_stops(controller):
    controller.reference.speed[:] = 0.
    Map2.control(controller)
    assert controller.commands[-1] == (0., 0.)


def test_missing_input_stops(controller):
    controller.odom = None
    Map2.control(controller)
    assert controller.commands[-1] == (0., 0.)
