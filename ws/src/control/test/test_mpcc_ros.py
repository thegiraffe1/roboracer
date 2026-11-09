"""ROS transport and safety with a controlled optimizer; no live simulator."""
import math
import time
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from control.mpcc_node import MPCC
from control.mpcc_model import Config
from control.input_health import InputHealth
from test_mpcc import circle


@pytest.mark.parametrize('deadline', [.035, .045])
def test_publication_deadline_records_elapsed_and_overrun(monkeypatch, deadline):
    clock = [0.]
    monkeypatch.setattr('control.mpcc_node.time', SimpleNamespace(monotonic=lambda: clock[0]))
    result = dict(ok=True, speed=.1, steering=0., request_time_s=deadline-.001, states=[])
    worker = Mock(pending=None)
    worker.poll.return_value = ('result', result)
    worker.hung.return_value = False
    logger = Mock()
    node = SimpleNamespace(config=Config(deadline=deadline), worker=worker, permanent_fault='', health=Mock(),
                           failed=False, last_steer=0., last_warning_time=None,
                           publish=Mock(), report=Mock(), get_logger=Mock(return_value=logger),
                           get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10**9)))

    def state(now, ros_now):
        clock[0] += .003  # state validation crosses the configured deadline
        return np.zeros(6)

    node.state = state
    node.stop = lambda *args, **kwargs: MPCC.stop(node, *args, **kwargs)
    MPCC.tick(node)
    node.publish.assert_called_once_with()  # only the stop, never the candidate
    details = node.report.call_args.args[0]['details']
    assert details['publication_time_s'] == pytest.approx(deadline+.002)
    assert details['deadline_overrun_s'] == pytest.approx(.002)
    assert details['deadline_s'] == pytest.approx(deadline)
    assert f'{(deadline+.002)*1000:.2f} ms total, 2.00 ms late' in logger.warning.call_args.args[0]
    worker.invalidate.assert_called_once()
    node.health.fault.assert_not_called()  # computation lateness is not a sensor fault


@pytest.mark.parametrize('kind,reason', [('fault', 'optimizer deadline exceeded'),
                                        ('result', 'nonlinear rollout violation')])
def test_solver_failure_can_retry_without_restarting_input_recovery(monkeypatch, kind, reason):
    clock = [0.]
    monkeypatch.setattr('control.mpcc_node.time', SimpleNamespace(monotonic=lambda: clock[0]))
    health = InputHealth()
    health.recovering = False
    health.expected_yaw = 0.
    health.fault = Mock(wraps=health.fault)
    worker = Mock(ready=True, error='', pending=None)
    worker.poll.return_value = (kind, dict(ok=False, reason=reason))
    worker.hung.return_value = False
    stamp = SimpleNamespace(sec=1, nanosec=0)
    message = SimpleNamespace(header=SimpleNamespace(stamp=stamp))
    node = SimpleNamespace(config=Config(deadline=.045), worker=worker, permanent_fault='', health=health,
                           failed=False, last_steer=0., last_warning_time=None,
                           publish=Mock(), report=Mock(), get_logger=Mock(return_value=Mock()),
                           pose=message, odom=message, visualize=False, next_request=100.,
                           get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10**9)))

    def state(now, ros_now):
        assert health.accept(0., (ros_now, ros_now), now) is None
        return np.zeros(6)

    node.state = state
    node.stop = lambda *args, **kwargs: MPCC.stop(node, *args, **kwargs)
    MPCC.tick(node)
    node.publish.assert_called_once_with()
    assert node.failed
    worker.invalidate.assert_called_once()
    health.fault.assert_not_called()

    clock[0] = .005  # next poll, well before the 250 ms sensor recovery duration
    worker.poll.return_value = ('result', dict(ok=True, speed=.1, steering=0., request_time_s=.04))
    MPCC.tick(node)
    node.publish.assert_called_with(.1, 0.)
    assert not node.failed
    health.fault.assert_not_called()


def test_stop_rate_limits_logger_calls_without_delaying_safety(monkeypatch):
    clock = [0.]
    monkeypatch.setattr('control.mpcc_node.time', SimpleNamespace(monotonic=lambda: clock[0]))
    logger = Mock()
    node = SimpleNamespace(failed=False, health=Mock(), worker=Mock(),
                           last_warning_time=None, publish=Mock(), report=Mock(),
                           get_logger=Mock(return_value=logger))

    MPCC.stop(node, 'missing pose')
    for i in range(1, 400):
        clock[0] = i*.005
        MPCC.stop(node, 'missing pose')
    assert node.publish.call_count == 400  # stop on every invocation
    node.report.assert_called_once_with(dict(ok=False, reason='missing pose', event='stop'))
    node.health.fault.assert_called_once()
    node.worker.invalidate.assert_called_once()
    node.get_logger.assert_called_once()  # do not even enter ROS logging again
    logger.warning.assert_called_once_with('MPCC stop: missing pose')

    # A different fault is reported immediately, even inside the warning window.
    MPCC.stop(node, 'stale odom')
    node.report.assert_called_with(dict(ok=False, reason='stale odom', event='stop'))
    assert node.publish.call_count == 401
    assert logger.warning.call_count == 1

    clock[0] = 2.
    MPCC.stop(node, 'stale odom')
    assert node.get_logger.call_count == 2
    logger.warning.assert_called_with('MPCC stop: stale odom')
    clock[0] = 2.005
    MPCC.stop(node, 'stale odom')
    assert node.publish.call_count == 403
    assert logger.warning.call_count == 2


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
        '-p', 'speed_scale:=0.3',
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

    def feed(duration=.45, frame='map', child='base_link', age=0., yaw=math.pi/2, frozen=False, speed=0.):
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
            odom.twist.twist.linear.x = speed
            poses.publish(pose)
            odoms.publish(odom)
            spin(.02)

    try:
        spin(.3)
        assert drives and drives[-1].drive.speed == 0
        feed()
        assert drives[-1].drive.speed > 0
        feed(speed=1.04, duration=.15)
        assert drives[-1].drive.speed > 0  # target overshoot is valid input for a braking solve
        feed(speed=3.2, duration=.1)
        assert drives[-1].drive.speed == 0  # absolute limit still faults immediately
        feed(speed=-.01, duration=.1)
        assert drives[-1].drive.speed == 0  # negative measured state is not silently clipped
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
