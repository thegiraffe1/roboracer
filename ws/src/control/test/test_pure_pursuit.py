"""Functional test: launch pure_pursuit.launch.py on synthetic racelines, publish poses, check /drive.

Needs a sourced ROS 2 environment with the control package built:
    python3 -m pytest ws/src/control/test
"""
import math
import os
import random
import signal
import subprocess
import tempfile
import time

import numpy as np
import pytest
import yaml

rclpy = pytest.importorskip('rclpy')
from ackermann_msgs.msg import AckermannDriveStamped  # noqa: E402
from geometry_msgs.msg import PoseStamped  # noqa: E402
from nav_msgs.msg import Path  # noqa: E402
from std_msgs.msg import Bool  # noqa: E402
from visualization_msgs.msg import Marker, MarkerArray  # noqa: E402

SPEED_SCALE = 0.5
DOMAIN = random.randint(1, 101)  # not 0: the live stack (bridge, sim car) runs there
with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config', 'vehicle.yaml')) as f:
    MAX_STEER = yaml.safe_load(f)['/**']['ros__parameters']['max_steering_angle']
HEADER = '# s_m; x_m; y_m; psi_rad; kappa_radpm; vx_mps; ax_mps2'


def write_raceline(tracks_dir, name, xy, psi, kappa, vx):
    d = os.path.join(tracks_dir, name)
    os.makedirs(d, exist_ok=True)
    s = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(xy, axis=0).T))])
    n = len(xy)
    rows = np.column_stack([s, xy, psi, np.full(n, kappa), np.full(n, vx), np.zeros(n)])
    np.savetxt(os.path.join(d, 'raceline.csv'), rows, delimiter=';', fmt='%.7f', header=HEADER[2:])


def circle(radius, n):
    t = np.linspace(0, 2 * np.pi, n, endpoint=False)  # counter-clockwise
    return np.column_stack([radius * np.cos(t), radius * np.sin(t)]), t + np.pi / 2


def hairpin(length, gap, step):
    """Counter-clockwise loop: +x along y = 0, a half turn, -x back along y = gap."""
    x = np.arange(0.0, length, step)
    t = np.linspace(-np.pi / 2, np.pi / 2, int(np.pi * gap / 2 / step), endpoint=False)
    r = gap / 2
    xy = np.concatenate([np.column_stack([x, np.zeros_like(x)]),
                         np.column_stack([length + r * np.cos(t), r + r * np.sin(t)]),
                         np.column_stack([length - x, np.full_like(x, gap)]),
                         np.column_stack([-r * np.cos(t), r - r * np.sin(t)])])
    seg = np.roll(xy, -1, axis=0) - xy
    return xy, np.arctan2(seg[:, 1], seg[:, 0])


@pytest.fixture(scope='module')
def tracks_dir():
    with tempfile.TemporaryDirectory() as d:
        xy, psi = circle(4.0, 200)
        write_raceline(d, 'circle', xy, psi, 0.25, 4.0)
        x = np.arange(0.0, 20.0, 0.2)
        write_raceline(d, 'straight', np.column_stack([x, np.zeros_like(x)]), np.zeros_like(x), 0.0, 3.0)
        xy, psi = circle(0.2, 10)  # every waypoint within the lookahead distance
        write_raceline(d, 'tiny', xy, psi, 5.0, 1.0)
        xy, psi = hairpin(10.0, 1.0, 0.1)
        write_raceline(d, 'hairpin', xy, psi, 0.0, 3.0)
        yield d


@pytest.fixture(scope='module')
def ros():
    rclpy.init(domain_id=DOMAIN)
    node = rclpy.create_node('pure_pursuit_test')
    yield node
    node.destroy_node()
    rclpy.shutdown()


def launch(tracks_dir, track, *args):
    env = dict(os.environ, TRACKS_DIR=tracks_dir, ROS_DOMAIN_ID=str(DOMAIN))
    return subprocess.Popen(
        ['ros2', 'launch', 'control', 'pure_pursuit.launch.py', f'track:={track}',
         f'speed_scale:={SPEED_SCALE}', *args],
        env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, start_new_session=True)


def stop(proc):
    if proc.poll() is None:
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait()
    return proc.stdout.read()


class Driver:
    def __init__(self, node):
        self.node = node
        self.pub = node.create_publisher(PoseStamped, '/tracked_pose', 10)
        self.drives = []
        self.sub = node.create_subscription(AckermannDriveStamped, '/drive', self.drives.append, 10)
        self.markers = []
        self.marker_sub = node.create_subscription(
            MarkerArray, '/pure_pursuit/markers', self.markers.append, 10)

    def wait_for_node(self, proc, timeout=20.0):
        end = time.time() + timeout
        while self.pub.get_subscription_count() == 0:
            assert proc.poll() is None, 'node exited:\n' + proc.stdout.read()
            assert time.time() < end, 'pure_pursuit_node did not subscribe to /tracked_pose'
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def drive(self, x, y, yaw, n=10, frame='map', age=0.0):
        """Publish the same pose n times, stamped age s ago, and return the last /drive."""
        self.drives.clear()
        msg = PoseStamped()
        msg.header.frame_id = frame
        msg.pose.position.x, msg.pose.position.y = x, y
        msg.pose.orientation.z, msg.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        for _ in range(n):
            t = self.node.get_clock().now().nanoseconds * 1e-9 - age
            msg.header.stamp.sec, msg.header.stamp.nanosec = int(t), int((t % 1) * 1e9)
            self.pub.publish(msg)
            end = time.time() + 0.1
            while time.time() < end:
                rclpy.spin_once(self.node, timeout_sec=0.02)
        end = time.time() + 2.0
        while len(self.drives) < n and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        assert self.drives, f'no /drive for pose ({x}, {y}, {yaw})'
        d = self.drives[-1].drive
        assert math.isfinite(d.steering_angle) and abs(d.steering_angle) <= MAX_STEER + 1e-6
        return d

    def close(self):
        self.node.destroy_publisher(self.pub)
        self.node.destroy_subscription(self.sub)
        self.node.destroy_subscription(self.marker_sub)


@pytest.fixture
def run(ros, tracks_dir):
    procs, drivers = [], []

    def start(track, *args):
        proc = launch(tracks_dir, track, *args)
        driver = Driver(ros)
        procs.append(proc)
        drivers.append(driver)
        driver.wait_for_node(proc)
        return driver, proc

    yield start
    for d in drivers:
        d.close()
    for p in procs:
        out = stop(p)
        # the only traceback allowed is rclpy.spin interrupted by our SIGINT
        assert out.count('Traceback') == out.count('] KeyboardInterrupt'), out


def test_circle(run):
    driver, _ = run('circle')
    on = driver.drive(4.0, 0.0, math.pi / 2)  # on the path heading along it (counter-clockwise)
    assert 0.0 < on.steering_angle
    assert on.speed == pytest.approx(4.0 * SPEED_SCALE)
    inside = driver.drive(3.0, 0.0, math.pi / 2)  # path is to the right
    assert inside.steering_angle < 0.0
    outside = driver.drive(5.0, 0.0, math.pi / 2)  # path is to the left: merge toward it, ahead along the line
    assert 0.0 < outside.steering_angle <= MAX_STEER


def test_straight(run):
    driver, _ = run('straight')  # kappa = 0 everywhere
    left = driver.drive(5.0, 0.5, 0.0)
    assert left.steering_angle < 0.0
    assert left.speed == pytest.approx(3.0 * SPEED_SCALE)
    right = driver.drive(5.0, -0.5, 0.0)
    assert right.steering_angle > 0.0
    assert driver.drive(5.0, 0.0, 0.0).steering_angle == pytest.approx(0.0, abs=1e-6)


def test_visualization_markers(run):
    driver, _ = run('straight')
    yaw = 0.2
    drive = driver.drive(5.0, 0.5, yaw)
    assert driver.markers
    target, arrow = driver.markers[-1].markers
    assert (target.id, arrow.id) == (0, 1)
    assert target.type == Marker.SPHERE and arrow.type == Marker.ARROW
    for marker in (target, arrow):
        assert marker.header.frame_id == 'map'
        assert marker.action == Marker.ADD
        assert marker.color.a == 1.0
        assert marker.lifetime.nanosec == 500_000_000
    assert target.pose.position.x > 5.0
    assert target.pose.position.y == pytest.approx(0.0)
    assert arrow.pose.position.x == pytest.approx(5.0)
    assert arrow.pose.position.y == pytest.approx(0.5)
    q = arrow.pose.orientation
    assert 2 * math.atan2(q.z, q.w) == pytest.approx(yaw + drive.steering_angle)
    driver.drive(5.0, 0.5, yaw, age=2.0)
    assert all(m.action == Marker.DELETE for m in driver.markers[-1].markers)


def test_tiny_raceline_within_lookahead(run):
    driver, _ = run('tiny')
    d = driver.drive(0.2, 0.0, math.pi / 2)
    assert d.speed == pytest.approx(1.0 * SPEED_SCALE)
    # inside the loop: the target is the farthest point, to the left, not the point just behind (right)
    assert driver.drive(0.1, 0.0, math.pi / 2).steering_angle > 0.0


def test_hairpin_keeps_its_leg(run):
    # 0.55 m off the +x leg, 0.45 m from the -x leg: steer back to its own leg, not onto the other
    driver, _ = run('hairpin')
    assert driver.drive(5.0, 0.55, 0.0).steering_angle < 0.0  # first pose: heading picks the leg
    driver.drive(5.0, 0.0, 0.0)
    assert driver.drive(5.1, 0.55, 0.0).steering_angle < 0.0  # drifted off: stays on the leg


@pytest.mark.parametrize('bad', [dict(age=2.0), dict(frame='world'), dict(y=10.0)],
                         ids=['stale', 'world_frame', 'off_line'])
def test_bad_pose_stops(run, bad):
    driver, _ = run('straight')
    assert driver.drive(5.0, 0.0, 0.0).speed > 0.0
    y = bad.pop('y', 0.0)
    stopped = driver.drive(5.0, y, 0.0, **bad)
    assert stopped.speed == 0.0 and stopped.steering_angle == 0.0
    assert driver.drive(5.0, 0.0, 0.0).speed == pytest.approx(3.0 * SPEED_SCALE)  # resumes


def test_missing_raceline(tracks_dir):
    proc = launch(tracks_dir, 'no_such_track')
    try:
        proc.wait(timeout=20)
    except subprocess.TimeoutExpired:
        pass
    out = stop(proc)
    assert 'Raceline not found' in out


class LocalPath:
    """Publishes /local_path (the straight raceline shifted by dy) and /local_path/valid, as the spliner does."""

    def __init__(self, node):
        self.node = node
        self.path_pub = node.create_publisher(Path, '/local_path', 10)
        self.valid_pub = node.create_publisher(Bool, '/local_path/valid', 10)

    def send(self, dy, valid=True, n=100):
        msg = Path()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.node.get_clock().now().to_msg()
        for x in np.arange(0.0, 20.0, 0.2)[:n]:
            pose = PoseStamped()
            pose.pose.position.x, pose.pose.position.y = float(x), dy
            msg.poses.append(pose)
        end = time.time() + 1.0
        while (self.path_pub.get_subscription_count() == 0 or self.valid_pub.get_subscription_count() == 0) \
                and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.path_pub.publish(msg)
        self.valid_pub.publish(Bool(data=valid))
        end = time.time() + 0.2
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def close(self):
        self.node.destroy_publisher(self.path_pub)
        self.node.destroy_publisher(self.valid_pub)


@pytest.fixture
def local_path(ros):
    lp = LocalPath(ros)
    yield lp
    lp.close()


def test_avoid_follows_local_path(run, local_path):
    driver, _ = run('straight', 'avoid:=true')
    stopped = driver.drive(5.0, 0.0, 0.0)   # no /local_path yet
    assert stopped.speed == 0.0
    local_path.send(0.5)   # path 0.5 m to the left of the raceline
    on_path = driver.drive(5.0, 0.5, 0.0, n=3)
    assert on_path.steering_angle == pytest.approx(0.0, abs=1e-6)
    assert on_path.speed == pytest.approx(3.0 * SPEED_SCALE)   # the raceline's speed
    local_path.send(0.5)
    assert driver.drive(5.0, 0.0, 0.0, n=3).steering_angle > 0.0   # on the raceline: steer left onto the path


@pytest.mark.parametrize('bad', [dict(valid=False), dict(n=50)], ids=['not_valid', 'wrong_length'])
def test_avoid_stops_without_a_good_path(run, local_path, bad):
    driver, _ = run('straight', 'avoid:=true')
    local_path.send(0.0)
    assert driver.drive(5.0, 0.0, 0.0, n=3).speed > 0.0
    local_path.send(0.0, **bad)
    assert driver.drive(5.0, 0.0, 0.0, n=3).speed == 0.0
