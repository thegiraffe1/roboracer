"""ROS-independent AutoDRIVE MAP2 path and steering adaptation."""
import numpy as np

from control.map2_vendor.mpc_map2_core import Map2Profile, wrap_angle


class Reference:
    """Closed TUM raceline, projected onto segments in the map frame."""

    def __init__(self, rows, speed_scale=0.5, max_speed=3.0, margin=0.4):
        rows = np.asarray(rows, dtype=float)
        if rows.ndim != 2 or rows.shape[1] < 7 or not np.isfinite(rows).all():
            raise ValueError('raceline must contain finite TUM rows with seven columns')
        if len(rows) > 1 and np.linalg.norm(rows[0, 1:3] - rows[-1, 1:3]) < 1e-6:
            rows = rows[:-1]
        self.xy = rows[:, 1:3]
        self.delta = np.roll(self.xy, -1, axis=0) - self.xy
        self.lengths = np.linalg.norm(self.delta, axis=1)
        if len(rows) < 3 or np.any(self.lengths < 1e-6):
            raise ValueError('raceline needs at least three distinct consecutive points')
        if not all(np.isfinite(v) and v > 0 for v in (speed_scale, max_speed, margin)):
            raise ValueError('speed_scale, max_speed and margin must be finite and positive')
        self.s = np.r_[0., np.cumsum(self.lengths)[:-1]]
        self.length = float(self.lengths.sum())
        self.heading = np.arctan2(self.delta[:, 1], self.delta[:, 0])
        self.speed = np.clip(rows[:, 5] * speed_scale, 0., max_speed)
        # TUM psi uses a different convention; derive both heading and curvature from geometry.
        turn = np.angle(np.exp(1j * (self.heading - np.roll(self.heading, 1))))
        self.curvature = turn / (0.5 * (self.lengths + np.roll(self.lengths, 1)))
        self.margin = margin
        self.previous_s = None

    def project(self, x, y, yaw, max_offset, search_window=2.0):
        if not np.isfinite([x, y, yaw]).all():
            raise ValueError('non-finite pose')
        point = np.array([x, y])
        t = np.clip(np.sum((point - self.xy) * self.delta, axis=1) / self.lengths**2, 0., 1.)
        error = point - (self.xy + t[:, None] * self.delta)
        distance = np.linalg.norm(error, axis=1)
        aligned = np.cos(yaw - self.heading) > 0.
        candidates = aligned.copy()
        if self.previous_s is not None:
            arc = (self.s + t * self.lengths - self.previous_s + self.length / 2) % self.length - self.length / 2
            local = aligned & (np.abs(arc) <= search_window)
            if np.any(local) and np.min(distance[local]) <= max_offset:
                candidates = local
        if not np.any(candidates):
            raise ValueError('car points against the raceline')
        i = int(np.argmin(np.where(candidates, distance, np.inf)))
        if distance[i] > max_offset:
            raise ValueError('car is too far from an aligned raceline segment')
        n = (self.delta[i, 0] * error[i, 1] - self.delta[i, 1] * error[i, 0]) / self.lengths[i]
        s = float((self.s[i] + t[i] * self.lengths[i]) % self.length)
        self.previous_s = s
        return np.array([0., n, wrap_angle(yaw - self.heading[i])]), s

    def profile(self, s0):
        # Include the interpolated current point and repeat the loop far enough for the horizon.
        distance = (self.s - s0) % self.length
        order = np.argsort(distance)
        d = distance[order]
        def periodic(values):
            values = values[order]
            current = np.interp(0., np.r_[d[-1] - self.length, d], np.r_[values[-1], values])
            return np.r_[current, values, values, values]
        return Map2Profile(np.r_[0., d, d + self.length, d + 2 * self.length],
                           periodic(self.speed), periodic(self.curvature), periodic(self.heading),
                           np.full(1 + 3 * len(d), self.margin),
                           np.full(1 + 3 * len(d), self.margin))


def sim_steering(u, wheelbase, max_steering, minimum_speed=0.5):
    """Analytic bicycle LUT substitute: a_y=|v|*yaw_rate, delta=atan(L*a_y/|v|²).

    This is an uncalibrated simulation baseline. No physical-car LUT or GP checkpoint is used.
    """
    vx, vy, yaw_rate = np.asarray(u, dtype=float).reshape(3)
    if not np.isfinite([vx, vy, yaw_rate, wheelbase, max_steering]).all() or vx < 0:
        raise ValueError('invalid optimized input')
    speed = max(float(np.hypot(vx, vy)), minimum_speed)
    return float(np.clip(np.arctan(wheelbase * yaw_rate / speed), -max_steering, max_steering))
