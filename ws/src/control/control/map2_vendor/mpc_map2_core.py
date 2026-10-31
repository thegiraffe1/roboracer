"""Pure Python helpers shared by the ROS and acados MPC_MAP2 adapters.

The waypoint buffer used by the ROS controller carries ten columns:

    [0] x_m          [1] y_m          [2] vx_mps (reference speed)
    [3] normalized track margin (unused here)
    [4] s_m          [5] kappa_radpm  [6] psi_rad      [7] ax_mps2
    [8] d_left       [9] d_right

``d_left`` / ``d_right`` are the signed distances from the reference line to
the left/right track boundary.  Because the Frenet lateral coordinate ``n`` is
positive to the left, the road-boundary box is ``[-d_right, +d_left]``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


def wrap_angle(angle: float) -> float:
    """Wrap an angle to ``[-pi, pi)``."""
    return float((float(angle) + math.pi) % (2.0 * math.pi) - math.pi)


def finite_or_raise(name: str, value: float) -> float:
    value = float(value)
    if not np.isfinite(value):
        raise ValueError(f"{name} must be finite, got {value}")
    return value


@dataclass
class Map2Profile:
    """Forward-ordered local profile for stage-wise MPC parameters.

    ``distance_m`` stores the forward distance from the measured Frenet
    position; all stage interpolation happens in this monotonic coordinate,
    which is immune to ``s_m`` wrapping at the finish line.
    """

    distance_m: np.ndarray
    speed_mps: np.ndarray
    curvature_radpm: np.ndarray
    heading_rad: np.ndarray
    d_left_m: np.ndarray
    d_right_m: np.ndarray

    @classmethod
    def from_waypoints(
        cls,
        waypoints: np.ndarray,
        s0: float,
        track_length: float,
    ) -> "Map2Profile":
        values = np.asarray(waypoints, dtype=float)
        if values.ndim != 2 or values.shape[0] < 2 or values.shape[1] < 10:
            raise ValueError("waypoints must have at least two rows and ten columns")
        track_length = finite_or_raise("track_length", track_length)
        s0 = finite_or_raise("s0", s0)
        if track_length <= 0.0:
            raise ValueError("track_length must be positive")

        s = values[:, 4]
        forward = np.mod(s - s0, track_length)
        order = np.argsort(forward, kind="mergesort")
        forward = forward[order]
        speed = values[order, 2]
        curvature = values[order, 5]
        heading = np.unwrap(values[order, 6])
        d_left = values[order, 8]
        d_right = values[order, 9]

        finite = (
            np.isfinite(forward)
            & np.isfinite(speed)
            & np.isfinite(curvature)
            & np.isfinite(heading)
            & np.isfinite(d_left)
            & np.isfinite(d_right)
        )
        forward = forward[finite]
        speed = speed[finite]
        curvature = curvature[finite]
        heading = heading[finite]
        d_left = d_left[finite]
        d_right = d_right[finite]
        if forward.size < 2:
            raise ValueError("waypoint profile has fewer than two finite rows")

        keep = np.r_[True, np.diff(forward) > 1.0e-9]
        forward, speed, curvature, heading, d_left, d_right = (
            array[keep] for array in (forward, speed, curvature, heading, d_left, d_right)
        )
        return cls(forward, speed, curvature, heading, d_left, d_right)

    def sample(self, distance_m: float) -> tuple[float, float, float, float, float]:
        distance_m = max(0.0, float(distance_m))
        return (
            float(np.interp(distance_m, self.distance_m, self.speed_mps)),
            float(np.interp(distance_m, self.distance_m, self.curvature_radpm)),
            float(np.interp(distance_m, self.distance_m, self.heading_rad)),
            float(np.interp(distance_m, self.distance_m, self.d_left_m)),
            float(np.interp(distance_m, self.distance_m, self.d_right_m)),
        )


def build_stage_parameters(
    profile: Map2Profile,
    measured_speed: float,
    horizon: int,
    dt: float,
    minimum_model_speed: float,
    speed_lookahead: float = 0.0,
    psi_dot_lookahead: float = 0.0,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Build ``[kappa, d_left, d_right]`` stage parameters and ``v_ref`` speeds.

    Every stage uses the waypoint reference speed sampled from the forward
    profile.  In particular stage zero must NOT be anchored to the measured
    speed: ``v_x`` is an optimization variable and the returned speed command
    is ``v_x(0)``, so anchoring stage zero to the current velocity makes the
    controller reproduce the current speed forever (the car never accelerates
    towards the waypoint speed).  The forward position is propagated with the
    reference speed itself, which keeps the sampled curvature and road margins
    aligned with the predicted motion.

    ``speed_lookahead`` (seconds, >= 0) forward-propagates the v_ref sample
    point by ``measured_speed * speed_lookahead`` metres.  This mirrors the
    PP/MAP ``speed_lookahead`` semantic: the MPC no longer "sees" the
    waypoint speed at the car, but the speed the car will see after
    ``speed_lookahead`` seconds.  The forward-propagation is applied to v_ref
    only; curvature and road boundaries are still sampled from the model's
    own predicted position so the road geometry stays consistent with the
    optimization variable.  ``speed_lookahead=0`` recovers the previous
    behaviour exactly.

    ``psi_dot_lookahead`` (seconds, >= 0) applies the same forward-propagation
    to the yaw-rate cost reference: the third output is
    ``kappa * v_waypoint`` sampled at ``measured_speed * psi_dot_lookahead``
    metres past the car (not the current point, which lags the car by the
    control latency), with the sample pointer advancing stage by stage just
    like the v_ref pointer.  ``psi_dot_lookahead=0`` gives the previous
    behaviour exactly.
    """
    measured_speed = finite_or_raise("measured_speed", measured_speed)
    dt = finite_or_raise("dt", dt)
    if horizon < 1 or dt <= 0.0:
        raise ValueError("horizon and dt must be positive")
    if minimum_model_speed <= 0.0:
        raise ValueError("minimum_model_speed must be positive")
    if speed_lookahead < 0.0:
        raise ValueError("speed_lookahead must be non-negative")
    if psi_dot_lookahead < 0.0:
        raise ValueError("psi_dot_lookahead must be non-negative")

    params = np.zeros((horizon, 3), dtype=float)
    v_refs = np.zeros(horizon, dtype=float)
    psi_dot_refs = np.zeros(horizon, dtype=float)
    # Initial forward offset: how far ahead of the car the v_ref sample point
    # starts.  Multiplying by ``measured_speed`` (not v_ref) keeps the offset
    # consistent with the MAP/PP definition, which uses the current velocity
    # to project the lookahead position.
    reference_s = max(0.0, float(measured_speed) * float(speed_lookahead))
    psi_reference_s = max(0.0, float(measured_speed) * float(psi_dot_lookahead))
    for stage in range(horizon):
        speed_ref, curvature, _, d_left, d_right = profile.sample(reference_s)
        v_ref = max(float(speed_ref), float(minimum_model_speed))
        v_refs[stage] = v_ref
        # Yaw-rate cost reference kappa * v, sampled at the psi_dot lookahead
        # point.  Taking the reference stage by stage keeps it aligned with
        # the curvature that the car will arrive at.
        psi_speed_ref, psi_curvature, _, _, _ = profile.sample(psi_reference_s)
        psi_v = max(float(psi_speed_ref), float(minimum_model_speed))
        psi_dot_refs[stage] = psi_curvature * psi_v
        # Guard against degenerate margins: a zero/negative margin makes the
        # soft-boundary box collapse and the QP infeasible.
        d_left = max(float(d_left), 0.05)
        d_right = max(float(d_right), 0.05)
        params[stage] = [curvature, d_left, d_right]
        reference_s += v_ref * dt
        psi_reference_s += psi_v * dt

    return params, v_refs, psi_dot_refs


def paper_dynamics(
    state: np.ndarray,
    u: np.ndarray,
    curvature: float,
) -> np.ndarray:
    """Evaluate the continuous MAP2 point-mass model used by the solver."""
    state = np.asarray(state, dtype=float).reshape(3)
    u = np.asarray(u, dtype=float).reshape(3)
    curvature = finite_or_raise("curvature", curvature)
    s, n, d_phi = state
    v_x, v_y, psi_dot = u
    denominator = 1.0 - curvature * n
    if abs(denominator) < 1.0e-6:
        raise ValueError("Frenet model denominator is singular")
    s_dot = (v_x * math.cos(d_phi) - v_y * math.sin(d_phi)) / denominator
    n_dot = v_x * math.sin(d_phi) + v_y * math.cos(d_phi)
    d_phi_dot = psi_dot - curvature * s_dot
    return np.array([s_dot, n_dot, d_phi_dot], dtype=float)


def measured_frenet_state(
    s_meas: float,
    n_meas: float,
    yaw: float,
    reference_heading: float,
) -> np.ndarray:
    """Assemble ``[s, n, d_phi]`` from the odometry streams.

    ``d_phi`` is the heading error ``yaw - reference_heading`` (wrapped).  The
    MAP2 model carries the sideslip through the ``v_y`` input, so no slip-angle
    correction is added to the heading error here.
    """
    s_meas = finite_or_raise("s_meas", s_meas)
    n_meas = finite_or_raise("n_meas", n_meas)
    yaw = finite_or_raise("yaw", yaw)
    reference_heading = finite_or_raise("reference_heading", reference_heading)
    d_phi = wrap_angle(yaw - reference_heading)
    return np.array([s_meas, n_meas, d_phi], dtype=float)


def lateral_acceleration(v_x: float, v_y: float, psi_dot: float) -> float:
    """Lateral acceleration implied by the optimized inputs.

    The draft writes ``a_c = (v_x^2 + v_y^2) / psi_dot``, which is
    dimensionally inconsistent; the correct kinematic relation is
    ``a_c = v_t * psi_dot`` with ``v_t = sqrt(v_x^2 + v_y^2)``, matching the
    ``a_c = v_t * psi_dot`` relation used in the pure-pursuit section of the
    draft.  A zero yaw rate therefore yields zero lateral acceleration instead
    of a division by zero.
    """
    v_x = finite_or_raise("v_x", v_x)
    v_y = finite_or_raise("v_y", v_y)
    psi_dot = finite_or_raise("psi_dot", psi_dot)
    return float(math.hypot(v_x, v_y) * psi_dot)


def update_waypoint_speeds(
    waypoints: np.ndarray,
    track_length: float,
    s0: float,
    stage_distances: np.ndarray,
    stage_speeds: np.ndarray,
) -> np.ndarray:
    """Write the MPC-optimized speed profile back into the waypoint buffer.

    Every waypoint whose forward distance lies inside the MPC horizon is
    assigned the interpolated optimized speed; waypoints beyond the horizon
    keep their original reference speed.  The buffer is modified in place and
    returned so the ROS-facing controller can log the change.

    ``stage_distances`` holds the predicted forward distance of each shooting
    stage (cumulative ``v * dt``) and ``stage_speeds`` the corresponding
    optimized ``v_x``.  Both have length equal to the horizon.
    """
    values = np.asarray(waypoints, dtype=float)
    if values.ndim != 2 or values.shape[1] < 10:
        raise ValueError("waypoints must have at least ten columns")
    track_length = finite_or_raise("track_length", track_length)
    s0 = finite_or_raise("s0", s0)
    stage_distances = np.asarray(stage_distances, dtype=float).reshape(-1)
    stage_speeds = np.asarray(stage_speeds, dtype=float).reshape(-1)
    if stage_distances.shape != stage_speeds.shape or stage_distances.size == 0:
        raise ValueError("stage_distances and stage_speeds must be non-empty and match")
    if not np.all(np.isfinite(stage_distances)) or not np.all(np.isfinite(stage_speeds)):
        raise ValueError("stage profile contains non-finite values")

    forward = np.mod(values[:, 4] - s0, track_length)
    mask = (forward >= 0.0) & (forward <= stage_distances[-1])
    if np.any(mask):
        values[mask, 2] = np.interp(forward[mask], stage_distances, stage_speeds)
    return values
