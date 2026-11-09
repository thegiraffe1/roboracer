"""Rear-axle bicycle model and validated MPCC configuration (SI units)."""
from dataclasses import asdict, dataclass
import math

import numpy as np


@dataclass(frozen=True)
class Config:
    horizon_nodes: int = 30
    dt: float = 0.05
    deadline: float = 0.035
    wheelbase: float = 0.33
    max_steering_angle: float = 0.524
    max_speed: float = 3.0
    nominal_speed: float = 3.0
    speed_scale: float = 0.5
    acceleration: float = 2.0
    deceleration: float = 3.0
    steering_rate: float = 2.0
    lateral_acceleration: float = 3.0
    # Conservative enclosing box until axle overhangs are measured: total length
    # 0.50 m, both axles inside body, wheelbase 0.33 m => front <= .50, rear <= .17.
    body_front: float = 0.50
    body_rear: float = 0.17
    body_width: float = 0.27
    clearance: float = 0.03
    progress_trust: float = 0.6
    max_lag: float = 0.25
    q_contour: float = 30.0
    q_lag: float = 100.0
    q_progress: float = 2.0
    r_acceleration: float = 0.1
    r_steering_rate: float = 0.5
    r_steering: float = 0.1
    r_progress: float = 0.05
    feasibility_tolerance: float = 0.005
    speed_feasibility_tolerance: float = 0.005
    dynamics_tolerance: float = 0.02
    max_input_age: float = 0.3
    max_input_skew: float = 0.1
    max_pose_jump: float = 1.0
    max_offset: float = 0.5
    search_window: float = 2.0

    def __post_init__(self):
        for name, value in asdict(self).items():
            if (not isinstance(value, (int, float)) or isinstance(value, bool) or
                    not math.isfinite(value) or value <= 0):
                raise ValueError(f'{name} must be finite and positive')
        if (int(self.horizon_nodes) != self.horizon_nodes or
                not 2 <= self.horizon_nodes <= 150):
            raise ValueError('horizon_nodes must be an integer in [2, 150]')
        if self.deadline >= self.dt or self.dt > 0.2:
            raise ValueError('require deadline < dt <= 0.2 seconds')
        if self.max_steering_angle >= math.pi / 2 or self.speed_scale > 1:
            raise ValueError('invalid steering lock or speed_scale (must be <= 1)')
        if self.acceleration > 2.5 or self.deceleration > 6.9:
            raise ValueError('MPCC acceleration limits exceed default adapter ramps')

    @property
    def speed_cap(self):
        """Absolute cap applies after scaling, matching the existing race interface."""
        return min(self.max_speed, self.nominal_speed*self.speed_scale)


class KinematicBicycle:
    """Model contract: symbolic dynamics, numerical step, bounds and command decode.

    State [X,Y,yaw,v,steering,theta]; input [acceleration,steering_rate,progress_rate].
    theta is virtual, unwrapped path arc length. A dynamic model can implement this
    contract with different dimensions, initialization and command decoding.
    """
    nx, nu = 6, 3

    def __init__(self, config):
        self.config = config

    def derivative(self, x, u):
        if x.ndim == 1:
            return np.array([x[3]*math.cos(x[2]), x[3]*math.sin(x[2]),
                             x[3]*math.tan(x[4])/self.config.wheelbase, u[0], u[1], u[2]])
        return np.stack([x[..., 3] * np.cos(x[..., 2]), x[..., 3] * np.sin(x[..., 2]),
                         x[..., 3] * np.tan(x[..., 4]) / self.config.wheelbase,
                         u[..., 0], u[..., 1], u[..., 2]], axis=-1)

    def symbolic(self, x, u, ca):
        return ca.vertcat(x[3]*ca.cos(x[2]), x[3]*ca.sin(x[2]),
                          x[3]*ca.tan(x[4])/self.config.wheelbase, u[0], u[1], u[2])

    def initialize(self, measured):
        """Extend the common six-state observation in a future dynamic model.

        The first six indexes retain the common pose/actuator/progress layout;
        a dynamic implementation may append estimated lateral speed/yaw rate.
        """
        state = np.asarray(measured, dtype=float).copy()
        if state.shape != (self.nx,):
            raise ValueError('model observation has wrong dimension')
        return state

    def step(self, x, u, dt=None):
        """RK4, matching the generated ERK integrator."""
        dt = self.config.dt if dt is None else dt
        k1 = self.derivative(x, u)
        k2 = self.derivative(x + dt*k1/2, u)
        k3 = self.derivative(x + dt*k2/2, u)
        k4 = self.derivative(x + dt*k3, u)
        return x + dt*(k1 + 2*k2 + 2*k3 + k4)/6

    def command(self, x0, u):
        c = self.config
        return (float(np.clip(x0[3] + c.dt*u[0], 0., c.speed_cap)),
                float(np.clip(x0[4] + c.dt*u[1], -c.max_steering_angle,
                              c.max_steering_angle)))

    def speed_recovery_slack(self, speed, target, elapsed):
        """Initial overspeed allowance removed by the configured braking ramp.

        Interpolate between control nodes, since acceleration is held constant
        within each interval. No allowance is created when starting below target.
        This does not increase command caps or the absolute vehicle speed bound.
        """
        excess = max(0., float(speed-target))
        whole, fraction = np.divmod(np.asarray(elapsed)/self.config.dt, 1.)
        decrement = self.config.deceleration*self.config.dt
        before = np.maximum(0., excess-decrement*whole)
        after = np.maximum(0., excess-decrement*(whole+1))
        return before*(1-fraction)+after*fraction

    def corners(self):
        c = self.config
        return np.array([[a, b] for a in (-c.body_rear, c.body_front)
                         for b in (-c.body_width/2, c.body_width/2)])
