"""acados SQP-RTI MPCC, with shifted warm starts and independent rollout checks."""
import os
import gc
import tempfile
import time

import numpy as np

from .mpcc_model import KinematicBicycle
from .mpcc_track import footprint_points


def check_rollout(model, track, free_map, initial, controls, predicted=None):
    """Return nonlinear violations in SI units; reject independently of solver status."""
    c = model.config
    controls = np.asarray(controls, dtype=float)
    if (not np.isfinite(initial).all() or not np.isfinite(controls).all() or
            controls.ndim != 2 or controls.shape[1] != model.nu):
        raise ValueError('invalid rollout input')
    lower = np.array([-c.deceleration, -c.steering_rate, 0.])-c.feasibility_tolerance
    upper = np.array([c.acceleration, c.steering_rate, c.speed_cap])+c.feasibility_tolerance
    if (np.any(controls < lower) or np.any(controls > upper) or
            abs(initial[4]) > c.max_steering_angle+c.feasibility_tolerance or
            not 0 <= initial[3] <= c.speed_cap+c.feasibility_tolerance):
        raise ValueError('rollout input exceeds actuator bounds')
    states = [np.array(initial, copy=True)]
    defects = 0.
    for k, u in enumerate(controls):
        next_state = model.step(states[-1], u)
        if predicted is not None:
            defects = max(defects, float(np.max(abs(next_state-predicted[k+1]))))
        states.append(next_state)
    states = np.asarray(states)
    speed = np.maximum(np.maximum(states[:-1, 3], states[1:, 3]), .1)
    steering = np.maximum(abs(states[:-1, 4]), abs(states[1:, 4]))
    body_radius = np.hypot(max(c.body_front, c.body_rear), c.body_width/2)
    corner_speed = speed*(1+abs(np.tan(steering))/c.wheelbase*body_radius)
    counts = np.maximum(2, np.ceil(corner_speed*c.dt/(free_map.resolution*.5)).astype(int))
    times = np.concatenate([c.dt*np.arange(1, count+1)/count for count in counts])
    dense = np.vstack([states[0], model.step(np.repeat(states[:-1], counts, axis=0),
                                            np.repeat(controls, counts, axis=0), times[:, None])])
    if not np.isfinite(dense).all() or not np.isfinite(controls).all():
        raise ValueError('nonfinite rollout')
    r, t, n, _, right, left = track.sample(dense[:, 5])
    corners = footprint_points(dense, c, max(c.body_front+c.body_rear, c.body_width))
    lateral = np.einsum('nki,ni->nk', corners-r[:, None, :], n)
    corridor = max(0., float(np.max(lateral-left[:, None]+c.clearance)),
                   float(np.max(-right[:, None]+c.clearance-lateral)))
    lag = np.einsum('ni,ni->n', dense[:, :2]-r, t)
    points = footprint_points(dense, c, free_map.resolution*.5)
    min_clearance = float(np.min(free_map.clearance(points)))
    map_violation = max(0., c.clearance-min_clearance)
    actuator = max(0., float(np.max(abs(dense[:, 4]))-c.max_steering_angle),
                   float(np.max(controls[:, 0])-c.acceleration),
                   float(-c.deceleration-np.min(controls[:, 0])),
                   float(np.max(abs(controls[:, 1]))-c.steering_rate),
                   float(-np.min(controls[:, 2])),
                   float(np.max(controls[:, 2])-c.speed_cap),
                   float(-np.min(dense[:, 3])))
    speed_violation = max(0., float(np.max(dense[:, 3]-track.speed_limit(dense[:, 5]))))
    ay = dense[:, 3]**2*np.tan(dense[:, 4])/c.wheelbase
    lateral_violation = max(0., float(np.max(abs(ay))-c.lateral_acceleration))
    lag_violation = max(0., float(np.max(abs(lag))-c.max_lag))
    metrics = dict(boundary_violation_m=corridor, map_violation_m=map_violation,
                   min_clearance_m=min_clearance, actuator_violation=actuator,
                   speed_violation_mps=speed_violation,
                   lateral_acceleration_violation_mps2=lateral_violation,
                   lag_violation_m=lag_violation, dynamics_defect=defects)
    safe = (max(corridor, map_violation, actuator, speed_violation,
                lateral_violation, lag_violation) <= c.feasibility_tolerance and
            defects <= c.dynamics_tolerance)
    return safe, states, metrics


class AcadosMPCC:
    def __init__(self, config, track, free_map, model_factory=KinematicBicycle):
        self.config, self.track, self.free_map = config, track, free_map
        self.model = model_factory(config)
        self.last_x = self.last_u = None
        self.directory = tempfile.TemporaryDirectory(prefix='autodrive_mpcc_')
        self._build()
        # Collect compilation/template objects before any timed control request.
        gc.collect()

    def _build(self):
        import casadi as ca
        from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
        c = self.config
        model = AcadosModel()
        model.name = 'autodrive_kinematic_mpcc'
        x, u, p = ca.SX.sym('x', self.model.nx), ca.SX.sym('u', self.model.nu), ca.SX.sym('p', 10)
        model.x, model.u, model.p = x, u, p
        model.xdot = ca.SX.sym('xdot', self.model.nx)
        model.f_expl_expr = self.model.symbolic(x, u, ca)
        model.f_impl_expr = model.xdot-model.f_expl_expr
        # First-order reference r(theta) around nominal theta, refreshed each solve.
        error = x[:2]-p[:2]-p[2:4]*(x[5]-p[6])
        contour, lag = ca.dot(p[4:6], error), ca.dot(p[2:4], error)
        model.cost_y_expr = ca.vertcat(contour, lag, u[0], u[1], x[4], u[2])
        model.cost_y_expr_e = ca.vertcat(contour, lag, x[4])
        # Boundary normals are frozen per RTI stage; body rotation is nonlinear.
        boundary = []
        for bx, by in self.model.corners():
            offset = ca.vertcat(ca.cos(x[2])*bx-ca.sin(x[2])*by,
                                ca.sin(x[2])*bx+ca.cos(x[2])*by)
            lateral = ca.dot(p[4:6], x[:2]+offset-p[:2])
            boundary.extend([lateral-p[8]+c.clearance, -lateral-p[7]+c.clearance])
        ay = x[3]**2*ca.tan(x[4])/c.wheelbase
        constraints = ca.vertcat(*boundary, ay, lag)
        model.con_h_expr = constraints
        model.con_h_expr_e = constraints
        ocp = AcadosOcp()
        ocp.model = model
        # p[9] is a reference-speed channel reserved for optional velocity-guided
        # cost extensions. The standard cost does not track it.
        ocp.parameter_values = np.zeros(10)
        ocp.solver_options.N_horizon = c.horizon_nodes
        ocp.solver_options.tf = c.horizon_nodes*c.dt
        ocp.solver_options.integrator_type = 'ERK'
        ocp.solver_options.sim_method_num_stages = 4
        ocp.solver_options.sim_method_num_steps = 1
        ocp.solver_options.nlp_solver_type = 'SQP_RTI'
        ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
        ocp.solver_options.qp_solver_cond_N = min(c.horizon_nodes, 10)
        ocp.solver_options.qp_solver_iter_max = 50
        ocp.solver_options.qp_solver_tol_stat = 1e-5
        ocp.solver_options.qp_solver_tol_eq = 1e-5
        ocp.solver_options.qp_solver_tol_ineq = 1e-5
        ocp.solver_options.qp_solver_tol_comp = 1e-5
        ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
        ocp.solver_options.levenberg_marquardt = 1e-4
        ocp.solver_options.print_level = 0
        ocp.cost.cost_type = ocp.cost.cost_type_e = 'NONLINEAR_LS'
        # acados uses 1/2 residual'W residual, integrated over dt. Shifting nu's
        # reference gives r_nu*nu² - q_progress*nu, up to a constant.
        ocp.cost.W = 2*np.diag([c.q_contour, c.q_lag, c.r_acceleration,
                               c.r_steering_rate, c.r_steering, c.r_progress])
        ocp.cost.yref = np.array([0., 0., 0., 0., 0., c.q_progress/(2*c.r_progress)])
        ocp.cost.W_e = 2*np.diag([c.q_contour, c.q_lag, c.r_steering])
        ocp.cost.yref_e = np.zeros(3)
        ocp.constraints.idxbu = np.arange(3)
        ocp.constraints.lbu = np.array([-c.deceleration, -c.steering_rate, 0.])
        ocp.constraints.ubu = np.array([c.acceleration, c.steering_rate, c.speed_cap])
        for suffix in ('', '_e'):
            setattr(ocp.constraints, 'idxbx'+suffix, np.array([3, 4, 5]))
            setattr(ocp.constraints, 'lbx'+suffix, np.array([0., -c.max_steering_angle, -1e6]))
            setattr(ocp.constraints, 'ubx'+suffix, np.array([c.speed_cap,
                                                         c.max_steering_angle, 1e6]))
            setattr(ocp.constraints, 'lh'+suffix, np.r_[np.full(8, -1e6),
                                                       -c.lateral_acceleration, -c.max_lag])
            setattr(ocp.constraints, 'uh'+suffix, np.r_[np.zeros(8),
                                                       c.lateral_acceleration, c.max_lag])
        ocp.constraints.x0 = np.zeros(self.model.nx)
        ocp.code_export_directory = os.path.join(self.directory.name, 'generated')
        self.solver = AcadosOcpSolver(ocp, json_file=os.path.join(self.directory.name, 'ocp.json'),
                                     verbose=False)

    def reset(self):
        self.last_x = self.last_u = None
        self.solver.reset()

    def seed(self, initial):
        c, n = self.config, self.config.horizon_nodes
        if self.last_u is not None:
            controls = np.vstack([self.last_u[1:], self.last_u[-1]])
        else:
            controls = np.zeros((n, 3))
        states = [initial.copy()]
        for k in range(n):
            state = states[-1]
            if self.last_u is None:
                ix = int(np.searchsorted(self.track.s, state[5] % self.track.length, side='right')-1)
                tangent, curvature = self.track.tangent[ix], self.track.curvature[ix]
                yaw_ref = np.arctan2(tangent[1], tangent[0])
                heading_error = np.arctan2(np.sin(yaw_ref-state[2]), np.cos(yaw_ref-state[2]))
                steer = np.clip(np.arctan(c.wheelbase*curvature)+0.4*heading_error,
                                -c.max_steering_angle, c.max_steering_angle)
                speed = self.track.speed_limit(state[5])
                controls[k] = [np.clip((speed-state[3])/c.dt, -c.deceleration, c.acceleration),
                               np.clip((steer-state[4])/c.dt, -c.steering_rate, c.steering_rate),
                               max(0., state[3])]
            controls[k, 2] = min(controls[k, 2], c.speed_cap)
            states.append(self.model.step(state, controls[k]))
        return np.asarray(states), controls

    def solve(self, initial):
        """Keep unpredictable full cyclic-GC scans out of native-call preparation.

        New cyclic garbage is collected every request (generation zero); automatic
        GC state is restored for the caller. Collection/cleanup is still charged to
        the request deadline, and the parent additionally charges IPC latency.
        """
        enabled = gc.isenabled()
        gc.disable()
        start = time.monotonic()
        try:
            result = self._solve(initial)
        finally:
            gc.collect(0)
            if enabled:
                gc.enable()
        result['total_time_s'] = time.monotonic()-start
        if result['ok'] and result['total_time_s'] >= self.config.deadline:
            result.update(ok=False, reason='optimizer computation deadline exceeded')
            self.reset()
        return result

    def _solve(self, initial):
        start = time.monotonic()
        initial = self.model.initialize(initial)
        if not np.isfinite(initial).all():
            raise ValueError('invalid initial state')
        # Carry virtual progress across accepted intervals, anchored near measured
        # projection. Measured projection remains separate for lap evaluation.
        if self.last_x is not None and abs(self.last_x[1, 5]-initial[5]) < self.config.max_lag:
            initial[5] = self.last_x[1, 5]
        states, controls = self.seed(initial)
        c, n = self.config, self.config.horizon_nodes
        # Warm-start primal states/controls explicitly. Do not reuse unshifted duals.
        r, tangent, normal, _, right, left = self.track.sample(states[:, 5])
        parameters = np.column_stack([r, tangent, normal, states[:, 5], right, left,
                                      self.track.speed_limit(states[:, 5])])
        preview = states[:, 5, None]+np.linspace(-c.progress_trust, c.progress_trust, 9)
        limits = np.min(self.track.speed_limit(preview), axis=1)
        for k in range(n+1):
            self.solver.set(k, 'x', states[k])
            self.solver.set(k, 'p', parameters[k])
            if k < n:
                self.solver.set(k, 'u', controls[k])
            if k:
                theta = states[k, 5]
                self.solver.constraints_set(k, 'lbx', np.array([0., -c.max_steering_angle,
                                                               theta-c.progress_trust]))
                self.solver.constraints_set(k, 'ubx', np.array([limits[k], c.max_steering_angle,
                                                               theta+c.progress_trust]))
        self.solver.constraints_set(0, 'lbx', initial)
        self.solver.constraints_set(0, 'ubx', initial)
        assembly = time.monotonic()-start
        status = self.solver.solve()
        solve_time = float(self.solver.get_stats('time_tot'))
        result = dict(status=int(status), assembly_time_s=assembly, solver_time_s=solve_time)
        result['qp_iterations'] = int(np.sum(self.solver.get_stats('qp_iter')))
        result['initial_state'] = initial.tolist()
        result['virtual_progress_m'] = float(initial[5])
        point, tangent, normal, _, _, _ = self.track.sample(initial[5])
        result['contouring_error_m'] = float((initial[:2]-point) @ normal)
        result['lag_error_m'] = float((initial[:2]-point) @ tangent)
        if status != 0:
            self.reset()
            return dict(result, ok=False, reason=f'acados status {status}',
                        total_time_s=time.monotonic()-start)
        states = np.array([self.solver.get(k, 'x') for k in range(n+1)])
        controls = np.array([self.solver.get(k, 'u') for k in range(n)])
        result['first_control'] = controls[0].tolist()
        valid, rollout, metrics = check_rollout(self.model, self.track, self.free_map,
                                               initial, controls, states)
        elapsed = time.monotonic()-start
        reason = 'nonlinear rollout violation' if not valid else ''
        if elapsed >= c.deadline:
            reason = 'optimizer computation deadline exceeded'
        valid = valid and elapsed < c.deadline
        if valid:
            self.last_x, self.last_u = states, controls
        else:
            self.reset()
        speed, steer = self.model.command(initial, controls[0])
        return dict(result, **metrics, ok=valid, states=rollout.tolist(),
                    speed=speed, steering=steer, total_time_s=elapsed,
                    reason=reason)

    def close(self):
        # Release generated shared library before cleaning its private directory.
        self.solver = None
        self.directory.cleanup()
