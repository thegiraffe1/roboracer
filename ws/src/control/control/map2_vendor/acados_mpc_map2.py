"""acados implementation of the MAP2-paper MPC (three-state, three-input).

The continuous model follows the "Curvature Model Predictive Control" section
of the MAP2 draft:

    x = [s, n, d_phi]            (Frenet longitudinal, lateral, heading error)
    u = [v_x, v_y, psi_dot]      (body-frame velocities and yaw rate)

        s_dot    = (v_x*cos(d_phi) - v_y*sin(d_phi)) / (1 - kappa*n)
        n_dot    =  v_x*sin(d_phi) + v_y*cos(d_phi)
        d_phi_dot = psi_dot - kappa * s_dot

The cost tracks lateral deviation and heading error, tracks the waypoint
reference speed (needed so the optimized v_x is meaningful), penalizes the
lateral inputs (penalizing ``psi_dot - kappa*v`` with ``kappa*v`` sampled from
the waypoint profile instead of zero makes the MPC follow the curve's yaw
rate), and adds a quadratic slack term rho*eps^2 through acados soft
constraints on the road boundaries:

        n <=  d_left  + eps
        n >= -d_right - eps
"""

from __future__ import annotations

import os
import tempfile
from pathlib import Path

import numpy as np


class AcadosUnavailable(RuntimeError):
    """Raised when the MPC_MAP2 runtime dependency is not installed."""


class AcadosMPCMap2:
    """Reusable acados OCP for the MAP2 point-mass Frenet lateral model.

    State: ``[s, n, d_phi]``
    Input: ``[v_x, v_y, psi_dot]``
    Stage parameters: ``[kappa, d_left, d_right]``
    Cost reference (set per stage):
    ``[n_ref, d_phi_ref, v_ref, 0, psi_dot_ref]`` (``psi_dot_ref = kappa * v``
    sampled on the waypoint profile, defaults to zero).
    """

    def __init__(self, config: dict, logger=None):
        self.config = dict(config)
        self.logger = logger
        self.nx = 3
        self.nu = 3
        self.np = 3
        self.ny = 5
        self.N = int(self.config.get("horizon_nodes", 32))
        self.dt = float(self.config.get("dt", 0.05))
        self._solver = None
        self._code_export_directory = None
        self._last_x = None
        self._last_u = None
        self._pending_cost_weights = None
        self._build_solver()

    def _build_solver(self):
        try:
            import casadi as ca
            from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
        except ImportError as exc:
            raise AcadosUnavailable(
                "MPC_MAP2 requires acados_template and CasADi. "
                "Install the repository's acados setup in the active environment."
            ) from exc

        model = AcadosModel()
        model.name = "mpc_map2_frenet"
        x = ca.MX.sym("x", self.nx)
        u = ca.MX.sym("u", self.nu)
        p = ca.MX.sym("p", self.np)
        s, n, d_phi = x[0], x[1], x[2]
        v_x, v_y, psi_dot = u[0], u[1], u[2]
        kappa = p[0]

        s_dot = (v_x * ca.cos(d_phi) - v_y * ca.sin(d_phi)) / (1.0 - kappa * n)
        n_dot = v_x * ca.sin(d_phi) + v_y * ca.cos(d_phi)
        d_phi_dot = psi_dot - kappa * s_dot

        model.x = x
        model.xdot = ca.MX.sym("xdot", self.nx)
        model.u = u
        model.p = p
        model.f_expl_expr = ca.vertcat(s_dot, n_dot, d_phi_dot)
        model.f_impl_expr = model.xdot - model.f_expl_expr

        model.cost_y_expr = ca.vertcat(n, d_phi, v_x, v_y, psi_dot)
        model.cost_y_expr_e = ca.vertcat(n, d_phi)

        # Soft road-boundary constraints (slack eps extends the margins):
        #   n <=  d_left  + eps   ->  n - d_left  <= eps  (upper slack)
        #   n >= -d_right - eps   -> -n - d_right <= eps  (upper slack)
        # acados soft constraints add rho/2 * slack^2 with zl/zu = 0, which is
        # the rho*eps^2 term of the paper.
        d_left = p[1]
        d_right = p[2]
        model.con_h_expr = ca.vertcat(n - d_left, -n - d_right)

        ocp = AcadosOcp()
        ocp.model = model
        ocp.parameter_values = np.array([0.0, 1.0, 1.0])
        ocp.solver_options.N_horizon = self.N
        ocp.solver_options.tf = self.N * self.dt
        ocp.solver_options.integrator_type = "ERK"
        ocp.solver_options.sim_method_num_stages = int(
            self.config.get("erk_num_stages", 4)
        )
        ocp.solver_options.sim_method_num_steps = int(
            self.config.get("erk_num_steps", 1)
        )
        ocp.solver_options.qp_solver = str(
            self.config.get("qp_solver", "PARTIAL_CONDENSING_HPIPM")
        )
        ocp.solver_options.nlp_solver_type = str(
            self.config.get("nlp_solver_type", "SQP_RTI")
        )
        ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
        ocp.solver_options.print_level = int(self.config.get("print_level", 0))
        ocp.solver_options.levenberg_marquardt = float(
            self.config.get("levenberg_marquardt", 1.0e-4)
        )

        q_n = float(self.config.get("q_n", 100.0))
        q_dphi = float(self.config.get("q_dphi", 30.0))
        q_v = float(self.config.get("q_v", 10.0))
        r_vy = float(self.config.get("r_vy", 0.5))
        r_psi = float(self.config.get("r_psi", 0.1))
        p_n = float(self.config.get("p_n", q_n))
        p_dphi = float(self.config.get("p_dphi", q_dphi))
        rho = float(self.config.get("rho", 500.0))
        ocp.cost.cost_type = "NONLINEAR_LS"
        ocp.cost.W = np.diag([q_n, q_dphi, q_v, r_vy, r_psi])
        ocp.cost.yref = np.zeros(self.ny)
        ocp.cost.cost_type_e = "NONLINEAR_LS"
        ocp.cost.W_e = np.diag([p_n, p_dphi])
        ocp.cost.yref_e = np.zeros(2)

        # Soft road-boundary constraints are declared on the model
        # (model.con_h_expr above); here we only attach the bounds and mark the
        # constraints as soft with slack weight rho (the draft's rho*eps^2).
        # acados renders the JSON with json.dump, which cannot serialize
        # np.inf, so use a large finite lower bound instead of -inf.
        ocp.constraints.lh = np.array([-1.0e9, -1.0e9])
        ocp.constraints.uh = np.array([0.0, 0.0])
        ocp.constraints.idxsh = np.array([0, 1])
        ocp.cost.Zl = np.zeros(2)
        ocp.cost.Zu = rho * np.ones(2)
        ocp.cost.zl = np.zeros(2)
        ocp.cost.zu = np.zeros(2)

        v_x_min = float(self.config.get("v_x_min", 0.5))
        v_x_max = float(self.config.get("v_x_max", 12.0))
        v_y_max = float(self.config.get("v_y_max", 2.0))
        psi_dot_max = float(self.config.get("psi_dot_max", 6.0))
        if v_x_min >= v_x_max:
            raise ValueError("v_x_min must be smaller than v_x_max")
        ocp.constraints.lbu = np.array([v_x_min, -v_y_max, -psi_dot_max])
        ocp.constraints.ubu = np.array([v_x_max, v_y_max, psi_dot_max])
        ocp.constraints.idxbu = np.array([0, 1, 2])
        ocp.constraints.x0 = np.zeros(self.nx)

        export_root = self.config.get("code_export_directory")
        if not export_root:
            export_root = os.path.join(
                tempfile.gettempdir(), f"map2_solver_{os.getpid()}"
            )
        self._code_export_directory = str(Path(export_root).absolute())
        ocp.code_export_directory = self._code_export_directory
        json_file = os.path.join(self._code_export_directory, "acados_ocp.json")
        Path(self._code_export_directory).mkdir(parents=True, exist_ok=True)
        self._solver = AcadosOcpSolver(ocp, json_file=json_file)

        if self.logger:
            self.logger(
                "[MPC_MAP2] acados solver ready: "
                f"N={self.N}, dt={self.dt:.4f}, export={self._code_export_directory}"
            )

    def update_cost_weights(
        self,
        q_n: float,
        q_dphi: float,
        q_v: float,
        r_vy: float,
        r_psi: float,
        p_n: float,
        p_dphi: float,
        rho: float,
    ) -> None:
        """Queue new cost weights for application at the next solve call."""
        weights = np.asarray(
            [q_n, q_dphi, q_v, r_vy, r_psi, p_n, p_dphi, rho], dtype=float
        )
        if not np.all(np.isfinite(weights)) or np.any(weights < 0.0):
            raise ValueError("MPC_MAP2 cost weights must be finite and non-negative")

        # The dynamic-reconfigure callback runs on a ROS subscriber thread.
        # Applying matrices here could race with solve(), so publish one
        # immutable update for the control thread to consume.
        self._pending_cost_weights = tuple(float(value) for value in weights)

    def _apply_pending_cost_weights(self) -> None:
        weights = self._pending_cost_weights
        if weights is None:
            return

        q_n, q_dphi, q_v, r_vy, r_psi, p_n, p_dphi, rho = weights
        stage_weight = np.diag([q_n, q_dphi, q_v, r_vy, r_psi])
        terminal_weight = np.diag([p_n, p_dphi])
        slack_weight = rho * np.ones(2)

        for stage in range(self.N):
            self._solver.cost_set(stage, "W", stage_weight, api="new")
        # With x0 eliminated, acados has no nonlinear/slack constraint block at
        # stage zero. Road-boundary slacks exist on intermediate stages only.
        for stage in range(1, self.N):
            self._solver.cost_set(stage, "Zu", slack_weight)
        self._solver.cost_set(self.N, "W", terminal_weight, api="new")

        # Keep a newer callback update that arrived while the matrices above
        # were being changed; it will be applied on the following cycle.
        if self._pending_cost_weights is weights:
            self._pending_cost_weights = None

        if self.logger:
            self.logger(
                "[MPC_MAP2] cost weights updated: "
                f"q_n={q_n:g}, q_dphi={q_dphi:g}, q_v={q_v:g}, "
                f"r_vy={r_vy:g}, r_psi={r_psi:g}, p_n={p_n:g}, "
                f"p_dphi={p_dphi:g}, rho={rho:g}"
            )

    def solve(
        self,
        x0: np.ndarray,
        stage_parameters: np.ndarray,
        v_refs: np.ndarray,
        psi_dot_refs: np.ndarray | None = None,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, int]:
        """Solve the OCP and return ``(u0, u_seq, x_seq, status)``.

        ``stage_parameters`` has shape ``(N, 3)`` with rows
        ``[kappa, d_left, d_right]``; ``v_refs`` has shape ``(N,)``;
        ``psi_dot_refs`` (optional) has shape ``(N,)`` and holds the yaw-rate
        cost reference ``kappa * v`` per stage (zero when omitted).
        On failure the first input falls back to the reference speed and zero
        lateral inputs, matching the MAP2 fail-safe behaviour.
        """
        x0 = np.asarray(x0, dtype=float).reshape(self.nx)
        parameters = np.asarray(stage_parameters, dtype=float)
        refs = np.asarray(v_refs, dtype=float).reshape(-1)
        if psi_dot_refs is None:
            psi_refs = np.zeros(self.N, dtype=float)
        else:
            psi_refs = np.asarray(psi_dot_refs, dtype=float).reshape(-1)
        if parameters.shape != (self.N, self.np):
            raise ValueError(
                f"stage_parameters must have shape {(self.N, self.np)}, got {parameters.shape}"
            )
        if refs.shape != (self.N,):
            raise ValueError(f"v_refs must have shape {(self.N,)}, got {refs.shape}")
        if psi_refs.shape != (self.N,):
            raise ValueError(
                f"psi_dot_refs must have shape {(self.N,)}, got {psi_refs.shape}"
            )
        if not np.all(np.isfinite(x0)) or not np.all(np.isfinite(parameters)) or not np.all(
            np.isfinite(refs)
        ) or not np.all(np.isfinite(psi_refs)):
            raise ValueError("MPC_MAP2 solver input contains non-finite values")

        solver = self._solver
        self._apply_pending_cost_weights()
        solver.set(0, "lbx", x0)
        solver.set(0, "ubx", x0)
        if self._last_x is None:
            x_guess = np.tile(x0, (self.N + 1, 1))
            u_guess = np.zeros((self.N, self.nu))
        else:
            x_guess = np.vstack((self._last_x[1:], self._last_x[-1]))
            u_guess = np.vstack((self._last_u[1:], self._last_u[-1]))

        for stage in range(self.N):
            solver.set(stage, "p", parameters[stage])
            solver.set(
                stage,
                "yref",
                np.array([0.0, 0.0, refs[stage], 0.0, psi_refs[stage]]),
            )
            solver.set(stage, "x", x_guess[stage])
            solver.set(stage, "u", u_guess[stage])
        solver.set(self.N, "p", parameters[-1])
        solver.set(self.N, "yref", np.zeros(2))
        solver.set(self.N, "x", x_guess[-1])

        status = int(solver.solve())
        if status != 0:
            self._last_x = None
            self._last_u = None
            fallback_u = np.zeros((self.N, self.nu))
            fallback_u[:, 0] = refs
            return refs[0], fallback_u, np.zeros((self.N + 1, self.nx)), status

        self._last_u = np.vstack([solver.get(k, "u") for k in range(self.N)])
        self._last_x = np.vstack([solver.get(k, "x") for k in range(self.N + 1)])
        return self._last_u[0].copy(), self._last_u.copy(), self._last_x.copy(), status
