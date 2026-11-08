# MPCC implementation plan

Initial design, 2026-11-07, based on `MAP`. The user subsequently selected acados
SQP-RTI/HPIPM instead of the OSQP proposal below. The implementation on `feat/mpcc`
is documented in [MPCC.md](../ws/src/control/MPCC.md); that document supersedes
implementation details below (RK4, curvature speed envelope, regularized progress
cost and a latched hung-worker fault). This file retains the original tradeoff review.

## Repository findings

The initial checkout was `MAP2`; inspection confirmed that `MAP` is the appropriate
base for the requested MAP/MPCC comparison. The checkout has been switched to `MAP`.
The branches differ substantially; merging MAP2 wholesale would replace working
localization, bridge, recovery, and evaluation behavior.

| Area | Existing architecture | MPCC change |
| --- | --- | --- |
| Controller | `control/map_node.py`: L1 guidance plus offline acceleration-to-steering LUT; 40 Hz timer | Separate predictive optimizer; retain MAP as a baseline |
| Model | `map_dynamics.py`: six-state dynamic bicycle used offline for LUT generation; Pacejka/linear tires | Online kinematic bicycle initially; explicit model interface for a later dynamic model |
| State | `/localized_pose` in `map`, `/odom` body-frame forward speed and measured yaw rate | Reuse these inputs and input-health recovery; no new estimator |
| Commands | `/drive`, Ackermann speed in m/s and steering in radians | Same interface; optimizer outputs converted to next speed/steering targets |
| Actuation | Adapter speed PI/feedforward, acceleration/deceleration ramp; normalized steering | Respect adapter limits; assess tracking mismatch before raising speeds |
| Geometry | Rear-axle `base_link`; wheelbase 0.33 m, steering lock 0.524 rad in `vehicle.yaml` | Load these shared parameters; add footprint and controller-specific limits explicitly |
| Track | `centerline.csv`: x,y,right width,left width; TUM `raceline.csv`: seven semicolon-separated columns | Periodic arc-length geometry and asymmetric boundaries; preserve existing files |
| Launch | `race.launch.py` selects `pure_pursuit` or `map`; MAP rejects avoidance | Add `mpcc` selection, preserve defaults and single `/drive` publisher |
| Evaluation | `validate_race.py`, analysis scripts, per-run artifacts | Extend controller selection and add common geometric/solver metrics |
| Docker | ROS Humble; bind-mounted `/root/ws`; TUM in isolated venv | Add small, pinned MPCC numerical environment without altering TUM |

The running x86_64 Compose service contains ROS Python (after sourcing Humble),
NumPy and SciPy. OSQP, CasADi and acados Python modules were absent in the checked
system interpreter. The Dockerfile on MAP does not install an online MPC solver.
The image is six days old; a Dockerfile edit alone will not update that container.

Important details:

- MAP's `InputHealth` checks clock discontinuities, heading consistency with yaw
  rate, and recovery across multiple new samples. Reuse it and preserve its tests.
- TUM `psi_rad` is not ROS yaw. Recompute reference heading from geometry.
- Existing centerline widths come from occupancy-map ray casts and are tightened
  to avoid crossing normals. They are conservative map-derived corridors, not
  surveyed boundaries. Spline interpolation must not silently widen them.
- MAP2's constant raceline margin is not a usable replacement for those widths.
- The bridge supports an optional monotonic transport `/clock`. It is not physics
  time. Use consistent ROS time for stamps, monotonic time for deadlines/watchdogs.
- Existing default localization/bridge paths have privileged simulator dependencies
  documented in `AGENTS.md`. MPCC adds none. Reuse the isolated evaluator wiring
  where applicable; do not label the default stack ground-truth independent.

## Mathematical formulation

Use global Cartesian position to avoid Frenet dynamics' curvature singularity.
Let the physical state plus virtual progress be

`z = [X, Y, psi, v, delta, theta]`, controls `u = [a, omega_delta, nu]`.

Here `theta` is unwrapped reference arc length, `nu` is optimized virtual progress
speed, `delta` is steering angle, and `omega_delta` is steering rate. The initial
model is referenced to the rear axle:

```text
X_dot     = v cos(psi)
Y_dot     = v sin(psi)
psi_dot   = v tan(delta) / wheelbase
v_dot     = a
delta_dot = omega_delta
theta_dot = nu
```

Use midpoint/RK2 discretization with consistent analytical Jacobians. Validate
Jacobians numerically in tests. The model contract supplies state/control metadata,
discrete propagation, Jacobians, measured-state initialization and command decoding.
A future dynamic bicycle adds lateral velocity/yaw rate and tire forces through
this contract; it will require additional estimation and calibration validation.
It is not a configuration-only substitution.

For reference point `r(theta)`, unit tangent `t(theta)` and left normal `n(theta)`:

```text
e_l = t(theta)^T ([X,Y] - r(theta))
e_c = n(theta)^T ([X,Y] - r(theta))

J = sum(k=0..N-1) [ q_c e_c,k^2 + q_l e_l,k^2
                   + r_a a_k^2 + r_omega omega_delta,k^2
                   + r_delta delta_k^2 + ||u_k-u_(k-1)||_Rdu^2
                   - q_progress nu_k dt ]
    + q_c_terminal e_c,N^2 + q_l_terminal e_l,N^2
```

Lag error couples virtual progress to physical motion. Bound `nu` and its changes,
and apply trust regions to progress/pose to prevent a local QP from jumping to a
different leg or gaining artificial progress. Preserve unwrapped theta across the
lap seam; modulo is only for track sampling. Re-anchor after invalidation/reset.

Constraints:

- Initial state equals the validated measured/estimated state. Steering initially
  uses the last applied command; do not invent a measured steering topic. Record
  this assumption and support a future calibrated feedback provider.
- Linearized discrete dynamics at every stage.
- `0 <= v <= v_max`, `|delta| <= 0.524`, bounded acceleration/braking,
  steering rate, progress speed and control increments. Initial configured
  acceleration/braking must not exceed the adapter's 2.5/6.9 m/s² ramp settings;
  start with more conservative braking for prediction until validated.
- Linearized lateral-acceleration bound `|v² tan(delta)/L| <= a_y_max`, followed
  by nonlinear validation. This is a kinematic validity/traction proxy, not tire
  force estimation. Configure conservatively; do not infer tire limits from slip.
- At each stage, constrain the vehicle footprint between left/right local boundary
  halfspaces, tightened by a configurable clearance margin. Linearize rotated body
  corners about nominal heading. Include terminal and intermediate rollout samples.
  Explicit front/rear extents relative to the rear axle are required: the existing
  0.50 m length and 0.27 m width do not establish axle overhangs by themselves.
- Validate the nonlinear rollout and footprint against the actual corridor and map
  occupancy, including unknown cells. A solved local QP is insufficient evidence
  of collision-free motion. Reject excessive dynamics defects or boundary errors.

Keep track constraints hard initially. If the current footprint is outside the
tightened corridor, stop and report infeasibility; do not silently add boundary
slack. This sacrifices recovery capability for an understandable initial contract.
There is no recursive-feasibility or collision-avoidance guarantee from this model.

## Reference geometry and speed policy

Default MPCC path is the existing centerline, enabling racing-line optimization
within its corridor. Fit a periodic differentiable reference, build arc-length
lookup/inversion, and project locally using previous progress and heading. Check
the fitted geometry and interpolated widths against the original map; reject
invalid/degenerate tracks. Preserve direction, seam continuity and conservative
widths, and distinguish optimization theta from independently projected progress.

Do not attach centerline widths directly to the shifted raceline. An optional
raceline reference would require recomputing its boundary intersections first.

The standard objective rewards progress without velocity-profile tracking. Retain
the existing raceline speed data as optional metadata and as an initial conservative
speed-envelope source: associate it spatially to centerline progress with continuity
checks, never by matching row indices or raw arc lengths. Define `speed_scale` for
MPCC as scaling this envelope, and label the semantics in benchmark configuration.
Keep an absolute speed cap. Preview braking beyond the optimization horizon to
avoid accelerating into a corner that cannot be reached at a feasible speed.
Document the envelope as a commissioning restriction on the standard formulation.

Provide an optional `reference_speed(theta)` provider and objective extension hook,
disabled initially, for future velocity tracking or terminal velocity cost inspired
by EVO-MPCC. No adaptive weights, tire learning, opponents or overtaking now.

## Solver choice and timing

Recommend a sequential linear-quadratic approximation with **OSQP** initially.
Linearize dynamics, contour/lag errors and constraints around the shifted previous
trajectory. Use Gauss-Newton quadratic costs plus small regularization to keep the
QP convex. One QP per control cycle initially; a bounded second iteration may be
enabled only inside the same total deadline.

OSQP supports sparse matrix value updates, primal/dual warm starts, iteration limits
and runtime limits. Allocate a fixed CSC sparsity pattern once per horizon/model,
including structural entries that can become zero. Update numerical values; changing
dynamics still requires numerical factorization. Shift primal trajectories and
stage-aligned duals, or reset duals where active constraints change. Rebuild/reset
on structural parameter changes, reset events and faults. Cold-start from a bounded
kinematic rollout with geometry-derived steering; never warm-start invalid output.

Tradeoffs:

| Option | Benefit | Cost / decision |
| --- | --- | --- |
| Sparse OSQP QP | Small Python integration; inspectable matrices; warm starts/time limit | Iterative tolerances and local approximation require rollout validation; choose initially |
| acados SQP-RTI + HPIPM | OCP structure, generated native model, strong performance path | More build/code-generation dependencies; preferred next backend if timing fails |
| CasADi + IPOPT NLP | Convenient nonlinear prototype | Variable convergence time and heavier runtime; avoid for initial online loop |
| Direct HPIPM | Strong structured QP solver, used by primary/ROS references | More custom bindings/toolchain work for current Python stack |

Proposed starting configuration: 20 Hz control, `dt=0.05 s`, `N=30` (1.5 s
prediction), total computation deadline 35 ms within the 50 ms period. These are
commissioning targets, not measured capabilities. Horizon, sampling interval,
weights, tolerances, iteration cap and deadline are configurable and validated.
Reject deadlines at or above the control period. Record p50/p95/p99/max total
latency, deadline misses and nonlinear feasibility; solver time alone is insufficient.
Linux/Python and Docker supply no hard real-time scheduling guarantee.

## ROS integration and failure handling

Implement `mpcc_node.py` as a small supervisor and put optimizer work in a dedicated
process with a bounded latest-only mailbox. Keep the safety/publish timer responsive
even if native solver code blocks. Timestamp requests and results with sequence
and reset epoch. Never queue old solves or apply late/old-epoch results. Terminate
and recreate a hung worker using bounded cleanup; no in-process solver cancellation
assumption. A missing result at the deadline produces a stop, even on the first solve.

| Interface | Behavior |
| --- | --- |
| `/localized_pose` (`PoseStamped`) | Required map pose, validated stamps/quaternion/continuity |
| `/odom` (`Odometry`) | Required body-frame speed and yaw rate, compatible publisher QoS |
| Existing reset event (`String`) | Clear projection, health latch, worker generation, warm start and lap segment |
| `/drive` (`AckermannDriveStamped`) | Only supervisor publishes; first optimized interval gives speed/steering targets |
| `/mpcc/predicted_path` (`Path`) | Optional bounded-rate visualization in `map` |
| `/mpcc/diagnostics` (`DiagnosticArray`) | Status, residuals, timings, violations, stop reason |

Snapshot and time-align inputs before optimization; bound pose/odometry skew and
recheck freshness at publication. Extend current input-health validation with
monotonic receive-age checks and explicit odometry child-frame checks. Stop for
missing/invalid/stale input, clock faults, bad projection, infeasibility, nonfinite
results, unacceptable residuals, invalid rollout, worker failure or deadline miss.
Initially accept only successful solves passing independent tolerances.

Publish repeated speed-zero commands while retaining bounded last steering during
braking, preserving MAP's steering continuity. Clear the candidate and warm start;
re-arm only after consistent fresh measurements and a new feasible timely solve.
On shutdown publish a bounded stop burst and clean up the worker. Retain the bridge
watchdog as an independent final layer. These stops depend on the existing adapter
and braking behavior; they do not guarantee the car remains inside the corridor.
Do not automatically hand control to MAP after an optimizer fault.

Add `controller:=mpcc` to race launch with exclusive selection. Direct comparisons
use stopped/restarted launches (`controller:=map` versus `controller:=mpcc`), not
two competing nodes or a live moving handoff. Reject `avoid:=true` for MPCC initially,
as MAP already does; the existing shifted `/local_path` is not a boundary corridor.

## Dependencies, files and benchmark logging

Add pinned OSQP and compatible NumPy/SciPy in an MPCC venv created with access to
system ROS packages; launch with that interpreter after sourcing ROS/workspace.
Validate Humble Python/NumPy ABI and imports in a rebuilt image. Keep TUM isolated;
no CVXPY, gym simulator, external reference stack or acados build is needed initially.
Add the installer to Dockerfile and `.dockerignore` allowlist. Declare SciPy and
diagnostic message runtime dependencies and install config/launch resources through
the existing package metadata. Freeze exact numerical versions after the Docker
compatibility check, and record them in benchmark manifests.

Planned files:

- `control/mpcc_model.py`, `mpcc_track.py`, `mpcc_solver.py`: ROS-independent core.
- `control/mpcc_node.py`, `mpcc_worker.py`: inputs, deadlines, command safety.
- `config/mpcc.yaml`, `launch/mpcc.launch.py`, dependency installer; update package
  metadata, race launch, README and validation CLI choices.
- A shared read-only benchmark logger/evaluator and offline summary command.
- Focused model, geometry, optimizer, worker, ROS and launch tests.

Write unique runs under `ws/log/mpcc/` or the existing validation output tree,
never overwrite tracks. Record run manifest (commit, controller, all parameters,
track/raceline/map hashes and provenance, localizer, solver versions and clock basis),
timestamped state/command CSV and per-solve diagnostics. Use bounded asynchronous
logging; expose dropped-row counts and keep disk I/O outside the control deadline.

Log X,Y,yaw,v,measured yaw rate, estimated steering, projected progress, virtual
theta, contour/lag errors, commands, solver status/iterations/residuals, assembly,
solve/validation/total timings, input ages, deadline misses and stop reasons.
Log signed minimum footprint clearance and maximum boundary/actuator/dynamics
violations with units; keep unavailable values null.

For MAP/MPCC comparison, an independent evaluator uses the same centerline and
footprint to compute contouring RMSE/p95/max and boundary violations for both.
MAP solver metrics are unavailable, not zero. Count geometric laps with unwrapped
measured projection and a guarded start-line crossing, minimum distance/time,
direction and jump checks; exclude startup partial laps and reset segments. Retain
native simulator lap/collision metrics separately through existing evaluation tools;
never feed those metrics into driving. Report flying-lap times, completion rate,
collisions, stops and deadline misses across repeated identical configurations.

## Implementation order and acceptance

1. Implement track/model contracts and deterministic geometry/Jacobian tests.
2. Implement sparse QP, shift warm starts and independent nonlinear validation.
   Exercise progress from rest, corners, asymmetric widths, seam crossing, actuator
   saturation, infeasible corridors, bad statuses and cold/warm starts.
3. Add supervisor/worker and fault tests: blocked solve, deadline, stale/future data,
   skew, clock discontinuity, heading jump, reset during solve, recovery, worker
   restart and shutdown. Verify no late result can produce a drive command.
4. Add Docker dependencies, package resources and exclusive launch selection.
   Run focused tests in an isolated ROS domain, then relevant existing MAP/bridge
   regressions. Measure actual CPU latency with the default and larger horizons.
5. Add common metrics and comparison reporting; check log/lap semantics using
   synthetic laps, resets and interrupted runs. Verify ROS transport separately.
6. Commission in AutoDRIVE at the existing conservative speed scale, with the same
   map/localizer/adapter settings for MAP and MPCC. Preserve failed attempts. Run
   repeated complete flying laps and controlled input-loss/deadline fault trials.
   Raise limits only after observed tracking, clearance and timing support it.

Acceptance requires tests demonstrating progress optimization and corridor/input
constraints, responsive stop/recovery under faults, exclusive controller launch,
complete comparable logs, and measured deadline behavior. Simulator performance
claims require recorded live runs; unit tests alone establish none. If OSQP misses
the target, profile assembly/factorization first, then use the model/solver contract
to move to native/acados code rather than relaxing safety checks.

## References and scope of reuse

- [alexliniger/MPCC](https://github.com/alexliniger/MPCC): augmented progress,
  contour/lag objective and local QP formulation. Its nonlinear tire model and
  complete software stack are not imported.
- [F110_MPCC_ROS2](https://github.com/prajwalthakur/F110_MPCC_ROS2): linearized QP,
  HPIPM and computation-latency considerations. Retain our own ROS/AutoDRIVE bridge.
- [f1tenth_benchmarks](https://github.com/BDEvan5/f1tenth_benchmarks): centerline
  progress optimization and repeatable comparisons. Reference README inspected;
  controller source retrieval was unavailable during this planning pass. No claim
  is made that its implementation details have been audited or reproduced.
- [EVO-MPCC](https://github.com/zhouhengli/EVO-MPCC): future reference-speed and
  terminal velocity objective hooks only. The referenced project uses ROS Noetic;
  its stack, auto-tuning and overtaking are outside this implementation.
- [OSQP Python interface](https://osqp.org/docs/interfaces/python.html) and
  [solver settings](https://osqp.org/docs/interfaces/solver_settings.html): sparse
  updates, warm starts, iteration/runtime limits and diagnostics.
