# MPCC for AutoDRIVE

MPCC is an additional controller on the MAP branch's existing estimation and
actuation interfaces. It uses acados SQP-RTI with partially condensed HPIPM. MAP
and pure pursuit remain selectable. No reference repository's stack is imported.

## Run and compare

Build the image after pulling these dependencies, then build/source the workspace:

```bash
docker compose build
docker compose up -d
docker compose exec ros bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

With the existing bridge running, select one controller at a time:

```bash
scripts/race.sh <track> controller:=mpcc speed_scale:=0.3
scripts/race.sh <track> controller:=map speed_scale:=0.3 record_metrics:=true
```

Stop the current launch before starting the next one. `race.sh` resets the simulator;
it is AutoDRIVE-only and now defaults to wall time and UDP-only DDS. The bridge must
use the same clock (`autodrive_launch.py use_sim_time:=false` for this mode).
`USE_SIM_TIME=true` selects an existing shared-clock bridge; `DDS_TRANSPORT=default`
restores the normal middleware transport. Hardware/replay need separate entrypoints.
Direct launches do not reset the simulator. MPCC requires `centerline.csv`, `map.yaml` and the referenced
map image. The race script still requires `map.pbstream` and `raceline.csv`; existing
localization and track formats are unchanged. `avoid:=true` is rejected for MPCC.

Racing sets Cartographer's Google and ROS logging to WARN, retaining warnings and
errors. Restore scan-matching INFO output with `cartographer_log_level:=info`.
Standalone localization keeps its INFO default; mapping logging is unchanged.

For an already localized car:

```bash
ros2 launch control mpcc.launch.py track:=<track> speed_scale:=0.3
ros2 launch control mpcc.launch.py track:=<track> horizon_nodes:=40 dt:=0.05 deadline:=0.035
ros2 launch control mpcc.launch.py track:=<track> config_file:=/absolute/path/mpcc.yaml
```

The standalone launch uses `/localized_pose`, `/odom`, `vehicle.yaml`, and
`config/mpcc.yaml`. Explicit horizon/dt/deadline launch arguments override the YAML;
otherwise YAML values apply. `max_speed:=0` selects the configured cap (3 m/s), not
unlimited speed. The absolute cap applies after scaling the nominal speed envelope,
matching MAP's `max_speed` semantics. `use_sim_time:=true` requires the bridge's transport `/clock`;
use `false` consistently on hardware or when the bridge does not publish it.
Controller parameters are read at startup; restart the controller to change them.

If the bridge has fresh sensors but Cartographer receives no TF or tracked poses
in Docker/WSL, the optional Fast DDS UDP transport can avoid shared-memory delivery
failures. It preserves topic QoS and does not bypass localization gating:

```bash
scripts/race.sh <track> controller:=mpcc speed_scale:=0.3
```

The direct ROS launch default `dds_transport:=default` preserves the existing
middleware configuration; the simulator racing wrapper explicitly selects `udp`.

`speed_scale` has different meanings: MAP scales raceline waypoint speeds; MPCC
scales its centerline curvature/absolute speed envelope. Equal scales are not
matched-speed experiments. Record both policies; use matched absolute caps as a
second comparison and retain full configurations. MPCC's default 0.5 scale limits
speed to at most 1.5 m/s. It does not initially chase the TUM speed profile.

The existing validation runner supports both controllers and native simulator metrics:

```bash
python3 scripts/validate_race.py --track <track> --controller mpcc --speed-scale .3
python3 scripts/validate_race.py --track <track> --controller map --speed-scale .3
```

Its isolated localizer remaps and native-telemetry limitations still apply. The
existing default localization gate and bridge retain documented privileged simulator
dependencies; MPCC itself introduces no ground-truth state input.

## Formulation and implementation

State `x = [X,Y,yaw,v,delta,theta]` is referenced to the rear axle. Inputs are
`[acceleration, steering_rate, virtual_progress_rate]`. The physical dynamics are
the kinematic bicycle; `theta_dot = virtual_progress_rate`. Numerical validation
and acados use four-stage RK4/ERK. CasADi generates dynamics/constraint derivatives.

At each stage the periodic reference is locally approximated as
`r(theta) = r(theta_bar) + tangent(theta_bar)*(theta-theta_bar)`.
Contouring error is the normal projection of position error; lag error is its
tangent projection. The running objective, integrated over time, is:

```text
q_contour*e_contour² + q_lag*e_lag²
+ r_acceleration*a² + r_steering_rate*delta_rate² + r_steering*delta²
+ r_progress*progress_rate² - q_progress*progress_rate
```

Terminal cost penalizes contouring, lag and steering. The progress reward is
implemented by a shifted least-squares residual, keeping a Gauss-Newton Hessian.
Positive `r_progress` is a small regularization on virtual progress, not a reference
velocity tracker. Steering rate is an input rather than a post-solve steering clip.

Bounds cover acceleration/braking, speed, steering, steering rate, forward virtual
progress, lateral acceleration, lag, and a local progress trust region. Local
boundary halfspaces constrain all four rotated footprint corners at every predicted
stage and the terminal state. Boundaries have no slack. Widths are conservative
map-derived centerline widths; cubic interpolation never widens them.

A cyclic backward braking pass limits the speed envelope using curvature and
configured lateral acceleration, including corners beyond the MPC horizon.
Braking preview uses 70% of configured deceleration. Speed caps over the progress
trust region prevent the QP from stepping past a local cap into a nearby corner.

Each solve shifts the last accepted controls and rerolls states from the new
observation. Virtual progress is carried only when close to geometric projection.
This is an explicit primal warm start; unshifted dual variables are not reused.
Failures/reset epochs discard warm starts. Cold starts use curvature-derived
steering and bounded acceleration. One RTI iteration runs per request.

Independent validation rerolls controls from the initial state and checks:

- Nonfinite values, dynamics defects and control/state/lateral-acceleration limits.
- Contour corridor and lag constraints against the actual periodic spline.
- Dense footprint interior/edge samples against occupied, unknown and outside map
  cells. Spatial sampling is at half a map cell; temporal sampling includes body
  corner rotation speed. Map distance bounds include a conservative cell allowance.
- Full computation time, including preparation and validation.

Map resolution must be between 0.005 and 0.5 m/cell. The default footprint is a
**conservative enclosing box**, not calibrated overhangs: 0.50 m forward and 0.17 m
rearward from the rear axle, width 0.27 m. It follows from a 0.50 m body length,
0.33 m wheelbase and both axles lying within the body. Configure measured extents
before aggressive use; a conservative box may make narrow tracks infeasible.

`mpcc_model.py` separates symbolic and numerical vehicle dynamics, observation
initialization, command conversion and footprint geometry. `AcadosMPCC` accepts a
model factory; future dynamic models can append states while retaining the common
six-state layout. Additional state estimates/constraints must be supplied and tested.
Stage parameter 9 carries a reference-speed channel, unused by the standard cost,
for later velocity-guided objective extensions. No tire estimation, adaptive weights,
opponent handling or overtaking is implemented.

## Safety and timing

The ROS supervisor alone publishes `/drive`. A spawned process owns acados;
bounded queues permit only one in-flight request. Sequence/epoch checks reject
reset, invalidated and late results. A steady-clock 5 ms polling timer keeps safety
checks running when ROS time stalls. Measured state checks include frame, quaternion,
freshness, timestamp advancement/skew, pose jumps, heading recovery and absolute speed limit.
Small pose delays are aligned using measured forward speed/yaw rate.
This currently extrapolates the latest pose with the latest odometry; it does not
interpolate buffered observations to a common timestamp. Skew faults report the
signed pose-minus-odometry stamp difference, its limit, and both input ages.
Position, quaternion, odometry, and frame failures have distinct reasons; a negative
forward odometry reading is not reported as a corrupt pose.

Measured overshoot of the scaled target speed is handled as a braking initial
condition rather than invalid localization. A fixed initial speed allowance decays
at the configured deceleration between prediction nodes; the dense validator checks
the same time-dependent bound. It is not a solver-selected soft constraint. Commands
remain capped, and the configured absolute `max_speed`, actuator, lateral-acceleration
and boundary limits remain enforced. Raw target overshoot and recovery-bound violation
are logged separately. Negative/nonfinite odometry still stops the controller.

Default control period is 50 ms, horizon 30 steps, and request deadline 35 ms.
The solver checks its own total duration and the supervisor checks IPC/publication
latency again. An overrun, bad solve, invalid rollout or stale state publishes
repeated zero-speed commands while holding the last bounded steering. Resume needs
valid fresh inputs and a new valid timely solve. Sensor/clock faults additionally
require 0.25 seconds and three advancing pose/odometry pairs; computation/trajectory
failures invalidate the optimizer without resetting that sensor recovery latch.
Recovery warnings show the fresh-pair count and elapsed recovery duration. There is
no automatic handoff to MAP. A worker hung for 0.5 s is terminated and latches a
fault requiring controller restart.

The original 35 ms deadline and 0.005 m/s speed-envelope acceptance tolerance are
restored. All rollout, input-validity, publication, worker-deadline and shutdown
checks are enforced. Experimental validation/runtime-stop bypass code has been
removed, including acceptance of failed QP iterates and reuse of failed warm starts.
The briefly tested 45 ms deadline, 0.02 m/s envelope tolerance, and disabled stops
are not the shipped behavior. See `docs/mpcc_safety_restoration_20261108.md` for
experiment observations, verification and remaining limitations.

Publication deadline faults record `publication_time_s` (request latency plus
supervisor checks), `deadline_s`, and `deadline_overrun_s` in stop-event details;
console warnings show total/budget/lateness in milliseconds. Earlier logs did not
capture this elapsed time at the publication check, so their exact publication
overrun cannot be reconstructed. Rollout diagnostics retain the individual metrics
and add `violated_constraints`, listing each failed check, excess over the bound,
tolerance, and unit. A successful RTI QP status is not proof of nonlinear feasibility:
the independent dense rollout must also pass before any candidate is published.

Steering state is estimated from the last command. Longitudinal dynamics assume
acceleration tracking through the existing speed PI/ramp, whose default limits are
2.5 m/s² acceleration and 6.9 m/s² braking. MPCC defaults are more conservative at
2.0/3.0 m/s². Changing the adapter requires checking both settings. Neither the
kinematic model nor these stop commands guarantee collision-free braking.

Python, Docker and ordinary Linux scheduling provide soft real-time behavior;
deadline misses are expected to trigger stops. Keep the bridge command watchdog.
This controller does not detect new obstacles from `/scan` or provide a certified
terminal invariant set. Commission at low speed before using larger caps.

## Logging and tests

`controller_metrics_node` is an independent evaluator that never publishes commands.
Racing MAP/MPCC enables it by default; use `record_metrics:=false` to disable it.
Standalone MPCC also enables it. New unique directories under `~/ws/log/mpcc/`
contain a manifest, estimated states/commands and solver events in `samples.jsonl`,
logging loss/error status, and `summary.json` after shutdown. Manifests include
geometry/configuration, input and source hashes, numerical versions and localizer.
`MPCC_REVISION` may be supplied in the environment; otherwise revision is explicitly
unknown. Source hashes still fingerprint uncommitted code.

The common evaluator computes contour errors and footprint boundary/map clearance
for both controllers. Its geometric flying-lap timer excludes the initial partial
lap and resets on discontinuities. These estimated lap times are distinct from the
validation runner's native simulator times/collision counts. MAP solver timings
remain null. Queue loss/errors are recorded, not silently ignored.

```bash
ros2 run control controller_metrics_summary ~/ws/log/mpcc/map-<id> ~/ws/log/mpcc/mpcc-<id>

# Inside a built/sourced container, isolated from the live simulator:
export ACADOS_SOURCE_DIR=/opt/acados
export LD_LIBRARY_PATH=/opt/acados/lib:$LD_LIBRARY_PATH
export OPENBLAS_NUM_THREADS=1
ROS_DOMAIN_ID=64 /opt/mpcc_venv/bin/python3 -m pytest \
  src/control/test/test_mpcc.py src/control/test/test_mpcc_worker.py \
  src/control/test/test_mpcc_ros.py src/control/test/test_mpcc_launch.py \
  src/control/test/test_controller_metrics.py -q

/opt/mpcc_venv/bin/python3 src/control/tools/benchmark_mpcc.py \
  --steps 300 --horizon 30 --output /tmp/mpcc-benchmark-new.json
```

The benchmark uses an exact kinematic plant and an open synthetic map. It records
all attempted solves and stops on the first invalid/late result; output creation
is exclusive. It establishes neither AutoDRIVE lap performance nor hardware safety.
Tests cover actual generated acados code, independent infeasibility rejection,
process deadlines/reset epochs, ROS faults/recovery, installed launch, logging and
shutdown stop commands. See `docs/MPCC_VALIDATION.md` for measured results.

## References

- [alexliniger/MPCC](https://github.com/alexliniger/MPCC): progress augmentation,
  contour/lag objectives and local optimization.
- [F110_MPCC_ROS2](https://github.com/prajwalthakur/F110_MPCC_ROS2): ROS integration
  and computation-delay considerations.
- [f1tenth_benchmarks](https://github.com/BDEvan5/f1tenth_benchmarks): common
  centerline evaluation and reproducible comparison structure.
- [EVO-MPCC](https://github.com/zhouhengli/EVO-MPCC): future velocity-guided costs.

The dependency installer adapts this repository's MAP2 branch's pinned acados setup.
The MPCC model, solver and ROS supervision are separate implementations.
