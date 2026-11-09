# MPCC safety restoration after simulator experiments

The experimental runtime-stop and rollout-validation bypasses have been removed
from the implementation, not merely switched off in YAML. The installed controller
again enforces successful QP status, finite outputs, independent nonlinear rollout
validation, actuator/publication checks, input health/recovery, localization gating,
worker epochs/deadlines, hung-worker handling, repeated braking and shutdown stops.
The deadline is restored to 35 ms within the 50 ms control period. Speed-envelope
acceptance slack is restored to 0.005 m/s. Default speed scale remains 0.5.

The checkpoint retains the earlier timing instrumentation and read-only profiling
tools, logger rate limiting before ROS logging, detailed fault/constraint messages,
explicit braking recovery for initial target-speed overshoot, separation of sensor
recovery from solver failures, simulator race-wrapper clock/UDP defaults, and quiet
Cartographer race logging. Commands remain capped and absolute vehicle limits are
checked independently of the speed-envelope acceptance tolerance.

## Experiment observations and limits

The user reported a completed lap with the temporary 45 ms deadline and 0.02 m/s
speed-envelope tolerance. That report is not an independently validated lap for the
restored settings. No new live simulator lap was run during restoration.

The later bypass run `ws/log/mpcc/mpcc-1s7i8xz3/` used speed scale 1.0. At inspection
it contained 51 successful solves followed by 677 outer acados status-4 QP failures.
Before the first failure, reported speed changed from approximately 1.25 to 0.25 m/s
between requests. Later failed iterates retained virtual progress rates near 3 m/s
while the car was nearly stationary. Predicted lag and boundary violations became
large; these are forecast failures, not measurements of the car crossing a boundary.
Accepting and warm-starting from unsuccessful QP iterates could perpetuate failure;
that experimental path is removed. Logs are preserved as generated artifacts.

The nine-sample speed-envelope approximation can still miss a local minimum that
dense validation catches. Timing overruns, input skew/negative odometry, and
localization problems remain possible stop causes. This checkpoint restores the
checks; it does not establish reliable racing performance or collision protection.
MPCC still consumes `/localized_pose`. The existing localization gate's use of
ground-truth/map-origin metadata remains review debt; this checkpoint does not
claim that the complete default stack is ground-truth independent.

## Verification

Built and sourced control/perception in the existing ROS container:

```bash
colcon build --symlink-install --packages-select control perception
# With ACADOS_SOURCE_DIR=/opt/acados, LD_LIBRARY_PATH including /opt/acados/lib,
# and OPENBLAS_NUM_THREADS=1:
ROS_DOMAIN_ID=42 /opt/mpcc_venv/bin/python3 -m pytest \
  src/control/test src/perception/test/test_localization_launch_logging.py \
  src/planning/test/test_vehicle_params.py -q
/opt/mpcc_venv/bin/python3 -m pytest scripts/test -q
```

- Build: both packages passed.
- Controller/launch/vehicle checks: 129 passed, one failed, no skips, in 219.90 s.
- Script checks: 31 passed in 1.41 s.
- `git diff --check` passed.
- Installed-launch regression confirmed the shipped 35 ms deadline and 0.005 m/s
  speed tolerance, with no experimental bypass fields in its manifest.
- A new regression confirms a failed QP clears warm starts and cannot expose a
  driving command. ROS checks cover stale/invalid input braking and shutdown.

The failed test was `test_acados_progress_warm_start_and_infeasibility`: its first
QP succeeded, but total request computation exceeded that test's 49 ms deadline.
A standalone invocation reproduced the timing rejection at 56.24 ms total,
including 19.29 ms assembly and 2.20 ms native solve, with no rollout constraint
violations. The deadline rejection remains enforced. This is an unresolved timing
limitation; this checkpoint does not claim a fully passing controller suite or a
validated simulator lap under the restored defaults.
