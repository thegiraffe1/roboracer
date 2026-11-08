# MPCC implementation validation

Local validation on 2026-11-07 (America/Chicago), branch `feat/mpcc` based on `MAP`.
The automated results below establish implementation behavior and synthetic timing,
not successful AutoDRIVE racing. A subsequent [Porto lap attempt](mpcc_porto_trial_20261107.md)
timed out without completing a lap; it is recorded separately. The automated tests
below did not drive the normal ROS domain. No hardware run was performed.

## Environment

- ROS 2 Humble, x86_64, Docker service `ros`, WSL2 Linux host.
- acados v0.5.1, revision `48e223e85f0408ebfd1d8c6d6fb0589e9c41b3aa`.
- Generic BLASFEO/HPIPM target; SQP-RTI with partial condensing, ERK/RK4.
- `/opt/mpcc_venv`: NumPy 1.24.4, SciPy 1.10.1, CasADi 3.7.2.
- `OPENBLAS_NUM_THREADS=1`; the MPCC launch also sets `OMP_NUM_THREADS=1`.

The dependency installer was exercised in the existing container. That container
predated MAP's SLAM Toolbox dependency, which was installed to build its existing
`slam_graph_tools` package. `slam_graph_tools`, planning and perception built before
control. An old MAP2 control build contained stale launch-file symlinks; its generated
directory was preserved under the run artifacts rather than deleted, then control
was built normally. No maps or recorded racing runs were overwritten. A fresh full
Docker image build was not performed; Dockerfile/allowlist and the actual installer
were checked separately.

## Checks

From a sourced container:

```bash
colcon build --symlink-install --packages-select control

export ACADOS_SOURCE_DIR=/opt/acados
export LD_LIBRARY_PATH=/opt/acados/lib:$LD_LIBRARY_PATH
export OPENBLAS_NUM_THREADS=1
ROS_DOMAIN_ID=64 /opt/mpcc_venv/bin/python3 -m pytest src/control/test -q
```

The controller suite passed **108 tests** in 187.19 s at that checkpoint. After
final configuration/metadata changes and an additional symbolic-model regression,
the focused command below passed **14 tests** in 30.30 s (overlapping the first
suite, not 14 additional distinct tests):

```bash
ROS_DOMAIN_ID=64 /opt/mpcc_venv/bin/python3 -m pytest \
  src/control/test/test_mpcc.py src/control/test/test_mpcc_worker.py \
  src/control/test/test_controller_metrics.py src/control/test/test_mpcc_ros.py \
  src/control/test/test_mpcc_launch.py src/control/test/test_map_ros.py -q
```

Coverage includes generated native solves, progress from rest, shifted warm starts,
asymmetric corridors, map interior obstacles/unknown cells, derivative consistency,
infeasibility, process deadlines/epoch invalidation, hung worker shutdown, ROS input
faults and recovery, and installed launch with a real acados worker and logger.
The installed launch test uses domain 67; MAP ROS tests use 42; MPCC fault tests use
64. A startup/exit regression was fixed so repeated launch SIGINT does not interrupt
the stop burst or log flush. Native solver failure output in the infeasibility test
is expected and does not indicate an accepted unsafe result.

Additional checks:

- `python3 -m unittest discover -s ws/scripts/test -q`: **28 passed**.
- `python3 -m pytest src/planning/test/test_vehicle_params.py -q`: **3 passed**.
- `ros2 launch control mpcc.launch.py --show-args`: installed launch resolved.
- `git diff --check`, shell syntax and Python compilation checks passed.

## Synthetic timing

Command, inside the MPCC environment:

```bash
/opt/mpcc_venv/bin/python3 src/control/tools/benchmark_mpcc.py \
  --steps 300 --horizon 30 --output /tmp/mpcc-circle-release-30.json
```

The plant follows exactly the same kinematic model, on a 3 m radius circle with
asymmetric widths 0.8/0.6 m and an open synthetic occupancy map. It starts from rest.
Each next observation uses geometric projection, not optimized virtual progress.
This excludes ROS/IPC and actuator/localization mismatch; separate worker/launch
tests cover ROS/IPC acceptance and deadline rejection, not its full latency distribution.

All **300/300** requests passed independent validation and the 35 ms deadline.
Measured geometric progress was **21.94 m** (including crossing the periodic seam).

| Computation | Median | p95 | p99 | Maximum |
| --- | --- | --- | --- | --- |
| acados solve | 1.51 ms | 2.49 ms | 3.09 ms | 3.63 ms |
| Total preparation + solve + validation/collection | 14.19 ms | 18.23 ms | 21.44 ms | 27.16 ms |

Maximum sampled corridor violation was 0 m; maximum nonlinear dynamics defect was
0.000190 in the state's native SI components. These figures do not bound unseen
obstacles, estimator error, actuator tracking, or worst-case operating-system delay.

Earlier attempts were retained. They exposed full Python cyclic garbage collection
inside solver preparation, producing roughly 54–62 ms requests. Compilation objects
are now collected before requests, automatic full collection is disabled during each
request, and generation-zero collection runs every request and is charged to its
deadline. The caller's GC setting is restored afterward. Both post-fix 300-step
30-node runs passed; the earlier run's maximum total was 25.10 ms.

A separate **40-node** cold-start attempt exceeded the unchanged 35 ms deadline
(54.25 ms total, 3.18 ms native solve), and was rejected. That configuration is not
validated at the default deadline. Horizon changes require remeasuring total timing;
do not infer support from native solver time alone or disable the deadline check.

Raw JSON, including failed attempts, is preserved locally under
`ws/log/mpcc_implementation_a5de8mwx/` (git-ignored). The final file is
`mpcc-circle-release-30.json`; the earlier successful run is `mpcc-circle-gc-30.json`.

## Remaining commissioning

Run repeated MAP/MPCC comparisons with the same track, estimator and adapter,
keeping native lap/collision metrics separate from estimated centerline metrics.
Measure front/rear overhangs, steering feedback/tracking and speed-loop response;
the current footprint is deliberately conservative and steering state is command
derived. Confirm map widths/clearance and fault-stop behavior on the selected track
at low speed before raising limits. Existing default localization/bridge privileged
telemetry dependencies remain documented review debt, not new MPCC inputs.
