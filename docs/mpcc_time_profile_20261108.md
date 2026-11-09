# MPCC timing investigation

Checkpoint `84fe4f1` was committed and pushed to `origin/feat/mpcc` before this
investigation. The subsequent instrumentation and this report are included in the
later safety-restoration checkpoint. This timing investigation itself changed no
weights, limits, deadlines or speed-envelope constraints.

The native solver is not the principal cost. Python trajectory construction,
acados interface calls, repeated supervisor work, and result-observation latency
consume most of the budget. During fault recovery, ROS warning caller lookup is
particularly expensive even when actual warning output is throttled.

## Measurements

The input is the recorded Porto trial `ws/log/validation/race-acj5bxm1/`, using its
actual configuration: 30 nodes, 50 ms period, 35 ms deadline, speed scale 0.3.
Optional instrumentation records both wall and process CPU time by phase. Native
solver calls are unchanged. Separate cProfile windows identify function calls;
their overhead is excluded from the phase distributions below.

Experiments:

- 200 uniformly selected independent cold snapshots, with no localizer running.
- 200 contiguous snapshots, following recorded recovery/reset events and retaining
  valid warm starts, again without a localizer.
- The same contiguous snapshots with Cartographer running and a read-only ROS
  observer receiving 1,003 localized poses and 295 odometry messages.
- Two 20-second runs of the actual supervisor with `/drive` replaced by an
  in-memory sink. An additional five-second call profile followed the detailed run.

No profiling process created a `/drive` publisher or requested a simulator reset.
Temporary localizers were stopped afterward; the existing bridge remained running.
The final graph contained only bridge/Foxglove nodes and zero drive publishers.

### Worker computation under localization load

| Phase | Median wall time |
| --- | ---: |
| Warm seed construction | 1.41 ms |
| Cold seed construction | 6.49 ms |
| Reference parameter sampling | 0.74 ms |
| Python acados updates + extraction | 4.16 ms |
| Native acados solve | 1.43 ms |
| Independent nonlinear validation, combined | 4.60 ms |
| Complete worker computation | 13.34 ms |

These are component medians and should not be summed as an exact accounting of
the total median. Validation includes physical rollout, intermediate states,
corridor checks and map checks. This replay had 61 dense states and 20,496 footprint
samples per request. Map clearance querying itself was 0.98 ms median; it was not
the dominant cost.

The worker's measured CPU and wall phase durations were close. These replays do
not show a large worker-side blocking/preemption gap, although they do not rule
out scheduling delays in the original moving trial.

### Request observation and process communication

| Experiment | Request median | Request p95 | Misses >=35 ms |
| --- | ---: | ---: | ---: |
| Contiguous replay, no localizer | 16.38 ms | 30.98 ms | 2/200 |
| Contiguous replay, Cartographer + ROS observer | 16.67 ms | 28.87 ms | 4/200 |
| Original moving Porto trial | 20.04 ms | 36.51 ms | See trial report |

Under localization load, inbound queue/scheduling latency was 0.61 ms median
(1.00 ms p95). Worker completion to observation was 2.94 ms median (5.43 ms p95).
The latter includes result transfer and the deliberately reproduced 5 ms polling
cycle; it is not pure serialization cost.

The replay observer does not execute supervisor state validation or command
publication. Consequently its request times cannot substitute for the original
trial's end-to-end distribution. The paired runs also do not demonstrate that
Cartographer alone causes the tail: their p95 values overlap and vary.

### Actual supervisor, with commands suppressed

The detailed 20-second run recorded 3,958 polling callbacks:

| Operation | Median | p95 | Maximum |
| --- | ---: | ---: | ---: |
| Complete polling callback | 1.74 ms | 3.38 ms | 14.13 ms |
| Successful state validation/projection | 2.01 ms | 4.31 ms | 13.62 ms |
| Rejected state check | 0.28 ms | 0.50 ms | 7.80 ms |
| Stop handling, including throttled warning | 1.20 ms | 2.27 ms | 8.52 ms |
| Constructing a command sent only to the sink | 0.16 ms | 0.31 ms | 7.27 ms |

The timer's actual interval was 5.00 ms median, 6.09 ms p95 and 15.81 ms maximum.
The earlier supervisor run observed a 24.38 ms callback maximum. This matters
because result polling happens on this same callback path, and state validation
runs again before an accepted result can be published.

The separate five-second call profile attributed **1.396 s to 890 warning calls**.
Most of that was caller discovery: `CallerId`, `_find_caller`, `inspect.getframeinfo`,
`realpath` and filesystem `stat`/`lstat`. Throttling suppresses output after this
work has already happened. The stop path calls the logger on almost every 5 ms
tick during recovery. The 1.396 s includes cProfile overhead; the unprofiled stop
timings independently establish the cost. This sample represents recovery behavior,
not steady healthy racing.

## Optimization order

1. Check a monotonic warning-rate limit **before** calling ROS logging. Continue
   publishing repeated stops and retaining diagnostic reasons.
2. Keep fast input-freshness/deadline checks on the polling path, but avoid repeating
   full track projection when no new observation or solve/publication needs it.
   Preserve explicit bounds on invalid-input stop latency and add regressions.
3. Reduce Python seed and acados interface overhead. The call profile identifies
   scalar NumPy clipping, repeated `speed_limit()` construction of padded periodic
   arrays, RK4 allocation and per-stage setters/getters. Cache immutable track
   arrays first; consider native/batched interface work after measuring that change.
4. Measure completion notification versus fixed polling to recover part of the
   approximately 3 ms median observation delay. Keep the independent deadline check.

These are findings and proposed changes, not implemented speedups. The independent
speed-envelope discretization issue from the failed lap remains unresolved.

## Reproduction and artifacts

Inside the sourced ROS container, with `ACADOS_SOURCE_DIR=/opt/acados`, the acados
library path, and `OPENBLAS_NUM_THREADS=1`:

```bash
/opt/mpcc_venv/bin/python3 src/control/tools/profile_mpcc.py \
  log/validation/race-acj5bxm1 --samples 200 --selection contiguous \
  --seed-mode recorded --output log/mpcc_profile_new

# With localization already running, append --ros-listen for the replay observer.
# The supervisor tool requires localization and creates a sink instead of /drive:
/opt/mpcc_venv/bin/python3 src/control/tools/profile_mpcc_supervisor.py \
  --track porto_odom_trial --seconds 20 --output log/mpcc_supervisor_new
```

Outputs are exclusively created and include samples, summaries and separate
`calls.prof`/`calls.txt` attribution. Local raw directories are:

- `ws/log/mpcc_profile_quiet_20261107/`
- `ws/log/mpcc_profile_quiet_recorded_20261107/`
- `ws/log/mpcc_profile_cartographer_recorded_20261107/`
- `ws/log/mpcc_profile_supervisor_20261107/`
- `ws/log/mpcc_profile_supervisor_detail_20261107/`

The read-only localization orchestration scripts/logs are preserved alongside these
directories. Source snapshots are under `ws/log/mpcc_profile_sources_20261108/`.

Verification after instrumentation: **13 focused tests passed** in 44.79 seconds,
including native acados solving, profiling accounting, worker deadlines, ROS safety
and the installed launch. `git diff --check` and Python compilation checks passed.
