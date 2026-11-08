# MPCC Porto one-lap attempt — 2026-11-07

The first live MPCC lap test **failed to complete a lap**. It stopped at the
180-second limit with zero native lap increments and zero native collisions.
This is a failed commissioning attempt, not a collision-free completed-lap result.

## Configuration

Existing Porto simulator/bridge on port 4567, track `porto_odom_trial`, Cartographer
localization, MPCC speed scale 0.3 (0.9 m/s nominal cap), default 30-node horizon,
50 ms period and 35 ms request deadline. The already-running bridge used wall time;
the validation launch matched it explicitly. No bridge restart or map regeneration.

```bash
python3 scripts/validate_race.py --track porto_odom_trial \
  --localizer cartographer --controller mpcc --speed-scale 0.3 \
  --laps 2 --timeout 180 --startup-timeout 45 --use-sim-time false
```

The two-increment target was chosen to capture one full flying lap after the
potential partial first lap. Neither increment occurred. Existing validation remaps
disabled privileged pose/reset hints to the localizer; the unchanged bridge's
privileged telemetry/safety dependencies remain as documented in the runner.

## Measured result

| Metric | Result |
| --- | --- |
| Runner status | Failed: total timeout |
| Native lap increments / collisions | 0 / 0 |
| Full lap time | Unavailable |
| Estimated projected progress | 16.36 m |
| Estimated centerline contour RMSE / p95 | 0.0122 / 0.0186 m |
| Maximum sampled footprint corridor violation | 0 m |
| Request deadline misses | 89 |
| Additional publication deadline stops | 30 |
| Nonlinear rollout rejections | 160 |
| Total request-time median / p95 / maximum | 20.04 / 36.51 / 46.26 ms |
| Native solve median / p95 / maximum | 1.63 / 3.02 / 13.63 ms |
| Logging drops / errors | 0 / none |
| Cleanup escalation | None |

The contour/clearance figures use the estimated localized trajectory and do not
establish independent physical clearance. The car spent substantial time stopped.

## Findings

1. Native solve time is not the complete timing budget. With live localization and
   ROS traffic, request p95 exceeded the 35 ms deadline. Those results were rejected.
2. Repeated nonlinear rollout rejection eventually prevented restart near progress
   39.5 m (unwrapped). A representative on-time solve returned status 0 but its dense
   rollout exceeded the speed envelope by approximately **0.0107 m/s**, above the
   configured 0.005 tolerance. Corridor/map checks and lateral acceleration passed
   for that result. This points to insufficient conservative speed-envelope bounds
   between sampled constraints; it needs correction and regression coverage.
3. The measured encoder speed included negative samples (minimum logged -0.377
   m/s) and occasional cap exceedance (maximum 0.942 m/s). Strict state checks,
   timestamp skew and the recovery latch added stop/restart events. Diagnose the
   measurements and actuator response before changing their acceptance policy.

No deadline, state, boundary or feasibility thresholds were relaxed to make this
attempt pass. Following analysis, the logger's ambiguous legacy reason text was
fixed so nonlinear validation failures are not incorrectly counted as deadline
misses. The regenerated summary reports 119 total deadline misses (89 request +
30 publication), rather than the original mislabeled 249. Raw observations remain
unchanged. The numerical controller behavior was not retuned during the attempt.

## Artifacts and final state

Local artifacts: `ws/log/validation/race-acj5bxm1/`, including `report.json`,
`race.log`, `observations.jsonl`, and the independent evaluator's manifest,
`samples.jsonl` and corrected `summary.json` under
`controller_metrics/mpcc-2zggidm_/`. Track/input hashes are recorded in the report.

After cleanup, `/drive` had zero publishers, controller/localizer processes had
exited, and five consecutive measured wheel-speed samples were exactly 0.0 m/s.
The pre-existing bridge and Foxglove remained running.
