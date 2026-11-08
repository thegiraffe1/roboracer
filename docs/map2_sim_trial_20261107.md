# MAP2 Porto simulator trial — 2026-11-07

MAP2 drove Porto successfully in one bounded trial at half raceline speed scale.
The trial stopped at two native lap-counter increments with zero native collisions.
This includes one timed flying lap; the first increment includes the initial
partial lap. It is not evidence of reliable full-speed operation.

## Configuration and outcome

Branch `MAP2`, base commit `aa814a7`, with the local executable/vendor packaging,
solver dependency setup and race controller selection changes. AutoDRIVE was
reopened on Porto and connected to the freshly rebuilt bridge. Command inside
the sourced Humble container:

```bash
scripts/race.sh porto_odom_trial controller:=map2 speed_scale:=0.5 max_speed:=3.0
```

Cartographer localization; avoidance disabled. MAP2 configuration remained the
branch defaults: 20 horizon nodes, 50 ms period, 0.5 m maximum offset, 0.4 m soft
reference margin, analytic steering and no GP. No controller thresholds, weights
or speed defaults were changed for this trial.

| Measurement | Result |
|---|---:|
| Native lap-counter change | 0 → 2 |
| Native collision-counter change | 0 → 0 |
| Duration from first positive drive to target | 28.53 s |
| Last native full-lap time | 14.2491 s |
| Mean positive commanded speed | 1.922 m/s |
| Maximum commanded speed | 3.000 m/s |
| Localizer-reported cross-track RMS | 0.0183 m |
| Localizer-reported cross-track P95 / maximum | 0.0368 / 0.0986 m |
| MAP2 solver-failure / deadline warning lines | 0 / 0 |
| Zero-speed commands during active interval | 0 |
| Observed command interval P95 / maximum | 54.33 / 90.27 ms |

Cross-track error uses nearest raceline segments and 2,818 localized-pose samples
between the first positive drive command and termination. It measures agreement
with Cartographer's reported pose, not independent physical tracking accuracy.
Command intervals use observer receipt times; their maximum shows that absence
of solver deadline warnings does not establish a strict 20 Hz command guarantee.
The earlier full-speed PP/MAP experiments used different controllers/settings
and cannot be ranked against this run.

## Initial failed attempt and recovery

The first attempt localized but never drove. The old bridge process stamped
odometry approximately 241,671 s (67.1 hours) behind the current ROS clock;
MAP2 correctly rejected it as stale. The attempt timed out after 45 seconds,
with zero lap increments and zero collisions. The bridge's prior monotonic
epoch stamping can retain an offset after host clock changes.

After the user closed AutoDRIVE, the bridge process group was stopped with
SIGINT and its nodes were confirmed gone. `autodrive_bridge` was rebuilt from
MAP2 and restarted. The user reopened/reconnected the simulator. The retry's
odometry ages were approximately 3–6 ms. No timestamp tolerance was relaxed.

## Evidence and final state

Successful trial: `ws/log/map2_trial_20261108T002025Z/`.
Initial failed attempt: `ws/log/map2_trial_20261108T001709Z/`.
Both retain the runner, parameter/artifact hashes, `race.log`, `events.jsonl`,
`summary.json` and a finalized ROS bag. The successful run also has `metrics.json`.
The runner observes native counters, stops the experiment on a collision, two
lap increments or timeout, and sends repeated terminal zero commands after the
race launch exits. Native counters are evaluation inputs, not MAP2 inputs.

All race/localization processes exited cleanly after the successful run.
Post-run checks found zero `/drive` publishers and three consecutive wheel-speed
samples of 0.0 m/s. The bridge and Foxglove remain running; maps and racelines
were not regenerated.

Existing review debt remains: the bridge publishes restricted telemetry and
automatic reset events, and `localization_gate` subscribes to ground-truth odometry
for reset/reinitialization behavior. This run used that existing stack and does
not establish ground-truth independence. MAP2 itself consumed `/localized_pose`
and `/odom`; no privileged pose was added as a controller input.

## Follow-up: speed scale 1.0

At the user's request, repeated the same bounded trial at `speed_scale:=1.0`,
retaining `max_speed:=3.0`, Cartographer, the same map/raceline and controller
configuration, and the two-increment target. The instruction was to reduce
the scale after a crash. The scale-1.0 attempt reached the target without a
collision, so no lower-scale retries were performed.

```bash
scripts/race.sh porto_odom_trial controller:=map2 speed_scale:=1.0 max_speed:=3.0
```

| Measurement | Result |
|---|---:|
| Native lap-counter / collision-counter changes | 0 → 2 / 0 → 0 |
| Duration from first positive drive to target | 20.84 s |
| Last native full-lap time | 10.3616 s |
| Mean / maximum positive commanded speed | 2.839 / 3.000 m/s |
| Localizer-reported cross-track RMS | 0.1540 m |
| Localizer-reported cross-track P95 / maximum | 0.3886 / 0.4833 m |
| MAP2 solver-failure / deadline warning lines | 0 / 0 |
| Zero-speed commands during active interval | 0 |
| Observed command interval P95 / maximum | 55.25 / 89.38 ms |

The speed cap was reached in about 56% of active drive messages, so this was a
scale-1.0 run **with a 3 m/s cap**, not an uncapped raceline-speed test. Tracking
agreement with the localized pose deteriorated substantially from the half-scale
trial, and maximum error approached the configured 0.5 m offset stop threshold.
One successful attempt establishes neither repeatability nor full-speed safety.
As above, localized-pose errors are not independent physical accuracy bounds.

Evidence: `ws/log/map2_trial_20261108T002652Z/`, with the runner, artifact/config
hashes, finalized ROS bag, `race.log`, `events.jsonl`, `summary.json`, and
`metrics.json`. The controller/config/raceline hashes match the prior successful
trial. No controller settings were retuned during the run.

## Follow-up: removing the 3 m/s cap

Kept `speed_scale:=1.0` and raised the finite MPC speed bound to 6.252 m/s,
just above the saved raceline maximum of 6.251986 m/s. After that attempt
failed to finish, retried with 5.0 and 4.5 m/s caps. Other controller settings
and artifact hashes remained unchanged.

| Speed bound | Lap increments | Collisions | Outcome |
|---|---:|---:|---|
| 6.252 m/s (full raceline range) | 0 | 0 | Deadline warning, then persistent offset stop; timed out |
| 5.0 m/s | 0 | 0 | Deadline warning, then persistent offset stop |
| 4.5 m/s | 0 | 0 | Deadline warning, then persistent offset stop |

Each run logged `MAP2 solve exceeded the control period` followed by
`car is too far from an aligned raceline segment`. Positive speed commands
spanned approximately 3.00, 3.09 and 4.20 seconds respectively. These were
tracking/safety-stop failures, not recorded collisions. The warning sequence
does not establish that the deadline event caused the tracking deviation.
Warnings are throttled, so warning-line counts do not count every failed tick.

The first run retained the original 60-second timeout after driving began.
For the capped retries, the evaluation runner also ended a trial after eight
seconds without positive commands while wheel speed was below 0.1 m/s.
This changed only experiment termination; no controller safety checks were
relaxed. A proposed 4.0 m/s attempt was interrupted before it started.

Evidence directories, in the same order:

- `ws/log/map2_trial_20261108T003147Z/`
- `ws/log/map2_trial_20261108T003319Z/`
- `ws/log/map2_trial_20261108T003445Z/`

Each retains its runner, manifest, summary, command/pose events, launch logs
and finalized ROS bag. These outcomes do not support using the higher bounds
for reliable laps; the 3 m/s result above remains a single successful trial.
