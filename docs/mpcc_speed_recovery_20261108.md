# MPCC target-speed overshoot recovery

The run `ws/log/mpcc/mpcc-9ck91sdp/` used speed scale 0.3, giving a 0.9 m/s target
cap and a 3.0 m/s configured absolute limit. Encoder measurements reached 1.038
m/s. The supervisor previously rejected measurements above 0.905 m/s, conflating
the scaled target with input validity. Each rejection restarted the normal
0.25-second/three-new-sample recovery latch. The logged run had seven cap-stop
events and 28 recovery-wait events, plus independent deadline/invalid-state faults.

The supervisor now accepts finite nonnegative measurements up to the existing
absolute limit. The optimizer handles target overshoot as an initial braking
condition. For initial excess `e0 = max(0, measured_speed - local_target)`, the
fixed allowance at prediction node k is `max(0, e0 - deceleration*k*dt)`.
Intermediate allowances are linearly interpolated between nodes to match constant
acceleration over an interval. This allowance is determined before optimization;
it is not a tunable solver slack variable.

Future speed bounds include only that decaying initial allowance, capped by the
absolute speed limit. The independent dense rollout checks the same bound. Commands
still cannot exceed the scaled target cap. Braking/steering-rate, lateral acceleration,
track footprint, map occupancy and computation-deadline checks remain enforced.
Negative/nonfinite odometry and measurements above the absolute limit still stop
the controller and require normal input recovery. No sensor values are clipped.

Diagnostics retain raw `speed_violation_mps` and additionally report
`initial_speed_excess_mps` and `speed_recovery_violation_mps`, distinguishing an
initial measured overshoot from an invalid predicted recovery trajectory.

Verification:

- Eight model/ROS tests passed, covering feasible recovery, rejection when overshoot
  is not corrected, rejection of newly introduced overspeed, the absolute-limit
  stop, negative-speed stop and the original input-freshness/recovery behavior.
- Four worker/installed-launch tests passed.
- All eight recorded overspeed snapshots produced trajectories passing the physical
  and recovery-envelope checks in offline replay. Seven were timely; one took
  35.8 ms and was correctly rejected by the unchanged 35 ms deadline.

Replay output is preserved at `ws/log/mpcc_speed_recovery_20261108/replay.json`.
No new live racing lap was performed for this change. Restarting the controller is
required to apply it. Independent deadline misses, negative encoder measurements,
timestamp skew and the separate between-node speed-envelope issue remain possible
stop causes and are not suppressed by this change.
