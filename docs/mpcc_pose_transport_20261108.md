# MPCC missing-pose diagnosis and transport workaround

The reported persistent `MPCC stop: waiting for pose` was upstream of MPCC.
The running race, Cartographer and localization gate all used wall time, matching
the existing bridge. Correcting the clock alone did not resolve it.

Observed with the default DDS transport:

- LiDAR, IMU and odometry arrived with timestamps approximately 3–6 ms old.
- Pose-topic QoS settings were compatible and MPCC subscribed to `/localized_pose`.
- Cartographer had an active trajectory but published no `/tracked_pose` messages
  and had zero map constraints. Consequently the gate published no localized poses.
- Cartographer's TF buffer reported no frames, although independent Python and C++
  TF listeners received the sensor transforms and odometry transforms.
- Restarting the race launch with identical settings reproduced the failure.

Changing only the new localization processes to an explicit UDP-only Fast DDS
profile restored delivery: an eight-second check received 633 tracked poses and
581 localized poses. This isolates a middleware transport problem in this container;
the exact underlying shared-memory failure has not been characterized.

## Durable option

`control/config/fastdds_udp.xml` is installed with the package. Race launch accepts
`dds_transport:=udp`, which sets `FASTRTPS_DEFAULT_PROFILES_FILE` for its children.
It selects a user UDPv4 transport and disables built-in transports, including shared
memory. Topic names, QoS, localization gating and controller safety checks are
unchanged. The default option preserves the existing middleware configuration.
See the [Fast DDS transport configuration](https://fast-dds.docs.eprosima.com/en/2.x/fastdds/transport/whitelist.html)
for the XML transport mechanism.

Inside the sourced container:

```bash
USE_SIM_TIME=false scripts/race.sh porto_odom_trial \
  controller:=mpcc speed_scale:=0.3 dds_transport:=udp
```

The simulator-only `race.sh` wrapper now supplies those clock/transport defaults,
so the explicit environment variable and `dds_transport` argument above are optional.
Direct ROS launch defaults and the shared mapping defaults remain unchanged.

## Verification

The control package rebuilt successfully. Two focused launch tests passed,
including optional/default transport selection and the installed XML profile.

A bounded launch through `race.sh` with the installed UDP option observed 1,205
tracked poses, 61 localized poses, and a nonzero MPCC command. The run was stopped
immediately after that command: this verifies the pose-flow problem was resolved,
not a completed lap or the previously identified speed-envelope issue.

After cleanup there were zero `/drive` publishers and five consecutive measured
wheel-speed samples were 0.0 m/s. The pre-existing bridge remained running. The
diagnostic launches were stopped rather than left running in the background.

Local artifacts are preserved under
`ws/log/mpcc_pose_recovery_20261108_ppao99x1/`, including default-transport failure
logs, the UDP-only localization log and `race_udp_verified.log`/`verification.json`.
