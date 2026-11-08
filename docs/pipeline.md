# Unseen-track pipeline

The stack is built to race a track it has never seen. Nothing track-specific is committed:
every map and raceline is generated on the day, into `ws/tracks/<name>/` (git-ignored).

```
new track ─► Cartographer mapping ─► centerline ─► TUM mintime raceline ─► localization + pure pursuit
             (keyboard teleop or
              wall follower lap)
```

## Commands

Inside the container (`docker compose exec ros bash`, working directory `~/ws`), with the simulator connected:

```bash
ros2 launch autodrive_bridge autodrive_launch.py     # shell 1: bridge (always running)          alias: sim
scripts/map_track.sh <name>                         # shell 2: map -> save -> centerline -> mintime   map-track
scripts/map_track.sh <name> --wall-follow           #   ... or let the wall follower drive 2 laps (--laps N), the default
scripts/map_track.sh <name> --gap-follow            #   ... or follow the gap (experimental: planning can fail, control/README.md)
ros2 run autodrive_bridge teleop_keyboard           # shell 3: drive the mapping lap(s)            kb
scripts/race.sh <name> [speed_scale:=0.5]           # localization + pure pursuit                  race
scripts/race.sh <name> controller:=map2             # localization + experimental MAP2 MPC
scripts/race.sh <name> avoid:=true                  #   ... passing obstacles the map did not have
ros2 launch control follow_the_gap.launch.py [speed:=1.0]   # no-map fallback, instead of race.sh   ftg
```

MAP2 is installed by the Docker image, including acados v0.5.1 and CasADi 3.6.7
in `/opt/map2_venv`. Rebuild the image after pulling this setup. To add its
dependencies to an already running container without restarting the bridge:

```bash
# Inside the ROS container:
bash src/control/docker/install_map2_deps.sh
colcon build --symlink-install --packages-select control
source install/setup.bash
scripts/race.sh <name> controller:=map2 speed_scale:=0.5 max_speed:=3.0
```

Pure pursuit remains the default controller. MAP2 uses `/localized_pose` and
`/odom`; its 3 m/s model bound applies when the shared `max_speed:=0.0` argument
is left at its default. An explicit positive `max_speed` overrides that bound.
`controller:=map2 avoid:=true` is rejected because MAP2 does not follow obstacle
avoidance paths. For an existing separately running localizer, launch only the
controller with `ros2 launch control map2.launch.py track:=<name>`.
That launch accepts `python_executable:=<path>` for a different solver environment;
`ACADOS_SOURCE_DIR` can override the default `/opt/acados` installation.
The analytic steering adapter and fixed soft reference corridor are experimental;
startup and synthetic solve checks do not establish collision-free laps.
Dependency setup follows the [acados installation instructions](https://docs.acados.org/installation/index.html)
and [Python interface instructions](https://docs.acados.org/python_interface/index.html#installation).

Setup validation (2026-11-07): the installer ran in the existing Humble container;
symlink and clean non-symlink control builds passed. The command
`ROS_DOMAIN_ID=68 python3 -m pytest src/control/test/test_map2.py src/control/test/test_map2_node.py -q`
passed all 23 tests. A separate `/opt/map2_venv/bin/python3` smoke check completed
320 real straight-reference solves successfully and verified standalone MAP2
and combined Cartographer/MAP2 startup on domain 68. Warm solver-only timings
were median 5.13 ms, P95 7.70 ms, maximum 62.36 ms; the maximum exceeded the
50 ms control period, so this does not validate deadline reliability. The checks
used no live sensor inputs and performed no simulator laps. The complete Docker
image was not rebuilt during these checks. Existing bridge/localizer privileged
reset and initialization dependencies remain; the default stack is not claimed
to be ground-truth independent.

`map_track.sh` resets the sim car, starts Cartographer mapping, waits for you to finish driving and
press Enter (with `--wall-follow` or `--gap-follow`: starts `control wall_follow.launch.py` or `disparity_gap.launch.py` and waits for its laps), runs `save_map`, stops mapping, then runs `ros2 run planning build_raceline <name>`.
It needs a terminal (it refuses to start without one) and exits early if `cartographer_node` dies on startup.
If `save_map` fails, mapping keeps running and it asks to retry (Enter) or quit (`q`, the map is lost; delete `tracks/<name>` before reusing the name).
`race.sh` resets the sim car (localization starts at the map origin = where mapping started) and
launches `control race.launch.py` (perception `localization.launch.py` + control `pure_pursuit.launch.py`).
With `avoid:=true` it also starts planning's `obstacle_spliner_node`, and pure pursuit steers on its
`/local_path` (the raceline shifted sideways around an obstacle on `/scan` that the map did not have)
instead of the raceline, at the raceline's speeds. With no fresh collision-free path the car stops.
If mapping fails on race day, `follow_the_gap.launch.py` drives on `/scan` alone (no map); run it instead of
`race.sh`, never alongside it, since both publish `/drive`.

### Stopping a launch

Stop `ros2 launch` with Ctrl-C, or from a script with SIGINT to its whole process group:
`kill -INT -- -<pgid>` (`map_track.sh` does this). SIGTERM to the `ros2 launch` process alone
(e.g. `timeout ros2 launch ...`, `kill <pid>`) exits the launch but leaves every node it started running.
Check with `ros2 node list` and kill leftovers before the next launch.

## Packages (`ws/src/`)

| Package | Contents |
|---|---|
| `autodrive_bridge` | `fast_bridge`, `adapter`, `wheel_odom`, `teleop_keyboard`; `autodrive_launch.py`. Sim only: replaced by f1tenth_system on the car. |
| `perception` | Cartographer configs, `mapping.launch.py`, `localization.launch.py`, `save_map`, `map_origin`. Particle filter localization instead of Cartographer's: `mcl.launch.py` (`mcl_node`, `mcl.py`, `config/mcl.yaml`). Experimental, not in the pipeline: `ekf.launch.py` (robot_localization IMU EKF + `corrector`, a drift-corrected forward speed on `/odometry/corrected`, sim only) |
| `planning` | `build_raceline` = `centerline` extraction + TUM `mintime` optimization. `obstacle_spliner_node`: stationary-obstacle avoidance, `/local_path` (`race.launch.py avoid:=true`) |
| `control` | `pure_pursuit_node`, `pure_pursuit.launch.py`, `race.launch.py`. `follow_the_gap_node` (`follow_the_gap.launch.py`): reactive `/scan` -> `/drive`, needs no map, the fallback. `wall_follow_node` (`wall_follow.launch.py`): drives the mapping laps (`map_track.sh --wall-follow`), the default. `disparity_gap_node` (`disparity_gap.launch.py`): follow the gap with disparity extension, also drives the mapping laps (`map_track.sh --gap-follow`, experimental: see control/README.md) |

## Interfaces

| Topic / frame | Type | Producer |
|---|---|---|
| `/scan` (frame `laser`) | `sensor_msgs/LaserScan` | adapter (sim) / urg_node (car) |
| `/odom`, TF `odom -> base_link` | `nav_msgs/Odometry` | wheel_odom (sim) / vesc_to_odom (car) |
| `/sensors/imu/raw` (frame `imu`) | `sensor_msgs/Imu`, accel includes gravity; Cartographer uses it | adapter (sim) / vesc (car) |
| `/drive` | `ackermann_msgs/AckermannDriveStamped` | teleop_keyboard, wall_follow or disparity_gap (mapping), pure_pursuit, follow_the_gap |
| TF `map -> odom` | | Cartographer (mapping and localization), or `mcl_node` with `localizer:=mcl` |
| `/tracked_pose` | `geometry_msgs/PoseStamped`, frame `map` | Cartographer (`publish_tracked_pose`) |
| `/localized_pose` | `geometry_msgs/PoseStamped`, frame `map` | localization_gate (or `mcl_node`): `/tracked_pose` (or the filter pose) relayed once localized; withheld after a sim reset (`sim_reset`) until relocalized (race.launch.py's `pose_topic`) |
| `/autodrive/roboracer_1/sim_reset` | `std_msgs/String`, the reason | fast_bridge (sim only): one event per sim reset. Reset commands always; respawns and other teleports (collision count change, position jump) only with restricted topics. fast_bridge also replies zeros for `reset_frames` frames |
| `/local_path` | `nav_msgs/Path`, frame `map`, one pose per raceline point | obstacle_spliner_node: the raceline, shifted around the nearest obstacle in the way; pure_pursuit follows it with `avoid:=true` |
| `/local_path/valid` | `std_msgs/Bool` | obstacle_spliner_node: false when its pose or scan is stale or no side is collision-free (pure_pursuit then stops) |
| `/ground_truth/pose`, TF `world -> ground_truth` | `geometry_msgs/PoseStamped`, frame `world` (sim) | adapter, debugging only |

Static TF `base_link -> laser, imu, front_camera` comes from the adapter. `base_link` is the rear axle.
Cartographer runs with `use_imu_data = true`, which requires `imu` at the origin of `base_link`
(rotation allowed). On the car, publish `base_link -> imu` with zero translation and the VESC's
mounting rotation, or Cartographer stops waiting for an IMU it cannot transform.

## Track directory `ws/tracks/<name>/`

| File | Written by | Format |
|---|---|---|
| `map.pbstream` | `save_map` | Cartographer state, loaded by `localization.launch.py` |
| `map.pgm`, `map.yaml` | `save_map` | ROS map_server occupancy grid, `map` frame |
| `trajectory.csv` | `save_map` | `# t,x_m,y_m,yaw_rad`: the mapping lap's optimized poses (`/trajectory_query`), gives driving direction |
| `map_to_world.yaml` | `save_map` | ground-truth pose of the `map` origin in the sim world frame (debugging) |
| `centerline.csv` | `build_raceline` | TUM input: `# x_m,y_m,w_tr_right_m,w_tr_left_m`, closed loop, driving direction |
| `raceline.csv` | `build_raceline` | TUM output: `# s_m; x_m; y_m; psi_rad; kappa_radpm; vx_mps; ax_mps2`, `map` frame |
| `preview.png` | `build_raceline` | map, centerline, track bounds, raceline coloured by speed |

## Car parameters (AutoDRIVE RoboRacer)

| | |
|---|---|
| wheelbase, max steering | `ws/src/control/config/vehicle.yaml` (0.33 m, 0.524 rad), loaded by every node that uses them |
| wheel radius | 0.0585 m (calibrated against ground truth) |
| throttle | motor torque (sim VehicleController; ~25 m/s per unit throttle is only its steady state), speed loop in `adapter.yaml` |
| acceleration limit | 2.5 m/s^2 up, 6.9 m/s^2 down (`adapter.yaml` `max_acceleration`, `max_deceleration`): the adapter ramps the /drive speed as the car's speed controller would, so a launch from a standstill does not spin the wheels; after a /drive gap the ramp restarts from the measured speed |
