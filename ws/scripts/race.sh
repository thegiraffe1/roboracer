#!/usr/bin/env bash
# Race a mapped track: localization (Cartographer, or the MCL particle filter) + pure pursuit on its minimum-time raceline.
#
#   ws/scripts/race.sh <name> [speed_scale:=0.5] [max_speed:=0.0] [pose_topic:=/localized_pose] [avoid:=false]
#                          [localizer:=mcl]    particle filter on map.yaml instead of Cartographer localization
#                          [controller:=map2] experimental MAP2 MPC (default: pp)
#
# Needs the bridge running and tracks/<name>/ from ws/scripts/map_track.sh <name>.
# Resets the sim car first: localization starts from the pose mapping started from. The car waits
# until the localizer has localized (~2-3 s) and after every sim reset or crash. Ctrl-C stops.
set -eo pipefail

name=${1:?usage: race.sh <track name> [launch args]}
shift
source "$(dirname "$0")/common.sh"

for f in map.pbstream raceline.csv; do
    [ -e "$tracks/$name/$f" ] || { echo "$tracks/$name/$f missing; run map-track $name  (scripts/map_track.sh)" >&2; exit 1; }
done

reset_sim_car

exec ros2 launch control race.launch.py track:="$name" "$@"
