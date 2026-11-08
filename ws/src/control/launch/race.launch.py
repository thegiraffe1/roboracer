"""Race a mapped track: localization + selected raceline controller.

    ros2 launch control race.launch.py track:=<name> [speed_scale:=0.5] [max_speed:=0.0] [pose_topic:=/localized_pose]
        [avoid:=false] [localizer:=cartographer|mcl] [controller:=pp|map2]

Needs tracks/<name>/map.pbstream (map.yaml for localizer:=mcl) and raceline.csv (ws/scripts/map_track.sh
<name>), and the car at the pose mapping started from (ws/scripts/race.sh resets the sim car there).
/localized_pose is published once the localizer has localized (Cartographer: localization_gate; mcl: the
particle cloud has converged), so the car waits for that. localizer:=mcl is the particle filter
(perception mcl.launch.py); the default is Cartographer pure localization.
avoid:=true also starts planning's obstacle_spliner_node on the track's raceline and map, and pure
pursuit follows its /local_path around obstacles the map did not have (stopping when there is none).
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from perception.tracks import track_dir


def controller(context):
    name = LaunchConfiguration('controller').perform(context)
    avoid = LaunchConfiguration('avoid').perform(context).lower() in ('true', '1')
    if name == 'map2' and avoid:
        raise ValueError('MAP2 does not support avoid:=true')
    args = {key: LaunchConfiguration(key) for key in ('track', 'pose_topic', 'speed_scale')}
    cap = LaunchConfiguration('max_speed').perform(context)
    if name == 'map2':
        # MAP2 requires a finite model speed bound. Preserve its 3 m/s default
        # when the shared race argument uses PP's zero/no-cap sentinel.
        args['max_speed'] = '3.0' if float(cap) == 0. else cap
    else:
        args.update(max_speed=cap, avoid=LaunchConfiguration('avoid'))
    filename = 'map2.launch.py' if name == 'map2' else 'pure_pursuit.launch.py'
    return [IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('control'), 'launch', filename]),
        launch_arguments=args.items())]


def obstacle_spliner(context):
    if LaunchConfiguration('avoid').perform(context).lower() not in ('true', '1'):
        return []
    d = track_dir(LaunchConfiguration('track').perform(context))
    return [Node(package='planning', executable='obstacle_spliner_node', output='screen', parameters=[{
        'raceline_path': os.path.join(d, 'raceline.csv'),
        'map_yaml': os.path.join(d, 'map.yaml'),
        'pose_topic': LaunchConfiguration('pose_topic'),
    }])]


def generate_launch_description():
    def include(package, launch_file, args, condition=None):
        return IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare(package), 'launch', launch_file]),
            launch_arguments={a: LaunchConfiguration(a) for a in args}.items(), condition=condition)

    use_mcl = PythonExpression(["'", LaunchConfiguration('localizer'), "' == 'mcl'"])

    return LaunchDescription([
        DeclareLaunchArgument('track', description='track name under ws/tracks/'),
        DeclareLaunchArgument('controller', default_value='pp', choices=['pp', 'map2'],
                              description='raceline controller'),
        DeclareLaunchArgument('pose_topic', default_value='/localized_pose',
                              description="a localizer's PoseStamped in the map frame"),
        DeclareLaunchArgument('speed_scale', default_value='0.5', description='drive speed = raceline vx * speed_scale'),
        DeclareLaunchArgument('max_speed', default_value='0.0', description='cap on the drive speed [m/s], 0 = no cap'),
        DeclareLaunchArgument('avoid', default_value='false',
                              description='avoid obstacles the map did not have (obstacle_spliner_node)'),
        DeclareLaunchArgument('localizer', default_value='cartographer',
                              description='cartographer: pure localization on map.pbstream; mcl: particle filter on map.yaml'),
        OpaqueFunction(function=controller),
        include('perception', 'localization.launch.py', ['track'], UnlessCondition(use_mcl)),
        include('perception', 'mcl.launch.py', ['track'], IfCondition(use_mcl)),
        OpaqueFunction(function=obstacle_spliner),
    ])
