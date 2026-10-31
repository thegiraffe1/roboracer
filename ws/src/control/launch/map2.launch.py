"""MAP2 controller only; run AutoDRIVE bridge and localization separately."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    defaults = {'track': '', 'raceline_path': '', 'pose_topic': '/localized_pose',
                'odom_topic': '/odom', 'speed_scale': '0.5', 'max_speed': '3.0',
                'use_sim_time': 'false'}
    types = {'speed_scale': float, 'max_speed': float, 'use_sim_time': bool}
    share = FindPackageShare('control')
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()],
        Node(package='control', executable='map2_node', output='screen', parameters=[
            PathJoinSubstitution([share, 'config', 'vehicle.yaml']),
            PathJoinSubstitution([share, 'config', 'map2.yaml']),
            {name: ParameterValue(LaunchConfiguration(name), value_type=types.get(name, str))
             for name in defaults},
        ]),
    ])
