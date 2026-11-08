"""MPCC on an existing map/centerline, bridge and localized estimate."""
import os
import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    defaults = dict(track='', centerline_path='', map_yaml='', pose_topic='/localized_pose',
                    odom_topic='/odom', speed_scale='0.5', max_speed='0.0',
                    use_sim_time='true', horizon_nodes='', dt='', deadline='',
                    publish_prediction='false', record_metrics='true',
                    log_root=os.path.expanduser('~/ws/log/mpcc'), localizer='unknown')
    share = FindPackageShare('control')
    parameters = [PathJoinSubstitution([share, 'config', 'vehicle.yaml']),
                  LaunchConfiguration('config_file')]
    types = dict(speed_scale=float, publish_prediction=bool)
    overrides = {name: ParameterValue(LaunchConfiguration(name), value_type=types.get(name, str))
                 for name in ('track', 'centerline_path', 'map_yaml', 'pose_topic', 'odom_topic',
                              'speed_scale', 'publish_prediction')}
    # Race's max_speed=0 means "use configured cap", not an unbounded MPCC.
    def nodes(context):
        cap = float(LaunchConfiguration('max_speed').perform(context))
        if not math.isfinite(cap) or cap < 0:
            raise ValueError('max_speed must be nonnegative')
        node_overrides = dict(overrides)
        for name, value_type in (('horizon_nodes', int), ('dt', float), ('deadline', float)):
            value = LaunchConfiguration(name).perform(context)
            if value:
                node_overrides[name] = value_type(value)
        if cap > 0:
            node_overrides['max_speed'] = cap
        env = dict(ACADOS_SOURCE_DIR=os.environ.get('ACADOS_SOURCE_DIR', '/opt/acados'),
                   LD_LIBRARY_PATH=os.path.join(os.environ.get('ACADOS_SOURCE_DIR', '/opt/acados'), 'lib')+
                                   os.pathsep+os.environ.get('LD_LIBRARY_PATH', ''),
                   OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1')
        return [
            Node(package='control', executable='mpcc_node', output='screen',
                 prefix=[LaunchConfiguration('python_executable')], additional_env=env,
                 parameters=[*parameters, node_overrides]),
            Node(package='control', executable='controller_metrics_node', output='screen',
                 condition=IfCondition(LaunchConfiguration('record_metrics')),
                 prefix=[LaunchConfiguration('python_executable')], additional_env=env,
                 parameters=[*parameters, node_overrides, dict(controller='mpcc',
                     log_root=LaunchConfiguration('log_root'), localizer=LaunchConfiguration('localizer'))]),
        ]
    return LaunchDescription([
        *[DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()],
        DeclareLaunchArgument('python_executable', default_value='/opt/mpcc_venv/bin/python3'),
        DeclareLaunchArgument('config_file', default_value=PathJoinSubstitution([share, 'config', 'mpcc.yaml'])),
        SetParameter(name='use_sim_time', value=ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)),
        OpaqueFunction(function=nodes),
    ])
