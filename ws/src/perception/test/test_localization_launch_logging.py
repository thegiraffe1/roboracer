"""Cartographer verbosity must cover both glog and ROS, with a racing override."""
from pathlib import Path
import runpy

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription


LAUNCH = Path(__file__).resolve().parents[1]/'launch'/'localization.launch.py'
RACE = Path(__file__).resolve().parents[2]/'control'/'launch'/'race.launch.py'


@pytest.mark.parametrize('level,minimum', [('info', 0), ('warn', 1)])
def test_cartographer_log_level_controls_glog_and_ros(tmp_path, monkeypatch, level, minimum):
    track = tmp_path/'fixture'
    track.mkdir()
    (track/'map.pbstream').touch()
    monkeypatch.setenv('TRACKS_DIR', str(tmp_path))
    module = runpy.run_path(str(LAUNCH))
    monkeypatch.setitem(module['nodes'].__globals__, 'Node', lambda **kwargs: kwargs)
    context = LaunchContext()
    context.launch_configurations.update(track='fixture', cartographer_log_level=level)
    cartographer, _, gate = module['nodes'](context)
    assert f'-minloglevel={minimum}' in cartographer['arguments']
    assert cartographer['ros_arguments'] == ['--log-level', level]
    assert '-start_trajectory_with_default_topics=false' in cartographer['arguments']
    assert cartographer['remappings'] == [('imu', '/sensors/imu/raw')]
    assert 'ros_arguments' not in gate  # gate warnings/diagnostics retain their own level


def test_racing_defaults_to_warn_and_forwards_explicit_info():
    description = runpy.run_path(str(RACE))['generate_launch_description']()
    declaration = next(a for a in description.entities if isinstance(a, DeclareLaunchArgument)
                       and a.name == 'cartographer_log_level')
    include = next(a for a in description.entities if isinstance(a, IncludeLaunchDescription)
                   and 'cartographer_log_level' in dict(a.launch_arguments))
    context = LaunchContext()
    declaration.execute(context)
    value = dict(include.launch_arguments)['cartographer_log_level']
    assert value.perform(context) == 'warn'
    context.launch_configurations['cartographer_log_level'] = 'info'
    declaration.execute(context)
    assert value.perform(context) == 'info'


def test_standalone_localization_keeps_info_default():
    description = runpy.run_path(str(LAUNCH))['generate_launch_description']()
    declaration = next(a for a in description.entities if isinstance(a, DeclareLaunchArgument)
                       and a.name == 'cartographer_log_level')
    context = LaunchContext()
    declaration.execute(context)
    assert context.launch_configurations['cartographer_log_level'] == 'info'
