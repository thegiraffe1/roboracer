"""Exercise the sim wrapper with fake ROS commands; never connect to a live graph."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / 'race.sh'


class RaceScriptTests(unittest.TestCase):
    def run_script(self, launch_args=(), overrides=None):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            track = root/'tracks'/'fixture'
            track.mkdir(parents=True)
            (track/'map.pbstream').touch()
            (track/'raceline.csv').write_text('# raceline: {"method": "mintime", "clean": true}\n')
            commands = root/'commands.jsonl'
            binaries = root/'bin'
            binaries.mkdir()
            ros2 = binaries/'ros2'
            ros2.write_text(f'#!{sys.executable}\nimport json,os,sys\n'
                'with open(os.environ["RACE_TEST_COMMANDS"], "a") as stream:\n'
                '    stream.write(json.dumps(sys.argv[1:])+"\\n")\n')
            ros2.chmod(0o755)
            sleep = binaries/'sleep'
            sleep.write_text('#!/bin/sh\nexit 0\n')
            sleep.chmod(0o755)
            env = dict(os.environ, TRACKS_DIR=str(root/'tracks'),
                       RACE_TEST_COMMANDS=str(commands), PATH=str(binaries)+os.pathsep+os.environ['PATH'])
            env.pop('USE_SIM_TIME', None)
            env.pop('DDS_TRANSPORT', None)
            env.update(overrides or {})
            # Source the real common.sh, but omit machine-owned ROS setup files.
            wrapper = '''
source() {
    case "$1" in
        /opt/ros/humble/setup.bash|*/ws/install/setup.bash) return 0 ;;
        *) builtin source "$@" ;;
    esac
}
export -f source
exec bash "$@"
'''
            result = subprocess.run(['bash', '-c', wrapper, 'race_script_test', str(SCRIPT),
                                     'fixture', *launch_args], env=env, text=True,
                                    stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=10)
            self.assertEqual(result.returncode, 0, result.stderr)
            return [json.loads(line) for line in commands.read_text().splitlines()]

    def test_sim_defaults_and_reset_are_forwarded(self):
        calls = self.run_script(['controller:=mpcc'])
        self.assertEqual(len(calls), 2)
        self.assertEqual(calls[0][:3], ['topic', 'pub', '-w'])
        self.assertIn('/autodrive/reset_command', calls[0])
        self.assertEqual(calls[1][:3], ['launch', 'control', 'race.launch.py'])
        self.assertIn('use_sim_time:=false', calls[1])
        self.assertIn('dds_transport:=udp', calls[1])
        self.assertIn('controller:=mpcc', calls[1])

    def test_environment_overrides(self):
        launch = self.run_script(overrides={'USE_SIM_TIME': 'true', 'DDS_TRANSPORT': 'default'})[-1]
        self.assertIn('use_sim_time:=true', launch)
        self.assertIn('dds_transport:=default', launch)

    def test_explicit_launch_overrides_follow_defaults(self):
        launch = self.run_script(['use_sim_time:=true', 'dds_transport:=default'])[-1]
        self.assertGreater(launch.index('use_sim_time:=true'), launch.index('use_sim_time:=false'))
        self.assertGreater(launch.index('dds_transport:=default'), launch.index('dds_transport:=udp'))


if __name__ == '__main__':
    unittest.main()
