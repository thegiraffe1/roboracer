#!/usr/bin/env python3
"""Measure the real ROS supervisor with its drive publisher replaced by a sink.

Requires existing bridge/localization. Does not reset, command, or move the car.
"""
import argparse
import cProfile
import io
import json
from pathlib import Path
import time
import pstats

import rclpy
from rclpy.signals import SignalHandlerOptions
from ament_index_python.packages import get_package_share_directory
from control.mpcc_node import MPCC
from profile_mpcc import stats


class CommandSink:
    def __init__(self):
        self.count = self.positive = 0

    def publish(self, msg):
        self.count += 1
        self.positive += int(msg.drive.speed > 0)


class ShadowMPCC(MPCC):
    def __init__(self):
        self.sink = CommandSink()
        self.tick_rows, self.state_rows, self.result_rows = [], [], []
        self.stop_rows, self.publish_rows, self.report_rows = [], [], []
        self.previous_tick = None
        super().__init__()

    def create_publisher(self, msg_type, topic, qos, *args, **kwargs):
        if topic == '/drive':
            return self.sink  # Never construct an actual ROS drive publisher.
        return super().create_publisher(msg_type, topic, qos, *args, **kwargs)

    def state(self, now, ros_now):
        wall, cpu = time.perf_counter(), time.process_time()
        accepted = False
        try:
            result = super().state(now, ros_now)
            accepted = True
            return result
        finally:
            self.state_rows.append(dict(wall_s=time.perf_counter()-wall,
                                        cpu_s=time.process_time()-cpu, accepted=accepted))

    def tick(self):
        wall, cpu = time.perf_counter(), time.process_time()
        interval = wall-self.previous_tick if self.previous_tick is not None else None
        self.previous_tick = wall
        try:
            super().tick()
        finally:
            self.tick_rows.append(dict(wall_s=time.perf_counter()-wall,
                                       cpu_s=time.process_time()-cpu, interval_s=interval))

    def report(self, result):
        wall = time.perf_counter()
        try:
            super().report(result)
            self.result_rows.append({k: v for k, v in result.items() if k != 'states'})
        finally:
            self.report_rows.append(time.perf_counter()-wall)

    def stop(self, *args, **kwargs):
        wall = time.perf_counter()
        try:
            return super().stop(*args, **kwargs)
        finally:
            self.stop_rows.append(time.perf_counter()-wall)

    def publish(self, *args, **kwargs):
        wall = time.perf_counter()
        try:
            return super().publish(*args, **kwargs)
        finally:
            self.publish_rows.append(time.perf_counter()-wall)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--track', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=20.)
    args = parser.parse_args()
    if not 1 <= args.seconds <= 120:
        parser.error('seconds must be in [1, 120]')
    args.output.mkdir(parents=True, exist_ok=False)
    share = Path(get_package_share_directory('control'))
    rclpy.init(args=['--ros-args', '--params-file', str(share/'config/vehicle.yaml'),
                    '--params-file', str(share/'config/mpcc.yaml'), '-p', 'use_sim_time:=false',
                    '-p', 'track:='+args.track, '-p', 'speed_scale:=0.3'],
               signal_handler_options=SignalHandlerOptions.NO)
    guard = rclpy.create_node('mpcc_shadow_profile_guard')
    for _ in range(20):
        rclpy.spin_once(guard, timeout_sec=.05)
    if guard.count_publishers('/drive'):
        raise RuntimeError('existing controller: profiling not started')
    node = None
    try:
        node = ShadowMPCC()
        end = time.monotonic()+120
        while time.monotonic() < end and not (node.worker.ready and node.pose is not None):
            rclpy.spin_once(node, timeout_sec=.01)
            if node.worker.error:
                raise RuntimeError(node.worker.error)
        if not node.worker.ready or node.pose is None:
            raise RuntimeError('worker/localization initialization timeout')
        node.tick_rows.clear()
        node.state_rows.clear()
        node.result_rows.clear()
        node.stop_rows.clear()
        node.publish_rows.clear()
        node.report_rows.clear()
        end = time.monotonic()+args.seconds
        check_at = 0.
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.01)
            if time.monotonic() >= check_at:
                if node.count_publishers('/drive'):
                    raise RuntimeError('unexpected actual drive publisher')
                check_at = time.monotonic()+1
        rows = dict(ticks=node.tick_rows, state_checks=node.state_rows, results=node.result_rows)
        (args.output/'samples.json').write_text(json.dumps(rows, indent=2)+'\n')
        summary = dict(duration_s=args.seconds, drive_publishers=node.count_publishers('/drive'),
                       suppressed_commands=node.sink.count, suppressed_positive_commands=node.sink.positive,
                       stop_wall_s=stats(node.stop_rows), sink_publish_wall_s=stats(node.publish_rows),
                       report_wall_s=stats(node.report_rows),
                       tick_wall_s=stats([r['wall_s'] for r in node.tick_rows]),
                       tick_cpu_s=stats([r['cpu_s'] for r in node.tick_rows]),
                       tick_interval_s=stats([r['interval_s'] for r in node.tick_rows if r['interval_s'] is not None]))
        for accepted, name in ((True, 'valid'), (False, 'rejected')):
            selected = [r for r in node.state_rows if r['accepted'] == accepted]
            summary['state_'+name] = {key: stats([r[key] for r in selected]) for key in ('wall_s', 'cpu_s')}
        (args.output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
        print(json.dumps(summary, indent=2), flush=True)
        # Separate call-attribution window; do not mix cProfile overhead into the
        # phase/timer distributions above.
        profiler = cProfile.Profile()
        end = time.monotonic()+5.
        profiler.enable()
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.01)
        profiler.disable()
        profiler.dump_stats(str(args.output/'calls.prof'))
        stream = io.StringIO()
        pstats.Stats(profiler, stream=stream).strip_dirs().sort_stats('cumulative').print_stats(45)
        (args.output/'calls.txt').write_text(stream.getvalue())
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        guard.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
