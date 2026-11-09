#!/usr/bin/env python3
"""Profile recorded MPCC observations without creating a /drive publisher.

Direct mode attributes solver wall/CPU time. Worker mode measures the existing
spawned process/queues with a 5 ms observer poll and optional read-only ROS traffic.
Late candidates are retained for analysis; none can actuate a vehicle.
"""
import argparse
from collections import Counter
import cProfile
from dataclasses import asdict
import hashlib
import io
import json
from pathlib import Path
import pstats
from queue import Empty
import signal
import time

import numpy as np

from control.mpcc_model import Config
from control.mpcc_solver import AcadosMPCC
from control.mpcc_track import Track, FreeMap
from control.mpcc_worker import Worker


def profiled_optimizer(requests, results, config, centerline, map_yaml):
    """Same one-job process layout, with boundary timestamps and phase profiling."""
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    c = Config(**config)
    solver = AcadosMPCC(c, Track.load(centerline, c), FreeMap(map_yaml), profile=True)
    results.put(dict(kind='ready'))
    epoch = None
    try:
        while True:
            request = requests.get()
            received = time.monotonic()
            if request is None:
                break
            if epoch != request['epoch']:
                solver.reset()
                epoch = request['epoch']
            started = time.monotonic()
            try:
                result = solver.solve(request['state'])
            except Exception as exc:
                solver.reset()
                result = dict(ok=False, reason=f'{type(exc).__name__}: {exc}')
            finished = time.monotonic()
            results.put(dict(kind='result', sequence=request['sequence'], result=result,
                             received=received, started=started, finished=finished))
    finally:
        solver.close()


def stats(values):
    values = np.asarray(values, dtype=float)
    return dict(count=len(values), mean=float(np.mean(values)), p50=float(np.percentile(values, 50)),
                p95=float(np.percentile(values, 95)), p99=float(np.percentile(values, 99)),
                maximum=float(np.max(values))) if len(values) else None


def aggregate(rows):
    phases = {}
    names = {key for row in rows for key in row.get('profile', {}).get('phases', {})}
    for name in sorted(names):
        values = [r['profile']['phases'][name] for r in rows
                  if name in r.get('profile', {}).get('phases', {})]
        phases[name] = {key: stats([v[key] for v in values]) for key in ('wall_s', 'cpu_s')}
    phases['total'] = {'wall_s': stats([r['total_time_s'] for r in rows if 'total_time_s' in r])}
    for name in ('request_time_s', 'inbound_s', 'worker_reset_s', 'outbound_and_poll_s', 'parent_cpu_s'):
        if any(name in r for r in rows):
            phases[name] = stats([r[name] for r in rows if name in r])
    seed_groups = {}
    for cold, name in ((0, 'warm'), (1, 'cold')):
        selected = [r['profile']['phases']['seed'] for r in rows if
                    r.get('profile', {}).get('counts', {}).get('cold_start') == cold and
                    'seed' in r.get('profile', {}).get('phases', {})]
        seed_groups[name] = {key: stats([v[key] for v in selected]) for key in ('wall_s', 'cpu_s')}
    return dict(samples=len(rows), accepted_candidates=sum(r.get('ok', False) for r in rows),
                reasons=dict(Counter(r.get('reason', '') for r in rows)), phases=phases,
                seed_by_start=seed_groups)


def load_trial(trial, count, selection):
    evaluator = list((trial/'controller_metrics').glob('mpcc-*'))
    if len(evaluator) != 1:
        raise ValueError('trial must contain exactly one MPCC evaluator log')
    meta = json.loads((evaluator[0]/'manifest.json').read_text())
    c = Config(**meta['config'])
    centerline = next(p for p in meta['files_sha256'] if Path(p).name == 'centerline.csv')
    map_yaml = next(p for p in meta['files_sha256'] if Path(p).name == 'map.yaml')
    observations = []
    reset_before = True
    for line in (evaluator[0]/'samples.jsonl').read_text().splitlines():
        row = json.loads(line)
        if row['kind'] == 'solver' and 'initial_state' in row['values']:
            observations.append(dict(row['values'], reset_before=reset_before))
            reset_before = not row['values'].get('ok', False)
        elif row['kind'] == 'solver' and row['values'].get('event') == 'stop':
            reset_before = True
    if not observations:
        raise ValueError('no recorded solver initial states')
    indexes = (np.arange(min(count, len(observations))) if selection == 'contiguous' else
               np.linspace(0, len(observations)-1, min(count, len(observations)), dtype=int))
    return c, centerline, map_yaml, [observations[i] for i in indexes], meta


def direct(c, track, grid, observations, output, seed_mode):
    solver = AcadosMPCC(c, track, grid, profile=True)
    rows = []
    try:
        for i, observation in enumerate(observations):
            # Sparse replay snapshots are independent cold starts, avoiding an
            # invalid warm start across a deliberately skipped section of track.
            if seed_mode == 'cold' or observation['reset_before']:
                solver.reset()
            result = solver.solve(observation['initial_state'])
            result.pop('states', None)
            rows.append(dict(result, input_index=i, original_reason=observation.get('reason', '')))
        profiler = cProfile.Profile()
        # Call-attribution run is separate: cProfile overhead must not be mistaken
        # for the uninstrumented/phase-timed request distribution.
        profiler.enable()
        for observation in observations[:min(40, len(observations))]:
            if seed_mode == 'cold' or observation['reset_before']:
                solver.reset()
            solver.solve(observation['initial_state'])
        profiler.disable()
        profiler.dump_stats(str(output/'calls.prof'))
        stream = io.StringIO()
        pstats.Stats(profiler, stream=stream).strip_dirs().sort_stats('cumulative').print_stats(35)
        (output/'calls.txt').write_text(stream.getvalue())
    finally:
        solver.close()
    return rows


def worker_run(c, centerline, map_yaml, observations, ros_listen, seed_mode):
    worker = Worker(c, centerline, map_yaml, target=profiled_optimizer)
    node = ros = None
    callback_counts = Counter()
    if ros_listen:
        import rclpy as ros
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import Odometry
        from rclpy.qos import qos_profile_sensor_data
        ros.init()
        node = ros.create_node('mpcc_timing_observer')
        for topic, kind in (('/localized_pose', PoseStamped), ('/odom', Odometry)):
            node.create_subscription(kind, topic,
                lambda msg, key=topic: callback_counts.update([key]), qos_profile_sensor_data)
    rows = []
    try:
        end = time.monotonic()+120
        while not worker.ready and time.monotonic() < end:
            event = worker.poll(time.monotonic())
            if event and event[0] == 'fault':
                raise RuntimeError(event[1])
            time.sleep(.002)
        if not worker.ready:
            raise RuntimeError('worker initialization timeout')
        next_request = time.monotonic()
        for i, observation in enumerate(observations):
            while time.monotonic() < next_request:
                if node is not None:
                    ros.spin_once(node, timeout_sec=0.)
                time.sleep(min(.001, max(0., next_request-time.monotonic())))
            if seed_mode == 'cold' or observation['reset_before']:
                worker.invalidate()
            submitted, cpu = time.monotonic(), time.process_time()
            assert worker.submit(observation['initial_state'], submitted)
            next_poll = submitted+.005
            message = None
            while time.monotonic()-submitted < 2.:
                if node is not None:
                    ros.spin_once(node, timeout_sec=0.)
                now = time.monotonic()
                if now >= next_poll:
                    next_poll = now+.005
                    try:
                        message = worker.results.get_nowait()
                        break
                    except Empty:
                        pass
                time.sleep(.0005)
            observed, cpu_elapsed = time.monotonic(), time.process_time()-cpu
            if message is None or message.get('kind') != 'result':
                raise RuntimeError('profile worker did not return a result')
            worker.pending = None
            result = message['result']
            result.pop('states', None)
            result.update(input_index=i, request_time_s=observed-submitted,
                          inbound_s=message['received']-submitted,
                          worker_reset_s=message['started']-message['received'],
                          outbound_and_poll_s=observed-message['finished'], parent_cpu_s=cpu_elapsed)
            result['request_deadline_missed'] = result['request_time_s'] >= c.deadline
            if result['request_deadline_missed']:
                worker.invalidate()
            rows.append(result)
            next_request = max(submitted+c.dt, observed)
    finally:
        worker.close()
        if node is not None:
            node.destroy_node()
            ros.try_shutdown()
    return rows, dict(callback_counts)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trial', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--samples', type=int, default=200)
    parser.add_argument('--mode', choices=['direct', 'worker', 'both'], default='both')
    parser.add_argument('--ros-listen', action='store_true')
    parser.add_argument('--selection', choices=['uniform', 'contiguous'], default='uniform')
    parser.add_argument('--seed-mode', choices=['cold', 'recorded'], default='cold')
    args = parser.parse_args()
    if args.samples < 1:
        parser.error('samples must be positive')
    if args.seed_mode == 'recorded' and args.selection != 'contiguous':
        parser.error('recorded warm starts require contiguous observations')
    c, centerline, map_yaml, observations, original = load_trial(args.trial, args.samples, args.selection)
    args.output.mkdir(parents=True, exist_ok=False)
    manifest = dict(config=asdict(c), source_trial=str(args.trial), original_manifest=original,
                    profile_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                    mode=args.mode, ros_listen=args.ros_listen, selection=args.selection, seed_mode=args.seed_mode,
                    limitation='No actuation; observer does not execute supervisor state validation/publication.')
    (args.output/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    summary = {}
    if args.mode in ('direct', 'both'):
        rows = direct(c, Track.load(centerline, c), FreeMap(map_yaml), observations, args.output, args.seed_mode)
        (args.output/'direct.json').write_text(json.dumps(rows, indent=2)+'\n')
        summary['direct'] = aggregate(rows)
    if args.mode in ('worker', 'both'):
        rows, callbacks = worker_run(c, centerline, map_yaml, observations, args.ros_listen, args.seed_mode)
        (args.output/'worker.json').write_text(json.dumps(rows, indent=2)+'\n')
        summary['worker'] = aggregate(rows)
        summary['worker']['request_deadline_misses'] = sum(r['request_deadline_missed'] for r in rows)
        summary['worker']['ros_callbacks'] = callbacks
    (args.output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    for mode, result in summary.items():
        print(mode, 'total', result['phases']['total'], flush=True)
    print('Output:', args.output, flush=True)


if __name__ == '__main__':
    main()
