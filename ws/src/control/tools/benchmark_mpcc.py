#!/usr/bin/env python3
"""Synthetic bicycle-in-the-loop timing/feasibility check, without ROS or AutoDRIVE.

Run in the sourced MPCC environment; not evidence of simulator or hardware safety.
"""
import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
import tempfile

import numpy as np
from PIL import Image
import yaml

from control.mpcc_model import Config
from control.mpcc_track import Track, FreeMap
from control.mpcc_solver import AcadosMPCC


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--steps', type=int, default=300)
    parser.add_argument('--horizon', type=int, default=30)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.steps < 1:
        parser.error('steps must be positive')
    c = Config(horizon_nodes=args.horizon)
    angle = np.linspace(0, 2*np.pi, 200, endpoint=False)
    track = Track(np.column_stack([3*np.cos(angle), 3*np.sin(angle),
                                  np.full(200, .8), np.full(200, .6)]), c)
    results = []
    with tempfile.TemporaryDirectory(prefix='mpcc_benchmark_') as directory:
        directory = Path(directory)
        Image.fromarray(np.full((500, 500), 254, dtype=np.uint8)).save(directory/'map.pgm')
        (directory/'map.yaml').write_text(yaml.safe_dump(dict(
            image='map.pgm', resolution=.04, origin=[-10., -10., 0.])))
        solver = AcadosMPCC(c, track, FreeMap(directory/'map.yaml'))
        state = np.array([3., 0., math.pi/2, 0., 0., 0.])
        initial = state.copy()
        try:
            for _ in range(args.steps):
                result = solver.solve(state)
                results.append({k: v for k, v in result.items() if k != 'states'})
                if not result['ok']:
                    break
                state = np.array(result['states'][1])
                # State acquisition uses geometric projection, never optimizer theta.
                state[5] = track.project(state[:2], state[2], state[5])[0]
        finally:
            solver.close()
    def timing(key):
        values = [r[key] for r in results if key in r]
        return dict(p50=float(np.percentile(values, 50)), p95=float(np.percentile(values, 95)),
                    p99=float(np.percentile(values, 99)), maximum=float(max(values)))
    report = dict(config=asdict(c), requested_steps=args.steps, attempted_steps=len(results),
                  successful_steps=sum(r['ok'] for r in results),
                  progress_m=float(state[5]-initial[5]),
                  solver_time_s=timing('solver_time_s'), total_time_s=timing('total_time_s'),
                  samples=results, limitation='Synthetic exact-model plant, no ROS, AutoDRIVE or hardware.')
    # Exclusive creation preserves prior results, including failures.
    with args.output.open('x') as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
    print(json.dumps({k: v for k, v in report.items() if k not in ('samples', 'config')}, indent=2))
    return 0 if report['successful_steps'] == args.steps else 1


if __name__ == '__main__':
    raise SystemExit(main())
