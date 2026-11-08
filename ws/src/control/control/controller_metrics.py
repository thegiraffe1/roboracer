"""Read-only, bounded logging and common centerline metrics for MAP and MPCC."""
from dataclasses import asdict
import hashlib
import importlib.metadata
import json
import math
import os
from pathlib import Path
from queue import Queue, Full
import tempfile
import threading
import time
import subprocess

import numpy as np


class LapTimer:
    """Flying laps from measured projection; discard partial first/reset segments."""
    def __init__(self, length):
        self.length = length
        self.reset()

    def reset(self):
        self.previous = self.previous_time = self.start = None
        self.start_progress = None

    def update(self, theta, now, max_speed):
        if self.previous is None:
            self.previous, self.previous_time = theta, now
            return None
        advance = theta-self.previous
        elapsed = now-self.previous_time
        if elapsed <= 0 or elapsed > .5 or advance < -.2 or advance > max_speed*elapsed+.2:
            self.reset()
            self.previous, self.previous_time = theta, now
            return None
        crossing = math.floor(theta/self.length) > math.floor(self.previous/self.length)
        lap = None
        if crossing:
            # Interpolate crossing time between observations rather than using
            # callback receive time as if it were an exact start-line measurement.
            fraction = (math.floor(theta/self.length)*self.length-self.previous)/max(advance, 1e-9)
            crossed_at = self.previous_time+fraction*elapsed
            if self.start is not None and theta-self.start_progress >= .9*self.length:
                duration = crossed_at-self.start
                if duration >= self.length/max_speed*.8:
                    lap = duration
            self.start, self.start_progress = crossed_at, math.floor(theta/self.length)*self.length
        self.previous, self.previous_time = theta, now
        return lap


class RunLog:
    """Disk writes on a daemon thread. Queue loss/errors are explicitly reported."""
    def __init__(self, root, manifest):
        root = Path(root).expanduser()
        root.mkdir(parents=True, exist_ok=True)
        self.directory = Path(tempfile.mkdtemp(prefix=manifest['controller']+'-', dir=root))
        self.queue = Queue(maxsize=2000)
        self.dropped = 0
        self.error = None
        (self.directory/'manifest.json').write_text(json.dumps(manifest, indent=2, allow_nan=False)+'\n')
        self.thread = threading.Thread(target=self._write, daemon=True)
        self.thread.start()

    def _write(self):
        try:
            with (self.directory/'samples.jsonl').open('w') as stream:
                while True:
                    row = self.queue.get()
                    if row is None:
                        break
                    stream.write(json.dumps(row, allow_nan=False)+'\n')
                    stream.flush()
        except Exception as exc:
            self.error = str(exc)

    def record(self, row):
        try:
            self.queue.put_nowait(row)
        except Full:
            self.dropped += 1

    def close(self):
        try:
            self.queue.put(None, timeout=.5)
        except Full:
            self.error = self.error or 'logging queue did not drain'
        self.thread.join(timeout=2.)
        if self.thread.is_alive():
            self.error = self.error or 'logging shutdown timeout'
        (self.directory/'logging_status.json').write_text(json.dumps(dict(
            dropped_rows=self.dropped, error=self.error), indent=2)+'\n')


def manifest(controller, config, paths, metadata=None):
    versions = {}
    for name in ('numpy', 'scipy', 'casadi', 'acados_template'):
        try:
            versions[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            versions[name] = None
    hashes = {str(path): hashlib.sha256(Path(path).read_bytes()).hexdigest()
              for path in paths if path and Path(path).is_file()}
    acados_revision = None
    try:
        acados_revision = subprocess.check_output(['git', '-C', os.environ.get('ACADOS_SOURCE_DIR', '/opt/acados'),
                                                   'rev-parse', 'HEAD'], text=True, stderr=subprocess.DEVNULL,
                                                  timeout=2.).strip()
    except (OSError, subprocess.SubprocessError):
        pass
    return dict(schema=1, controller=controller, config=asdict(config), files_sha256=hashes,
                acados_revision=acados_revision,
                versions=versions, metadata=metadata or {},
                revision=os.environ.get('MPCC_REVISION', 'unknown'),
                clock='ROS stamps and monotonic receive times; not physics time')


def summarize(directory):
    """Offline summary; unavailable solver/lap metrics remain null."""
    metadata = json.loads((Path(directory)/'manifest.json').read_text())
    deadline = metadata.get('config', {}).get('deadline')
    contours, solver, total, laps, boundary = [], [], [], [], []
    faults, deadline_misses, stops, samples = 0, 0, 0, 0
    for line in (Path(directory)/'samples.jsonl').read_text().splitlines():
        row = json.loads(line)
        if row['kind'] == 'state':
            samples += 1
            contours.append(row['contour_m'])
            boundary.append(row['boundary_violation_m'])
            if row.get('lap_time_s') is not None:
                laps.append(row['lap_time_s'])
        elif row['kind'] == 'solver':
            values = row['values']
            if values.get('solver_time_s') is not None:
                solver.append(values['solver_time_s'])
            if values.get('request_time_s') is not None:
                total.append(values['request_time_s'])
            is_stop = values.get('event') == 'stop'
            stops += int(is_stop)
            faults += int(not values.get('ok', False) and not is_stop)
            reason = values.get('reason', '')
            missed = 'deadline' in reason
            # Old logs used one ambiguous reason for validation and computation
            # failure. Use their measured duration, not that wording, to classify.
            if reason == 'nonlinear validation or computation deadline':
                missed = deadline is not None and values.get('total_time_s', 0.) >= deadline
            deadline_misses += int(missed and (not is_stop or reason == 'publication deadline exceeded'))
    def timing(values):
        return dict(p50=float(np.percentile(values, 50)), p95=float(np.percentile(values, 95)),
                    p99=float(np.percentile(values, 99)), maximum=float(max(values))) if values else None
    return dict(state_samples=samples, flying_laps_s=laps,
                contour_rmse_m=float(np.sqrt(np.mean(np.square(contours)))) if contours else None,
                contour_p95_m=float(np.percentile(abs(np.asarray(contours)), 95)) if contours else None,
                maximum_boundary_violation_m=max(boundary) if boundary else None,
                solver_time_s=timing(solver), request_time_s=timing(total), solver_faults=faults,
                deadline_misses=deadline_misses, stop_events=stops,
                logging=json.loads((Path(directory)/'logging_status.json').read_text())
                if (Path(directory)/'logging_status.json').exists() else None)


def summary_main():
    import argparse
    parser = argparse.ArgumentParser(description='Summarize common MAP/MPCC controller logs')
    parser.add_argument('directories', nargs='+')
    args = parser.parse_args()
    print(json.dumps({d: summarize(d) for d in args.directories}, indent=2, allow_nan=False))
