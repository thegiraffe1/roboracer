"""Real process/IPC regressions for deadline and epoch safety."""
import time
import math
import os

import numpy as np
import pytest

from control.mpcc_model import Config
from control.mpcc_worker import Worker


def fake_optimizer(requests, results, config, centerline, map_yaml):
    results.put(dict(kind='ready'))
    while True:
        request = requests.get()
        if request is None:
            return
        if centerline == 'late':
            time.sleep(.08)
        elif centerline == 'hung':
            time.sleep(10)
        results.put(dict(kind='result', sequence=request['sequence'], epoch=request['epoch'],
                         result=dict(ok=True, speed=.1, steering=.01)))


def wait_event(worker, kind, timeout=4):
    end = time.monotonic()+timeout
    while time.monotonic() < end:
        event = worker.poll(time.monotonic())
        if event and event[0] == kind:
            return event[1]
        time.sleep(.001)
    raise AssertionError('no event '+kind)


def test_deadline_and_invalidated_results_never_accepted():
    worker = Worker(Config(deadline=.02), 'late', '', target=fake_optimizer)
    try:
        wait_event(worker, 'ready')
        assert worker.submit([0.]*6, time.monotonic())
        assert not worker.submit([0.]*6, time.monotonic())  # no request backlog
        fault = wait_event(worker, 'fault')
        assert 'deadline' in fault['reason']
        wait_event(worker, 'discarded')  # successful but late result cannot drive
        assert worker.pending is None
    finally:
        worker.close()
    worker = Worker(Config(), 'normal', '', target=fake_optimizer)
    try:
        wait_event(worker, 'ready')
        worker.submit([0.]*6, time.monotonic())
        worker.invalidate()  # reset during solve
        wait_event(worker, 'discarded')
    finally:
        worker.close()


def test_hung_worker_shutdown_is_bounded():
    worker = Worker(Config(), 'hung', '', target=fake_optimizer)
    try:
        wait_event(worker, 'ready')
        worker.submit([0.]*6, time.monotonic()-.6)
        assert worker.hung(time.monotonic())
    finally:
        start = time.monotonic()
        worker.close()
        assert time.monotonic()-start < 1.
        assert not worker.process.is_alive()


@pytest.mark.skipif(not os.environ.get('ACADOS_SOURCE_DIR'), reason='requires pinned acados environment')
def test_real_acados_worker_startup_and_ipc(tmp_path):
    from test_mpcc import circle, free_map
    track = circle()
    path = tmp_path/'centerline.csv'
    np.savetxt(path, np.column_stack([track.xy, track.width]), delimiter=',')
    free_map(tmp_path)
    worker = Worker(Config(), path, tmp_path/'map.yaml')
    try:
        wait_event(worker, 'ready', timeout=60)
        state = [3., 0., math.pi/2, 0., 0., 0.]
        # Scheduling can exceed the soft deadline on shared CI hosts. Every late
        # attempt must fault/discard; require a timely successful solve to pass.
        accepted = None
        for _ in range(4):
            assert worker.submit(state, time.monotonic())
            end = time.monotonic()+1
            while time.monotonic() < end:
                event = worker.poll(time.monotonic())
                if event and event[0] == 'result':
                    if event[1]['ok']:
                        accepted = event[1]
                    break
                if event and event[0] == 'discarded':
                    break
                time.sleep(.001)
            if accepted is not None:
                break
        assert accepted is not None
        assert accepted['speed'] > 0
        assert accepted['request_time_s'] < worker.config.deadline
    finally:
        worker.close()
