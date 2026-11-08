"""Bounded, spawned optimizer process; no ROS imports or command publishing."""
from dataclasses import asdict
import multiprocessing as mp
import signal
from queue import Empty, Full
import time

from .mpcc_model import Config


def optimizer_main(requests, results, config, centerline, map_yaml):
    # The supervisor owns orderly shutdown. ros2 launch may send SIGINT both to
    # the process group and again to each child; avoid interrupting native cleanup.
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    from .mpcc_track import Track, FreeMap
    from .mpcc_solver import AcadosMPCC
    solver = None
    try:
        c = Config(**config)
        solver = AcadosMPCC(c, Track.load(centerline, c), FreeMap(map_yaml))
        results.put(dict(kind='ready'))
        epoch = None
        while True:
            request = requests.get()
            if request is None:
                break
            if request['epoch'] != epoch:
                solver.reset()
                epoch = request['epoch']
            try:
                result = solver.solve(request['state'])
            except Exception as exc:
                solver.reset()
                result = dict(ok=False, reason=f'{type(exc).__name__}: {exc}')
            results.put(dict(kind='result', sequence=request['sequence'], epoch=epoch,
                             submitted=request['submitted'], result=result))
    except Exception as exc:
        results.put(dict(kind='error', reason=f'{type(exc).__name__}: {exc}'))
    finally:
        if solver is not None:
            solver.close()


class Worker:
    """At most one solve in flight. Old epochs and late results never become commands.

    Polling/submit are nonblocking. Slow process reap is separated from the safety
    timer; a terminated child is reaped on later polls. Startup has no driving output.
    """
    def __init__(self, config, centerline, map_yaml, target=optimizer_main):
        self.config = config
        self.context = mp.get_context('spawn')
        self.requests = self.context.Queue(maxsize=1)
        self.results = self.context.Queue(maxsize=1)
        self.process = self.context.Process(target=target, args=(self.requests, self.results,
            asdict(config), str(centerline), str(map_yaml)), daemon=True)
        self.ready = False
        self.error = ''
        self.epoch = self.sequence = 0
        self.pending = None
        self.started = time.monotonic()
        self.process.start()

    def invalidate(self):
        self.epoch += 1

    def submit(self, state, now):
        if not self.ready or self.pending is not None or self.error:
            return False
        self.sequence += 1
        request = dict(sequence=self.sequence, epoch=self.epoch,
                       state=list(state), submitted=now)
        try:
            self.requests.put_nowait(request)
        except Full:
            return False
        self.pending = request
        return True

    def poll(self, now):
        """Return ('ready'|'result'|'fault'|'discarded', payload) or None."""
        try:
            message = self.results.get_nowait()
        except Empty:
            message = None
        if message is not None:
            if message['kind'] == 'ready':
                self.ready = True
                return 'ready', {}
            if message['kind'] == 'error':
                self.error = message['reason']
                return 'fault', dict(reason=self.error)
            pending = self.pending
            self.pending = None
            if (pending is None or message['sequence'] != pending['sequence'] or
                    message['epoch'] != self.epoch):
                return 'discarded', dict(reason='invalidated optimizer result')
            elapsed = now-pending['submitted']
            result = message['result']
            result['request_time_s'] = elapsed
            if elapsed >= self.config.deadline:
                self.invalidate()
                return 'fault', dict(result, ok=False, reason='optimizer deadline exceeded')
            return 'result', result
        if not self.error and not self.process.is_alive():
            self.error = 'optimizer process exited'
            return 'fault', dict(reason=self.error)
        if self.pending and now-self.pending['submitted'] >= self.config.deadline:
            # Mark as expired once; leave it in flight until the child replies or
            # the supervisor kills a hung process. Subsequent result is discarded.
            if self.pending['epoch'] == self.epoch:
                self.invalidate()
                return 'fault', dict(reason='optimizer deadline exceeded', ok=False,
                                      request_time_s=now-self.pending['submitted'])
        if not self.ready and now-self.started > 120 and not self.error:
            self.error = 'optimizer initialization timeout'
            return 'fault', dict(reason=self.error)
        return None

    def hung(self, now):
        return self.pending is not None and now-self.pending['submitted'] > max(.5, 5*self.config.dt)

    def close(self):
        """Bounded shutdown, called outside the driving callback."""
        try:
            self.requests.put_nowait(None)
        except Full:
            pass
        self.process.join(timeout=.1)
        if self.process.is_alive():
            self.process.terminate()
            self.process.join(timeout=.2)
        if self.process.is_alive():
            self.process.kill()
            self.process.join(timeout=.2)
        for queue in (self.requests, self.results):
            queue.cancel_join_thread()
            queue.close()
