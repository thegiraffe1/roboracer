"""Optional wall/CPU timing spans, with no collection when disabled."""
import time


class Timing:
    def __init__(self, enabled=False):
        self.enabled = enabled
        self.phases = {}
        self.counts = {}
        if enabled:
            self.wall = time.perf_counter()
            self.cpu = time.process_time()

    def mark(self, name):
        if self.enabled:
            wall, cpu = time.perf_counter(), time.process_time()
            self.phases[name] = dict(wall_s=wall-self.wall, cpu_s=cpu-self.cpu)
            self.wall, self.cpu = wall, cpu

    def count(self, name, value):
        if self.enabled:
            self.counts[name] = int(value)

    def result(self):
        return dict(phases=self.phases, counts=self.counts)
