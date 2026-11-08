"""Common metric semantics: partial laps, discontinuities and unavailable data."""
import json

import pytest

from control.controller_metrics import LapTimer, RunLog, summarize


def test_flying_laps_exclude_partial_and_reset():
    timer = LapTimer(10.)
    laps = []
    for i in range(301):
        result = timer.update(2.+i*.1, i*.1, 3.)
        if result is not None:
            laps.append(result)
    assert laps == pytest.approx([10., 10.])
    timer.reset()
    assert timer.update(35., 31., 3.) is None
    assert timer.update(100., 31.1, 3.) is None  # teleport is not a lap
    assert timer.start is None


def test_common_log_and_unavailable_solver_metrics(tmp_path):
    log = RunLog(tmp_path, dict(controller='map'))
    log.record(dict(kind='state', contour_m=.1, boundary_violation_m=0., lap_time_s=None))
    log.record(dict(kind='state', contour_m=-.1, boundary_violation_m=.02, lap_time_s=12.))
    log.close()
    result = summarize(log.directory)
    assert result['contour_rmse_m'] == pytest.approx(.1)
    assert result['flying_laps_s'] == [12.]
    assert result['solver_time_s'] is None
    assert result['logging'] == dict(dropped_rows=0, error=None)


def test_nonlinear_failure_is_not_reported_as_deadline_miss(tmp_path):
    log = RunLog(tmp_path, dict(controller='mpcc', config=dict(deadline=.035)))
    for values in [dict(ok=False, reason='nonlinear validation or computation deadline', total_time_s=.02),
                   dict(ok=False, reason='optimizer deadline exceeded', request_time_s=.036),
                   dict(ok=False, reason='optimizer deadline exceeded', event='stop'),
                   dict(ok=False, reason='publication deadline exceeded', event='stop')]:
        log.record(dict(kind='solver', values=values))
    log.close()
    result = summarize(log.directory)
    assert result['solver_faults'] == 2
    assert result['deadline_misses'] == 2
