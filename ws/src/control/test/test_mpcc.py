"""Behavioral MPCC model/geometry/solver regressions; solver tests need acados."""
import math
import os
from unittest.mock import Mock

import numpy as np
import pytest
from PIL import Image
import yaml

from control.mpcc_model import Config, KinematicBicycle
from control.mpcc_track import Track, FreeMap
from control.mpcc_solver import AcadosMPCC, check_rollout
from control.mpcc_timing import Timing


def circle(config=None, radius=3., right=.8, left=.6):
    config = config or Config()
    angle = np.linspace(0., 2*np.pi, 200, endpoint=False)
    return Track(np.column_stack([radius*np.cos(angle), radius*np.sin(angle),
                                  np.full(len(angle), right), np.full(len(angle), left)]), config)


def free_map(directory):
    path = directory / 'map.yaml'
    Image.fromarray(np.full((500, 500), 254, dtype=np.uint8)).save(directory/'map.pgm')
    path.write_text(yaml.safe_dump(dict(image='map.pgm', resolution=.04, origin=[-10., -10., 0.])))
    return FreeMap(path)


def test_limits_and_bicycle():
    for kw in [dict(deadline=.1), dict(horizon_nodes=3.5), dict(speed_scale=float('nan')),
               dict(acceleration=3.), dict(max_steering_angle=2.)]:
        with pytest.raises(ValueError):
            Config(**kw)
    model = KinematicBicycle(Config())
    state = np.array([0., 0., 0., 2., .2, 0.])
    moved = model.step(state, np.array([0., 0., 2.]))
    assert moved[0] > 0 and moved[1] > 0 and moved[2] > 0
    assert moved[5] == pytest.approx(.1)
    assert model.step(np.zeros(6), np.zeros(3)) == pytest.approx(np.zeros(6))
    capped = Config(max_speed=1., nominal_speed=3., speed_scale=.5)
    assert capped.speed_cap == 1.  # absolute cap is not multiplied by speed_scale
    assert np.max(circle(capped).speed) <= 1.


def test_qp_failure_discards_failed_iterate_and_warm_start():
    c = Config()
    solver = AcadosMPCC.__new__(AcadosMPCC)
    solver.config, solver.model, solver.track = c, KinematicBicycle(c), circle(c)
    solver.last_x, solver.last_u = np.zeros((c.horizon_nodes+1, 6)), np.zeros((c.horizon_nodes, 3))
    solver.solver = Mock()
    solver.solver.solve.return_value = 4
    solver.solver.get_stats.side_effect = lambda field: .001 if field == 'time_tot' else np.array([2])
    result = solver._solve(np.array([3., 0., math.pi/2, 0., 0., 0.]), Timing())
    assert not result['ok'] and result['reason'] == 'acados status 4'
    assert 'speed' not in result and 'first_control' not in result
    assert solver.last_x is None and solver.last_u is None
    solver.solver.reset.assert_called_once()
    solver.solver.get.assert_not_called()  # never forward the failed iterate


def test_initial_overspeed_allowance_expires_and_does_not_excuse_acceleration(tmp_path):
    c = Config(speed_scale=.3)
    model = KinematicBicycle(c)
    elapsed = np.array([0., .025, .05, .1])
    assert model.speed_recovery_slack(1.04, .9, elapsed) == pytest.approx([.14, .07, 0., 0.])
    assert model.speed_recovery_slack(.8, .9, elapsed) == pytest.approx(np.zeros(4))
    track = circle(c)
    grid = free_map(tmp_path)
    steer = math.atan(c.wheelbase/3.)
    initial = np.array([3., 0., math.pi/2, 1.04, steer, 0.])
    target = float(track.speed_limit(initial[5]))
    braking = np.array([[(target-initial[3])/c.dt, 0., .9], [0., 0., .9]])
    safe, states, metrics = check_rollout(model, track, grid, initial, braking)
    assert safe, metrics
    assert states[1, 3] == pytest.approx(target)
    assert metrics['speed_violation_mps'] == pytest.approx(initial[3]-target)
    assert metrics['speed_recovery_violation_mps'] <= c.feasibility_tolerance
    assert metrics['initial_speed_excess_mps'] == pytest.approx(initial[3]-target)
    assert metrics['violated_constraints'] == []
    uncorrected = np.array([[0., 0., .9], [0., 0., .9]])
    safe, _, metrics = check_rollout(model, track, grid, initial, uncorrected)
    assert not safe and metrics['speed_recovery_violation_mps'] > .1
    assert [v['constraint'] for v in metrics['violated_constraints']] == ['speed envelope']
    initial[3] = .8
    safe, _, metrics = check_rollout(model, track, grid, initial,
                                    np.array([[2., 0., .9], [2., 0., .9]]))
    assert not safe and metrics['speed_recovery_violation_mps'] > .09


def test_projection_seam_and_asymmetric_widths():
    track = circle()
    before, _, _ = track.project([3., -.03], -0.01+math.pi/2, track.length-.05)
    after, error, _ = track.project([3., .03], math.pi/2, before)
    assert after > track.length and after > before
    assert abs(error) < .01
    _, _, _, curvature, right, left = track.sample(after)
    assert curvature == pytest.approx(1/3, rel=.01)
    assert right == .8 and left == .6
    assert track.speed_limit(after) <= 1.5


def test_map_unknown_outside_and_footprint_rejection(tmp_path):
    c = Config()
    track = circle(c)
    grid = free_map(tmp_path)
    assert grid.clearance(np.array([[0., 0.], [30., 30.]]))[1] < 0
    pixels = np.full((500, 500), 254, dtype=np.uint8)
    pixels[250, 250] = 205
    Image.fromarray(pixels).save(tmp_path/'map.pgm')
    unknown = FreeMap(tmp_path/'map.yaml')
    assert unknown.clearance(np.array([[.02, -.02]]))[0] < 0
    model = KinematicBicycle(c)
    state = np.array([2.45, 0., math.pi/2, 0., 0., 0.])
    safe, _, metrics = check_rollout(model, track, grid, state, np.zeros((2, 3)))
    assert not safe and metrics['boundary_violation_m'] > 0
    assert 'track boundary' in [v['constraint'] for v in metrics['violated_constraints']]
    # Corners can be free while an obstacle intersects the middle of the body.
    pixels = np.full((500, 500), 254, dtype=np.uint8)
    pixels[249, 325] = 0
    Image.fromarray(pixels).save(tmp_path/'map.pgm')
    blocked = FreeMap(tmp_path/'map.yaml')
    state[0] = 3.
    safe, _, metrics = check_rollout(model, track, blocked, state, np.zeros((2, 3)))
    assert not safe and metrics['map_violation_m'] > 0


def test_symbolic_model_and_generated_jacobians_match_numerical():
    ca = pytest.importorskip('casadi')
    model = KinematicBicycle(Config())
    x, u = ca.SX.sym('x', 6), ca.SX.sym('u', 3)
    expression = model.symbolic(x, u, ca)
    function = ca.Function('bicycle_test', [x, u], [expression, ca.jacobian(expression, x),
                                                   ca.jacobian(expression, u)])
    for speed, steering in [(0., 0.), (1.5, .3), (1., -.4)]:
        state = np.array([1., -2., .7, speed, steering, 12.])
        controls = np.array([.3, -.1, .5])
        value, dx, du = function(state, controls)
        assert np.array(value).ravel() == pytest.approx(model.derivative(state, controls))
        eps = 1e-6
        numeric_x = np.column_stack([(model.derivative(state+eps*e, controls)-
                                      model.derivative(state-eps*e, controls))/(2*eps) for e in np.eye(6)])
        numeric_u = np.column_stack([(model.derivative(state, controls+eps*e)-
                                      model.derivative(state, controls-eps*e))/(2*eps) for e in np.eye(3)])
        assert np.array(dx) == pytest.approx(numeric_x, abs=1e-7)
        assert np.array(du) == pytest.approx(numeric_u, abs=1e-7)


@pytest.mark.skipif(not os.environ.get('ACADOS_SOURCE_DIR'), reason='requires pinned acados environment')
def test_acados_progress_warm_start_and_infeasibility(tmp_path):
    c = Config(deadline=.049)
    track = circle(c)
    solver = AcadosMPCC(c, track, free_map(tmp_path), profile=True)
    try:
        state = np.array([3., 0., math.pi/2, 0., 0., 0.])
        result = solver.solve(state)
        assert result['ok'], {k: result.get(k) for k in (
            'reason', 'status', 'total_time_s', 'assembly_time_s', 'solver_time_s', 'violated_constraints')}
        profile = result['profile']
        assert profile['counts']['footprint_samples'] > profile['counts']['dense_states']
        assert profile['phases']['validation_map']['wall_s'] >= 0
        assert sum(p['wall_s'] for p in profile['phases'].values()) <= result['total_time_s']+.001
        assert all(p['cpu_s'] >= 0 for p in profile['phases'].values())
        assert 0 < result['speed'] <= c.acceleration*c.dt+.001
        assert result['boundary_violation_m'] <= c.feasibility_tolerance
        assert solver.last_u is not None
        states = np.array(result['states'])
        second = solver.solve(states[1])
        assert second['ok'], second
        assert second['states'][-1][5] > states[1, 5]
        # A measured target overshoot must produce a feasible braking command,
        # while keeping the target, absolute speed and boundary limits intact.
        solver.reset()
        overspeed = state.copy()
        overspeed[3] = c.speed_cap+.1
        overspeed[4] = math.atan(c.wheelbase/3.)
        recovery = solver.solve(overspeed)
        assert recovery['ok'], recovery
        assert recovery['first_control'][0] < 0
        assert recovery['speed'] <= c.speed_cap
        assert recovery['states'][1][3] <= c.speed_cap+c.feasibility_tolerance
        assert recovery['initial_speed_excess_mps'] == pytest.approx(
            overspeed[3]-track.speed_limit(overspeed[5]))
        assert recovery['speed_recovery_violation_mps'] <= c.feasibility_tolerance
        solver.reset()
        assert solver.last_u is None
        bad = state.copy()
        bad[0] = 2.45
        result = solver.solve(bad)
        assert not result['ok']
    finally:
        solver.close()
