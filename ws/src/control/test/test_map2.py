"""MAP2 geometry and numerical adapter tests; no acados build required."""
import numpy as np
import pytest

from control.map2 import Reference, sim_steering
from control.map2_vendor.mpc_map2_core import build_stage_parameters, paper_dynamics


def circle(clockwise=False):
    t = np.linspace(0, 2 * np.pi, 120, endpoint=False)
    if clockwise:
        t = -t
    rows = np.zeros((len(t), 7))
    rows[:, 1:3] = np.column_stack([2 * np.cos(t), 2 * np.sin(t)])
    rows[:, 5] = 4.
    return rows


@pytest.mark.parametrize('clockwise,sign', [(False, 1), (True, -1)])
def test_signed_curvature_and_equilibrium(clockwise, sign):
    ref = Reference(circle(clockwise))
    assert ref.curvature == pytest.approx(np.full(120, sign * 0.5), rel=0.001)
    params, speeds, yaw_rates = build_stage_parameters(ref.profile(ref.length - 0.01), 0., 20, 0.05, 0.1)
    assert speeds == pytest.approx(np.full(20, 2.))  # accelerates from standstill
    assert params[:, 0] == pytest.approx(np.full(20, sign * 0.5), rel=0.001)
    dx = paper_dynamics([0., 0., 0.], [speeds[0], 0., yaw_rates[0]], params[0, 0])
    assert dx == pytest.approx([2., 0., 0.])


def test_projection_sign_seam_and_heading():
    ref = Reference(circle())
    state, s = ref.project(1.9, 0., ref.heading[0], 0.5)
    assert state[1] > 0  # inside CCW circle is left
    ref.previous_s = ref.length - 0.01
    _, s = ref.project(*ref.xy[1], ref.heading[1], 0.5)
    assert s < 0.2
    with pytest.raises(ValueError):
        ref.project(20., 20., 0., 0.5)
    with pytest.raises(ValueError):
        ref.project(2., 0., -np.pi / 2, 0.5)


def test_profile_interpolates_current_and_wraps():
    rows = circle()
    rows[:, 5] = np.arange(120) + 1
    ref = Reference(rows, speed_scale=1., max_speed=200.)
    p = ref.profile(ref.length - ref.lengths[-1] / 2)
    assert p.sample(0.)[0] == pytest.approx(60.5)
    assert p.sample(ref.lengths[-1] / 2)[0] == pytest.approx(1.)


@pytest.mark.parametrize('rows', [np.zeros((2, 7)), np.full((3, 7), np.nan), np.zeros((4, 3))])
def test_reject_bad_reference(rows):
    with pytest.raises(ValueError):
        Reference(rows)


def test_steering_sign_limits_and_zero_rate():
    assert sim_steering([2., 0., 1.], 0.33, 0.524) == pytest.approx(np.arctan(0.33 / 2))
    assert sim_steering([2., 0., -1.], 0.33, 0.524) < 0
    assert sim_steering([0., 0., 0.], 0.33, 0.524) == 0
    assert sim_steering([0.1, 0., 10.], 0.33, 0.524) == 0.524
    with pytest.raises(ValueError):
        sim_steering([np.nan, 0., 0.], 0.33, 0.524)
