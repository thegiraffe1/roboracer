"""Fault categories and measured deadline diagnostics without a live graph."""
import json

import pytest

from control.mpcc_diagnostics import ControlFault, fault_summary, validate_state_values, rollout_violations
from control.mpcc_model import Config


@pytest.mark.parametrize('position,quaternion,twist,reason,field', [
    ([float('nan'), 0.], [0., 0., 0., 1.], [0., 0., 0.], 'nonfinite pose position', 'x'),
    ([0., 0.], [0., float('inf'), 0., 1.], [0., 0., 0.], 'nonfinite pose quaternion', 'qy'),
    ([0., 0.], [0., 0., 0., 1.], [0., 0., float('nan')], 'nonfinite odometry twist', 'yaw_rate'),
])
def test_nonfinite_state_fields_are_distinct(position, quaternion, twist, reason, field):
    with pytest.raises(ControlFault, match=reason) as caught:
        validate_state_values(position, quaternion, twist)
    assert caught.value.details == {'nonfinite_fields': [field]}
    json.dumps(caught.value.details, allow_nan=False)
    assert field in fault_summary(caught.value.details)


def test_quaternion_and_reverse_odometry_are_distinct():
    validate_state_values([1., 2.], [0., 0., 0., 1.], [0., 0., 0.])
    with pytest.raises(ControlFault, match='invalid quaternion norm') as caught:
        validate_state_values([1., 2.], [0., 0., 0., 0.], [0., 0., 0.])
    assert caught.value.details['quaternion_norm'] == 0.
    with pytest.raises(ControlFault, match='negative forward odometry speed') as caught:
        validate_state_values([1., 2.], [0., 0., 0., 1.], [-.02, 0., 0.])
    assert caught.value.details['forward_speed_mps'] == -.02
    assert 'vx=-0.0200' in fault_summary(caught.value.details)


def test_relaxed_envelope_tolerance_does_not_relax_physical_limits():
    metrics = dict.fromkeys((
        'boundary_violation_m', 'map_violation_m', 'steering_angle_violation_rad',
        'acceleration_violation_mps2', 'braking_violation_mps2', 'steering_rate_violation_radps',
        'progress_speed_violation_mps', 'reverse_speed_violation_mps',
        'absolute_speed_violation_mps', 'speed_recovery_violation_mps',
        'lateral_acceleration_violation_mps2', 'lag_violation_m', 'dynamics_defect'), 0.)
    metrics['speed_recovery_violation_mps'] = .0108
    assert rollout_violations(metrics, Config())[0]['constraint'] == 'speed envelope'
    relaxed = Config(deadline=.045, speed_feasibility_tolerance=.02)
    assert rollout_violations(metrics, relaxed) == []
    metrics['speed_recovery_violation_mps'] = .021
    assert rollout_violations(metrics, relaxed)[0]['constraint'] == 'speed envelope'
    metrics['speed_recovery_violation_mps'] = .0108
    for key, name in [('boundary_violation_m', 'track boundary'),
                      ('absolute_speed_violation_mps', 'absolute speed'),
                      ('steering_angle_violation_rad', 'steering angle')]:
        metrics[key] = .006
        assert [v['constraint'] for v in rollout_violations(metrics, relaxed)] == [name]
        metrics[key] = 0.
