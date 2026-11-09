"""Specific MPCC fault reasons and numeric details, independent of ROS."""
import math

import numpy as np


class ControlFault(ValueError):
    def __init__(self, reason, **details):
        super().__init__(reason)
        self.details = details


def validate_state_values(position, quaternion, twist):
    """Same acceptance rules as the supervisor, with separate fault categories."""
    for values, names, reason in (
            (position, ('x', 'y'), 'nonfinite pose position'),
            (quaternion, ('qx', 'qy', 'qz', 'qw'), 'nonfinite pose quaternion'),
            (twist, ('forward_speed', 'lateral_speed', 'yaw_rate'), 'nonfinite odometry twist')):
        bad = [name for name, value in zip(names, values) if not math.isfinite(value)]
        if bad:
            raise ControlFault(reason, nonfinite_fields=bad)
    norm = float(np.linalg.norm(quaternion))
    if not math.isfinite(norm):
        raise ControlFault('nonfinite quaternion norm', nonfinite_fields=['quaternion_norm'])
    if abs(norm-1.) > .05:
        raise ControlFault('invalid quaternion norm', quaternion_norm=norm, norm_tolerance=.05)
    if twist[0] < 0:
        raise ControlFault('negative forward odometry speed', forward_speed_mps=float(twist[0]))


def rollout_violations(metrics, config):
    """List failed constraints; values are excess over the physical bound."""
    specs = (
        ('boundary_violation_m', 'track boundary', 'm'),
        ('map_violation_m', 'map clearance', 'm'),
        ('steering_angle_violation_rad', 'steering angle', 'rad'),
        ('acceleration_violation_mps2', 'acceleration', 'm/s^2'),
        ('braking_violation_mps2', 'braking', 'm/s^2'),
        ('steering_rate_violation_radps', 'steering rate', 'rad/s'),
        ('progress_speed_violation_mps', 'virtual progress speed', 'm/s'),
        ('reverse_speed_violation_mps', 'reverse speed', 'm/s'),
        ('absolute_speed_violation_mps', 'absolute speed', 'm/s'),
        ('speed_recovery_violation_mps', 'speed envelope', 'm/s'),
        ('lateral_acceleration_violation_mps2', 'lateral acceleration', 'm/s^2'),
        ('lag_violation_m', 'lag error', 'm'),
        ('dynamics_defect', 'dynamics consistency', 'state units'),
    )
    failed = []
    for key, name, unit in specs:
        value = metrics[key]
        if key == 'dynamics_defect':
            tolerance = config.dynamics_tolerance
        elif key == 'speed_recovery_violation_mps':
            tolerance = config.speed_feasibility_tolerance
        else:
            tolerance = config.feasibility_tolerance
        if not math.isfinite(value) or value > tolerance:
            failed.append(dict(constraint=name, metric=key,
                               excess=value if math.isfinite(value) else str(value),
                               tolerance=tolerance, unit=unit))
    return failed


def fault_summary(details):
    """Short console details; complete values remain in structured diagnostics."""
    if not details:
        return ''
    if 'publication_time_s' in details:
        return (f" ({details['publication_time_s']*1000:.2f} ms total, "
                f"{details['deadline_overrun_s']*1000:.2f} ms late; "
                f"budget {details['deadline_s']*1000:.2f} ms)")
    if details.get('violated_constraints'):
        return ' ['+'; '.join(
            f"{v['constraint']}: excess {v['excess']:.6g} {v['unit']} > {v['tolerance']:.6g}"
            if isinstance(v['excess'], (int, float)) else f"{v['constraint']}: {v['excess']}"
            for v in details['violated_constraints'])+']'
    if 'forward_speed_mps' in details:
        return f" (vx={details['forward_speed_mps']:.4f} m/s)"
    if 'quaternion_norm' in details:
        return f" (norm={details['quaternion_norm']:.6g}, tolerance={details['norm_tolerance']})"
    if 'nonfinite_fields' in details:
        return ' ('+', '.join(details['nonfinite_fields'])+')'
    if 'pose_odom_skew_s' in details:
        return (f" (pose-odom={details['pose_odom_skew_s']*1000:.2f} ms, "
                f"limit {details['max_input_skew_s']*1000:.2f} ms)")
    if 'recovery_samples' in details:
        return (f" ({details['recovery_samples']}/{details['recovery_required_samples']} fresh pairs, "
                f"{details['recovery_elapsed_s']:.3f}/{details['recovery_required_s']:.3f} s)")
    return ''
