"""Lateral/planar kinematic helpers for Model 6 bicycle motion."""

import math

from constants import MAX_STEER


def compute_steering_angle(steering_input, max_steer_deg=MAX_STEER):
    """Map normalized steering input [-1, 1] to steering angle in radians."""
    try:
        steer = float(steering_input)
    except (TypeError, ValueError):
        steer = 0.0
    steer = max(-1.0, min(1.0, steer))
    return steer * math.radians(max_steer_deg)


def compute_yaw_rate(speed, steering_angle, wheelbase):
    """Compute bicycle-model yaw rate from speed and steering angle."""
    wb = max(1e-6, float(wheelbase))
    if abs(steering_angle) <= 0.001:
        return 0.0
    return (float(speed) * math.sin(float(steering_angle))) / wb


def integrate_heading(heading, yaw_rate, dt):
    """Integrate heading angle using current yaw rate."""
    return float(heading) + float(yaw_rate) * float(dt)


def world_velocity_from_heading(speed, heading):
    """Project scalar speed into world-frame velocity components."""
    v = float(speed)
    h = float(heading)
    return v * math.cos(h), v * math.sin(h)


def integrate_position(x, y, vx, vy, dt):
    """Integrate world position from world-frame velocity."""
    step = float(dt)
    return float(x) + float(vx) * step, float(y) + float(vy) * step
