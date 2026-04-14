# ─────────────────────────────────────────────────────────────────────────────
# lateral_dynamics.py
# ─────────────────────────────────────────────────────────────────────────────

import math
from constants import MAX_STEER, LOW_SPEED_EPSILON


def compute_steering_angle(steering_input, max_steer_deg=MAX_STEER):
    try:
        steer = float(steering_input)
    except (TypeError, ValueError):
        steer = 0.0
    steer = max(-1.0, min(1.0, steer))
    return steer * math.radians(max_steer_deg)


def compute_slip_angles(delta, beta, r, v, b, c):
    v_abs = abs(v)
    v_safe = math.sqrt(v_abs * v_abs + LOW_SPEED_EPSILON * LOW_SPEED_EPSILON)
    
    alpha_f = delta - beta - (r * b) / v_safe
    alpha_r = -beta + (r * c) / v_safe

    return alpha_f, alpha_r


def compute_lateral_forces(alpha_f, alpha_r, c_af, c_ar, max_grip_f, max_grip_r):
    # Smoothly saturate tire force with an atan curve.
    # This keeps low-slip slope equal to cornering stiffness while preserving
    # more sensitivity than tanh under large slip, improving counter-steer authority.
    grip_f = max(1e-6, float(max_grip_f))
    grip_r = max(1e-6, float(max_grip_r))

    scale_f = (math.pi * 0.5) * (c_af * alpha_f) / grip_f
    scale_r = (math.pi * 0.5) * (c_ar * alpha_r) / grip_r
    f_yf = grip_f * (2.0 / math.pi) * math.atan(scale_f)
    f_yr = grip_r * (2.0 / math.pi) * math.atan(scale_r)
    
    return f_yf, f_yr


def compute_lateral_derivatives(f_yf, f_yr, r, v, b, c, mass, i_z):
    v_abs = abs(v)
    v_safe = math.sqrt(v_abs * v_abs + LOW_SPEED_EPSILON * LOW_SPEED_EPSILON)
    d_r = (f_yf * b - f_yr * c) / i_z
    d_beta = ((f_yf + f_yr) / (mass * v_safe)) - r
    return d_r, d_beta


def integrate_state(state, derivative, dt):
    return float(state) + float(derivative) * float(dt)


def world_velocity_from_heading(speed, heading, beta):
    v = float(speed)
    travel_angle = float(heading) + float(beta)
    return v * math.cos(travel_angle), v * math.sin(travel_angle)


def integrate_position(x, y, vx, vy, dt):
    step = float(dt)
    return float(x) + float(vx) * step, float(y) + float(vy) * step