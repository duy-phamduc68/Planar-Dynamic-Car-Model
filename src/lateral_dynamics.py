# ─────────────────────────────────────────────────────────────────────────────
# lateral_dynamics.py
# ─────────────────────────────────────────────────────────────────────────────

import math
from constants import MAX_STEER


def compute_steering_angle(steering_input, max_steer_deg=MAX_STEER):
    try:
        steer = float(steering_input)
    except (TypeError, ValueError):
        steer = 0.0
    steer = max(-1.0, min(1.0, steer))
    return steer * math.radians(max_steer_deg)


def compute_slip_angles(delta, beta, r, v, b, c):
    # THE PURGE: Back to 0.1 epsilon
    v_safe = max(abs(v), 0.1) 
    
    alpha_f = delta - beta - (r * b) / v_safe
    alpha_r = -beta + (r * c) / v_safe
    
    limit = 0.17
    alpha_f = max(-limit, min(limit, alpha_f))
    alpha_r = max(-limit, min(limit, alpha_r))
    
    return alpha_f, alpha_r


def compute_lateral_forces(alpha_f, alpha_r, c_af, c_ar, max_grip_f, max_grip_r):
    f_yf = c_af * alpha_f
    f_yr = c_ar * alpha_r
    
    f_yf = max(-max_grip_f, min(max_grip_f, f_yf))
    f_yr = max(-max_grip_r, min(max_grip_r, f_yr))
    
    return f_yf, f_yr


def compute_lateral_derivatives(f_yf, f_yr, r, v, b, c, mass, i_z):
    v_safe = max(abs(v), 0.1)
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