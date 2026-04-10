# ─────────────────────────────────────────────────────────────────────────────
# physics.py - Vehicle2D wrapper with split longitudinal/lateral dynamics
# ─────────────────────────────────────────────────────────────────────────────

import math

from constants import (
    L,
    CAR_BODY,
    I_Z,
    C_AF,
    C_AR,
    YAW_DAMPING_MULTIPLIER,
    BETA_DAMPING,
    BETA_HARD_LIMIT,
)
from lateral_dynamics import (
    compute_steering_angle,
    compute_slip_angles,
    compute_lateral_forces,
    compute_lateral_derivatives,
    integrate_state,
    world_velocity_from_heading,
    integrate_position,
)
from longitudinal_dynamics import (
    LONG5_ENGINE_ID,
    LONG5_ENGINE_LABEL,
    create_longitudinal_engine,
    update_longitudinal_speed,
)


class Vehicle2D:
    """2D dynamic bicycle model wrapper with Long.5 longitudinal dynamics."""
    def __init__(self, engine_id=LONG5_ENGINE_ID):
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0        # radians (0 = facing East/Right)
        self.steering_angle = 0.0 # radians (relative to chassis)
        self.yaw_rate = 0.0       # r: radians/second
        self.beta = 0.0           # sideslip angle: radians
        self.chassis_color = tuple(CAR_BODY)
        
        # Base lateral constants
        self.L = L
        self.I_Z = I_Z
        self.C_AF = C_AF
        self.C_AR = C_AR
        self.YAW_DAMPING_MULTIPLIER = float(YAW_DAMPING_MULTIPLIER)
        self.BETA_DAMPING = float(BETA_DAMPING)
        self.BETA_HARD_LIMIT = float(BETA_HARD_LIMIT)

        self.engine_id = LONG5_ENGINE_ID
        self.engine = None
        self.set_engine(engine_id, preserve_speed=False)

        # Telemetry for the renderer
        self.last_alpha_f = 0.0
        self.last_alpha_r = 0.0
        self.last_f_yf = 0.0
        self.last_f_yr = 0.0

    def _sync_vehicle_from_engine(self):
        if hasattr(self.engine, "L"):
            self.L = float(self.engine.L)
        for attr in ["M", "F_ENGINE_MAX", "C_RR", "C_DRAG", "C_BRAKING"]:
            if hasattr(self.engine, attr):
                setattr(self, attr, getattr(self.engine, attr))

    def _sync_engine_from_vehicle(self):
        if hasattr(self.engine, "L"):
            self.engine.L = self.L
        for attr in ["M", "F_ENGINE_MAX", "C_RR", "C_DRAG", "C_BRAKING"]:
            if hasattr(self, attr) and hasattr(self.engine, attr):
                setattr(self.engine, attr, getattr(self, attr))

    def set_engine(self, engine_id, preserve_speed=True):
        if engine_id != LONG5_ENGINE_ID:
            engine_id = LONG5_ENGINE_ID

        prev_v = self.v if (preserve_speed and self.engine is not None) else 0.0
        self.engine_id = LONG5_ENGINE_ID
        self.engine = create_longitudinal_engine(engine_id)
        self.engine.v = max(0.0, prev_v)
        self._sync_vehicle_from_engine()

    def get_hud_data(self):
        hud = {}
        if hasattr(self.engine, "get_hud_data"):
            hud = self.engine.get_hud_data()
        
        # Inject lateral telemetry into HUD data
        hud.update({
            "beta": self.beta,
            "yaw_rate": self.yaw_rate,
            "alpha_f": self.last_alpha_f,
            "alpha_r": self.last_alpha_r,
            "f_yf": self.last_f_yf,
            "f_yr": self.last_f_yr,
        })
        return hud
        
    def reset(self):
        self.x, self.y, self.heading = 0.0, 0.0, 0.0
        self.yaw_rate = 0.0
        self.beta = 0.0
        self.last_alpha_f = 0.0
        self.last_alpha_r = 0.0
        self.last_f_yf = 0.0
        self.last_f_yr = 0.0
        self.engine.reset()

    @property
    def v(self): return self.engine.v

    def update(self, dt, throttle, brake, steering_input, enable_scrub=True):
        
        # --- NEW: CALCULATE MODEL 7.5 SCRUB ---
        f_scrub_mag = 0.0
        if enable_scrub:
            f_lat_total = abs(self.last_f_yf) + abs(self.last_f_yr)
            # F_scrub = F_lat * sin(beta) * multiplier
            scrub_multiplier = getattr(self, 'SCRUB_MULTIPLIER', 2.5)
            f_scrub_mag = f_lat_total * abs(math.sin(self.beta)) * scrub_multiplier

        # 1. Longitudinal channel (speed update) - Pass scrub!
        v = update_longitudinal_speed(self.engine, dt, throttle, brake, f_scrub=f_scrub_mag)

        # 2. Get lever arms from the engine's CG geometry mapping
        b = getattr(self.engine, 'b', self.L * 0.5)
        c = getattr(self.engine, 'c', self.L * 0.5)

        # 3. Lateral/planar channel (Model 7 Dynamics)
        self.steering_angle = compute_steering_angle(steering_input)

        # Use travel direction for steering influence so reverse motion naturally
        # inverts yaw response while wheel telemetry stays true to steering angle.
        gear = int(getattr(self.engine, "gear", 0)) if self.engine is not None else 0
        if abs(v) > 0.2:
            travel_sign = 1.0 if v >= 0.0 else -1.0
        elif gear < 0:
            travel_sign = -1.0
        else:
            travel_sign = 1.0
        dyn_steer = self.steering_angle * travel_sign
        
        # Compute slip angles
        alpha_f, alpha_r = compute_slip_angles(
            dyn_steer, self.beta, self.yaw_rate, v, b, c
        )
        
        # Grab dynamic load from the engine
        Wf = getattr(self.engine, 'Wf', self.M * 9.81 * 0.5)
        Wr = getattr(self.engine, 'Wr', self.M * 9.81 * 0.5)
        mu = getattr(self.engine, 'MU', 1.0)
        
        max_grip_f = mu * Wf
        max_grip_r = mu * Wr

        # Compute lateral forces
        f_yf, f_yr = compute_lateral_forces(
            alpha_f, alpha_r, self.C_AF, self.C_AR, max_grip_f, max_grip_r
        )

        # Fade lateral authority in the parking-lot speed range to avoid
        # unrealistic spin-in-place behavior at near-zero longitudinal speed.
        lat_speed = abs(v)
        lat_gain = min(1.0, lat_speed / 1.5)
        f_yf *= lat_gain
        f_yr *= lat_gain
        
        # Compute ODE derivatives
        d_r, d_beta = compute_lateral_derivatives(
            f_yf, f_yr, self.yaw_rate, v, b, c, self.M, self.I_Z
        )
        d_beta -= self.beta * getattr(self, 'BETA_DAMPING', 0.15)
        
        # Always-on rotational damping keeps yaw dynamics bounded.
        d_r -= self.yaw_rate * getattr(self, 'YAW_DAMPING_MULTIPLIER', 1.75)

        # Optional extra scrub-specific damping to retain scrub behavior.
        if enable_scrub:
            d_r -= self.yaw_rate * 0.5
        
        # 4. Integrate states (Euler)
        self.yaw_rate = integrate_state(self.yaw_rate, d_r, dt)
        self.beta = integrate_state(self.beta, d_beta, dt)
        self.beta = math.atan2(math.sin(self.beta), math.cos(self.beta))
        beta_limit = abs(getattr(self, 'BETA_HARD_LIMIT', 0.8))
        self.beta = max(-beta_limit, min(beta_limit, self.beta))
        self.heading = integrate_state(self.heading, self.yaw_rate, dt)
        
        # 5. Position integration with sideslip
        vx, vy = world_velocity_from_heading(v, self.heading, self.beta)
        self.x, self.y = integrate_position(self.x, self.y, vx, vy, dt)

        # Save telemetry
        self.last_alpha_f = alpha_f
        self.last_alpha_r = alpha_r
        self.last_f_yf = f_yf
        self.last_f_yr = f_yr

        # Keep attributes in sync for options menu overrides
        self._sync_engine_from_vehicle()