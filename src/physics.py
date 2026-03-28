# ─────────────────────────────────────────────────────────────────────────────
# physics.py — Vehicle2D wrapper with split longitudinal/lateral dynamics
# ─────────────────────────────────────────────────────────────────────────────

from constants import L, CAR_BODY
from lateral_dynamics import (
    compute_steering_angle,
    compute_yaw_rate,
    integrate_heading,
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
    """2D kinematic bicycle model wrapper with Long.5 longitudinal dynamics."""
    def __init__(self, engine_id=LONG5_ENGINE_ID):
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0        # radians (0 = facing East/Right)
        self.steering_angle = 0.0 # radians (relative to chassis)
        self.yaw_rate = 0.0
        self.chassis_color = tuple(CAR_BODY)
        
        self.L = L

        self.engine_id = LONG5_ENGINE_ID
        self.engine = None
        self.set_engine(engine_id, preserve_speed=False)

    def _sync_vehicle_from_engine(self):
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
        if hasattr(self.engine, "get_hud_data"):
            return self.engine.get_hud_data()
        return {
            "mode_label": LONG5_ENGINE_LABEL,
            "gear": "N/A",
            "rpm": "N/A",
            "shift": "AUTO",
            "slip": "N/A",
            "traction": "N/A",
            "placeholder": True,
        }
        
    def reset(self):
        self.x, self.y, self.heading = 0.0, 0.0, 0.0
        self.engine.reset()

    @property
    def v(self): return self.engine.v

    def update(self, dt, throttle, brake, steering_input):
        # Longitudinal channel (speed update).
        v = update_longitudinal_speed(self.engine, dt, throttle, brake)

        # Lateral/planar channel (bicycle yaw + pose integration).
        self.steering_angle = compute_steering_angle(steering_input)
        self.yaw_rate = compute_yaw_rate(v, self.steering_angle, self.L)
        self.heading = integrate_heading(self.heading, self.yaw_rate, dt)
        vx, vy = world_velocity_from_heading(v, self.heading)
        self.x, self.y = integrate_position(self.x, self.y, vx, vy, dt)

        # Keep attributes in sync for options menu overrides
        self._sync_engine_from_vehicle()