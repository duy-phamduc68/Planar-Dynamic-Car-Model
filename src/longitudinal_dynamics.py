"""Longitudinal dynamics model and helpers (Long.5-only runtime)."""

import math


LONG5_ENGINE_ID = 5
LONG5_ENGINE_LABEL = "Long.5"


def _clamp(val, lo, hi):
    return max(lo, min(hi, val))


class Long5Engine:
    """Long.5: decoupled wheel slip and traction saturation drivetrain model."""

    engine_id = LONG5_ENGINE_ID
    label = LONG5_ENGINE_LABEL
    placeholder = False

    def __init__(self):
        self.v = 0.0

        # Vehicle / tire model constants.
        self.M = 1500.0
        self.I_W = 3.0
        self.C_RR = 13.0
        self.C_DRAG = 0.43
        self.AIR_DENSITY = 1.225
        self.FRONTAL_AREA = 2.4
        self.WHEEL_DAMPING = 0.0
        self.C_BRAKE_TORQUE = 4500.0
        self.C_BRAKING = self.C_BRAKE_TORQUE
        self.MU = 1.0
        self.C_T = 30000.0
        self.g = 9.81

        # Geometry. Vehicle2D may override L at runtime.
        self.L = 2.8
        self.h = 0.5
        self._b_ratio = 1.7 / 2.8
        self._c_ratio = 1.1 / 2.8
        self.b = self.L * self._b_ratio
        self.c = self.L * self._c_ratio

        # Drivetrain.
        self.R_W = 0.33
        self.FINAL_DRIVE = 3.42
        self.ETA = 0.7
        self.GEAR_RATIOS = {
            -1: 3.166,
            0: 0.0,
            1: 2.97,
            2: 2.07,
            3: 1.43,
            4: 1.00,
            5: 0.84,
        }
        self.RPM_IDLE = 800.0
        self.RPM_REDLINE = 6000.0
        self.TORQUE_CURVE = [
            (800.0, 200.0),
            (1000.0, 250.0),
            (2000.0, 320.0),
            (3000.0, 380.0),
            (4000.0, 400.0),
            (5000.0, 380.0),
            (6000.0, 300.0),
        ]
        self.upshift_rpm = 5200.0
        self.downshift_rpm = 2200.0
        self.enable_auto_shift = False
        self.min_shift_gap_s = 0.18
        self.rev_limiter_active = False
        self.rev_limiter_release_delta = 180.0

        self.gear = 1
        self.rpm = self.RPM_IDLE
        self.true_rpm = self.RPM_IDLE
        self._sim_clock = 0.0
        self._last_shift_t = -1e9

        # Optional hybrid assist.
        self.HYBRID_TORQUE_MAX = 0.0
        self.HYBRID_ASSIST_FADE_START_RPM = 6000.0
        self.HYBRID_ASSIST_FADE_END_RPM = 8000.0

        # Wheel and slip state.
        self.omega = 0.0
        self.wheel_omega = 0.0
        self.wheel_angle = 0.0
        self.is_slipping = False

        # Dynamic load transfer state.
        self.W = 0.0
        self.dW = 0.0
        self.Wf_static = 0.0
        self.Wr_static = 0.0
        self.Wf = 0.0
        self.Wr = 0.0

        # Longitudinal-only position telemetry channel.
        self.long_x = 0.0

        # Last-step telemetry channels.
        self.last_v = 0.0
        self.last_a = 0.0
        self.last_f_traction = 0.0
        self.last_f_drag = 0.0
        self.last_f_rr = 0.0
        self.last_f_net = 0.0
        self.last_alpha = 0.0
        self.last_slip_ratio = 0.0
        self.last_t_engine = 0.0
        self.last_t_drive = 0.0
        self.last_t_brake = 0.0
        self.last_t_traction = 0.0
        self.last_t_net = 0.0

        # Display stabilizers.
        self._rpm_display = self.RPM_IDLE
        self._wheel_omega_display = 0.0
        self._wheel_alpha_display = 0.0
        self._slip_ratio_display = 0.0

        self._update_force_hint()
        self._update_load_state(0.0)

    def _sync_cg_geometry(self):
        self.L = max(0.5, float(self.L))
        self.b = self.L * self._b_ratio
        self.c = self.L * self._c_ratio

    def _update_force_hint(self):
        peak_torque = max((tq for _rpm, tq in self.TORQUE_CURVE), default=0.0)
        first = abs(float(self.GEAR_RATIOS.get(1, 0.0)))
        self.F_ENGINE_MAX = (peak_torque * first * self.FINAL_DRIVE * self.ETA) / max(self.R_W, 1e-6)

    def _update_load_state(self, a):
        self._sync_cg_geometry()
        self.W = self.M * self.g
        self.dW = (self.h / max(self.L, 1e-6)) * self.M * a
        self.Wf_static = (self.c / max(self.L, 1e-6)) * self.W
        self.Wr_static = (self.b / max(self.L, 1e-6)) * self.W
        self.Wf = max(0.0, self.Wf_static - self.dW)
        self.Wr = max(0.0, self.Wr_static + self.dW)

    def _torque_at_rpm(self, rpm):
        curve = sorted((float(r), float(t)) for r, t in self.TORQUE_CURVE)
        if not curve:
            return 0.0
        if rpm <= curve[0][0]:
            return curve[0][1]
        if rpm >= curve[-1][0]:
            return curve[-1][1]
        for i in range(len(curve) - 1):
            r1, t1 = curve[i]
            r2, t2 = curve[i + 1]
            if r1 <= rpm <= r2:
                k = (rpm - r1) / max(1e-9, (r2 - r1))
                return t1 + k * (t2 - t1)
        return 0.0

    def _apply_auto_shift_hysteresis(self):
        if not self.enable_auto_shift:
            return
        if self.gear < 1:
            return
        if (self._sim_clock - self._last_shift_t) < max(0.0, float(self.min_shift_gap_s)):
            return

        up = float(self.upshift_rpm)
        down = float(self.downshift_rpm)
        if down >= up:
            down = up - 200.0

        max_fwd_gear = max((g for g in self.GEAR_RATIOS.keys() if g > 0), default=5)
        if self.rpm > up and self.gear < max_fwd_gear:
            self.gear += 1
            self._last_shift_t = self._sim_clock
        elif self.rpm < down and self.gear > 1:
            self.gear -= 1
            self._last_shift_t = self._sim_clock

    def _hybrid_assist_torque_at_rpm(self, rpm):
        max_t = max(0.0, float(getattr(self, "HYBRID_TORQUE_MAX", 0.0)))
        if max_t <= 0.0:
            return 0.0

        start = float(getattr(self, "HYBRID_ASSIST_FADE_START_RPM", 6000.0))
        end = float(getattr(self, "HYBRID_ASSIST_FADE_END_RPM", 8000.0))
        if end <= start:
            end = start + 1.0

        if rpm <= start:
            return max_t
        if rpm >= end:
            return 0.0
        return max_t * (end - rpm) / (end - start)

    def _rolling_resistance_force(self, v):
        if abs(v) < 1e-9:
            return 0.0

        c_rr = float(self.C_RR)
        if c_rr < 1.0:
            return c_rr * self.M * self.g * math.copysign(1.0, v)
        return c_rr * v

    def _aero_drag_force(self, v):
        c_drag = float(self.C_DRAG)
        if c_drag < 1.0:
            rho = max(0.0, float(getattr(self, "AIR_DENSITY", 1.225)))
            area = max(0.0, float(getattr(self, "FRONTAL_AREA", 2.4)))
            return 0.5 * rho * c_drag * area * v * abs(v)
        return c_drag * v * abs(v)

    def _update_rev_limiter(self):
        release_rpm = self.RPM_REDLINE - max(10.0, float(self.rev_limiter_release_delta))
        if self.rev_limiter_active:
            if self.rpm <= release_rpm:
                self.rev_limiter_active = False
            return
        if self.rpm >= self.RPM_REDLINE:
            self.rev_limiter_active = True

    @staticmethod
    def _smooth(prev, target, alpha):
        alpha = _clamp(float(alpha), 0.0, 1.0)
        return prev + alpha * (target - prev)

    def _gear_text(self):
        if self.gear < 0:
            return "R"
        if self.gear == 0:
            return "N"
        return str(self.gear)

    def reset(self):
        self.v = 0.0
        self.omega = 0.0
        self.wheel_omega = 0.0
        self.wheel_angle = 0.0
        self.long_x = 0.0
        self._sim_clock = 0.0
        self._last_shift_t = -1e9

        self.gear = 1
        self.rpm = self.RPM_IDLE
        self.true_rpm = self.RPM_IDLE
        self.is_slipping = False

        self.last_v = 0.0
        self.last_a = 0.0
        self.last_f_traction = 0.0
        self.last_f_drag = 0.0
        self.last_f_rr = 0.0
        self.last_f_net = 0.0
        self.last_alpha = 0.0
        self.last_slip_ratio = 0.0
        self.last_t_engine = 0.0
        self.last_t_drive = 0.0
        self.last_t_brake = 0.0
        self.last_t_traction = 0.0
        self.last_t_net = 0.0
        self.rev_limiter_active = False
        self._rpm_display = self.RPM_IDLE
        self._wheel_omega_display = 0.0
        self._wheel_alpha_display = 0.0
        self._slip_ratio_display = 0.0

        self._update_force_hint()
        self._update_load_state(0.0)

    def update(self, dt, throttle, brake, f_scrub=0.0):
        dt = max(1e-6, float(dt))
        throttle = _clamp(float(throttle), 0.0, 1.0)
        brake = _clamp(float(brake), 0.0, 1.0)
        self._sim_clock += dt

        self.C_BRAKE_TORQUE = max(0.0, float(getattr(self, "C_BRAKING", self.C_BRAKE_TORQUE)))
        self.C_BRAKING = self.C_BRAKE_TORQUE

        self._sync_cg_geometry()
        self._update_force_hint()
        self._apply_auto_shift_hysteresis()

        v_prev = self.v
        omega_prev = self.omega
        gear_ratio = float(self.GEAR_RATIOS.get(self.gear, 0.0))

        true_rpm_raw = omega_prev * gear_ratio * self.FINAL_DRIVE * 60.0 / (2.0 * math.pi)
        self.true_rpm = true_rpm_raw

        clutch_bite = 1.0
        if self.gear == 0:
            rpm = self.RPM_IDLE
            clutch_bite = 0.0
        else:
            rpm = max(abs(true_rpm_raw), self.RPM_IDLE)
            if abs(true_rpm_raw) < self.RPM_IDLE:
                clutch_bite = max(0.55, abs(true_rpm_raw) / max(self.RPM_IDLE, 1e-6))
        self.rpm = rpm
        self._update_rev_limiter()

        if self.rev_limiter_active:
            t_engine = 0.0
        else:
            t_engine_base = self._torque_at_rpm(self.rpm)
            t_engine_hybrid = self._hybrid_assist_torque_at_rpm(self.rpm)
            t_engine = throttle * (t_engine_base + t_engine_hybrid)
            t_engine *= clutch_bite

        t_drive = t_engine * gear_ratio * self.FINAL_DRIVE * self.ETA
        if self.gear < 0:
            t_drive = -t_drive
        if self.gear > 0 and v_prev < -0.1:
            t_drive = 0.0
        if self.gear < 0 and v_prev > 0.1:
            t_drive = 0.0

        i_engine = 0.25
        i_eff = self.I_W + i_engine * (gear_ratio * self.FINAL_DRIVE) ** 2

        v_abs = max(abs(v_prev), 1.0)
        slip_ratio = (omega_prev * self.R_W - v_prev) / v_abs

        f_trac_raw = self.C_T * slip_ratio
        f_max = self.MU * self.Wr
        f_traction = _clamp(f_trac_raw, -f_max, f_max)
        t_traction = f_traction * self.R_W
        self.is_slipping = abs(f_trac_raw) > f_max

        f_rr = self._rolling_resistance_force(v_prev)
        f_drag = self._aero_drag_force(v_prev)
        t_drag_wheel = float(getattr(self, "WHEEL_DAMPING", 0.0)) * omega_prev

        # --- NEW: APPLY MODEL 7.5 SCRUB FORCE ---
        f_scrub_directed = 0.0
        if abs(v_prev) > 1e-3:
            # Direct the magnitude against the current direction of travel
            f_scrub_directed = abs(f_scrub) * math.copysign(1.0, v_prev)
        # ----------------------------------------

        t_ext = t_drive - t_traction - t_drag_wheel
        t_brake = 0.0
        if abs(omega_prev) > 0.1:
            max_brake_t = (i_eff * abs(omega_prev)) / dt
            actual_b = min(self.C_BRAKE_TORQUE * brake, max_brake_t)
            t_brake = actual_b * math.copysign(1.0, omega_prev)
        else:
            max_hold = self.C_BRAKE_TORQUE * brake
            if abs(t_ext) <= max_hold:
                t_brake = t_ext
            else:
                t_brake = math.copysign(max_hold, t_ext)

        # Apply scrub explicitly to the net longitudinal force
        f_net = f_traction - f_drag - f_rr - f_scrub_directed
        t_net = t_drive - t_brake - t_traction - t_drag_wheel
        a = f_net / max(self.M, 1e-6)
        alpha = t_net / max(i_eff, 1e-6)

        self.v = v_prev + dt * a
        self.omega = omega_prev + dt * alpha

        is_braking_to_stop = (brake > 0.1 and abs(self.v) < 0.5)
        is_coasting_to_stop = (throttle < 0.01 and brake <= 0.1 and abs(self.v) < 0.1 and abs(self.omega) < 0.5)
        if is_braking_to_stop or is_coasting_to_stop:
            self.v = 0.0
            self.omega = 0.0
            a = 0.0
            alpha = 0.0
            f_traction = 0.0
            t_traction = 0.0
            slip_ratio = 0.0
            t_net = 0.0
            f_net = 0.0
            self.is_slipping = False

        self.long_x += dt * self.v
        self.wheel_omega = self.omega
        self.wheel_angle = (self.wheel_angle + self.wheel_omega * dt) % (2.0 * math.pi)
        self._update_load_state(a)

        self.last_v = self.v
        self.last_a = a
        self.last_f_traction = f_traction
        self.last_f_drag = f_drag
        self.last_f_rr = f_rr
        self.last_f_net = f_net
        self.last_alpha = alpha
        self.last_slip_ratio = slip_ratio
        self.last_t_engine = t_engine
        self.last_t_drive = t_drive
        self.last_t_brake = t_brake
        self.last_t_traction = t_traction
        self.last_t_net = t_net

        self._rpm_display = self._smooth(self._rpm_display, self.rpm, 0.25)
        self._wheel_omega_display = self._smooth(self._wheel_omega_display, self.wheel_omega, 0.30)
        self._wheel_alpha_display = self._smooth(self._wheel_alpha_display, self.last_alpha, 0.30)
        self._slip_ratio_display = self._smooth(self._slip_ratio_display, self.last_slip_ratio, 0.30)
        return self.v

    def get_hud_data(self):
        slip_pct = 100.0 * self._slip_ratio_display
        f_max = self.MU * max(abs(self.Wr), 1e-6)
        traction_pct = 100.0 * abs(self.last_f_traction) / max(f_max, 1e-6)
        slip_txt = f"{slip_pct:+.1f}%"
        traction_txt = f"{traction_pct:.1f}%"

        telemetry = {
            "v": self.last_v,
            "a": self.last_a,
            "x": self.long_x,
            "f_traction": self.last_f_traction,
            "f_drag": self.last_f_drag,
            "f_rr": self.last_f_rr,
            "f_net": self.last_f_net,
            "wheel_omega": self.wheel_omega,
            "wheel_alpha": self.last_alpha,
            "slip_ratio": self.last_slip_ratio,
            "t_drive": self.last_t_drive,
            "t_brake": self.last_t_brake,
            "t_traction": self.last_t_traction,
            "t_net": self.last_t_net,
        }

        return {
            "mode_label": self.label,
            "gear": self._gear_text(),
            "rpm": f"{self._rpm_display:.0f}",
            "rpm_idle": self.RPM_IDLE,
            "rpm_redline": self.RPM_REDLINE,
            "shift": "AUTO" if self.enable_auto_shift else "MANUAL",
            "slip": slip_txt,
            "traction": traction_txt,
            "placeholder": self.placeholder,
            "ax": f"{self.last_a:.2f}",
            "ay": "0.00",
            "slip_fl": "0.0%",
            "slip_fr": "0.0%",
            "slip_rl": slip_txt,
            "slip_rr": slip_txt,
            "engine_torque": f"{self.last_t_engine:.1f}",
            "wheel_torque": f"{self.last_t_drive:.1f}",
            "fz_front": f"{self.Wf:.0f}",
            "fz_rear": f"{self.Wr:.0f}",
            "wheel_omega": self._wheel_omega_display,
            "wheel_radius_m": self.R_W,
            "wheel_angle": self.wheel_angle,
            "is_slipping": self.is_slipping,
            "telemetry_mode": "long5",
            "telemetry": telemetry,
            "f_traction": f"{self.last_f_traction:.1f}",
            "f_drag": f"{self.last_f_drag:.1f}",
            "f_rr": f"{self.last_f_rr:.1f}",
            "f_net": f"{self.last_f_net:.1f}",
            "wheel_alpha": f"{self._wheel_alpha_display:.2f}",
            "slip_ratio": f"{self._slip_ratio_display:+.4f}",
            "t_drive": f"{self.last_t_drive:.1f}",
            "t_brake": f"{self.last_t_brake:.1f}",
            "t_traction": f"{self.last_t_traction:.1f}",
            "t_net": f"{self.last_t_net:.1f}",
            "x_long": f"{self.long_x:.2f}",
        }


def create_longitudinal_engine(engine_id=None):
    """Factory kept for compatibility with previous engine-selection wiring."""
    _ = engine_id
    return Long5Engine()


def update_longitudinal_speed(engine, dt, throttle, brake, f_scrub=0.0):
    """Advance the active longitudinal engine and return vehicle speed."""
    if engine is None:
        return 0.0
    return float(engine.update(dt, throttle, brake, f_scrub=f_scrub))