"""
Car Physics Simulator - Model 6 (Top-Down Kinematics)
=======================================================
Runs Model 6 (Bicycle Kinematics) with fixed Long.5 longitudinal dynamics.
"""

import os
import json
import math
import pygame
from constants import (
    THROTTLE_RAMP_DEFAULT,
    BRAKE_RAMP_DEFAULT,
    STEER_RAMP_DEFAULT,
    GRID_SIZE,
    YAW_DAMPING_MULTIPLIER,
    BETA_DAMPING,
    BETA_HARD_LIMIT,
    SKID_MARK_VISIBILITY_SCALE,
    SKID_MARK_LONG_SLIP_THRESHOLD,
    SKID_MARK_LAT_SLIP_THRESHOLD,
    SKID_MARK_LONG_GAIN,
    SKID_MARK_LAT_GAIN,
    TIMESTEP_OPTIONS,
    FPS_OPTIONS,
    CAR_BODY,
    PIXELS_PER_METER,
)
from physics import Vehicle2D
from controls import (
    load_xinput,
    get_xinput_state,
    XINPUT_BUTTON_START,
    XINPUT_BUTTON_BACK,
    XINPUT_BUTTON_DPAD_UP,
    XINPUT_BUTTON_DPAD_DOWN,
    XINPUT_BUTTON_DPAD_LEFT,
    XINPUT_BUTTON_DPAD_RIGHT,
    XINPUT_BUTTON_A,
    XINPUT_BUTTON_B,
    XINPUT_BUTTON_X,
    XINPUT_BUTTON_RIGHT_THUMB,
    XINPUT_BUTTON_LEFT_SHOULDER,
    XINPUT_BUTTON_RIGHT_SHOULDER,
)
from renderer import (
    draw_skidpad,
    draw_slip_patches,
    draw_trc_slip_warning,
    draw_car_topdown,
    draw_hud_planar,
)
from ui import OptionsMenu, CheckBox


def _focus_window_on_startup():
    """Best-effort focus handoff for the simulator window on Windows."""
    if os.name != "nt":
        return False

    try:
        import ctypes

        pygame.event.pump()
        wm_info = pygame.display.get_wm_info() or {}
        hwnd = wm_info.get("window") or wm_info.get("hwnd")
        if not hwnd:
            return False

        user32 = ctypes.windll.user32
        SW_SHOWNORMAL = 1
        SW_RESTORE = 9
        HWND_TOPMOST = -1
        HWND_NOTOPMOST = -2
        SWP_NOMOVE = 0x0002
        SWP_NOSIZE = 0x0001
        SWP_NOACTIVATE = 0x0010

        fg_hwnd = user32.GetForegroundWindow()
        this_tid = user32.GetCurrentThreadId()
        fg_tid = user32.GetWindowThreadProcessId(fg_hwnd, None) if fg_hwnd else 0

        if fg_tid and fg_tid != this_tid:
            user32.AttachThreadInput(fg_tid, this_tid, True)

        # Nudge z-order so Windows is more likely to accept foreground activation.
        user32.SetWindowPos(hwnd, HWND_TOPMOST, 0, 0, 0, 0, SWP_NOMOVE | SWP_NOSIZE | SWP_NOACTIVATE)
        user32.SetWindowPos(hwnd, HWND_NOTOPMOST, 0, 0, 0, 0, SWP_NOMOVE | SWP_NOSIZE)
        user32.ShowWindow(hwnd, SW_SHOWNORMAL)
        user32.ShowWindow(hwnd, SW_RESTORE)
        user32.BringWindowToTop(hwnd)
        user32.SetActiveWindow(hwnd)
        focused = bool(user32.SetForegroundWindow(hwnd))

        if fg_tid and fg_tid != this_tid:
            user32.AttachThreadInput(fg_tid, this_tid, False)

        return focused
    except Exception:
        # Keep startup resilient if focus APIs fail in restricted contexts.
        return False


def _parse_scalar(val):
    try:
        return float(val.strip())
    except (TypeError, ValueError, AttributeError):
        return None


def _parse_bool(val):
    if val is None:
        return None
    txt = str(val).strip().lower()
    if txt in ("true", "1", "yes", "on"):
        return True
    if txt in ("false", "0", "no", "off"):
        return False
    return None


def _parse_text(val):
    if val is None:
        return None
    txt = str(val).strip()
    if not txt:
        return None
    if (txt.startswith('"') and txt.endswith('"')) or (txt.startswith("'") and txt.endswith("'")):
        txt = txt[1:-1].strip()
    return txt or None


def _load_presets_registry():
    """Read presets from presets.json; returns a dict keyed by preset name."""
    path = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "presets.json"))
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
    except (OSError, json.JSONDecodeError):
        return {}

    if not isinstance(data, dict):
        return {}

    out = {}
    for key, spec in data.items():
        if isinstance(key, str) and isinstance(spec, dict):
            out[key] = spec
    return out


def _load_global_settings():
    """Read startup settings from global config with safe fallbacks."""
    dt_default = 0.01
    fps_default = 60
    true_form_default = False
    auto_shift_default = False
    inverse_steering_default = False
    kb_throttle_ramp_engage_default = 0.5
    kb_throttle_ramp_release_default = 0.2
    kb_brake_ramp_engage_default = 0.5
    kb_brake_ramp_release_default = 0.2
    kb_steer_ramp_engage_default = 0.5
    kb_steer_ramp_release_default = 0.1
    preset_default = "DEFAULT"
    cfg_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "config.yaml"))

    cfg_dt = None
    cfg_fps = None
    cfg_true_form = None
    cfg_auto_shift = None
    cfg_inverse_steering = None
    cfg_kb_throttle_ramp_engage = None
    cfg_kb_throttle_ramp_release = None
    cfg_kb_brake_ramp_engage = None
    cfg_kb_brake_ramp_release = None
    cfg_kb_steer_ramp_engage = None
    cfg_kb_steer_ramp_release = None
    cfg_preset = None
    try:
        with open(cfg_path, "r", encoding="utf-8") as f:
            for raw in f:
                line = raw.split("#", 1)[0].strip()
                if not line or ":" not in line:
                    continue
                key, val = line.split(":", 1)
                key = key.strip().lower()
                if key == "timestep":
                    parsed = _parse_scalar(val)
                    if parsed is None:
                        continue
                    cfg_dt = parsed
                elif key == "fps":
                    parsed = _parse_scalar(val)
                    if parsed is None:
                        continue
                    cfg_fps = parsed
                elif key == "true_form":
                    cfg_true_form = _parse_bool(val)
                elif key == "auto_shift":
                    cfg_auto_shift = _parse_bool(val)
                elif key == "inverse_steering":
                    cfg_inverse_steering = _parse_bool(val)
                elif key == "kb_throttle_ramp_engage":
                    cfg_kb_throttle_ramp_engage = _parse_scalar(val)
                elif key == "kb_throttle_ramp_release":
                    cfg_kb_throttle_ramp_release = _parse_scalar(val)
                elif key == "kb_brake_ramp_engage":
                    cfg_kb_brake_ramp_engage = _parse_scalar(val)
                elif key == "kb_brake_ramp_release":
                    cfg_kb_brake_ramp_release = _parse_scalar(val)
                elif key == "kb_steer_ramp_engage":
                    cfg_kb_steer_ramp_engage = _parse_scalar(val)
                elif key == "kb_steer_ramp_release":
                    cfg_kb_steer_ramp_release = _parse_scalar(val)
                elif key == "preset":
                    cfg_preset = _parse_text(val)
    except OSError:
        pass

    dt_options = [dt for dt, _label in TIMESTEP_OPTIONS]
    fps_options = list(FPS_OPTIONS)

    if cfg_dt is None or cfg_dt <= 0:
        dt = dt_default
    else:
        dt = min(dt_options, key=lambda opt: abs(opt - cfg_dt))

    if cfg_fps is None or cfg_fps <= 0:
        fps = fps_default
    else:
        fps = int(min(fps_options, key=lambda opt: abs(opt - cfg_fps)))

    true_form = true_form_default if cfg_true_form is None else cfg_true_form
    auto_shift = auto_shift_default if cfg_auto_shift is None else cfg_auto_shift
    inverse_steering = inverse_steering_default if cfg_inverse_steering is None else cfg_inverse_steering

    def _pick_pos(parsed, default):
        if parsed is None or parsed <= 0:
            return default
        return float(parsed)

    kb_throttle_ramp_engage = _pick_pos(cfg_kb_throttle_ramp_engage, kb_throttle_ramp_engage_default)
    kb_throttle_ramp_release = _pick_pos(cfg_kb_throttle_ramp_release, kb_throttle_ramp_release_default)
    kb_brake_ramp_engage = _pick_pos(cfg_kb_brake_ramp_engage, kb_brake_ramp_engage_default)
    kb_brake_ramp_release = _pick_pos(cfg_kb_brake_ramp_release, kb_brake_ramp_release_default)
    kb_steer_ramp_engage = _pick_pos(cfg_kb_steer_ramp_engage, kb_steer_ramp_engage_default)
    kb_steer_ramp_release = _pick_pos(cfg_kb_steer_ramp_release, kb_steer_ramp_release_default)
    preset_name = preset_default if cfg_preset is None else cfg_preset

    return (
        dt,
        fps,
        true_form,
        auto_shift,
        inverse_steering,
        kb_throttle_ramp_engage,
        kb_throttle_ramp_release,
        kb_brake_ramp_engage,
        kb_brake_ramp_release,
        kb_steer_ramp_engage,
        kb_steer_ramp_release,
        preset_name,
    )

class Simulator:
    def __init__(self):
        if os.name == "nt":
            os.environ.setdefault("SDL_HINT_FORCE_RAISEWINDOW", "1")
        pygame.init()
        pygame.display.set_caption("Car Physics Simulator - Planar Dynamic Car Model")
        self._windowed_size = (1280, 720)
        self._is_fullscreen = False
        self.screen = pygame.display.set_mode(self._windowed_size, pygame.RESIZABLE)
        _focus_window_on_startup()
        self.screen_w, self.screen_h = self.screen.get_size()

        self.font_sm = pygame.font.SysFont("Consolas", 13)
        self.font_md = pygame.font.SysFont("Consolas", 17, bold=True)
        self.font_lg = pygame.font.SysFont("Consolas", 20, bold=True)

        (
            self.dt,
            self.target_fps,
            cfg_true_form,
            cfg_auto_shift,
            cfg_inverse_steering,
            cfg_kb_throttle_ramp_engage,
            cfg_kb_throttle_ramp_release,
            cfg_kb_brake_ramp_engage,
            cfg_kb_brake_ramp_release,
            cfg_kb_steer_ramp_engage,
            cfg_kb_steer_ramp_release,
            cfg_preset,
        ) = _load_global_settings()
        self.grid_size = GRID_SIZE
        self._spawn_tile = (0, 0)
        self.inverse_steering = bool(cfg_inverse_steering)
        self.kb_throttle_ramp_engage = cfg_kb_throttle_ramp_engage
        self.kb_throttle_ramp_release = cfg_kb_throttle_ramp_release
        self.kb_brake_ramp_engage = cfg_kb_brake_ramp_engage
        self.kb_brake_ramp_release = cfg_kb_brake_ramp_release
        self.kb_steer_ramp_engage = cfg_kb_steer_ramp_engage
        self.kb_steer_ramp_release = cfg_kb_steer_ramp_release
        self.yaw_damping_multiplier = float(YAW_DAMPING_MULTIPLIER)
        self.beta_damping = float(BETA_DAMPING)
        self.beta_hard_limit = float(BETA_HARD_LIMIT)
        self.skid_mark_visibility_scale = float(SKID_MARK_VISIBILITY_SCALE)
        self.skid_mark_long_slip_threshold = float(SKID_MARK_LONG_SLIP_THRESHOLD)
        self.skid_mark_lat_slip_threshold = float(SKID_MARK_LAT_SLIP_THRESHOLD)
        self.skid_mark_long_gain = float(SKID_MARK_LONG_GAIN)
        self.skid_mark_lat_gain = float(SKID_MARK_LAT_GAIN)

        # Backward-compatible aliases for existing UI/runtime access.
        self.throttle_ramp = self.kb_throttle_ramp_engage
        self.steer_ramp = self.kb_steer_ramp_engage
        self.input_mode = "keyboard"
        
        # Physics Wrapper
        self.car = Vehicle2D()
        self.car.YAW_DAMPING_MULTIPLIER = self.yaw_damping_multiplier
        self.car.BETA_DAMPING = self.beta_damping
        self.car.BETA_HARD_LIMIT = self.beta_hard_limit
        self.sim_time = 0.0
        self._preset_registry = _load_presets_registry()
        self.active_preset = None
        
        # Camera
        self.zoom = 1.0

        # Analog Inputs
        self.throttle, self.brake, self.steering = 0.0, 0.0, 0.0
        self._drive_throttle, self._drive_brake = 0.0, 0.0
        self._w_held, self._s_held, self._a_held, self._d_held = False, False, False, False

        # World-space rear-wheel slip visuals.
        self._slip_patches = []
        self._slip_layer = None
        self._toggled_grid_tiles = set()
        self._grid_paint_active = False
        self._grid_paint_value = True
        self._grid_paint_last_tile = None

        # Input state
        self._xinput_ok = load_xinput()
        self._btns_prev = 0
        self._last_keyboard_input_t = 0.0
        self._last_controller_input_t = 0.0
        self.enable_auto_shift = bool(cfg_auto_shift)
        self._apply_auto_shift_mode()

        # UI Elements
        self._menu_btn = pygame.Rect(8, 8, 110, 30)
        self._true_form_cb = CheckBox(130, 14, "True Form", checked=cfg_true_form)
        self.options = OptionsMenu(self)

        # Apply config-selected preset after car/options initialization.
        if not self.apply_preset(cfg_preset, reset=True):
            self.apply_preset("DEFAULT", reset=True)

        self._clock = pygame.time.Clock()
        self._fps_display, self._fps_acc, self._fps_frames = 0.0, 0.0, 0

        # Three-state timer: idle -> running -> stopped -> idle.
        self._timer_state = "idle"
        self._timer_start_t = 0.0
        self._timer_elapsed = 0.0

        # Manual shift warning state (shown below timer when direction change is blocked).
        self._shift_warning_t = 0.0
        self._shift_warning_msg = ""
        self._startup_focus_pending = (os.name == "nt")
        self._startup_focus_deadline_ms = pygame.time.get_ticks() + 1500
        self._layout()

    def get_preset_names(self):
        names = tuple(self._preset_registry.keys())
        if any(name.upper() == "DEFAULT" for name in names):
            return names
        return ("DEFAULT",) + names

    def get_preset_label(self):
        return self.active_preset or "DEFAULT"

    def _normalize_preset_name(self, preset_name):
        if not isinstance(preset_name, str):
            return None
        if preset_name.strip().upper() == "DEFAULT":
            return "DEFAULT"
        if preset_name in self._preset_registry:
            return preset_name

        target = preset_name.strip().lower()
        for key in self._preset_registry.keys():
            if key.lower() == target:
                return key
        return None

    def _apply_default_preset(self, reset=True, preserve_speed=None):
        """Reset active engine/car fields to constructor defaults for the current engine."""
        if preserve_speed is None:
            preserve_speed = not reset
        engine_id = self.car.engine_id
        self.car.set_engine(engine_id, preserve_speed=bool(preserve_speed))
        self._apply_auto_shift_mode()
        if hasattr(self.car, "chassis_color"):
            self.car.chassis_color = tuple(CAR_BODY)
        self.active_preset = "DEFAULT"
        if reset:
            self.reset_scenario()
        return True

    def _coerce_gear_ratios(self, raw):
        if not isinstance(raw, dict):
            return None
        out = {}
        for k, v in raw.items():
            try:
                ki = int(k)
                fv = float(v)
            except (TypeError, ValueError):
                return None
            out[ki] = fv
        return out

    def _coerce_torque_curve(self, raw):
        if not isinstance(raw, list):
            return None
        out = []
        for pt in raw:
            if not isinstance(pt, (list, tuple)) or len(pt) != 2:
                return None
            try:
                out.append((float(pt[0]), float(pt[1])))
            except (TypeError, ValueError):
                return None
        return out

    def apply_preset(self, preset_name, reset=True):
        """Apply a named preset to the current car/engine. Returns True when applied."""
        key = self._normalize_preset_name(preset_name)
        if key is None:
            return False

        spec = self._preset_registry.get(key)
        if key != "DEFAULT" and not isinstance(spec, dict):
            return False

        # Always start from true engine defaults so any omitted preset field
        # falls back to canonical defaults instead of the previous preset state.
        self._apply_default_preset(reset=False, preserve_speed=not reset)

        engine = getattr(self.car, "engine", None)
        if engine is None:
            return False

        shared_vehicle_fields = {"M", "F_ENGINE_MAX", "C_RR", "C_DRAG", "C_BRAKING", "L"}
        cg_height = None
        cg_to_front = None
        cg_to_rear = None
        has_hybrid_torque = False
        has_hybrid_fade_start = False
        has_hybrid_fade_end = False

        for field, value in (spec or {}).items():
            if field == "CG_HEIGHT":
                try:
                    cg_height = float(value)
                except (TypeError, ValueError):
                    cg_height = None
                continue

            if field == "CG_TO_FRONT":
                try:
                    cg_to_front = float(value)
                except (TypeError, ValueError):
                    cg_to_front = None
                continue

            if field == "CG_TO_REAR":
                try:
                    cg_to_rear = float(value)
                except (TypeError, ValueError):
                    cg_to_rear = None
                continue

            if field in ("C_BRAKE_TORQUE", "C_BRAKING"):
                try:
                    fval = float(value)
                except (TypeError, ValueError):
                    continue

                if hasattr(engine, "C_BRAKE_TORQUE"):
                    engine.C_BRAKE_TORQUE = fval
                if hasattr(engine, "C_BRAKING"):
                    engine.C_BRAKING = fval
                if hasattr(self.car, "C_BRAKING"):
                    self.car.C_BRAKING = fval
                continue

            if field == "GEAR_RATIOS":
                ratios = self._coerce_gear_ratios(value)
                if ratios is not None and hasattr(engine, "GEAR_RATIOS"):
                    engine.GEAR_RATIOS = dict(ratios)
                continue

            if field == "TORQUE_CURVE":
                curve = self._coerce_torque_curve(value)
                if curve is not None and hasattr(engine, "TORQUE_CURVE"):
                    engine.TORQUE_CURVE = list(curve)
                continue

            if field == "UPSHIFT_RPM" and hasattr(engine, "upshift_rpm"):
                try:
                    engine.upshift_rpm = float(value)
                except (TypeError, ValueError):
                    pass
                continue

            if field == "DOWNSHIFT_RPM" and hasattr(engine, "downshift_rpm"):
                try:
                    engine.downshift_rpm = float(value)
                except (TypeError, ValueError):
                    pass
                continue

            if field == "CHASSIS_COLOR" and hasattr(self.car, "chassis_color"):
                if isinstance(value, (list, tuple)) and len(value) == 3:
                    try:
                        self.car.chassis_color = tuple(max(0, min(255, int(c))) for c in value)
                    except (TypeError, ValueError):
                        self.car.chassis_color = tuple(CAR_BODY)
                else:
                    # If key exists but value is null/invalid, reset to canonical default color.
                    self.car.chassis_color = tuple(CAR_BODY)
                continue

            try:
                fval = float(value)
            except (TypeError, ValueError):
                continue

            if field == "HYBRID_TORQUE_MAX":
                has_hybrid_torque = True
            elif field == "HYBRID_ASSIST_FADE_START_RPM":
                has_hybrid_fade_start = True
            elif field == "HYBRID_ASSIST_FADE_END_RPM":
                has_hybrid_fade_end = True

            if hasattr(engine, field):
                setattr(engine, field, fval)
            if field in shared_vehicle_fields and hasattr(self.car, field):
                setattr(self.car, field, fval)

        # Prevent hybrid-assist carry-over when switching from a hybrid preset
        # to a preset that does not declare hybrid fields.
        if hasattr(engine, "HYBRID_TORQUE_MAX") and not has_hybrid_torque:
            engine.HYBRID_TORQUE_MAX = 0.0
        if hasattr(engine, "HYBRID_ASSIST_FADE_START_RPM") and not has_hybrid_fade_start:
            engine.HYBRID_ASSIST_FADE_START_RPM = 6000.0
        if hasattr(engine, "HYBRID_ASSIST_FADE_END_RPM") and not has_hybrid_fade_end:
            engine.HYBRID_ASSIST_FADE_END_RPM = 8000.0

        # Optional CG geometry overrides for Long.5 style models.
        if cg_height is not None and hasattr(engine, "h"):
            engine.h = max(0.0, cg_height)

        if (cg_to_front is not None or cg_to_rear is not None) and hasattr(engine, "L"):
            b_val = float(getattr(engine, "b", 0.0))
            c_val = float(getattr(engine, "c", 0.0))

            if cg_to_front is not None:
                b_val = max(0.0, cg_to_front)
            if cg_to_rear is not None:
                c_val = max(0.0, cg_to_rear)

            if b_val + c_val > 1e-6:
                engine.L = b_val + c_val
                if hasattr(self.car, "L"):
                    self.car.L = engine.L
                if hasattr(engine, "_b_ratio"):
                    engine._b_ratio = b_val / engine.L
                if hasattr(engine, "_c_ratio"):
                    engine._c_ratio = c_val / engine.L
                if hasattr(engine, "b"):
                    engine.b = b_val
                if hasattr(engine, "c"):
                    engine.c = c_val

        # Keep vehicle-engine synchronized for shared fields and derived hints.
        if hasattr(self.car, "_sync_vehicle_from_engine"):
            self.car._sync_vehicle_from_engine()
        if hasattr(engine, "_update_force_hint"):
            engine._update_force_hint()

        self.active_preset = key
        if reset:
            self.reset_scenario()
        return True

    def _layout(self):
        self.scene_rect = pygame.Rect(0, 0, self.screen_w, int(self.screen_h * 0.75))
        self.hud_rect = pygame.Rect(0, self.scene_rect.height, self.screen_w, self.screen_h - self.scene_rect.height)

    def _snap_spawn_to_tile_center(self):
        grid = max(1e-6, float(self.grid_size))
        # Keep default spawn tile at world-origin tile, but centered.
        self.car.x = 0.5 * grid
        self.car.y = 0.5 * grid
        self._spawn_tile = (0, 0)

    def _camera_center(self):
        cam_x = self.car.x + 0.5 * self.car.L * math.cos(self.car.heading)
        cam_y = self.car.y + 0.5 * self.car.L * math.sin(self.car.heading)
        return cam_x, cam_y

    def _grid_tile_from_screen(self, sx, sy):
        if self.grid_size <= 0.0:
            return None
        ppm = max(1e-6, PIXELS_PER_METER * self.zoom)
        cam_x, cam_y = self._camera_center()
        wx = ((sx - (self.screen_w / 2.0)) / ppm) + cam_x
        wy = -((sy - (self.scene_rect.height / 2.0)) / ppm) + cam_y

        ix = int(math.floor(wx / self.grid_size))
        iy = int(math.floor(wy / self.grid_size))
        return (ix, iy)

    def _set_grid_tile(self, key, enabled):
        if key is None:
            return
        if enabled:
            self._toggled_grid_tiles.add(key)
        else:
            self._toggled_grid_tiles.discard(key)

    def _toggle_fullscreen(self):
        if self._is_fullscreen:
            self.screen = pygame.display.set_mode(self._windowed_size, pygame.RESIZABLE)
            self._is_fullscreen = False
        else:
            self._windowed_size = self.screen.get_size()
            self.screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
            self._is_fullscreen = True

        self.screen_w, self.screen_h = self.screen.get_size()
        self._layout()
        self._slip_layer = None
        if hasattr(self, "options") and self.options is not None:
            self.options._build()

    def reset_scenario(self):
        self.car.reset()
        self._snap_spawn_to_tile_center()
        self._apply_auto_shift_mode()
        self._drive_throttle, self._drive_brake = 0.0, 0.0
        self.sim_time, self.throttle, self.brake, self.steering = 0.0, 0.0, 0.0, 0.0
        self._slip_patches.clear()
        self._timer_state = "idle"
        self._timer_elapsed = 0.0
        self._timer_start_t = 0.0

    def reset_grid_tiles(self):
        self._toggled_grid_tiles.clear()
        self._grid_paint_active = False
        self._grid_paint_last_tile = None

    def _apply_auto_shift_mode(self):
        if hasattr(self.car.engine, "enable_auto_shift"):
            self.car.engine.enable_auto_shift = bool(self.enable_auto_shift)

    def _toggle_auto_shift(self):
        self.enable_auto_shift = not self.enable_auto_shift
        self._apply_auto_shift_mode()

    def _apply_auto_direction_logic(self):
        """Model3-style auto mode: brake can command reverse and direction changes require near stop."""
        if not hasattr(self.car.engine, "gear"):
            self._drive_throttle = self.throttle
            self._drive_brake = self.brake
            return

        v = self.car.v
        near_stop = abs(v) <= 0.08
        throttle_in = self.throttle
        brake_in = self.brake

        if self.car.engine.gear == 0:
            self.car.engine.gear = 1

        # Forward gear behavior.
        if self.car.engine.gear >= 1:
            if near_stop and brake_in > 0.05 and throttle_in < 0.1:
                self.car.engine.gear = -1
                self._drive_throttle = brake_in
                self._drive_brake = 0.0
                return

            self._drive_throttle = throttle_in
            self._drive_brake = brake_in
            return

        # Reverse gear behavior: brake doubles as reverse throttle.
        if brake_in > 0.05:
            self._drive_throttle = brake_in
            self._drive_brake = 0.0
            return

        # Forward throttle while reversing applies brake first.
        if v < -0.08 and throttle_in > 0.05:
            self._drive_throttle = 0.0
            self._drive_brake = throttle_in
            return

        if near_stop and throttle_in > 0.05:
            self.car.engine.gear = 1
            self._drive_throttle = throttle_in
            self._drive_brake = 0.0
            return

        self._drive_throttle = 0.0
        self._drive_brake = 0.0

    def _request_shift(self, delta):
        if not hasattr(self.car.engine, "gear"):
            return

        # Explicitly ignore manual shift commands while auto shift is active.
        if self.enable_auto_shift:
            self._shift_warning_msg = "Manual shifting disabled while AUTO SHIFT is ON"
            self._shift_warning_t = 2.0
            return

        if hasattr(self.car.engine, "GEAR_RATIOS"):
            max_fwd_gear = max((g for g in self.car.engine.GEAR_RATIOS.keys() if g > 0), default=5)
        else:
            max_fwd_gear = 5

        current_gear = int(self.car.engine.gear)
        target_gear = max(-1, min(max_fwd_gear, current_gear + int(delta)))

        # Direction safety for manual shifting:
        # require near-stop before selecting reverse while moving forward (and vice versa).
        if not self.enable_auto_shift:
            near_stop_speed = 0.08
            v = float(self.car.v)
            selecting_reverse_while_forward = target_gear < 0 and v > near_stop_speed
            selecting_forward_while_reverse = target_gear > 0 and v < -near_stop_speed
            if selecting_reverse_while_forward or selecting_forward_while_reverse:
                self._shift_warning_msg = "Go full stop before changing direction"
                self._shift_warning_t = 2.0
                return

        self.car.engine.gear = target_gear

    def _toggle_timer(self):
        if self._timer_state == "idle":
            self._timer_state = "running"
            self._timer_start_t = self.sim_time
            self._timer_elapsed = 0.0
            return
        if self._timer_state == "running":
            self._timer_elapsed = max(0.0, self.sim_time - self._timer_start_t)
            self._timer_state = "stopped"
            return
        self._timer_state = "idle"
        self._timer_elapsed = 0.0

    def _timer_display(self):
        if self._timer_state == "idle":
            return "TIMER IDLE", (90, 255, 255)
        if self._timer_state == "running":
            elapsed = max(0.0, self.sim_time - self._timer_start_t)
            return f"{elapsed:.2f}s", (90, 255, 255)
        return f"{self._timer_elapsed:.2f}s", (255, 215, 80)

    def _handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT: return False
            if event.type == pygame.VIDEORESIZE:
                self.screen_w, self.screen_h = event.w, event.h
                self._layout()
                self._slip_layer = None
                self.options._build()

            if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE and not self.options.editing_active:
                self.options.toggle(); continue

            # Top UI Overlays
            if event.type == pygame.MOUSEMOTION:
                self._true_form_cb.handle_event(event)
                if (
                    self._grid_paint_active
                    and event.buttons[0]
                    and not self.options.visible
                    and self.scene_rect.collidepoint(event.pos)
                ):
                    tile = self._grid_tile_from_screen(event.pos[0], event.pos[1])
                    if tile is not None and tile != self._grid_paint_last_tile:
                        self._set_grid_tile(tile, self._grid_paint_value)
                        self._grid_paint_last_tile = tile

            if event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                self._grid_paint_active = False
                self._grid_paint_last_tile = None

            if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1 and not self.options.visible:
                if self._true_form_cb.handle_event(event): continue
                if self._menu_btn.collidepoint(event.pos): self.options.toggle(); continue
                if self.scene_rect.collidepoint(event.pos):
                    tile = self._grid_tile_from_screen(event.pos[0], event.pos[1])
                    current_on = (tile in self._toggled_grid_tiles)
                    self._grid_paint_value = not current_on
                    self._set_grid_tile(tile, self._grid_paint_value)
                    self._grid_paint_active = True
                    self._grid_paint_last_tile = tile
                    continue

            if self.options.handle_event(event): continue

            # Keyboard Input
            if event.type in (pygame.KEYDOWN, pygame.KEYUP):
                now = pygame.time.get_ticks() * 0.001
                is_down = (event.type == pygame.KEYDOWN)
                if event.key in (pygame.K_w, pygame.K_UP):
                    self._w_held = is_down
                    if is_down:
                        self._last_keyboard_input_t = now
                if event.key in (pygame.K_SPACE, pygame.K_LSHIFT, pygame.K_s, pygame.K_DOWN):
                    self._s_held = is_down
                    if is_down:
                        self._last_keyboard_input_t = now
                if event.key in (pygame.K_a, pygame.K_LEFT):
                    self._a_held = is_down
                    if is_down:
                        self._last_keyboard_input_t = now
                if event.key in (pygame.K_d, pygame.K_RIGHT):
                    self._d_held = is_down
                    if is_down:
                        self._last_keyboard_input_t = now

                if is_down:
                    if event.key == pygame.K_1:
                        self._toggle_auto_shift()
                        self._last_keyboard_input_t = now
                    if event.key == pygame.K_f:
                        self._toggle_fullscreen()
                        self._last_keyboard_input_t = now
                    if event.key == pygame.K_j:
                        self._request_shift(-1)
                        self._last_keyboard_input_t = now
                    if event.key == pygame.K_k:
                        self._request_shift(+1)
                        self._last_keyboard_input_t = now
                    if event.key == pygame.K_2:
                        self.zoom = min(3.0, self.zoom + 0.2)
                        self._last_keyboard_input_t = now
                    if event.key == pygame.K_3:
                        self.zoom = max(0.2, self.zoom - 0.2)
                        self._last_keyboard_input_t = now
                    if event.key == pygame.K_4:
                        self._true_form_cb.checked = not self._true_form_cb.checked
                        self._last_keyboard_input_t = now
                    if event.key == pygame.K_q:
                        self._toggle_timer()
                        self._last_keyboard_input_t = now
                    if event.key == pygame.K_r:
                        self.reset_scenario()
                        self._last_keyboard_input_t = now
        return True

    def _update_input(self, dt):
        now = pygame.time.get_ticks() * 0.001
        xi = get_xinput_state(0)
        rt = lt = steer = 0.0
        btns = 0
        controller_active = False
        if xi is not None:
            rt, lt, steer, btns = xi
            controller_active = (rt > 0.05 or lt > 0.05 or abs(steer) > 0.05 or btns != 0)
            if controller_active:
                self._last_controller_input_t = now

        keyboard_active = (self._w_held or self._s_held or self._a_held or self._d_held)
        if keyboard_active:
            self._last_keyboard_input_t = now

        recent_window = 0.35
        kb_recent = (now - self._last_keyboard_input_t) <= recent_window
        ctrl_recent = (xi is not None) and ((now - self._last_controller_input_t) <= recent_window)

        if kb_recent and not ctrl_recent:
            self.input_mode = "keyboard"
        elif ctrl_recent and not kb_recent:
            self.input_mode = "controller"
        elif kb_recent and ctrl_recent:
            self.input_mode = (
                "keyboard"
                if self._last_keyboard_input_t >= self._last_controller_input_t
                else "controller"
            )
        elif xi is None:
            self.input_mode = "keyboard"

        if xi is not None:
            # Controller buttons (edge detect)
            if (btns & XINPUT_BUTTON_START) and not (self._btns_prev & XINPUT_BUTTON_START):
                if not self.options.editing_active:
                    self.options.toggle()
            if (btns & XINPUT_BUTTON_BACK) and not (self._btns_prev & XINPUT_BUTTON_BACK):
                self.reset_scenario()
            if (btns & XINPUT_BUTTON_A) and not (self._btns_prev & XINPUT_BUTTON_A):
                self._toggle_auto_shift()
            if (
                ((btns & XINPUT_BUTTON_X) and not (self._btns_prev & XINPUT_BUTTON_X))
                or ((btns & XINPUT_BUTTON_LEFT_SHOULDER) and not (self._btns_prev & XINPUT_BUTTON_LEFT_SHOULDER))
            ):
                self._request_shift(-1)
            if (
                ((btns & XINPUT_BUTTON_B) and not (self._btns_prev & XINPUT_BUTTON_B))
                or ((btns & XINPUT_BUTTON_RIGHT_SHOULDER) and not (self._btns_prev & XINPUT_BUTTON_RIGHT_SHOULDER))
            ):
                self._request_shift(+1)
            if (btns & XINPUT_BUTTON_DPAD_UP) and not (self._btns_prev & XINPUT_BUTTON_DPAD_UP):
                self.zoom = min(3.0, self.zoom + 0.2)
            if (btns & XINPUT_BUTTON_DPAD_DOWN) and not (self._btns_prev & XINPUT_BUTTON_DPAD_DOWN):
                self.zoom = max(0.2, self.zoom - 0.2)
            if (btns & XINPUT_BUTTON_DPAD_LEFT) and not (self._btns_prev & XINPUT_BUTTON_DPAD_LEFT):
                self._toggle_timer()
            if (btns & XINPUT_BUTTON_DPAD_RIGHT) and not (self._btns_prev & XINPUT_BUTTON_DPAD_RIGHT):
                self._true_form_cb.checked = not self._true_form_cb.checked
            if (btns & XINPUT_BUTTON_RIGHT_THUMB) and not (self._btns_prev & XINPUT_BUTTON_RIGHT_THUMB):
                self._toggle_fullscreen()
            self._btns_prev = btns
        else:
            self._btns_prev = 0

        # Keep controller/menu button handling responsive while options are open,
        # but do not update analog driving inputs under the menu.
        if self.options.visible:
            return

        if self.input_mode == "keyboard" or xi is None:
            # Analog Ramps for Keyboard
            tr_up = 1.0 / max(1e-6, self.kb_throttle_ramp_engage)
            tr_down = 1.0 / max(1e-6, self.kb_throttle_ramp_release)
            br_up = 1.0 / max(1e-6, self.kb_brake_ramp_engage)
            br_down = 1.0 / max(1e-6, self.kb_brake_ramp_release)
            sr_up = 1.0 / max(1e-6, self.kb_steer_ramp_engage)
            sr_down = 1.0 / max(1e-6, self.kb_steer_ramp_release)

            self.throttle = min(1.0, self.throttle + tr_up * dt) if self._w_held else max(0.0, self.throttle - tr_down * dt)
            self.brake = min(1.0, self.brake + br_up * dt) if self._s_held else max(0.0, self.brake - br_down * dt)

            left_held = self._a_held
            right_held = self._d_held
            if self.inverse_steering:
                left_held, right_held = right_held, left_held

            if left_held:
                self.steering = max(-1.0, self.steering - sr_up * dt)
            elif right_held:
                self.steering = min(1.0, self.steering + sr_up * dt)
            else:
                if self.steering > 0:
                    self.steering = max(0.0, self.steering - sr_down * dt)
                if self.steering < 0:
                    self.steering = min(0.0, self.steering + sr_down * dt)
        else:
            self.throttle = rt
            self.brake = lt
            self.steering = max(-1.0, min(1.0, -steer if self.inverse_steering else steer))

        # Drive command mapping (supports model3-style auto reverse behavior).
        has_gears = hasattr(self.car.engine, "gear")
        if self.enable_auto_shift and has_gears:
            self._apply_auto_shift_mode()
            self._apply_auto_direction_logic()
        else:
            if has_gears and hasattr(self.car.engine, "enable_auto_shift"):
                self.car.engine.enable_auto_shift = False
            self._drive_throttle = self.throttle
            self._drive_brake = self.brake

    def _update_slip_visuals(self, dt):
        """Spawn and decay dark rear-wheel slip patches on wheelspin and lateral slip."""
        engine = getattr(self.car, "engine", None)
        slip_ratio = abs(float(getattr(engine, "last_slip_ratio", 0.0))) if engine is not None else 0.0
        vis_scale = max(0.05, float(getattr(self, "skid_mark_visibility_scale", SKID_MARK_VISIBILITY_SCALE)))
        long_slip_threshold = max(1e-6, float(getattr(self, "skid_mark_long_slip_threshold", SKID_MARK_LONG_SLIP_THRESHOLD))) / vis_scale
        long_gain = float(getattr(self, "skid_mark_long_gain", SKID_MARK_LONG_GAIN))
        long_slip_intensity = max(0.0, (slip_ratio - long_slip_threshold) * (long_gain * vis_scale))
        is_long_slipping = long_slip_intensity > 0.0
        
        # Model 7: Introduce lateral slip detection
        alpha_f = abs(getattr(self.car, "last_alpha_f", 0.0))
        alpha_r = abs(getattr(self.car, "last_alpha_r", 0.0))
        
        # Determine if we are slipping laterally (baseline threshold ~ 6 degrees / 0.1 rad)
        lat_slip_threshold = max(1e-6, float(getattr(self, "skid_mark_lat_slip_threshold", SKID_MARK_LAT_SLIP_THRESHOLD))) / vis_scale
        lat_gain = float(getattr(self, "skid_mark_lat_gain", SKID_MARK_LAT_GAIN))
        lat_slip_intensity_r = max(0.0, (alpha_r - lat_slip_threshold) * (lat_gain * vis_scale))
        lat_slip_intensity_f = max(0.0, (alpha_f - lat_slip_threshold) * (lat_gain * vis_scale))
        
        is_lat_slipping_r = lat_slip_intensity_r > 0.0
        speed = abs(self.car.v)

        if is_long_slipping or is_lat_slipping_r:
            half_track = 0.78
            lat_x = -math.sin(self.car.heading)
            lat_y = math.cos(self.car.heading)
            
            # Rear Axle Patches (Longitudinal OR Lateral)
            if is_long_slipping or is_lat_slipping_r:
                intensity_r = max(min(1.0, long_slip_intensity), min(1.0, lat_slip_intensity_r))
                rear_x = self.car.x
                rear_y = self.car.y
                
                for sign in (-1.0, 1.0):
                    wx = rear_x + sign * half_track * lat_x
                    wy = rear_y + sign * half_track * lat_y
                    self._slip_patches.append(
                        {
                            "x": wx,
                            "y": wy,
                            "radius_m": 0.06 + 0.16 * intensity_r,
                            "alpha": 105.0 + 110.0 * intensity_r,
                            "darkness": 20,
                            "decay": 32.0 + 42.0 * intensity_r,
                        }
                    )

                # Extra central smear for burnouts
                if is_long_slipping and speed < 1.5:
                    self._slip_patches.append(
                        {
                            "x": rear_x,
                            "y": rear_y,
                            "radius_m": 0.08 + 0.10 * intensity_r,
                            "alpha": 70.0 + 90.0 * intensity_r,
                            "darkness": 24,
                            "decay": 30.0 + 28.0 * intensity_r,
                        }
                    )
            
        if self._slip_patches:
            for patch in self._slip_patches:
                patch["alpha"] -= dt * patch.get("decay", 40.0)
            self._slip_patches = [p for p in self._slip_patches if p.get("alpha", 0.0) > 0.0]
            if len(self._slip_patches) > 2000:
                self._slip_patches = self._slip_patches[-2000:]

    def run(self):
        running = True
        accumulator = 0.0
        while running:
            frame_dt = min(self._clock.tick(self.target_fps) / 1000.0, 0.1)
            
            self._fps_acc += frame_dt
            self._fps_frames += 1
            if self._fps_acc >= 0.5:
                self._fps_display, self._fps_acc, self._fps_frames = self._fps_frames / self._fps_acc, 0.0, 0

            if (self.screen.get_size() != (self.screen_w, self.screen_h)):
                self.screen_w, self.screen_h = self.screen.get_size(); self._layout(); self._slip_layer = None

            running = self._handle_events()

            self._update_input(frame_dt)
            if not self.options.visible:
                accumulator += frame_dt
                while accumulator >= self.dt:
                    self.car.update(self.dt, self._drive_throttle, self._drive_brake, self.steering)
                    self._update_slip_visuals(self.dt)
                    self.sim_time += self.dt
                    accumulator -= self.dt

            if self._shift_warning_t > 0.0:
                self._shift_warning_t = max(0.0, self._shift_warning_t - frame_dt)
                if self._shift_warning_t <= 0.0:
                    self._shift_warning_msg = ""

            # Drawing Pipeline
            cam_x, cam_y = self._camera_center()

            draw_skidpad(
                self.screen,
                cam_x,
                cam_y,
                self.zoom,
                self.screen_w,
                self.scene_rect.height,
                self.grid_size,
                self._toggled_grid_tiles,
                self._spawn_tile,
            )
            if self._slip_layer is None or self._slip_layer.get_size() != (self.screen_w, self.scene_rect.height):
                self._slip_layer = pygame.Surface((self.screen_w, self.scene_rect.height), pygame.SRCALPHA)
            draw_slip_patches(
                self.screen,
                self._slip_patches,
                cam_x,
                cam_y,
                self.zoom,
                self.screen_w,
                self.scene_rect.height,
                layer=self._slip_layer,
            )
            draw_car_topdown(self.screen, self.car, cam_x, cam_y, self.zoom, self.screen_w, self.scene_rect.height, self._true_form_cb.checked)
            draw_trc_slip_warning(self.screen, self.font_sm, self.sim_time, self.scene_rect.height, self.car)
            draw_hud_planar(self.screen, self.hud_rect, self.font_sm, self.font_lg, self.car, self.throttle, self.brake, self.steering, self._fps_display, self.sim_time)

            # Bottom-right scene tag for active preset (paper validation aid).
            preset_name = self.get_preset_label()
            if preset_name == "MCLAREN_P1":
                preset_display = "MCLAREN_P1 (Hybrid Assist)"
            else:
                preset_display = preset_name

            preset_txt = self.font_sm.render(preset_display, True, (230, 235, 255))
            pad_x = 8
            pad_y = 6
            box_w = preset_txt.get_width() + pad_x * 2
            box_h = preset_txt.get_height() + pad_y * 2
            box_x = self.screen_w - box_w - 10
            box_y = self.scene_rect.height - box_h - 10
            box = pygame.Surface((box_w, box_h), pygame.SRCALPHA)
            box.fill((12, 14, 20, 180))
            self.screen.blit(box, (box_x, box_y))
            pygame.draw.rect(self.screen, (80, 170, 255), (box_x, box_y, box_w, box_h), 1, border_radius=4)
            self.screen.blit(preset_txt, (box_x + pad_x, box_y + pad_y))
            
            # Top Overlays
            # Menu Btn
            pygame.draw.rect(self.screen, (65, 72, 96) if self._menu_btn.collidepoint(pygame.mouse.get_pos()) else (45, 50, 68), self._menu_btn, border_radius=5)
            pygame.draw.rect(self.screen, (80, 170, 255), self._menu_btn, 1, border_radius=5)
            self.screen.blit(self.font_sm.render("Options", True, (230, 235, 255)), (25, 15))
            # Checkboxes
            self._true_form_cb.draw(self.screen, self.font_sm)
            # Top-Right Text
            input_src = "Pad" if self.input_mode == "controller" else "KB"
            hud = self.car.get_hud_data()
            mode_label = hud.get("mode_label", f"Long.{self.car.engine_id}")
            top_right_line_1 = self.font_sm.render(
                f"FPS: {self._fps_display:.0f} | dt: {self.dt} | Zoom: {self.zoom:.2f}x",
                True,
                (230, 235, 255),
            )
            top_right_line_2 = self.font_sm.render(
                f"Grid: {self.grid_size:.0f}m | Input: {input_src}",
                True,
                (230, 235, 255),
            )
            self.screen.blit(top_right_line_1, (self.screen_w - top_right_line_1.get_width() - 10, 10))
            self.screen.blit(top_right_line_2, (self.screen_w - top_right_line_2.get_width() - 10, 28))
            # Top-Center Text
            txt = self.font_sm.render(f"t = {self.sim_time:.1f}s", True, (255, 255, 255))
            self.screen.blit(txt, (self.screen_w//2 - 30, 10))
            timer_text, timer_col = self._timer_display()
            timer_lbl = self.font_sm.render(timer_text, True, timer_col)
            self.screen.blit(timer_lbl, (self.screen_w // 2 - timer_lbl.get_width() // 2, 28))
            if self._shift_warning_t > 0.0 and self._shift_warning_msg:
                warn_col = (255, 214, 96)
                warn_lbl = self.font_sm.render(self._shift_warning_msg, True, warn_col)
                self.screen.blit(warn_lbl, (self.screen_w // 2 - warn_lbl.get_width() // 2, 46))

            self.options.draw(self.screen, self.font_sm, self.font_md)
            pygame.display.flip()

            if self._startup_focus_pending:
                if _focus_window_on_startup() or pygame.time.get_ticks() >= self._startup_focus_deadline_ms:
                    self._startup_focus_pending = False
        pygame.quit()

if __name__ == "__main__":
    Simulator().run()