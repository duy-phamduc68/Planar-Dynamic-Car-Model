# ─────────────────────────────────────────────────────────────────────────────
# ui.py — Button, CheckBox widgets and a model5-style options overlay
# ─────────────────────────────────────────────────────────────────────────────

import pygame

from constants import (
    TEXT_BRIGHT,
    TEXT_DIM,
    ACCENT,
    GRAPH_AXIS,
    BTN_NORMAL,
    BTN_HOVER,
    BTN_ACTIVE,
    GRAPH_GRID,
    PANEL_BG,
    TIMESTEP_OPTIONS,
    FPS_OPTIONS,
    CONST_FIELDS,
)

try:
    import pyperclip
    _HAS_PYPERCLIP = True
except Exception:
    pyperclip = None
    _HAS_PYPERCLIP = False


def _sec_label(surface, font, text, x, y, color=None):
    col = color if color is not None else TEXT_DIM
    lbl = font.render(text, True, col)
    surface.blit(lbl, (x, y))


def _fmt_const(val):
    if isinstance(val, bool):
        return "true" if val else "false"
    try:
        fv = float(val)
    except (TypeError, ValueError):
        return str(val)
    if fv == int(fv):
        return str(int(fv))
    return f"{fv:g}"


def _parse_float(text):
    try:
        return float(text)
    except (TypeError, ValueError):
        return None


def _parse_bool(text):
    if text is None:
        return None
    txt = str(text).strip().lower()
    if txt in ("true", "1", "yes", "on"):
        return True
    if txt in ("false", "0", "no", "off"):
        return False
    return None


def _clamp(val, lo, hi):
    return max(lo, min(hi, val))


def _valid_gear_ratios(text):
    if not text.strip():
        return False
    pairs = [p.strip() for p in text.split(",") if p.strip()]
    if not pairs:
        return False
    for pair in pairs:
        if ":" not in pair:
            return False
        g, r = pair.split(":", 1)
        gi = _parse_float(g)
        rr = _parse_float(r)
        if gi is None or rr is None or gi <= 0 or rr <= 0:
            return False
    return True


def _valid_torque_curve(text):
    if not text.strip():
        return False
    pts = [p.strip() for p in text.split(",") if p.strip()]
    if not pts:
        return False
    for pt in pts:
        if ":" not in pt:
            return False
        rpm, tq = pt.split(":", 1)
        rv = _parse_float(rpm)
        tv = _parse_float(tq)
        if rv is None or tv is None or rv <= 0 or tv <= 0:
            return False
    return True


def _placeholder_for_field(spec):
    default = spec.get("default")
    lo = spec.get("lo")
    hi = spec.get("hi")
    ftype = spec.get("type")
    if ftype == "bool":
        return f"default: {_fmt_const(bool(default))}; true/false"
    if ftype == "gear_ratios":
        return "default: 1:3.8, 2:2.2, 3:1.5"
    if ftype == "torque_curve":
        return "default: 1000:140, 3000:220, 5500:170"
    if default is not None and lo is not None and hi is not None:
        return f"default: {_fmt_const(default)}; {lo:g} - {hi:g}"
    if default is not None:
        return f"default: {_fmt_const(default)}"
    if lo is not None and hi is not None:
        return f"{lo:g} - {hi:g}"
    return ""


def _field_valid(spec, text):
    if text is None:
        return False
    txt = text.strip()
    if not txt:
        return True

    ftype = spec.get("type", "float")
    if ftype == "float":
        val = _parse_float(txt)
        if val is None:
            return False
        lo = spec.get("lo")
        hi = spec.get("hi")
        if lo is not None and val < lo:
            return False
        if hi is not None and val > hi:
            return False
        return True
    if ftype == "bool":
        return _parse_bool(txt) is not None
    if ftype == "gear_ratios":
        return _valid_gear_ratios(txt)
    if ftype == "torque_curve":
        return _valid_torque_curve(txt)
    return True


class Button:
    def __init__(self, rect, label, toggle=False, active=False):
        self.rect = pygame.Rect(rect)
        self.label = label
        self.toggle = toggle
        self.active = active
        self.disabled = False
        self._hover = False

    def handle_event(self, event, mapped_pos=None):
        if self.disabled:
            return False

        pos = mapped_pos if mapped_pos is not None else getattr(event, "pos", None)
        if pos is None:
            return False

        if event.type == pygame.MOUSEMOTION:
            self._hover = self.rect.collidepoint(pos)
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1 and self.rect.collidepoint(pos):
            if self.toggle:
                self.active = not self.active
            return True
        return False

    def draw(self, surface, font):
        if self.disabled:
            col = (30, 32, 42)
            border_col = (55, 58, 72)
            txt_col = (80, 82, 98)
        elif self.active:
            col = BTN_ACTIVE
            border_col = ACCENT
            txt_col = TEXT_BRIGHT
        elif self._hover:
            col = BTN_HOVER
            border_col = GRAPH_AXIS
            txt_col = TEXT_BRIGHT
        else:
            col = BTN_NORMAL
            border_col = GRAPH_AXIS
            txt_col = TEXT_BRIGHT

        pygame.draw.rect(surface, col, self.rect, border_radius=5)
        pygame.draw.rect(surface, border_col, self.rect, 1, border_radius=5)
        txt = font.render(self.label, True, txt_col)
        surface.blit(txt, txt.get_rect(center=self.rect.center))


class CheckBox:
    def __init__(self, x, y, label, checked=True):
        self.rect = pygame.Rect(x, y, 18, 18)
        self.label = label
        self.checked = checked
        self._hover = False

    def handle_event(self, event, mapped_pos=None):
        pos = mapped_pos if mapped_pos is not None else getattr(event, "pos", None)
        if pos is None:
            return False

        if event.type == pygame.MOUSEMOTION:
            self._hover = self.rect.collidepoint(pos)
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1 and self.rect.collidepoint(pos):
            self.checked = not self.checked
            return True
        return False

    def draw(self, surface, font):
        col = BTN_HOVER if self._hover else BTN_NORMAL
        pygame.draw.rect(surface, col, self.rect, border_radius=3)
        pygame.draw.rect(surface, ACCENT, self.rect, 1, border_radius=3)
        if self.checked:
            pygame.draw.line(surface, ACCENT, (self.rect.x + 3, self.rect.y + 9), (self.rect.x + 7, self.rect.y + 14), 2)
            pygame.draw.line(surface, ACCENT, (self.rect.x + 7, self.rect.y + 14), (self.rect.x + 15, self.rect.y + 4), 2)
        txt = font.render(self.label, True, TEXT_BRIGHT)
        surface.blit(txt, (self.rect.right + 8, self.rect.y))


class OptionsMenu:
    _PW_MIN = 880
    _ROW = 32
    _SMALL_GAP = 8
    _SEC_GAP = 14
    _HEADER_H = 28
    _TOP_MARGIN = 20

    def __init__(self, sim):
        self.sim = sim
        self.visible = False
        self.scroll_y = 0
        self.panel = pygame.Rect(0, 0, 0, 0)

        self._tab_order = ["Simulation", "Tuning", "Constants"]
        self._active_tab = "Simulation"
        self._section_order = [
            "Simulation Rates",
            "Runtime Tuning",
            "Preset Profiles",
        ]
        self._tab_sections = {
            "Simulation": ["Simulation Rates"],
            "Tuning": ["Runtime Tuning"],
            "Constants": [
                "Preset Profiles",
            ],
        }
        self._collapsed = {name: False for name in self._section_order}

        self._ui = {}
        self._field_specs = {}
        self._field_texts = {}
        self._visible_field_order = []

        self._const_dirty = False
        self._const_editing = None
        self._cursor_pos = 0
        self._selection_anchor = None
        self._clipboard = ""
        self._mouse_dragging = False

        self._init_field_specs()
        self._sync_real_constant_texts()

    @property
    def editing_active(self):
        return self._const_editing is not None

    def _build(self):
        self._sync_real_constant_texts()

    def _init_field_specs(self):
        self._field_specs = {}
        for name, attr, unit, default in CONST_FIELDS:
            lo, hi = self._infer_limits(attr, default)
            key = f"const:{attr}"
            self._field_specs[key] = {
                "key": key,
                "label": name,
                "unit": unit,
                "section": "Core Physics",
                "type": "float",
                "default": default,
                "lo": lo,
                "hi": hi,
                "apply_attr": attr,
                "is_dummy": False,
            }

        self._tune_fields = [
            {
                "key": "tune:grid_size",
                "label": "Grid Size",
                "unit": "m",
                "section": "Runtime Tuning",
                "type": "float",
                "default": getattr(self.sim, "grid_size", 10.0),
                "lo": 10.0,
                "hi": 50.0,
                "apply_attr": "grid_size",
                "is_dummy": False,
            },
            {
                "key": "tune:inverse_steering",
                "label": "Inverse Steering",
                "unit": "",
                "section": "Runtime Tuning",
                "type": "bool",
                "default": bool(getattr(self.sim, "inverse_steering", False)),
                "lo": None,
                "hi": None,
                "apply_attr": "inverse_steering",
                "is_dummy": False,
            },
            {
                "key": "tune:kb_throttle_ramp_engage",
                "label": "KB Throttle Engage Ramp",
                "unit": "s",
                "section": "Runtime Tuning",
                "type": "float",
                "default": getattr(self.sim, "kb_throttle_ramp_engage", 0.5),
                "lo": 0.05,
                "hi": 10.0,
                "apply_attr": "kb_throttle_ramp_engage",
                "is_dummy": False,
            },
            {
                "key": "tune:kb_throttle_ramp_release",
                "label": "KB Throttle Release Ramp",
                "unit": "s",
                "section": "Runtime Tuning",
                "type": "float",
                "default": getattr(self.sim, "kb_throttle_ramp_release", 0.2),
                "lo": 0.05,
                "hi": 10.0,
                "apply_attr": "kb_throttle_ramp_release",
                "is_dummy": False,
            },
            {
                "key": "tune:kb_brake_ramp_engage",
                "label": "KB Brake Engage Ramp",
                "unit": "s",
                "section": "Runtime Tuning",
                "type": "float",
                "default": getattr(self.sim, "kb_brake_ramp_engage", 0.5),
                "lo": 0.05,
                "hi": 10.0,
                "apply_attr": "kb_brake_ramp_engage",
                "is_dummy": False,
            },
            {
                "key": "tune:kb_brake_ramp_release",
                "label": "KB Brake Release Ramp",
                "unit": "s",
                "section": "Runtime Tuning",
                "type": "float",
                "default": getattr(self.sim, "kb_brake_ramp_release", 0.2),
                "lo": 0.05,
                "hi": 10.0,
                "apply_attr": "kb_brake_ramp_release",
                "is_dummy": False,
            },
            {
                "key": "tune:kb_steer_ramp_engage",
                "label": "KB Steer Engage Ramp",
                "unit": "s",
                "section": "Runtime Tuning",
                "type": "float",
                "default": getattr(self.sim, "kb_steer_ramp_engage", 0.5),
                "lo": 0.05,
                "hi": 10.0,
                "apply_attr": "kb_steer_ramp_engage",
                "is_dummy": False,
            },
            {
                "key": "tune:kb_steer_ramp_release",
                "label": "KB Steer Release Ramp",
                "unit": "s",
                "section": "Runtime Tuning",
                "type": "float",
                "default": getattr(self.sim, "kb_steer_ramp_release", 0.1),
                "lo": 0.05,
                "hi": 10.0,
                "apply_attr": "kb_steer_ramp_release",
                "is_dummy": False,
            },
        ]
        for spec in self._tune_fields:
            self._field_specs[spec["key"]] = spec
            self._field_texts[spec["key"]] = _fmt_const(getattr(self.sim, spec["apply_attr"], spec["default"]))

    def _infer_limits(self, attr, default):
        if attr == "L":
            return 1.0, 6.0
        if attr == "M":
            return 300.0, 8000.0
        if attr in ("F_ENGINE_MAX", "C_BRAKING"):
            return 100.0, 50000.0
        if attr == "C_RR":
            return 0.1, 500.0
        if attr == "C_DRAG":
            return 0.01, 5.0
        lo = 0.000001
        hi = max(float(default) * 10.0, 1.0)
        return lo, hi

    def _sync_real_constant_texts(self):
        for key, spec in self._field_specs.items():
            attr = spec.get("apply_attr")
            if key.startswith("const:") and attr:
                self._field_texts[key] = _fmt_const(getattr(self.sim.car, attr, spec.get("default", 0.0)))
            if key.startswith("tune:") and attr:
                self._field_texts[key] = _fmt_const(getattr(self.sim, attr, spec.get("default", 0.0)))

    def _panel_width(self):
        return max(self._PW_MIN, int(self.sim.screen_w * 0.78))

    def _all_tab_sections_expanded(self):
        for sec in self._tab_sections[self._active_tab]:
            if self._collapsed.get(sec, False):
                return False
        return True

    def _apply_resettable_field(self, key):
        spec = self._field_specs[key]
        attr = spec.get("apply_attr")
        text = self._field_texts.get(key, "")
        if not _field_valid(spec, text):
            return False
        if spec.get("type") == "bool":
            val = _parse_bool(text)
            if val is None:
                return False
            if key.startswith("tune:") and attr:
                setattr(self.sim, attr, bool(val))
                self._field_texts[key] = _fmt_const(bool(val))
                return True
            return False

        val = _parse_float(text)
        if val is None:
            return False
        lo = spec.get("lo")
        hi = spec.get("hi")
        if lo is not None and hi is not None:
            val = _clamp(val, lo, hi)
        if key.startswith("const:") and attr:
            setattr(self.sim.car, attr, val)
            # Apply immediately to the active engine so the next physics step uses new values.
            engine = getattr(self.sim.car, "engine", None)
            if engine is not None and hasattr(engine, attr):
                setattr(engine, attr, val)
            if attr == "C_BRAKING" and engine is not None and hasattr(engine, "C_BRAKE_TORQUE"):
                engine.C_BRAKE_TORQUE = val
            if hasattr(self.sim.car, "_sync_engine_from_vehicle"):
                self.sim.car._sync_engine_from_vehicle()
            self._field_texts[key] = _fmt_const(val)
            return True
        if key.startswith("tune:") and attr:
            setattr(self.sim, attr, val)
            self._field_texts[key] = _fmt_const(val)
            # Keep legacy aliases in sync for compatibility.
            if attr == "kb_throttle_ramp_engage" and hasattr(self.sim, "throttle_ramp"):
                self.sim.throttle_ramp = val
            if attr == "kb_steer_ramp_engage" and hasattr(self.sim, "steer_ramp"):
                self.sim.steer_ramp = val
            return True
        return False

    def _recompute_const_dirty(self):
        dirty = False
        for key, spec in self._field_specs.items():
            if not key.startswith("const:"):
                continue
            txt = self._field_texts.get(key, "")
            if not _field_valid(spec, txt) or not txt.strip():
                continue
            val = _parse_float(txt)
            if val is None:
                continue
            baseline = float(getattr(self.sim.car, spec["apply_attr"], spec.get("default", 0.0)))
            if abs(val - baseline) > 1e-9:
                dirty = True
                break
        self._const_dirty = dirty

    def _active_text(self):
        if self._const_editing is None:
            return ""
        return self._field_texts.get(self._const_editing, "")

    def _set_active_text(self, text):
        if self._const_editing is None:
            return
        self._field_texts[self._const_editing] = text

    def _active_selection_range(self):
        if self._selection_anchor is None:
            return (self._cursor_pos, self._cursor_pos)
        return tuple(sorted((self._cursor_pos, self._selection_anchor)))

    def _has_selection(self):
        a, b = self._active_selection_range()
        return b > a

    def _clear_selection(self):
        self._selection_anchor = None

    def _ensure_cursor_bounds(self):
        txt = self._active_text()
        self._cursor_pos = max(0, min(self._cursor_pos, len(txt)))

    def _delete_selection(self):
        if not self._has_selection():
            return False
        a, b = self._active_selection_range()
        txt = self._active_text()
        self._set_active_text(txt[:a] + txt[b:])
        self._cursor_pos = a
        self._clear_selection()
        return True

    def _select_all(self):
        txt = self._active_text()
        self._selection_anchor = 0
        self._cursor_pos = len(txt)

    def _cursor_index_from_x(self, text, rect, x, font):
        local = x - rect.x - 8
        if local <= 0:
            return 0
        for i in range(len(text) + 1):
            if font.size(text[:i])[0] >= local:
                return i
        return len(text)

    def _copy_selection(self):
        if not self._has_selection():
            return
        a, b = self._active_selection_range()
        text = self._active_text()[a:b]
        if _HAS_PYPERCLIP:
            try:
                pyperclip.copy(text)
                return
            except Exception:
                pass
        self._clipboard = text

    def _cut_selection(self):
        self._copy_selection()
        self._delete_selection()

    def _paste_clipboard(self):
        clip = None
        if _HAS_PYPERCLIP:
            try:
                clip = pyperclip.paste()
            except Exception:
                clip = None
        if clip is None:
            clip = self._clipboard
        if not clip:
            return

        self._delete_selection()
        txt = self._active_text()
        a = self._cursor_pos
        txt = txt[:a] + clip + txt[a:]
        self._set_active_text(txt)
        self._cursor_pos = a + len(clip)

    def _begin_editing(self, field_key, cursor_pos=None):
        self._const_editing = field_key
        text = self._active_text()
        if cursor_pos is None:
            self._cursor_pos = len(text)
        else:
            self._cursor_pos = max(0, min(len(text), cursor_pos))
        self._selection_anchor = self._cursor_pos

    def _commit_active_text(self):
        if self._const_editing is None:
            return
        key = self._const_editing
        spec = self._field_specs.get(key)
        if spec is None:
            return

        txt = self._field_texts.get(key, "")
        if key.startswith("dummy:"):
            return

        if txt.strip() and _field_valid(spec, txt):
            self._apply_resettable_field(key)
            if key.startswith("const:"):
                self._const_dirty = False
        else:
            attr = spec.get("apply_attr")
            if key.startswith("const:") and attr:
                self._field_texts[key] = _fmt_const(getattr(self.sim.car, attr, spec.get("default", 0.0)))
            elif key.startswith("tune:") and attr:
                self._field_texts[key] = _fmt_const(getattr(self.sim, attr, spec.get("default", 0.0)))

    def _next_field_key(self):
        if self._const_editing not in self._visible_field_order:
            return None
        idx = self._visible_field_order.index(self._const_editing)
        if not self._visible_field_order:
            return None
        return self._visible_field_order[(idx + 1) % len(self._visible_field_order)]

    def _handle_text_key(self, event):
        if event.type != pygame.KEYDOWN or not self.editing_active:
            return False

        mods = event.mod if hasattr(event, "mod") else 0
        ctrl = bool(mods & pygame.KMOD_CTRL)
        shift = bool(mods & pygame.KMOD_SHIFT)
        key = event.key

        if ctrl and key == pygame.K_a:
            self._select_all()
            return True
        if ctrl and key == pygame.K_c:
            self._copy_selection()
            return True
        if ctrl and key == pygame.K_x:
            self._cut_selection()
            return True
        if ctrl and key == pygame.K_v:
            self._paste_clipboard()
            return True

        if key == pygame.K_LEFT:
            if shift and self._selection_anchor is None:
                self._selection_anchor = self._cursor_pos
            self._cursor_pos = max(0, self._cursor_pos - 1)
            if not shift:
                self._clear_selection()
            return True
        if key == pygame.K_RIGHT:
            if shift and self._selection_anchor is None:
                self._selection_anchor = self._cursor_pos
            self._cursor_pos = min(len(self._active_text()), self._cursor_pos + 1)
            if not shift:
                self._clear_selection()
            return True
        if key == pygame.K_HOME:
            if shift and self._selection_anchor is None:
                self._selection_anchor = self._cursor_pos
            self._cursor_pos = 0
            if not shift:
                self._clear_selection()
            return True
        if key == pygame.K_END:
            if shift and self._selection_anchor is None:
                self._selection_anchor = self._cursor_pos
            self._cursor_pos = len(self._active_text())
            if not shift:
                self._clear_selection()
            return True

        if key == pygame.K_BACKSPACE:
            if not self._delete_selection() and self._cursor_pos > 0:
                txt = self._active_text()
                self._set_active_text(txt[: self._cursor_pos - 1] + txt[self._cursor_pos :])
                self._cursor_pos -= 1
            self._clear_selection()
            self._recompute_const_dirty()
            return True

        if key == pygame.K_DELETE:
            if not self._delete_selection():
                txt = self._active_text()
                if self._cursor_pos < len(txt):
                    self._set_active_text(txt[: self._cursor_pos] + txt[self._cursor_pos + 1 :])
            self._clear_selection()
            self._recompute_const_dirty()
            return True

        if key in (pygame.K_RETURN, pygame.K_TAB):
            self._commit_active_text()
            if key == pygame.K_TAB and self._const_editing is not None:
                nxt = self._next_field_key()
                if nxt is not None:
                    self._begin_editing(nxt, cursor_pos=0)
            else:
                self._const_editing = None
            self._recompute_const_dirty()
            return True

        if key == pygame.K_ESCAPE:
            self._const_editing = None
            self._sync_real_constant_texts()
            self._clear_selection()
            self._recompute_const_dirty()
            return True

        if event.unicode and len(event.unicode) == 1 and event.unicode.isprintable() and not ctrl:
            txt = self._active_text()
            if self._has_selection():
                self._delete_selection()
                txt = self._active_text()
            if len(txt) < 240:
                self._set_active_text(txt[: self._cursor_pos] + event.unicode + txt[self._cursor_pos :])
                self._cursor_pos += 1
                self._recompute_const_dirty()
            return True

        return False

    def _map_mouse(self, event):
        pos = getattr(event, "pos", None)
        if pos and self.panel.collidepoint(pos):
            return (pos[0] - self.panel.x, pos[1] - self.panel.y + self.scroll_y)
        return None

    def _rebuild_layout(self, viewport_h, pw):
        row = self._ROW
        gap = self._SMALL_GAP
        sec_gap = self._SEC_GAP
        hdr_h = self._HEADER_H

        self._ui = {
            "tab_buttons": [],
            "expand_btn": None,
            "close_top_btn": None,
            "section_headers": {},
            "section_bodies": {},
            "ts_buttons": [],
            "fps_buttons": [],
            "preset_buttons": [],
            "preset_label_pos": None,
            "field_rects": {},
            "reset_btn": None,
            "close_btn": None,
        }
        self._visible_field_order = []

        y = 8
        tab_w = 140
        for i, tab in enumerate(self._tab_order):
            btn = Button(pygame.Rect(16 + i * (tab_w + 8), y, tab_w, 28), tab, toggle=True, active=(tab == self._active_tab))
            self._ui["tab_buttons"].append((tab, btn))

        exp_label = "Collapse All" if self._all_tab_sections_expanded() else "Expand All"
        self._ui["expand_btn"] = Button(pygame.Rect(pw - 274, y, 130, 28), exp_label)
        self._ui["close_top_btn"] = Button(pygame.Rect(pw - 136, y, 120, 28), "Close")
        y += 38

        for sec_name in self._tab_sections[self._active_tab]:
            header = pygame.Rect(12, y, pw - 24, hdr_h)
            self._ui["section_headers"][sec_name] = header
            y += hdr_h
            if self._collapsed.get(sec_name, False):
                y += sec_gap
                continue

            body_top = y
            inner_x = 24
            inner_w = pw - 48

            if sec_name == "Simulation Rates":
                ts_y = y + 6
                for i, (dt, label) in enumerate(TIMESTEP_OPTIONS):
                    r = pygame.Rect(inner_x, ts_y + i * (row + gap), inner_w, row)
                    self._ui["ts_buttons"].append((dt, Button(r, label, toggle=True, active=(dt == self.sim.dt))))
                y = ts_y + len(TIMESTEP_OPTIONS) * (row + gap) + 2

                _fps_y = y + 6
                col_w = max(1, inner_w // len(FPS_OPTIONS))
                for i, fps in enumerate(FPS_OPTIONS):
                    r = pygame.Rect(inner_x + i * col_w, _fps_y, col_w - 6, row)
                    self._ui["fps_buttons"].append((fps, Button(r, str(fps), toggle=True, active=(fps == self.sim.target_fps))))
                y = _fps_y + row + 8

            elif sec_name == "Preset Profiles":
                names = list(getattr(self.sim, "get_preset_names", lambda: tuple())())
                py = y + 8
                self._ui["preset_label_pos"] = (inner_x, py)
                py += 22

                col_w = max(140, (inner_w - 20) // 3)
                for i, name in enumerate(names):
                    col = i % 3
                    row_i = i // 3
                    r = pygame.Rect(inner_x + col * (col_w + 8), py + row_i * (row + gap), col_w, row)
                    self._ui["preset_buttons"].append((name, Button(r, name.replace("_", " "), toggle=True, active=False)))
                rows = (len(names) + 2) // 3
                y = py + rows * (row + gap) + 2

            elif sec_name == "Runtime Tuning":
                label_w = 330
                input_x = inner_x + label_w
                input_w = inner_w - label_w - 10
                rows = [self._field_specs[k] for k in self._field_specs if self._field_specs[k]["section"] == sec_name]
                for spec in rows:
                    r = pygame.Rect(input_x, y + 6, input_w, row)
                    self._ui["field_rects"][spec["key"]] = r
                    self._visible_field_order.append(spec["key"])
                    y += row + gap
                y += 2

            else:
                label_w = 330
                input_x = inner_x + label_w
                input_w = inner_w - label_w - 10
                rows = [self._field_specs[k] for k in self._field_specs if self._field_specs[k]["section"] == sec_name]
                for spec in rows:
                    r = pygame.Rect(input_x, y + 6, input_w, row)
                    self._ui["field_rects"][spec["key"]] = r
                    self._visible_field_order.append(spec["key"])
                    y += row + gap
                y += 2

            self._ui["section_bodies"][sec_name] = {"rect": pygame.Rect(12, body_top, pw - 24, y - body_top)}
            y += sec_gap

        self._ui["reset_btn"] = Button(pygame.Rect((pw // 2) - 220, y + 6, 210, row + 2), "Apply + Reset")
        self._ui["close_btn"] = Button(pygame.Rect((pw // 2) + 10, y + 6, 210, row + 2), "Close")

        y += row + 24

        self._total_height = y + 10

    def toggle(self):
        self.visible = not self.visible
        if self.visible:
            self.scroll_y = 0
            self._const_editing = None
            self._sync_real_constant_texts()
            self._recompute_const_dirty()

    def handle_event(self, event):
        if not self.visible:
            return False

        pw = self._panel_width()
        viewport_h = min(getattr(self, "_total_height", 880), self.sim.screen_h - 60)
        panel_x = max(20, (self.sim.screen_w - pw) // 2)
        panel_y = self._TOP_MARGIN
        self.panel = pygame.Rect(panel_x, panel_y, pw, viewport_h)
        self._rebuild_layout(viewport_h, pw)

        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            pos = getattr(event, "pos", None)
            if pos and not self.panel.collidepoint(pos):
                self.visible = False
                self._const_editing = None
                return True

        if event.type == pygame.MOUSEWHEEL and self.panel.collidepoint(pygame.mouse.get_pos()):
            self.scroll_y -= event.y * 32
            max_scroll = max(0, self._total_height - viewport_h)
            self.scroll_y = max(0, min(self.scroll_y, max_scroll))
            return True

        mapped_pos = self._map_mouse(event)

        for tab, btn in self._ui["tab_buttons"]:
            if btn.handle_event(event, mapped_pos):
                self._active_tab = tab
                self._const_editing = None
                return True

        if self._ui["expand_btn"].handle_event(event, mapped_pos):
            now_expanded = self._all_tab_sections_expanded()
            for sec in self._tab_sections[self._active_tab]:
                self._collapsed[sec] = now_expanded
            return True

        if self._ui["close_top_btn"].handle_event(event, mapped_pos):
            self.visible = False
            self._const_editing = None
            return True

        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1 and mapped_pos:
            for sec_name, rect in self._ui["section_headers"].items():
                if rect.collidepoint(mapped_pos):
                    self._collapsed[sec_name] = not self._collapsed[sec_name]
                    self._const_editing = None
                    return True

        if self._ui["reset_btn"].handle_event(event, mapped_pos):
            if self.editing_active:
                self._commit_active_text()
                self._const_editing = None

            changed = False
            any_applied = False
            for key in self._field_specs:
                if key.startswith("const:"):
                    applied = self._apply_resettable_field(key)
                    changed = applied or changed
                    any_applied = applied or any_applied
                if key.startswith("tune:"):
                    applied = self._apply_resettable_field(key)
                    any_applied = applied or any_applied

            # Ensure latest values are coherent before scenario reset.
            if hasattr(self.sim.car, "_sync_engine_from_vehicle"):
                self.sim.car._sync_engine_from_vehicle()

            if any_applied:
                self.sim.reset_scenario()

            self._recompute_const_dirty()
            return True

        if self._ui["close_btn"].handle_event(event, mapped_pos):
            self.visible = False
            self._const_editing = None
            return True

        if not self._const_dirty:
            for dt, btn in self._ui["ts_buttons"]:
                if btn.handle_event(event, mapped_pos):
                    for dt2, b2 in self._ui["ts_buttons"]:
                        b2.active = (dt2 == dt)
                    if dt != self.sim.dt:
                        self.sim.dt = dt
                        self.sim.reset_scenario()
                    return True

            for fps, btn in self._ui["fps_buttons"]:
                if btn.handle_event(event, mapped_pos):
                    for fps2, b2 in self._ui["fps_buttons"]:
                        b2.active = (fps2 == fps)
                    self.sim.target_fps = fps
                    return True

        # Presets should always be selectable and should always reset on apply.
        for preset_name, btn in self._ui["preset_buttons"]:
            if btn.handle_event(event, mapped_pos):
                applied = False
                if hasattr(self.sim, "apply_preset"):
                    applied = self.sim.apply_preset(preset_name, reset=True)
                if applied:
                    self._sync_real_constant_texts()
                    self._const_dirty = False
                    self._const_editing = None
                    self._clear_selection()
                return True

        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            self._mouse_dragging = True
            if mapped_pos:
                for key, rect in self._ui["field_rects"].items():
                    if rect.collidepoint(mapped_pos):
                        spec = self._field_specs.get(key, {})
                        if spec.get("type") == "bool":
                            # Bool tuning rows are direct-toggle checkboxes, not text-edit fields.
                            current = bool(getattr(self.sim, spec.get("apply_attr", ""), spec.get("default", False)))
                            new_val = not current
                            self._field_texts[key] = _fmt_const(new_val)
                            self._apply_resettable_field(key)
                            self._const_editing = None
                            return True
                        cursor = self._cursor_index_from_x(self._field_texts.get(key, ""), rect, mapped_pos[0], self.sim.font_sm)
                        self._begin_editing(key, cursor)
                        return True
            self._const_editing = None
            return True

        if event.type == pygame.MOUSEBUTTONUP and event.button == 1:
            self._mouse_dragging = False

        if event.type == pygame.MOUSEMOTION and self._mouse_dragging and mapped_pos and self.editing_active:
            rect = self._ui["field_rects"].get(self._const_editing)
            if rect and rect.collidepoint(mapped_pos):
                self._cursor_pos = self._cursor_index_from_x(self._active_text(), rect, mapped_pos[0], self.sim.font_sm)
                return True

        if self._handle_text_key(event):
            return True

        return True

    def draw(self, surface, font_sm, font_md):
        if not self.visible:
            return

        pw = self._panel_width()
        viewport_h = min(getattr(self, "_total_height", 880), surface.get_height() - 60)
        panel_x = max(20, (surface.get_width() - pw) // 2)
        panel_y = self._TOP_MARGIN
        self.panel = pygame.Rect(panel_x, panel_y, pw, viewport_h)
        self._rebuild_layout(viewport_h, pw)

        max_scroll = max(0, self._total_height - viewport_h)
        self.scroll_y = max(0, min(self.scroll_y, max_scroll))

        overlay = pygame.Surface((surface.get_width(), surface.get_height()), pygame.SRCALPHA)
        overlay.fill((0, 0, 0, 120))
        surface.blit(overlay, (0, 0))

        panel_bg = pygame.Surface((pw, viewport_h), pygame.SRCALPHA)
        pygame.draw.rect(panel_bg, PANEL_BG, panel_bg.get_rect(), border_radius=8)
        pygame.draw.rect(panel_bg, ACCENT, panel_bg.get_rect(), 1, border_radius=8)
        surface.blit(panel_bg, self.panel.topleft)

        content = pygame.Surface((pw, self._total_height), pygame.SRCALPHA)

        for _tab, btn in self._ui["tab_buttons"]:
            btn.active = (_tab == self._active_tab)
            btn.draw(content, font_sm)

        expand_btn = self._ui["expand_btn"]
        expand_btn.label = "Collapse All" if self._all_tab_sections_expanded() else "Expand All"
        expand_btn.draw(content, font_sm)
        self._ui["close_top_btn"].draw(content, font_sm)

        y = 46
        for sec_name in self._tab_sections[self._active_tab]:
            hdr = self._ui["section_headers"][sec_name]
            pygame.draw.rect(content, (34, 38, 50), hdr, border_radius=6)
            pygame.draw.rect(content, GRAPH_AXIS, hdr, 1, border_radius=6)
            marker = "+" if self._collapsed.get(sec_name, False) else "-"
            htxt = font_sm.render(f"{marker} {sec_name}", True, TEXT_BRIGHT)
            content.blit(htxt, (hdr.x + 10, hdr.y + 6))
            y = hdr.bottom

            body = self._ui["section_bodies"].get(sec_name)
            if body:
                pygame.draw.rect(content, (23, 26, 35), body["rect"], border_radius=6)
                pygame.draw.rect(content, (56, 61, 78), body["rect"], 1, border_radius=6)
                y = body["rect"].bottom + self._SEC_GAP
            else:
                y += self._SEC_GAP

        if self._active_tab == "Simulation" and not self._collapsed.get("Simulation Rates", False):
            sim_rect = self._ui["section_bodies"].get("Simulation Rates", {}).get("rect")
            if sim_rect:
                _sec_label(content, font_sm, "Timestep", sim_rect.x + 12, sim_rect.y + 6)
                for _, btn in self._ui["ts_buttons"]:
                    btn.disabled = self._const_dirty
                    btn.draw(content, font_sm)

                _sec_label(content, font_sm, "Target FPS", sim_rect.x + 12, sim_rect.y + 6 + len(TIMESTEP_OPTIONS) * (self._ROW + self._SMALL_GAP) + 12)
                for _, btn in self._ui["fps_buttons"]:
                    btn.disabled = self._const_dirty
                    btn.draw(content, font_sm)
                fixed_y = sim_rect.y + 6 + len(TIMESTEP_OPTIONS) * (self._ROW + self._SMALL_GAP) + self._ROW + 22
                _sec_label(content, font_sm, "Longitudinal Model: Long.5 (fixed)", sim_rect.x + 12, fixed_y, color=TEXT_BRIGHT)

        if self._active_tab == "Constants" and not self._collapsed.get("Preset Profiles", False):
            if self._ui.get("preset_label_pos") is not None:
                px, py = self._ui["preset_label_pos"]
                active = getattr(self.sim, "get_preset_label", lambda: "None")()
                _sec_label(content, font_sm, f"Active Preset: {active}", px, py, color=TEXT_BRIGHT)
            for preset_name, btn in self._ui["preset_buttons"]:
                btn.disabled = False
                btn.active = (preset_name == getattr(self.sim, "active_preset", None))
                btn.draw(content, font_sm)

        for key, rect in self._ui["field_rects"].items():
            spec = self._field_specs[key]
            label_x = rect.x - 330
            label_txt = f"{spec['label']}"
            unit_txt = f"[{spec['unit']}]" if spec.get("unit") else ""
            content.blit(font_sm.render(label_txt, True, TEXT_DIM), (label_x, rect.y + 8))
            content.blit(font_sm.render(unit_txt, True, (120, 125, 145)), (label_x + 195, rect.y + 8))

            if spec.get("type") == "bool":
                is_checked = bool(getattr(self.sim, spec.get("apply_attr", ""), spec.get("default", False)))
                box_col = BTN_NORMAL
                border_col = GRAPH_AXIS
                pygame.draw.rect(content, box_col, rect, border_radius=4)
                pygame.draw.rect(content, border_col, rect, 1, border_radius=4)

                cb_size = min(18, rect.height - 8)
                cb_rect = pygame.Rect(rect.x + 8, rect.y + (rect.height - cb_size) // 2, cb_size, cb_size)
                pygame.draw.rect(content, (24, 28, 38), cb_rect, border_radius=3)
                pygame.draw.rect(content, ACCENT, cb_rect, 1, border_radius=3)
                if is_checked:
                    pygame.draw.line(content, ACCENT, (cb_rect.x + 3, cb_rect.y + cb_size // 2), (cb_rect.x + 7, cb_rect.y + cb_size - 4), 2)
                    pygame.draw.line(content, ACCENT, (cb_rect.x + 7, cb_rect.y + cb_size - 4), (cb_rect.x + cb_size - 3, cb_rect.y + 3), 2)

                state_txt = "Enabled" if is_checked else "Disabled"
                state_col = TEXT_BRIGHT if is_checked else TEXT_DIM
                content.blit(font_sm.render(state_txt, True, state_col), (cb_rect.right + 10, rect.y + 8))
                continue

            txt = self._field_texts.get(key, "")
            valid = _field_valid(spec, txt)
            is_active = (self._const_editing == key)
            is_pending = key.startswith("const:") and txt.strip() and valid and abs(float(txt) - float(getattr(self.sim.car, spec["apply_attr"], spec.get("default", 0.0)))) > 1e-9

            if is_active:
                box_col = (30, 40, 60)
                border_col = ACCENT
            elif not valid:
                box_col = (58, 23, 23)
                border_col = (200, 70, 70)
            elif is_pending:
                box_col = (45, 40, 18)
                border_col = (230, 180, 70)
            else:
                box_col = BTN_NORMAL
                border_col = GRAPH_AXIS

            pygame.draw.rect(content, box_col, rect, border_radius=4)
            pygame.draw.rect(content, border_col, rect, 1, border_radius=4)

            display_text = txt
            display_color = TEXT_BRIGHT
            if is_active and self._has_selection() and txt:
                a, b = self._active_selection_range()
                before = txt[:a]
                selected = txt[a:b]
                sel_x = rect.x + 8 + font_sm.size(before)[0]
                sel_w = max(1, font_sm.size(selected)[0])
                pygame.draw.rect(content, (60, 90, 160), (sel_x, rect.y + 6, sel_w, rect.height - 12))

            if not txt:
                display_text = _placeholder_for_field(spec)
                display_color = TEXT_DIM

            content.blit(font_sm.render(display_text, True, display_color), (rect.x + 8, rect.y + 8))

            if is_active:
                self._ensure_cursor_bounds()
                caret_x = rect.x + 8 + font_sm.size((txt or "")[: self._cursor_pos])[0]
                pygame.draw.line(content, TEXT_BRIGHT, (caret_x, rect.y + 6), (caret_x, rect.y + rect.height - 6), 1)

        self._ui["reset_btn"].draw(content, font_md)
        self._ui["close_btn"].draw(content, font_sm)

        surface.blit(content, self.panel.topleft, area=pygame.Rect(0, self.scroll_y, pw, viewport_h))

        if max_scroll > 0:
            bar_w = 6
            bar_h = max(20, int(viewport_h * (viewport_h / self._total_height)))
            bar_x = self.panel.right - 10
            bar_y = self.panel.y + int((self.scroll_y / max_scroll) * (viewport_h - bar_h))
            pygame.draw.rect(surface, GRAPH_AXIS, (bar_x, bar_y, bar_w, bar_h), border_radius=3)