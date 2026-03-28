# ─────────────────────────────────────────────────────────────────────────────
# renderer.py — Top-Down 2D Renderer and Simcade HUD
# ─────────────────────────────────────────────────────────────────────────────

import math
import pygame
from constants import (ROAD_COLOR, ROAD_LINE, CAR_BODY, CAR_WHEEL, GRAPH_BG, GRAPH_GRID, 
                       PANEL_BG, TEXT_BRIGHT, TEXT_DIM, TEXT_DISABLED, ACCENT, PIXELS_PER_METER)

def _transform(wx, wy, cx, cy, zoom, sw, sh):
    """World coordinates to screen pixels."""
    ppm = PIXELS_PER_METER * zoom
    sx = int((wx - cx) * ppm + sw / 2)
    sy = int(-(wy - cy) * ppm + sh / 2) # Y is inverted in Pygame
    return sx, sy

def draw_skidpad(surface, cx, cy, zoom, sw, sh, grid_size_m):
    surface.fill(ROAD_COLOR)
    ppm = PIXELS_PER_METER * zoom
    grid_px = max(1.0, grid_size_m * ppm)
    
    offset_x = (cx * ppm - sw/2) % grid_px
    offset_y = (-cy * ppm - sh/2) % grid_px

    for x in range(int(-offset_x), sw, int(grid_px)):
        pygame.draw.line(surface, ROAD_LINE, (x, 0), (x, sh), 1)
    for y in range(int(-offset_y), sh, int(grid_px)):
        pygame.draw.line(surface, ROAD_LINE, (0, y), (sw, y), 1)


def draw_slip_patches(surface, patches, cx, cy, zoom, sw, sh):
    """Draw decaying dark tire slip patches in world-space."""
    if not patches:
        return

    layer = pygame.Surface((sw, sh), pygame.SRCALPHA)
    ppm = PIXELS_PER_METER * zoom
    for patch in patches:
        sx, sy = _transform(patch["x"], patch["y"], cx, cy, zoom, sw, sh)
        radius_m = max(0.02, float(patch.get("radius_m", 0.1)))
        r_px = max(1, int(radius_m * ppm))
        alpha = int(max(0.0, min(255.0, float(patch.get("alpha", 120.0)))))
        darkness = int(max(0, min(80, int(patch.get("darkness", 26)))))
        pygame.draw.circle(layer, (darkness, darkness, darkness, alpha), (sx, sy), r_px)
    surface.blit(layer, (0, 0))


def draw_trc_slip_warning(surface, font_sm, sim_t, scene_h, car):
    """Blinking model5-style TRC/slip warning inside the planar scene."""
    slipping = bool(getattr(car.engine, "is_slipping", False))
    if not slipping:
        return
    if int(sim_t * 15.0) % 2 != 0:
        return

    text = "!! TRC / SLIP !!"
    lbl = font_sm.render(text, True, (255, 255, 80))
    text_w, text_h = lbl.get_size()
    x = 12
    y = max(8, scene_h - text_h - 10)

    bg = pygame.Surface((text_w + 10, text_h + 6), pygame.SRCALPHA)
    bg.fill((14, 14, 20, 185))
    surface.blit(bg, (x - 5, y - 3))
    surface.blit(lbl, (x, y))

def draw_trajectory(surface, car, cx, cy, zoom, sw, sh):
    """Draws the predicted kinematic path for the next 3 seconds."""
    sim_x, sim_y, sim_h = car.x, car.y, car.heading
    v, delta, L = car.v, car.steering_angle, car.L

    # Render trajectory from the bicycle-model center (mid-wheelbase),
    # while dynamics continue integrating the rear-axle state.
    center_x = sim_x + 0.5 * L * math.cos(sim_h)
    center_y = sim_y + 0.5 * L * math.sin(sim_h)
    pts = [_transform(center_x, center_y, cx, cy, zoom, sw, sh)]

    approx_car_len = max(0.1, L * 1.75)
    max_path_len = 3.0 * approx_car_len
    traced_len = 0.0
    
    dt = 0.1
    for _ in range(30):
        prev_x, prev_y = sim_x, sim_y
        prev_h = sim_h
        yaw_rate = (v * math.sin(delta)) / L if abs(delta) > 0.001 else 0.0
        sim_h += yaw_rate * dt
        sim_x += v * math.cos(sim_h) * dt
        sim_y += v * math.sin(sim_h) * dt

        center_prev_x = prev_x + 0.5 * L * math.cos(prev_h)
        center_prev_y = prev_y + 0.5 * L * math.sin(prev_h)
        center_x = sim_x + 0.5 * L * math.cos(sim_h)
        center_y = sim_y + 0.5 * L * math.sin(sim_h)

        step_len = math.hypot(center_x - center_prev_x, center_y - center_prev_y)
        if traced_len + step_len > max_path_len and step_len > 1e-9:
            t = (max_path_len - traced_len) / step_len
            center_x = center_prev_x + (center_x - center_prev_x) * t
            center_y = center_prev_y + (center_y - center_prev_y) * t
            sx, sy = _transform(center_x, center_y, cx, cy, zoom, sw, sh)
            pts.append((sx, sy))
            break

        traced_len += step_len
        sx, sy = _transform(center_x, center_y, cx, cy, zoom, sw, sh)
        pts.append((sx, sy))
        if traced_len >= max_path_len:
            break
        
    if len(pts) > 1:
        pygame.draw.lines(surface, (255, 255, 0), False, pts, 2)

def draw_car_topdown(surface, car, cx, cy, zoom, sw, sh, true_form):
    ppm = PIXELS_PER_METER * zoom
    raw_body_col = getattr(car, "chassis_color", CAR_BODY)
    if isinstance(raw_body_col, (list, tuple)) and len(raw_body_col) == 3:
        try:
            body_col = tuple(max(0, min(255, int(c))) for c in raw_body_col)
        except (TypeError, ValueError):
            body_col = CAR_BODY
    else:
        body_col = CAR_BODY
    
    # Bicycle Model anchors
    rear_x, rear_y = car.x, car.y 
    front_x = rear_x + car.L * math.cos(car.heading)
    front_y = rear_y + car.L * math.sin(car.heading)
    
    rsx, rsy = _transform(rear_x, rear_y, cx, cy, zoom, sw, sh)
    fsx, fsy = _transform(front_x, front_y, cx, cy, zoom, sw, sh)

    if true_form:
        # 1. Bicycle Frame
        pygame.draw.line(surface, body_col, (rsx, rsy), (fsx, fsy), 3)
        
        # 2. Wheels
        w_len = int(0.4 * ppm)
        # Rear Wheel
        dx, dy = math.cos(car.heading) * w_len, -math.sin(car.heading) * w_len
        pygame.draw.line(surface, (255, 50, 50), (rsx - dx, rsy - dy), (rsx + dx, rsy + dy), 6)
        # Front Wheel (Rotated by steering angle!)
        fh = car.heading + car.steering_angle
        fdx, fdy = math.cos(fh) * w_len, -math.sin(fh) * w_len
        pygame.draw.line(surface, (50, 255, 50), (fsx - fdx, fsy - fdy), (fsx + fdx, fsy + fdy), 6)
        
        # 3. ICR (Instantaneous Center of Rotation)
        if abs(car.steering_angle) > 0.01:
            R_rear = car.L / math.tan(car.steering_angle)
            icr_x = rear_x - R_rear * math.sin(car.heading)
            icr_y = rear_y + R_rear * math.cos(car.heading)
            icrx_s, icry_s = _transform(icr_x, icr_y, cx, cy, zoom, sw, sh)
            
            pygame.draw.line(surface, (100, 100, 100), (rsx, rsy), (icrx_s, icry_s), 1)
            pygame.draw.line(surface, (100, 100, 100), (fsx, fsy), (icrx_s, icry_s), 1)
            pygame.draw.circle(surface, (255, 255, 0), (int(icrx_s), int(icry_s)), 5)
            
    else:
        # Standard Chassis Box with symmetric wheel placement and slight front bias
        wheelbase_m = max(0.1, car.L)
        front_overhang_m = 0.35
        rear_overhang_m = 0.25
        body_len_m = wheelbase_m + front_overhang_m + rear_overhang_m

        body_l = max(12, int(body_len_m * ppm))
        body_w = max(8, int(2.0 * ppm))
        car_surf = pygame.Surface((body_l, body_w), pygame.SRCALPHA)
        pygame.draw.rect(car_surf, body_col, (0, 0, body_l, body_w), border_radius=4)

        # Front window marker to clearly indicate heading direction.
        win_w = max(3, int(0.12 * body_l))
        win_h = max(4, int(0.62 * body_w))
        win_x = int(body_l * 0.70) - (win_w // 2)
        win_y = (body_w - win_h) // 2
        pygame.draw.rect(car_surf, (130, 210, 255), (win_x, win_y, win_w, win_h), border_radius=3)
        
        # Draw front wheels slightly turned
        w_l = max(4, int(0.7 * ppm))
        w_w = max(2, int(0.3 * ppm))
        fw_surf = pygame.Surface((w_l, w_w), pygame.SRCALPHA)
        fw_surf.fill(CAR_WHEEL)
        fw_rot = pygame.transform.rotate(fw_surf, math.degrees(car.steering_angle))
        
        wheel_margin_y = max(2, int(0.08 * body_w))
        rear_ax_x = int((rear_overhang_m / body_len_m) * body_l)
        front_ax_x = int(((rear_overhang_m + wheelbase_m) / body_len_m) * body_l)

        rear_x = max(0, min(body_l - w_l, rear_ax_x - w_l // 2))
        front_x = max(0, min(body_l - w_l, front_ax_x - w_l // 2))
        top_y = wheel_margin_y
        bot_y = body_w - w_w - wheel_margin_y

        # Position front wheels by axle-center so rotation doesn't bias right.
        front_cx = front_x + (w_l / 2)
        top_cy = top_y + (w_w / 2)
        bot_cy = bot_y + (w_w / 2)
        car_surf.blit(fw_rot, fw_rot.get_rect(center=(front_cx, top_cy)))
        car_surf.blit(fw_rot, fw_rot.get_rect(center=(front_cx, bot_cy)))
        pygame.draw.rect(car_surf, CAR_WHEEL, (rear_x, top_y, w_l, w_w))
        pygame.draw.rect(car_surf, CAR_WHEEL, (rear_x, bot_y, w_l, w_w))

        rotated_car = pygame.transform.rotate(car_surf, math.degrees(car.heading))
        center_from_rear_m = (body_len_m * 0.5) - rear_overhang_m
        center_factor = center_from_rear_m / wheelbase_m
        r_rect = rotated_car.get_rect(center=(rsx + (fsx-rsx)*center_factor, rsy + (fsy-rsy)*center_factor))
        surface.blit(rotated_car, r_rect)

def draw_hud_planar(surface, rect, font_sm, font_lg, car, throttle, brake, steering, fps, sim_t):
    pygame.draw.rect(surface, GRAPH_BG, rect)
    pygame.draw.line(surface, GRAPH_GRID, (rect.x, rect.y), (rect.right, rect.y), 2)
    
    # --- LEFT SIDE: TELEMETRY TEXT ---
    cx, cy = rect.x + 14, rect.y + 12
    line_h = 18
    hud = car.get_hud_data() if hasattr(car, "get_hud_data") else {}
    def _fmt_or_na(val, unit=""):
        if val is None:
            return "N/A"
        txt = str(val)
        if txt.strip() == "":
            return "N/A"
        if unit:
            return f"{txt} {unit}"
        return txt

    def _tele_color(txt):
        t = str(txt).strip().upper()
        return TEXT_DISABLED if ("N/A" in t or t in ("PLH", "-")) else TEXT_BRIGHT

    def _to_float(val):
        try:
            return float(val)
        except (TypeError, ValueError):
            return None

    def _fmt_float_or_na(val, decimals=2, unit=""):
        f = _to_float(val)
        if f is None:
            return "N/A"
        txt = f"{f:.{decimals}f}"
        if unit:
            return f"{txt} {unit}"
        return txt

    left_lines = [
        ("Velocity", f"{car.v:.2f} m/s"),
        ("Position", f"X {car.x:.2f}  Y {car.y:.2f}"),
        ("Heading", f"{math.degrees(car.heading)%360:.1f} deg"),
        ("Yaw Rate", f"{car.yaw_rate:.3f} rad/s"),
        ("Long. Accel", _fmt_or_na(hud.get("ax"), "m/s^2")),
        ("F Traction", _fmt_or_na(hud.get("f_traction"), "N")),
        ("F Drag", _fmt_or_na(hud.get("f_drag"), "N")),
        ("F RR", _fmt_or_na(hud.get("f_rr"), "N")),
    ]

    right_lines = [
        ("Net Force", _fmt_or_na(hud.get("f_net"), "N")),
        ("Slip Ratio", _fmt_or_na(hud.get("slip_ratio"))),
        ("Wheel w", _fmt_float_or_na(hud.get("wheel_omega"), 2, "rad/s")),
        ("Wheel a", _fmt_or_na(hud.get("wheel_alpha"), "rad/s^2")),
        ("T Drive", _fmt_or_na(hud.get("t_drive"), "Nm")),
        ("T Brake", _fmt_or_na(hud.get("t_brake"), "Nm")),
        ("Gear", _fmt_or_na(hud.get("gear"))),
        ("RPM", _fmt_or_na(hud.get("rpm"))),
    ]

    for i, (label, val) in enumerate(left_lines):
        col = _tele_color(val)
        surface.blit(font_sm.render(f"{label:12}: {val}", True, col), (cx, cy + i * line_h))

    cx2 = cx + 250
    for i, (label, val) in enumerate(right_lines):
        col = _tele_color(val)
        surface.blit(font_sm.render(f"{label:13}: {val}", True, col), (cx2, cy + i * line_h))

    # --- RIGHT SIDE: CONTROLS HUD (horizontal layout) ---
    base_x = max(rect.x + 550, rect.right - 590)
    base_y = rect.y + 12

    # Steering wheel column
    sw_cx, sw_cy, sw_r = base_x + 44, base_y + 56, 40
    pygame.draw.circle(surface, (40, 40, 40), (sw_cx, sw_cy), sw_r, 6)
    steer_vis = -steering * 90
    hand_x = sw_cx + math.sin(math.radians(steer_vis)) * sw_r
    hand_y = sw_cy - math.cos(math.radians(steer_vis)) * sw_r
    pygame.draw.line(surface, ACCENT, (sw_cx, sw_cy), (hand_x, hand_y), 4)
    surface.blit(font_sm.render("STEER", True, TEXT_DIM), (sw_cx - 20, sw_cy + sw_r + 8))

    # Pedals column
    ped_x = base_x + 100
    ped_y = base_y + 6
    bar_w, bar_h = 20, 86
    pygame.draw.rect(surface, (40, 40, 40), (ped_x, ped_y, bar_w, bar_h))
    pygame.draw.rect(surface, (40, 40, 40), (ped_x + 36, ped_y, bar_w, bar_h))
    pygame.draw.rect(surface, (100, 200, 100), (ped_x, ped_y + bar_h - int(throttle * bar_h), bar_w, int(throttle * bar_h)))
    pygame.draw.rect(surface, (200, 100, 100), (ped_x + 36, ped_y + bar_h - int(brake * bar_h), bar_w, int(brake * bar_h)))
    surface.blit(font_sm.render("THR", True, TEXT_DIM), (ped_x - 2, ped_y + bar_h + 8))
    surface.blit(font_sm.render("BRK", True, TEXT_DIM), (ped_x + 34, ped_y + bar_h + 8))

    # Numeric readout column
    read_x = base_x + 180
    read_y = base_y + 14
    steer_deg = math.degrees(car.steering_angle)
    input_lines = [
        f"Throttle: {throttle:.2f}",
        f"Brake:    {brake:.2f}",
        f"Steer:    {steer_deg:+.2f} deg",
    ]
    for i, line in enumerate(input_lines):
        surface.blit(font_sm.render(line, True, TEXT_BRIGHT), (read_x, read_y + i * 18))

    # Big speed readout directly under decimal inputs.
    speed_kmh = car.v * 3.6
    speed_label_y = read_y + len(input_lines) * 18 + 6
    speed_value_y = speed_label_y + 16
    surface.blit(font_sm.render("SPEED", True, TEXT_DIM), (read_x, speed_label_y))
    surface.blit(font_lg.render(f"{speed_kmh:+.1f} km/h", True, TEXT_BRIGHT), (read_x, speed_value_y))

    # Engine + drive-wheel column
    eng_x = base_x + 330
    eng_y = base_y
    gear_val = str(hud.get("gear", "N/A"))
    rpm_val_txt = str(hud.get("rpm", "N/A"))
    shift_val = str(hud.get("shift", "AUTO"))
    is_placeholder = bool(hud.get("placeholder", False))

    hdr_col = TEXT_DISABLED if is_placeholder else TEXT_DIM
    val_col = TEXT_BRIGHT
    shift_col = TEXT_DISABLED if is_placeholder else ACCENT

    # RPM tachometer gauge (model3-inspired: redline starts at ~90% band)
    rpm_val = _to_float(hud.get("rpm"))
    rpm_idle = _to_float(hud.get("rpm_idle"))
    rpm_redline = _to_float(hud.get("rpm_redline"))
    if rpm_idle is None:
        rpm_idle = 800.0
    if rpm_redline is None or rpm_redline <= rpm_idle:
        rpm_redline = rpm_idle + 1.0
    rpm_available = rpm_val is not None

    g_cx, g_cy, g_r = eng_x + 58, eng_y + 44, 34
    ring_col = (90, 95, 110) if rpm_available else (58, 62, 74)
    pygame.draw.circle(surface, (36, 38, 48), (g_cx, g_cy), g_r + 4, 4)
    pygame.draw.circle(surface, (20, 22, 30), (g_cx, g_cy), g_r - 10)

    start_deg = 210.0
    span_deg = 240.0
    for i in range(11):
        pct = i / 10.0
        deg = start_deg - pct * span_deg
        a = math.radians(deg)
        x0 = g_cx + math.cos(a) * (g_r - 2)
        y0 = g_cy - math.sin(a) * (g_r - 2)
        x1 = g_cx + math.cos(a) * (g_r + 5)
        y1 = g_cy - math.sin(a) * (g_r + 5)
        if rpm_available and pct >= 0.9:
            tick_col = (230, 90, 90)
        else:
            tick_col = ring_col
        pygame.draw.line(surface, tick_col, (x0, y0), (x1, y1), 2)

    if rpm_available:
        rpm_pct = max(0.0, min(1.0, (rpm_val - rpm_idle) / (rpm_redline - rpm_idle)))
        needle_deg = start_deg - rpm_pct * span_deg
        na = math.radians(needle_deg)
        nx = g_cx + math.cos(na) * (g_r - 10)
        ny = g_cy - math.sin(na) * (g_r - 10)
        pygame.draw.line(surface, ACCENT, (g_cx, g_cy), (nx, ny), 3)
        pygame.draw.circle(surface, ACCENT, (g_cx, g_cy), 3)

    rpm_center = rpm_val_txt if rpm_available else "N/A"
    rpm_col = val_col if rpm_available else TEXT_DISABLED
    surface.blit(font_sm.render("RPM", True, hdr_col), (g_cx - 14, g_cy + 10))
    surface.blit(font_sm.render(rpm_center, True, rpm_col), (g_cx - 18, g_cy - 10))
    redline_col = (230, 90, 90) if rpm_available else TEXT_DISABLED
    surface.blit(font_sm.render(f"RED {int(rpm_redline)}", True, redline_col), (eng_x + 4, eng_y + 84))

    gear_is_na = str(gear_val).strip().upper() in ("N/A", "PLH", "-")
    surface.blit(font_sm.render(f"GEAR: {gear_val}", True, TEXT_DISABLED if gear_is_na else val_col), (eng_x + 4, eng_y + 102))
    surface.blit(font_sm.render(f"SHIFT: {shift_val}", True, shift_col), (eng_x + 4, eng_y + 120))

    # Drive wheel visual (active for Long.3 / Long.5)
    wheel_cx, wheel_cy, wheel_r = eng_x + 190, eng_y + 52, 34
    show_drive_wheel = getattr(car, "engine_id", 1) in (3, 5)
    wheel_col = ACCENT if show_drive_wheel else TEXT_DISABLED
    drive_ring_col = (75, 75, 85) if show_drive_wheel else (55, 55, 65)

    pygame.draw.circle(surface, drive_ring_col, (wheel_cx, wheel_cy), wheel_r, 6)
    pygame.draw.circle(surface, (28, 28, 36), (wheel_cx, wheel_cy), max(2, wheel_r - 10))

    wheel_radius_m = 0.31
    if hud.get("wheel_radius_m") is not None:
        try:
            wheel_radius_m = float(hud.get("wheel_radius_m"))
        except (TypeError, ValueError):
            wheel_radius_m = 0.31

    omega = None
    if hud.get("wheel_omega") is not None:
        try:
            omega = abs(float(hud.get("wheel_omega")))
        except (TypeError, ValueError):
            omega = None
    if omega is None:
        omega = abs(car.v) / max(0.05, wheel_radius_m)

    wheel_angle = None
    if hud.get("wheel_angle") is not None:
        try:
            wheel_angle = float(hud.get("wheel_angle"))
        except (TypeError, ValueError):
            wheel_angle = None
    if wheel_angle is None:
        wheel_angle = omega * sim_t

    rim_angle = (math.degrees(wheel_angle) % 360.0) if show_drive_wheel else 0.0
    for spoke in (0.0, 90.0, 180.0, 270.0):
        a = math.radians(rim_angle + spoke)
        sx = wheel_cx + math.cos(a) * (wheel_r - 8)
        sy = wheel_cy + math.sin(a) * (wheel_r - 8)
        pygame.draw.line(surface, wheel_col, (wheel_cx, wheel_cy), (sx, sy), 2)

    wtxt = "DRIVE WHL" if show_drive_wheel else "DRIVE WHL \n (L3/L5)"
    surface.blit(font_sm.render(wtxt, True, TEXT_DIM if not show_drive_wheel else wheel_col), (wheel_cx - 32, wheel_cy + wheel_r + 8))