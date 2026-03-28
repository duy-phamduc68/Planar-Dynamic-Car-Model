# ─────────────────────────────────────────────────────────────────────────────
# constants.py — All named constants for the Car Physics Simulator
# ─────────────────────────────────────────────────────────────────────────────

# ── Physics defaults ──────────────────────────────────────────────────────────
M            = 1500    # kg
F_ENGINE_MAX = 3000    # N
C_RR         = 13.0    # kg/s  (rolling resistance coefficient)
C_DRAG       = 0.43    # kg/m  (aerodynamic drag coefficient)
C_BRAKING    = 12000   # N

# ── Geometry (Model 6) ────────────────────────────────────────────────────────
L            = 2.6     # m (Wheelbase)
MAX_STEER    = 35.0    # degrees (Max steering angle lock-to-lock)

PIXELS_PER_METER = 20    # Base scale (modified by zoom)
GRID_SIZE        = 10.0  # m (Spacing of the skidpad gridlines)

# ── Colour palette ────────────────────────────────────────────────────────────
ROAD_COLOR     = (40,  42,  45)
ROAD_LINE      = (60,  62,  68)
CAR_BODY       = (230, 110,  20)
CAR_WHEEL      = (20,   20,  20)
CAR_WHEEL_RIM  = (140, 140, 140)

GRAPH_BG       = (20,  20,  28)
GRAPH_GRID     = (50,  50,  60)
GRAPH_AXIS     = (120, 120, 130)
PANEL_BG       = (25,  28,  38, 240)
TEXT_BRIGHT    = (230, 235, 255)
TEXT_DIM       = (100, 105, 120)
TEXT_DISABLED  = (60,  65,  75)
ACCENT         = (80,  170, 255)
ACCENT2        = (255, 140,  50)
BTN_NORMAL     = (45,  50,  68)
BTN_HOVER      = (65,  72,  96)
BTN_ACTIVE     = (80, 170, 255)

# ── Simulation option tables ──────────────────────────────────────────────────
TIMESTEP_OPTIONS = [(0.001, "1 ms"), (0.01, "10 ms"), (0.016, "16 ms")]
FPS_OPTIONS = [60, 120, 144]

THROTTLE_RAMP_DEFAULT = 0.5   # seconds to go 0→1
BRAKE_RAMP_DEFAULT    = 0.3
STEER_RAMP_DEFAULT    = 0.4

# ── Physics constants field table ─────────────────────────────────────────────
CONST_FIELDS = [
    ("Mass",         "M",            "kg",   M),
    ("Wheelbase",    "L",            "m",    L),
    ("Engine Force", "F_ENGINE_MAX", "N",    F_ENGINE_MAX),
    ("Rolling Res.", "C_RR",         "kg/s", C_RR),
    ("Aero Drag",    "C_DRAG",       "kg/m", C_DRAG),
    ("Brake Force",  "C_BRAKING",    "N",    C_BRAKING),
]