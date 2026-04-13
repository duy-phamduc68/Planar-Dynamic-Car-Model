# ─────────────────────────────────────────────────────────────────────────────
# constants.py — All named constants for the Car Physics Simulator
# ─────────────────────────────────────────────────────────────────────────────

# ── Physics defaults ──────────────────────────────────────────────────────────
M            = 1500    # kg
F_ENGINE_MAX = 3000    # N
C_RR         = 13.0    # kg/s  (rolling resistance coefficient)
C_DRAG       = 0.43    # kg/m  (aerodynamic drag coefficient)
C_BRAKING    = 20000   # N

# NEW MODEL 7 LATERAL CONSTANTS
# Increased yaw inertia slightly so the car resists whipping around quite as fast
I_Z          = 2500.0  # kg·m² (Yaw moment of inertia)

# STAGGERED GRIP: Making C_AR > C_AF creates inherent "understeer" stability. 
# Once the car spins to 180 degrees, the massive rear grip acts like a parachute, 
# fighting the rotation and forcing the car to stabilize backward.
C_AF         = 85000.0  # N/rad (Front cornering stiffness)
C_AR         = 140000.0 # N/rad (Rear cornering stiffness)

YAW_DAMPING_MULTIPLIER = 2.0

# Lateral stability safeguards
BETA_DAMPING = 0.15
BETA_HARD_LIMIT = 0.8
LOW_SPEED_EPSILON = 0.1

# Skid mark visibility scaling (continuous):
# 1.0 = baseline, <1.0 = less visible/harder to trigger, >1.0 = more visible/easier to trigger
SKID_MARK_VISIBILITY_SCALE = 0.4

# Base slip thresholds for spawning skid patches.
# Longitudinal uses wheel slip ratio (unitless), lateral uses slip angle (radians).
SKID_MARK_LONG_SLIP_THRESHOLD = 0.08
SKID_MARK_LAT_SLIP_THRESHOLD = 0.06

# Intensity gains for converting slip above threshold into visual strength.
SKID_MARK_LONG_GAIN = 2.6
SKID_MARK_LAT_GAIN = 2.0

# ── Geometry (Model 6 & 7) ────────────────────────────────────────────────────
L            = 2.6     # m (Wheelbase)
MAX_STEER    = 35.0    # degrees (Max steering angle lock-to-lock)

PIXELS_PER_METER = 20
GRID_SIZE     = 10.0     # m (Grid spacing for rendering)

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
    
    # NEW MODEL 7 UI FIELDS
    ("Yaw Inertia",  "I_Z",          "kg.m2", I_Z),
    ("Front C_a",    "C_AF",         "N/rad", C_AF),
    ("Rear C_a",     "C_AR",         "N/rad", C_AR),
    ("Beta Damping", "BETA_DAMPING", "",      BETA_DAMPING),
    ("Beta Limit",   "BETA_HARD_LIMIT", "rad", BETA_HARD_LIMIT),
]