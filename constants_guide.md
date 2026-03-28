# Constants Guide (Model 6)

This document describes how fields in `presets.json` map to the three longitudinal models:

* Long.1 (`src/longitudinal/long1.py`)
* Long.3 (`src/longitudinal/long3.py`)
* Long.5 (`src/longitudinal/long5.py`)

It also records:

* default values defined in each engine,
* the range observed in current presets,
* and practical limits that avoid unstable behavior.

---

## How Preset Fields Are Applied

`simulator.apply_preset()` assigns values from the preset to the active engine when the corresponding attributes exist.

Special cases:

* `C_BRAKE_TORQUE` sets both `engine.C_BRAKE_TORQUE` and `engine.C_BRAKING` (if present).
* `UPSHIFT_RPM` maps to `engine.upshift_rpm`.
* `DOWNSHIFT_RPM` maps to `engine.downshift_rpm`.
* `CHASSIS_COLOR` affects rendering only (RGB clamped to 0–255).

Notes:

* Fields not used by the active engine are ignored.
* Long.1 does not use drivetrain-related fields such as `GEAR_RATIOS` or `TORQUE_CURVE`.

---

## Missing Constants and Engine Defaults

Runtime behavior follows this order:

1. Engine constructor defaults.
2. `apply_preset()` overrides fields present in the selected preset.
3. Any missing field retains its current value.

### Missing Fields in `presets.json`

* Missing fields are not zeroed.
* No error is raised.
* The engine continues using its existing value.

Example (`F_ENGINE_MAX` in Long.1):

* Most presets do not define it.
* Long.1 remains functional because `Long1Engine.__init__` assigns a default value.
* To override it, the field must be explicitly added to the preset.

---

## Engine Switching Behavior

When switching between Long.1, Long.3, and Long.5 via `Simulator.set_longitudinal_engine()`:

1. A new engine instance is created.
2. Constructor defaults are applied.
3. The scenario resets.
4. The preset label is retained, but values are not re-applied.

Effect:

* After switching, the engine runs on its defaults until a preset is applied again.
* This is most noticeable in Long.1, where `F_ENGINE_MAX` comes only from defaults unless specified.

---

## Preset Authoring Guidelines

For consistent behavior across engines:

* Include shared fields in all presets:

  * `M`, `C_RR`, `C_DRAG`, `C_BRAKE_TORQUE`, `CHASSIS_COLOR`
* For Long.1:

  * Add `F_ENGINE_MAX` if control over engine force is required.
* For Long.3 and Long.5:

  * Always define:

    * `R_W`, `FINAL_DRIVE`, `ETA`
    * `RPM_IDLE`, `RPM_REDLINE`
    * `UPSHIFT_RPM`, `DOWNSHIFT_RPM`
    * `GEAR_RATIOS`, `TORQUE_CURVE`
* For Long.5:

  * Include `I_W`, `MU`, `C_T` where traction behavior matters.

---

## Engine Usage Matrix

| Constant                       | Long.1            | Long.3            | Long.5      | Notes                             |
| ------------------------------ | ----------------- | ----------------- | ----------- | --------------------------------- |
| `M`                            | Yes               | Yes               | Yes         | Vehicle mass                      |
| `F_ENGINE_MAX`                 | Yes               | Derived           | Derived     | Not in presets; computed in L3/L5 |
| `C_RR`                         | Yes               | Yes               | Yes         | Rolling resistance                |
| `C_DRAG`                       | Yes               | Yes               | Yes         | Aerodynamic drag                  |
| `C_BRAKING` / `C_BRAKE_TORQUE` | Yes (`C_BRAKING`) | Yes (`C_BRAKING`) | Yes (both)  | Preset uses `C_BRAKE_TORQUE`      |
| `L`                            | Via vehicle       | Via vehicle       | Via vehicle | Not preset-controlled             |
| `I_W`                          | No                | No                | Yes         | Wheel inertia                     |
| `MU`                           | No                | No                | Yes         | Traction limit                    |
| `C_T`                          | No                | No                | Yes         | Slip stiffness                    |
| `R_W`                          | No                | Yes               | Yes         | Wheel radius                      |
| `FINAL_DRIVE`                  | No                | Yes               | Yes         | Gear reduction                    |
| `ETA`                          | No                | Yes               | Yes         | Efficiency                        |
| `RPM_IDLE`                     | No                | Yes               | Yes         | Idle RPM                          |
| `RPM_REDLINE`                  | No                | Yes               | Yes         | Redline                           |
| `UPSHIFT_RPM`                  | No                | Yes               | Yes         | Shift threshold                   |
| `DOWNSHIFT_RPM`                | No                | Yes               | Yes         | Shift threshold                   |
| `GEAR_RATIOS`                  | No                | Yes               | Yes         | Gear map                          |
| `TORQUE_CURVE`                 | No                | Yes               | Yes         | Torque map                        |
| `CHASSIS_COLOR`                | Visual            | Visual            | Visual      | No physics effect                 |

---

## Defaults and Recommended Limits

### Shared Constants

| Constant         | Defaults (L1 / L3 / L5) | Observed range | Recommended range |
| ---------------- | ----------------------- | -------------- | ----------------- |
| `M` (kg)         | 1500 / 1500 / 1500      | 1100 – 2400    | 800 – 3500        |
| `C_RR`           | 13 / 13 / 13            | 0.015 – 18.0   | 0.01 – 30         |
| `C_DRAG`         | 0.43 / 0.43 / 0.43      | 0.28 – 0.55    | 0.10 – 1.20       |
| `C_BRAKE_TORQUE` | 12000 / 12000 / 4500    | 3500 – 9000    | 1500 – 15000      |
| `L` (m)          | ~2.6                    | not set        | 1.8 – 3.6         |

---

### Drivetrain (Long.3, Long.5)

| Constant        | Defaults         | Observed range         | Recommended range         |
| --------------- | ---------------- | ---------------------- | ------------------------- |
| `R_W` (m)       | 0.33             | 0.30 – 0.38            | 0.25 – 0.45               |
| `FINAL_DRIVE`   | 3.42             | 3.2 – 9.0              | 2.5 – 10.0                |
| `ETA`           | 0.7              | 0.80 – 0.95            | 0.65 – 1.00               |
| `RPM_IDLE`      | 800              | 0 – 900                | 0 – 1200                  |
| `RPM_REDLINE`   | 6000             | 5500 – 16000           | 4000 – 18000              |
| `UPSHIFT_RPM`   | 5200             | 4500 – 16000           | ≥ `DOWNSHIFT_RPM` + 200   |
| `DOWNSHIFT_RPM` | 2200             | 0 – 3000               | ≤ `UPSHIFT_RPM` - 200     |
| `GEAR_RATIOS`   | standard 5-speed | defined in all presets | forward gears decreasing  |
| `TORQUE_CURVE`  | piecewise        | defined in all presets | monotonic RPM, torque ≥ 0 |

---

### Slip Model (Long.5)

| Constant | Default | Observed range | Recommended range |
| -------- | ------- | -------------- | ----------------- |
| `I_W`    | 3.0     | 2.0 – 4.5      | 1.0 – 8.0         |
| `MU`     | 1.0     | 0.9 – 1.3      | 0.6 – 1.8         |
| `C_T`    | 30000   | 20000 – 80000  | 10000 – 120000    |

---

### Visual

| Constant        | Default        | Range     |
| --------------- | -------------- | --------- |
| `CHASSIS_COLOR` | (230, 110, 20) | RGB 0–255 |

---

## Preset Observations

Available presets:

* `ECONOMY_COMPACT`
* `SPORTS_CAR`
* `TURBO_PERFORMANCE`
* `SUV_TRUCK`
* `ELECTRIC_EV`
* `MCLAREN_P1`

Notes:

* McLaren P1 is added to validate a mathematical modeling paper of mine for school.
* `MCLAREN_P1` is estimated from publicly available real-world data. As a result, its parameters differ noticeably from the other presets, which are derived and interpreted from Macro Monster’s Guide (2003).
* `MCLAREN_P1` uses `C_RR = 0.015`, significantly lower than other presets.
* `ELECTRIC_EV` uses `RPM_IDLE = 0`, `DOWNSHIFT_RPM = 0`, and a high `FINAL_DRIVE = 9.0`, consistent with a single-speed drivetrain.
* Long.1 ignores most drivetrain fields; add `F_ENGINE_MAX` if stronger preset control is needed.

---

## Tuning Notes

1. For Long.3 and Long.5, tune in sequence:

   * `M` → `R_W` / `FINAL_DRIVE` → `TORQUE_CURVE` → `C_BRAKE_TORQUE`
2. If launch behavior is too aggressive in Long.5:

   * reduce `C_T` before adjusting `MU`
3. Maintain at least ~300 RPM between upshift and downshift thresholds to avoid oscillation
4. If top speed is excessive:

   * reduce high-RPM torque or increase `C_DRAG`
