# Planar Dynamic Car Model

Main reference: [Macro Monster's Car Physics Guide](https://www.asawicki.info/Mirror/Car%20Physics%20for%20Games/Car%20Physics%20for%20Games.html)

Check out the learning journey on my blog: [yuk068.github.io](https://yuk068.github.io/)

**Model 1-5** (Longitudinal Simulators) are in this repository: [duy-phamduc68/Longitudinal-Car-Physics](https://github.com/duy-phamduc68/Longitudinal-Car-Physics)

**Model 6** (Planar Kinematic Car Model) is in this repository: [duy-phamduc68/Planar-Kinematic-Car-Model](https://github.com/duy-phamduc68/Planar-Kinematic-Car-Model)

I try to break down each model both mathematically (continuous math) and implement them in code.

This repository contains code for **Model 7** of the roadmap.

## Model 7: High-Speed Lateral Tire Model (2D)

![simulator 7 thumbnail 1](/media/simulator7-1.png)

![simulator 7 thumbnail 2](/media/simulator7-2.png)

[Technical Breakdown](https://yuk068.github.io/2026/03/28/car-physics-model7)

To be honest, I've tuned the default settings and reworked this model to get it to be more video-game-fun, all constants are customizable if you want to try to make it more realistic, but I've decided to not go deeper into realism for this model.

### Input Guide

Click on the grid tiles to toggle their highlighted state.

#### Keyboard Controls

| Key(s)                            | Action                                        |
| --------------------------------- | --------------------------------------------- |
| `W` / `Up`                        | Throttle                                      |
| `Space` / `Down` / `S` / `LShift` | Brake                                         |
| `A` / `Left`                      | Steer left                                    |
| `D` / `Right`                     | Steer right                                   |
| `J`                               | Downshift                                     |
| `K`                               | Upshift                                       |
| `1`                               | Toggle auto shift                             |
| `2`                               | Zoom in                                       |
| `3`                               | Zoom out                                      |
| `4`                               | Toggle true form                              |
| `Q`                               | Timer cycle (idle → running → stopped → idle) |
| `R`                               | Reset scenario                                |
| `F`                               | Toggle fullscreen                             |
| `Esc`                             | Open/close options                            |

#### Xbox Controller (XInput)

| Control           | Action                                        |
| ----------------- | --------------------------------------------- |
| `RT`              | Throttle                                      |
| `LT`              | Brake                                         |
| `Left Stick X`    | Steering                                      |
| `X` / `LB`        | Downshift                                     |
| `B` / `RB`        | Upshift                                       |
| `A`               | Toggle auto shift                             |
| `D-pad Left`      | Timer cycle (idle → running → stopped → idle) |
| `D-pad Right`     | Toggle true form                              |
| `Back` / `Select` | Reset scenario                                |
| `D-pad Up`        | Zoom in                                       |
| `D-pad Down`      | Zoom out                                      |
| `RS`              | Toggle fullscreen                             |
| `Start`           | Open/close options                            |

Completed models:

- [Model 1: Longitudinal Point Mass (1D)](https://github.com/duy-phamduc68/Longitudinal-Car-Physics#model-1-longitudinal-point-mass-1d)
- [Model 2: Load Transfer Without Traction Limits (1D)](https://github.com/duy-phamduc68/Longitudinal-Car-Physics#model-2-load-transfer-without-traction-limits-1d)
- [Model 3: Engine Torque + Gearing without Slip (1D)](https://github.com/duy-phamduc68/Longitudinal-Car-Physics#model-3-engine-torque--gearing-without-slip-1d)
- [Model 4: Wheel Rotational Dynamics (1D)](https://github.com/duy-phamduc68/Longitudinal-Car-Physics#model-4-wheel-rotational-dynamics-1d)
- [Model 5: Slip Ratio + Traction Curve (1D)](https://github.com/duy-phamduc68/Longitudinal-Car-Physics#model-5-slip-ratio--traction-curve-1d)
- [Model 6: Low-Speed Kinematic Turning (2D)](https://github.com/duy-phamduc68/Planar-Kinematic-Car-Model#model-6-low-speed-kinematic-turning-2d)
- [Model 7: High-Speed Lateral Tire Model (2D)](#model-7-high-speed-lateral-tire-model-2d)