# MedBot · Hospital Medicine Delivery Robot

**▶ Live demo:** https://samarahdhanyaa-pixel.github.io/medbot-simulator

Kinematic and dynamic simulation of a differential-drive robot that carries medicine from a hospital pharmacy to a ward and back.
Capstone for **23AID202 Introduction to Robotics**, Amrita School of Engineering, Chennai.

## Features
- Hospital occupancy map with 0.3 m wall inflation
- A* path planning (8-connected) with 5 cm path resampling
- Pure Pursuit path tracking, compared with a PID heading controller
- Forward/inverse kinematics and Euler–Lagrange dynamics
- Empty vs loaded (25 / 35 kg) motor torque and motor sizing
- Live animation, telemetry and plots

## Results (Ward B round trip)
| Round trip | Path | RMS error | Peak torque (empty → loaded) |
|---|---|---|---|
| 60.7 s | 32 m | 6.7 cm | 0.85 → 1.18 N·m |

## Files
- `index.html` — interactive web simulator
- `main_simulation.m` + 6 MATLAB modules — original simulation (run `main_simulation` in MATLAB)

*Dhanyaasri M · CH.AI.U4AID25063*
