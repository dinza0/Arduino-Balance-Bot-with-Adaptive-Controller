# Adaptive Self-Balancing Robot

![CAD model of the adaptive self-balancing robot](assets/balance_bot_cad.png)

## Overview

This repository contains the embedded Arduino code for a two-wheeled self-balancing robot using an MPU6050 IMU, Dynamixel MX-28AR actuators, and an adaptive inverse-dynamics balancing controller. The robot is modeled as a planar wheeled inverted pendulum, and the controller stabilizes the body pitch angle by commanding wheel velocity through the Dynamixel motors.

The final controller uses a tuned fixed inverse-dynamics controller as the baseline and adds an adaptive gain-update layer that modifies the effective pitch gains online. The adaptive controller is intended to improve robustness to modeling uncertainty, payload changes, IMU trim error, friction, and differences between the CAD-based model and the physical robot.

## Main Features

- MPU6050 IMU-based body tilt estimation
- Dynamixel MX-28AR Protocol 2.0 velocity-mode control
- CAD-based lumped inverse-dynamics model
- Fixed inverse-dynamics balancing controller
- Online adaptive gain tuning for pitch control
- Fractional adaptive corrections for \(K_p\) and \(K_d\)
- Adaptive gating to prevent learning during fall events or actuator saturation
- Safety pause / re-arm behavior
- Serial CSV logging for MATLAB plotting and performance analysis
- Autonomous plug-and-play adaptive mode for untethered testing

## Hardware

The robot uses the following main hardware components:

- Arduino Mega or compatible board
- MPU6050 IMU
- Dynamixel MX-28AR motors using Protocol 2.0
- RS-485 half-duplex communication module, such as MAX485
- Two wheels, tested with 65 mm and 90 mm diameters
- Custom CAD-designed chassis
- External motor power supply for the Dynamixels

## Coordinate Definitions

The main variables used in the controller are:

| Symbol | Meaning |
|---|---|
| \(\theta\) | Body pitch angle measured by the IMU |
| \(\dot{\theta}\) | Body pitch rate measured by the IMU gyro |
| \(\phi\) | Average wheel angle |
| \(\dot{\phi}\) | Average wheel angular velocity |
| \(\omega_{cmd}\) | Commanded wheel velocity sent to the Dynamixels |
| \(\theta_{ref}^{eff}\) | Effective upright reference including trim |

The IMU angle is the balance angle. The wheel angle \(\phi\) is not used as the body tilt angle. It is used for logging and for estimating base velocity.

## Controller Summary

### Fixed Inverse-Dynamics Controller

The fixed controller computes a desired pitch angular acceleration using pitch error and pitch-rate error:

```math
\ddot{\theta}_{cmd} = K_p e_\theta + K_d e_{\dot{\theta}} + K_i z_\theta
```

where

```math
e_\theta = \theta_{ref}^{eff} - \theta,
\qquad
 e_{\dot{\theta}} = 0 - \dot{\theta}.
```

The effective reference is

```math
\theta_{ref}^{eff} = \theta_{ref} + \theta_{trim}.
```

The inverse-dynamics model maps the desired pitch acceleration to a wheel-speed command:

```math
\omega_{cmd} = f_{ID}(\theta, \dot{\theta}, \dot{x}, \ddot{\theta}_{cmd}).
```

### Adaptive Gain Update

The adaptive controller modifies the fixed baseline gains online:

```math
K_p^{eff}=K_{p,b}(1+\Delta K_p)
```

```math
K_d^{eff}=K_{d,b}(1+\Delta K_d)
```

where \(\Delta K_p\) and \(\Delta K_d\) are fractional corrections. For example, \(\Delta K_p=0.05\) means a 5% increase in the effective proportional gain.

The reference model is

```math
\ddot{\theta}_m = K_{p,b}(\theta_{ref}^{eff}-\theta_m) + K_{d,b}(0-\dot{\theta}_m).
```

The model-following error is

```math
e_m =
\begin{bmatrix}
\theta - \theta_m \\
\dot{\theta} - \dot{\theta}_m
\end{bmatrix}.
```

The adaptive signal is computed as

```math
\sigma = B^T P e_m.
```

In code, the required coefficients are computed directly from the Lyapunov weighting:

```math
A_m^T P + P A_m = -Q,
\qquad
Q = \begin{bmatrix}10 & 0 \\ 0 & 1\end{bmatrix}.
```

The adaptive updates are normalized and bounded:

```math
\dot{\Delta K}_p
=
\frac{-\gamma_{K_p}\sigma\Phi_p}{N}
-
\lambda_{K_p}\Delta K_p
```

```math
\dot{\Delta K}_d
=
\frac{-\gamma_{K_d}\sigma\Phi_d}{N}
-
\lambda_{K_d}\Delta K_d
```

with

```math
\Phi_p = K_{p,b}e_\theta,
\qquad
\Phi_d = K_{d,b}e_{\dot{\theta}},
```

```math
N = N_0 + \Phi_p^2 + \Phi_d^2.
```

## Adaptive Safety Logic

Adaptation is only allowed when the robot is in a valid balancing region. The adaptive update is gated using:

- IMU read validity
- controller armed state
- safety pause state
- upright angle range
- motor command saturation check
- model-following deadzone

This prevents the robot from learning from bad data during large fall events, IMU glitches, or motor saturation.

## Code Structure

Typical important sections in the Arduino file are:

| Section | Purpose |
|---|---|
| IMU setup and update | Reads MPU6050 and estimates \(\theta\), \(\dot{\theta}\) |
| Dynamixel communication | Raw Protocol 2.0 read/write functions |
| CAD model constants | Lumped mass, COM, inertia, wheel radius, and gravity terms |
| Fixed controller | Baseline inverse-dynamics balance controller |
| Adaptive controller | Online update of \(\Delta K_p\), \(\Delta K_d\) |
| Safety logic | Tip detection, torque-off, and re-arm behavior |
| CSV logging | Serial output for MATLAB plotting |
| Autonomous mode | Plug-and-play adaptive balancing without serial commands |

## Quick Start

1. Connect the MPU6050 and Dynamixel RS-485 interface.
2. Upload the Arduino sketch to the board.
3. Power the Dynamixel motors from the external supply.
4. Place or hold the robot near upright.
5. If using the autonomous version, the robot waits until it is upright and still, then arms automatically.
6. For serial-test versions, use the Serial Monitor to select modes.

## Serial Commands

The test-framework versions of the code may include the following commands:

| Command | Function |
|---|---|
| `p` | Print state / diagnostics |
| `t` | Dynamixel step test |
| `c` | Fixed inverse-dynamics controller |
| `a` | Adaptive inverse-dynamics controller |
| `l` | Toggle CSV logging |
| `d` | Toggle verbose/raw IMU output |
| `x` | Disable/reset adaptive gains |
| `r` | Reset controller states |
| `z` | Zero wheel position/integrators |
| `s` or `0` | Stop / torque off |

The autonomous plug-and-play version does not require serial commands to start balancing.

## CSV Logging and Plotting

The code can output CSV-formatted serial data for MATLAB plotting. Important logged signals include:

- `time_ms`
- `theta`
- `theta_ref`
- `theta_error`
- `thetadot`
- `omega_cmd`
- `last_wcmd`
- `dKp_pct`
- `dKd_pct`
- `Kp_eff`
- `Kd_eff`
- `adapt_gate`
- `safety_pause`

Recommended plots for performance evaluation:

- pitch angle tracking versus time
- pitch error versus time
- motor command with saturation limits
- adaptive gain corrections versus time
- fixed versus adaptive controller comparison
- payload-added adaptive response

## Tuning Notes

The final balancing performance is sensitive to:

- IMU zero offset
- `theta_ref_trim_rad`
- wheel radius
- motor command sign
- Dynamixel command rate
- maximum wheel-speed command
- proportional and derivative gains
- adaptive rate gains
- adaptive deadzone and saturation gate

The trim angle should be tuned separately from the IMU offset. The IMU offset defines the measured upright angle, while `theta_ref_trim_rad` defines the balance point preferred by the controller.

## Known Limitations

The robot is limited by the available wheel speed and motor response. When the wheel-speed command reaches saturation, increasing the gains or adaptive corrections cannot produce more corrective wheel motion. The adaptive controller improves local robustness around upright but does not guarantee recovery from large tilt angles.

The controller also assumes planar motion. Yaw dynamics, lateral slip, asymmetric wheel friction, battery-voltage variation, and floor-contact changes are not explicitly modeled.

## Suggested Demonstration

A useful demonstration is to compare fixed and adaptive balancing with and without a small payload placed near the top of the robot. The payload should be centered above the wheel axle so that it primarily changes the mass, inertia, and center-of-mass height rather than introducing a large forward/backward trim bias.

Recommended payload range:

- start with 30 g
- typical demonstration target: 50 g
- upper test range: 75 g

## Repository Contents

A typical repository layout is:

```text
.
├── README.md
├── assets/
│   └── balance_bot_cad.png
├── Arduino/
│   └── SelfBalancingBot_AutonomousAdaptive_PlugAndPlay.ino
├── MATLAB/
│   └── arduino_adaptive_plotter_white_deg.m
└── docs/
    └── report_figures/
```

## Notes

This project was developed as a hardware adaptive-control demonstration. The adaptive controller should be interpreted as a bounded robustness layer around a working fixed inverse-dynamics controller, not as a replacement for calibration, mechanical balance, or actuator capability.
