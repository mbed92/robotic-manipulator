# Copilot Instructions

## Build, test, lint
- No build, lint, or automated test commands are documented in this repository.
- Firmware sketches are Arduino `.ino` files under `arduino/robot_arm/` and `arduino/robot_arm/scripts/`; use your local Arduino tooling to build/upload the specific sketch you are working on.
- Manual verification helpers:
  - `arduino/robot_arm/scripts/kinematics_debug/kinematics_debug.ino` runs FK/IK and PWM interpolation checks via Serial output.
  - `arduino/robot_arm/scripts/go_zero/go_zero.ino` drives joints to configured zero positions and validates FK/IK consistency.

## High-level architecture
- The core firmware lives in `arduino/robot_arm/`:
  - `robot_arm.ino` is a standalone Cartesian trajectory demo that precomputes waypoints, solves IK, converts to PWM, and drives the arm via the PCA9685.
  - `kinematics.*` implements FK/IK for a 5-DOF arm (J0–J4) plus joint-angle-to-PWM conversion; results are returned as `KinematicsResult` with status codes.
  - `pca9685_servo_driver.*` wraps the Adafruit PWM servo driver for low-level PWM writes.
  - `robot_calibration.h` centralizes joint calibration (PWM min/zero/max, angle limits) and link offsets used by both kinematics and motion demos.
- `arduino/robot_arm/scripts/` contains focused sketches for validation and demos (dance, zeroing, kinematics debug) that reuse the shared kinematics/driver/calibration code.
- `pcb/` contains KiCad hardware design files and should stay decoupled from firmware changes.

## Key conventions
- Joint indexing follows the physical arm: J0 base, J1–J3 planar arm, J4 TCP rotation, J5 gripper; only J0–J4 participate in FK/IK (J5 is gripper only).
- Units are consistent across kinematics: meters for lengths/positions, radians for angles, microseconds for PWM.
- Safety-sensitive calibration lives in `robot_calibration.h`; do not change joint limits, offsets, or PWM ranges without hardware validation.
- Kinematics functions return `KinematicsResult<T>` with explicit `KinematicsStatus` error codes; callers check status and avoid silent fallbacks.
- Motion demos typically write PWM per-joint through `Pca9685ServoDriver::setServoUs()`; use `setServoUsImmediate()` if you need simultaneous channel updates.
