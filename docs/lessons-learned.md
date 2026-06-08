# Lessons Learned

This project is hardware-adjacent, so the useful lessons are mostly about
keeping the software model honest against the physical manipulator.

## Servo Zero Is Mechanical First

`1500 us` does not automatically mean a good-looking robot pose. It is only the
electrical midpoint of a typical hobby servo command range.

Mechanical zero should be established by mounting each servo horn so the joint
is physically at its intended zero position when the servo is commanded to its
zero PWM. Only after that should `pwm_zero_us` be calibrated in
`arduino/robot_arm/robot_calibration.h`.

If the horn is installed one spline off, software calibration can hide the
symptom, but the usable range becomes asymmetric. The joint can then hit a
mechanical stop before the configured software limit is reached.

## Servo Range Is Not Always 180 Degrees

The real MG996R range does not have to match a full 180 degrees. A commonly
referenced MG996R datasheet describes about 120 degrees of movement, which is
roughly 60 degrees in each direction from center:
[MG996R datasheet](https://mm.digikey.com/Volume0/opasdata/d220001/medias/docus/204/MG996R.pdf).

That is why the current calibration uses conservative joint limits of about
`-60` to `+60` degrees:

```cpp
constexpr float JOINT_ANGLE_MIN_RAD = -1.0472f; // -60 degrees
constexpr float JOINT_ANGLE_MAX_RAD = 1.0472f; // 60 degrees
```

This is a practical safety choice, not just a kinematic preference. It keeps
software commands closer to the range the servos and mechanism can actually
reach.

## Calibrate Every Joint Separately

Each joint needs its own calibration pass:

- physical zero,
- minimum safe PWM and angle,
- maximum safe PWM and angle,
- sign direction,
- mechanical-stop margin.

Do not assume that all servos are mounted with the same direction. In this
project, `J3` is reversed in `JOINT_CALIBRATIONS`:

```cpp
{3, 2000, 1500, 1000, JOINT_ANGLE_MIN_RAD, JOINT_ANGLE_ZERO_RAD, JOINT_ANGLE_MAX_RAD},
```

That is intentionally different from the normal ordering used by most other
joints:

```cpp
{0, 1000, 1500, 2000, JOINT_ANGLE_MIN_RAD, JOINT_ANGLE_ZERO_RAD, JOINT_ANGLE_MAX_RAD},
```

The reversed PWM order is clearer and safer than compensating for the inversion
somewhere else in the kinematic code.

## Verify IK With FK

Inverse kinematics should be debugged by comparing the requested IK target with
the forward-kinematics result of the computed joint angles.

The firmware already does this in `robot_arm.ino`: after solving IK and
converting the command, it runs `forwardKinematics(angles)` and prints the
model TCP pose. That makes it easier to catch:

- sign errors,
- wrong link lengths,
- joint-limit surprises,
- IK solutions that are mathematically valid but not the solution expected on
  the real arm.

This is one of the most useful checks before trusting a new target on hardware.

## Keep The Kinematic Model As Simple As The Robot

The kinematic model should match the actual mechanism instead of solving a more
general problem than the robot needs.

In this project, the IK can stay simple:

- `J0` provides independent yaw,
- `J1-J3` form a planar `Reacher2D` chain,
- `J4` is a user-defined local wrist/TCP rotation,
- `J5` is the gripper and is controlled outside the kinematic model.

This keeps the firmware easier to debug and reduces the number of ambiguous IK
solutions.

## Power Wiring Is Part Of The Control System

Common ground and separated power domains are not details. They are conditions
for the robot to work reliably.

The PCB wiring follows this split:

- Arduino powers the PCA9685 logic side through VCC/VDD,
- the external PSU powers PCA9685 `V+` and the servos,
- Arduino ground, PSU ground, PCA9685 ground, and servo grounds are connected
  together.

The Arduino should not power the servos. The servo rail can draw high current
and create voltage drops or resets that should not be imposed on the logic
supply. At the same time, all grounds must be common or the PWM signal has no
shared electrical reference.
