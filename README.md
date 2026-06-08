# robotic-manipulator

<img src="assets/robot.gif" alt="Robotic manipulator prototype on a workbench" width="360">

A small robotic manipulator project controlled by an Arduino Nano through a
PCA9685 servo driver. The repository contains Arduino firmware, the kinematic
model, angle-to-PWM mapping, calibration constants, and PCB files. This is my side project, 
and I had a lot of fun working on it. 

## Structure

- `arduino/robot_arm/robot_arm.ino` - main firmware sketch.
- `arduino/robot_arm/kinematics.*` - forward kinematics, inverse kinematics, and angle-to-PWM conversion.
- `arduino/robot_arm/robot_calibration.h` - servo channels, joint limits, HOME pose, and arm dimensions.
- `arduino/robot_arm/pca9685_servo_driver.*` - PCA9685 driver wrapper.
- `pcb/` - KiCad PCB project.

## Hardware

The firmware assumes:

- an Arduino Nano microcontroller,
- a PCA9685 PWM driver on the I2C bus,
- six logical joints `J0-J5`, where `J0-J4` are part of the kinematic model
  and `J5` is the gripper.

Joint indices map to PCA9685 channels defined in
`arduino/robot_arm/robot_calibration.h`.

## Kinematic Model

The manipulator is modeled as an arm with five kinematic joints:

- `J0` - base rotation around the vertical axis,
- `J1-J3` - pitch joints forming the planar arm chain,
- `J4` - local wrist/TCP rotation, independent of TCP position in the current model,
- `J5` - gripper, controlled outside the kinematic model.

TCP position is represented by the `TcpPose` structure:

- `reach_x_m` - reach in the arm plane, in meters,
- `reach_z_m` - TCP height, in meters,
- `yaw_rad` - base rotation, in radians.

Angle convention:

- all angles are in radians,
- `J1-J3 = 0` means that the arm segments point vertically upward,
- positive horizontal reach is computed with `sin(angle)`,
- height is computed with `cos(angle)`,
- `J0` controls the yaw of the whole arm,
- `J4` does not change the TCP position reported by forward kinematics.

Arm dimensions are stored in `JOINT_OFFSETS`:

- `L1` - vertical distance from base `J0` to shoulder `J1`,
- `L2-L4` - planar arm segment lengths,
- `L5` - offset from the last link to the TCP.

## Forward Kinematics

`forwardKinematics()` takes `J0-J4` joint angles and returns a `TcpPose`.
The computation accumulates consecutive pitch angles:

- `pitch_1 = J1`,
- `pitch_2 = J1 + J2`,
- `pitch_3 = J1 + J2 + J3`.

The TCP reach and height are then computed from link lengths `L2-L5`.
In the current model, `L4` and `L5` have the same direction as the final pitch
`J1 + J2 + J3`.

## Inverse Kinematics

The firmware uses geometric IK:

- `inverseKinematicsPositionYaw()` solves TCP position and yaw while using `J3`
  from the HOME pose,
- `inverseKinematicsPositionYawPitch()` solves TCP position and yaw for an
  explicitly provided `J3`.

IK does not solve the full tool orientation. `J3` is part of the input command,
and `J4` remains an independent local rotation.

The IK model behaves like a `Reacher2D` system after rotating the base to the
requested yaw. The solution works in a plane:

1. `J0` is set to `yaw_rad`.
2. The requested `J3` changes the effective length and offset of the distal link.
3. Two elbow configurations are computed for the two-link arm.
4. Configurations outside the geometry or joint limits are rejected.
5. If both configurations are valid, the one closer to the HOME pose is selected.

Possible error statuses include:

- `OutOfReach` - the point is outside the geometric workspace,
- `JointLimitViolation` - a geometric solution exists, but it exceeds joint limits,
- `InvalidInput` - the input contains invalid values, such as `NaN` or infinity.

## Control Rules

The current firmware executes one requested Cartesian command:

1. `TARGET` in `robot_arm.ino` defines the target `TcpPose`, explicit `J3`,
   explicit `J4`, and gripper state.
2. `setup()` initializes the PCA9685 and computes the command for the target.
3. If IK and PWM conversion succeed, `loop()` sends the command to the servos
   exactly once.
4. After sending the command, the program stays idle.

There is currently no trajectory planning, velocity ramping, motion
interpolation, or feedback control loop. Servos receive the target PWM pulse
width directly, so every new target should be tested carefully.

## Angle-To-PWM Mapping

`jointAnglesToPwmUs()` uses calibration values from `JOINT_CALIBRATIONS`.
For each joint, the calibration defines:

- PCA9685 channel,
- `pwm_min_us`, `pwm_zero_us`, `pwm_max_us`,
- `angle_min_rad`, `angle_zero_rad`, `angle_max_rad`.

The mapping is piecewise linear:

- from `angle_min_rad` to `angle_zero_rad`,
- from `angle_zero_rad` to `angle_max_rad`.

This also supports reversed servos, for example when `pwm_min_us` is greater
than `pwm_max_us`. The current `J3` calibration is one such case.

The `J5` gripper is controlled separately:

- `Open` uses `pwm_min_us`,
- `Closed` uses `pwm_max_us`.

## Lessons Learned

Mechanical zero must be established physically, not only in software. Mount
each servo horn so the joint is at its real zero position when the servo is
commanded to `pwm_zero_us`. If the horn is installed one spline off, later
calibration can hide the symptom, but the usable range becomes asymmetric and
the joint can hit a mechanical stop before the software limit is reached.

Angle-to-PWM mapping should be calibrated per joint and checked against both
software and mechanical limits. For each joint, confirm:

- the PWM value for physical zero,
- the PWM values for the minimum and maximum safe angles,
- whether the servo direction is normal or reversed,
- that the configured angle limits reject commands before the mechanism reaches
  a hard stop.

Joint sign conventions must be verified on the real manipulator. Move one joint
at a time by a small positive command and record whether the physical motion
matches the kinematic model. Do the same for a small negative command. This is
especially important before trusting FK/IK results, because a single inverted
joint sign can make a mathematically valid command move the arm in the wrong
direction.

The kinematic model should match the actual mechanism instead of solving a more
general problem than the robot needs. In this project the IK can be kept simple:
`J0` provides independent yaw, `J1-J3` form a planar `Reacher2D` chain, and `J4`
is a user-defined local wrist/TCP rotation rather than a variable inferred from
TCP position. This keeps the firmware easier to debug and reduces the number of
ambiguous IK solutions.

## Startup Safety

Before testing on the manipulator, check:

- whether the target is within `JOINT_CALIBRATIONS` limits,
- whether `J3` and `J4` can cause a mechanical collision,
- whether the gripper PWM values are calibrated correctly,
- whether the unramped move can hit mechanical stops,
- whether the servo power supply is sufficient for a sudden move to the target.

The safest test sequence is to compile first, then test without load, and only
then move with the final tool or object.

## Related Material

For broader teaching material on forward and inverse kinematics, see
[robotics-ai-classes](https://github.com/mbed92/robotics-ai-classes). That
repository contains ROS-based class exercises for FK and IK on a robotic
manipulator.

## Validation

Use Arduino CLI to validate firmware before testing on hardware:

```sh
arduino-cli compile --fqbn arduino:avr:nano arduino/robot_arm
```

For throwaway local builds, specify an explicit build path so generated
artifacts do not end up in the repository:

```sh
arduino-cli compile --fqbn arduino:avr:nano --build-path /tmp/robot_arm_build arduino/robot_arm
```
