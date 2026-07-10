# robotic-manipulator

<img src="assets/robot.gif" alt="Robotic manipulator prototype on a workbench" width="360">

A small robotic manipulator project controlled by an Arduino Nano through a
PCA9685 servo driver. The repository contains Arduino firmware, the kinematic
model, angle-to-PWM mapping, calibration constants, and PCB files. This is my side project, 
and I had a lot of fun working on it. 

## Structure

- [`arduino/robot_arm/robot_arm.ino`](arduino/robot_arm/robot_arm.ino) - main firmware sketch.
- [`arduino/robot_arm/kinematics.h`](arduino/robot_arm/kinematics.h) / [`kinematics.cpp`](arduino/robot_arm/kinematics.cpp) - forward kinematics, Jacobian IK step generation, and angle-to-PWM conversion.
- [`arduino/robot_arm/robot_calibration.h`](arduino/robot_arm/robot_calibration.h) - servo channels, joint limits, HOME pose, and arm dimensions.
- [`arduino/robot_arm/pca9685_servo_driver.h`](arduino/robot_arm/pca9685_servo_driver.h) / [`pca9685_servo_driver.cpp`](arduino/robot_arm/pca9685_servo_driver.cpp) - PCA9685 driver wrapper.
- [`docs/bom.md`](docs/bom.md) - V1 prototype bill of materials.
- [`docs/lessons-learned.md`](docs/lessons-learned.md) - practical notes from calibration, IK debugging, and wiring.
- [`docs/TODO.md`](docs/TODO.md) - project task notes and planned improvements.
- [`pcb/`](pcb/) - KiCad PCB project.

## Hardware

The firmware assumes:

- an Arduino Nano microcontroller,
- a PCA9685 PWM driver on the I2C bus,
- six logical joints `J0-J5`, where `J0-J4` are part of the kinematic model
  and `J5` is the gripper.

The mechanical design and servo selection are based on material from
[Elektroweb](https://elektroweb.pl).

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

The firmware uses an open-loop Jacobian IK trajectory generator based on the
commanded joint state:

- `computeJacobianIkStep()` is a pure numerical function. It computes one IK
  step from the current commanded joint angles, target, and configuration. It
  does not write PWM, wait, or access hardware.
- `moveToTargetJacobianIk()` in `robot_arm.ino` is the blocking hardware-facing
  trajectory generator. It calls the step function, checks limits, converts the
  next angles to PWM, writes the PCA9685 channels, waits until the next control
  period, and updates the commanded joint state.
- Before the planar DLS loop, the generator moves the explicitly commanded
  `J0`, `J3`, and `J4` axes toward their targets with the same fixed period and
  velocity limits. This gives the local Jacobian solve the intended `J3`
  geometry instead of starting from the vertical HOME singularity.

The IK task is:

- TCP reach,
- TCP height,
- base yaw,
- explicit `J3`,
- explicit `J4`.

Only the coupled planar subproblem `[reach, height, J3] <-> [J1, J2, J3]`
uses damped least squares. `J0` yaw and `J4` are handled as direct proportional
controllers with the same joint velocity and joint limit handling as the planar
joints. Task rows have explicit weights so meter and radian errors are scaled
before the DLS solve.

The solver limits joint velocity by computing a global `alpha` scale. If any
requested joint velocity exceeds its limit, the whole `q_dot` vector is scaled
by the smallest required ratio. For example, if a joint requests `3 rad/s` and
the limit is `2 rad/s`, the whole command is scaled by `2/3`. The current
default joint velocity limit is `0.5 rad/s` for each kinematic joint.

The default motion configuration uses a `20 ms` control period and allows up to
`500` iterations per target. With the default period this gives a maximum of
about `10 s` for one blocking target move before the target is reported as not
converged.

Joint limits also shrink the global step before failing the motion. A target is
reported as blocked only when the step scale is effectively exhausted while the
task error is still too large. The blocking motion loop also detects lack of
progress so the robot does not spend several seconds making tiny noisy motions
near an unreachable target.

Damped least squares does not remove physical singularities. Near a singular
configuration, some TCP directions are still not achievable by the mechanism.
DLS only prevents the numerical controller from asking for unbounded joint
velocities.

Because `J3` is an explicit task requirement and also part of the reach/height
kinematics, it restricts the reachable workspace. Some points that are
geometrically reachable with free `J3` may not be reachable when a specific
`J3` value is required.

Possible error statuses include:

- `OutOfReach` - the target did not converge within the configured iteration budget,
- `JointLimitViolation` - the current or computed command violates joint limits,
- `JointLimitBlocked` - joint limits prevent useful progress toward the target,
- `NoProgress` - the solver stopped reducing the normalized task error,
- `InvalidInput` - the input contains invalid values, such as `NaN` or infinity.

## Control Rules

The current firmware executes a fixed pick-and-place demo sequence during
startup:

1. `TARGET1` through `TARGET11` in `robot_arm.ino` define target `TcpPose`
   values, explicit `J3`, explicit `J4`, and gripper state.
2. `setup()` initializes the PCA9685, sends the arm to `ZERO`/HOME, and opens
   the gripper.
3. The robot moves through a blocking fixed-period Jacobian IK sequence: ready
   pose, pick-side yaw, descend, close gripper, lift, carry across a larger
   base rotation, descend, open gripper, retreat, and return to HOME.
4. The final HOME return is a normal Jacobian IK target, not a direct
   all-servo zero command.
5. Once the sequence is complete, `loop()` stays idle.

This is not closed-loop feedback from joint sensors. It is an open-loop
trajectory generator that assumes the commanded state is a reasonable estimate
of the servo state. Every new target and transition should still be tested
carefully on hardware.

## V1 Limitations

- No closed-loop feedback from joints.
- The Jacobian IK trajectory generator is blocking during the fixed demo
  sequence.
- IK solves TCP position and base yaw, not full 6D pose.
- `J3` and `J4` are explicit task inputs; `J3` constrains reachable TCP
  positions.
- Calibration is specific to this physical build and should not be copied
  blindly.
- Servo limits are conservative to avoid mechanical stops.

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

The main practical lessons are:

- mechanical zero is set by servo horn mounting first, then refined with
  `pwm_zero_us`,
- each joint needs its own zero, min, max, sign direction, and mechanical-stop
  check,
- IK commands should be sanity-checked by running FK on the resulting joint
  angles,
- Arduino logic power and servo power should be separated, with a common ground.

See [`docs/lessons-learned.md`](docs/lessons-learned.md) for the detailed notes.

## Startup Safety

Before testing on the manipulator, check:

- whether the target is within `JOINT_CALIBRATIONS` limits,
- whether `J3` and `J4` can cause a mechanical collision,
- whether the gripper PWM values are calibrated correctly,
- whether the open-loop IK motion can hit mechanical stops,
- whether the servo power supply is sufficient for multi-joint motion under load.

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
