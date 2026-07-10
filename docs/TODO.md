# TODO

## Accept target commands from an external controller

Current state:
- The firmware uses a hardcoded `TARGETS[]` pick-and-place demo sequence in `robot_arm.ino`.
- Changing the target TCP pose, `J3`, `J4`, or gripper state requires editing and reflashing the Arduino sketch.
- This is useful for local demo tuning, but it is not a practical control interface.

Target state:
- Receive target commands from an external controller over a board-level bus such as I2C or SPI.
- The external controller could be ROS running on a Jetson or PC.
- Keep the Arduino responsible for low-level command validation, IK, angle-to-PWM conversion, and servo output.
- Keep the higher-level system responsible for planning, sequencing, and deciding what target should be sent next.

Recommended first implementation:
- Define a small binary command frame for `TcpPose`, requested `J3`, requested `J4`, and gripper command.
- Include a protocol version, command type, payload length, and checksum or CRC.
- Return an explicit status response such as accepted, invalid input, out of reach, or joint limit violation.
- Start with one command-at-a-time execution before adding queues or streaming trajectories.

Main trade-off:
- I2C is simple and common for embedded peripherals, but the Arduino must behave predictably as a bus device.
- SPI can provide cleaner timing and higher throughput, but needs more wiring and a slightly stricter framing design.
- A serial transport may be easier for early debugging, even if I2C or SPI remains the target hardware interface.

## Make Jacobian IK motion non-blocking

Current state:
- The main demo uses a blocking open-loop Jacobian IK trajectory generator.
- Each generated step writes all kinematic joint PWM targets, waits until the next fixed control period, and updates the commanded joint state.
- `Pca9685ServoDriver::setServoUs()` remains a low-level immediate PWM write.

Target state:
- Keep `computeJacobianIkStep()` as a pure numerical function.
- Move the blocking loop into a non-blocking `millis()`-scheduled motion state machine when the robot needs to accept commands, status requests, or stop conditions during motion.
- Keep the PCA9685 driver focused on low-level PWM output.

Recommended first implementation:
- Define an active motion state containing target, config, last deadline, current commanded angles, last error, and no-progress counter.
- On each loop tick, run at most one Jacobian IK step when the next period deadline has elapsed.
- Return explicit status for running, converged, invalid input, joint-limit blocked, no progress, and missed deadline.
- Add a way to cancel the active motion before adding command queues.

Main trade-off:
- The current blocking implementation is simpler for bring-up and keeps timing easy to inspect.
- A non-blocking scheduler is more complex, but it is needed before the Arduino can safely handle external command streaming or emergency-stop style inputs during motion.
