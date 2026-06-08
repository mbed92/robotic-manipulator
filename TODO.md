# TODO

## Accept target commands from an external controller

Current state:
- The firmware uses a manually edited `const ArmTarget TARGET` in `robot_arm.ino`.
- Changing the target TCP pose, `J3`, `J4`, or gripper state requires editing and reflashing the Arduino sketch.
- This is useful for a simple bring-up demo, but it is not a practical control interface.

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

## Generate coordinated multi-joint ramps

Current state:
- Ramp generation is disabled for now.
- `Pca9685ServoDriver::setServoUs()` writes the target PWM immediately.
- Demo scripts still call `setServoUs()` joint by joint, but each call is now a direct PWM update.

Target state:
- Generate the ramp at the motion/script level, where targets for all joints are known at the same time.
- Keep the PCA9685 driver focused on low-level PWM output, using immediate channel writes for each ramp sample.
- For each ramp step, compute intermediate PWM values for all joints and write all channels before delaying.

Recommended first implementation:
- Add a helper such as `moveAllJointsRamp(targets_us)` in the main demo/control script.
- Read or track the current PWM for each joint.
- Compute the maximum required delta across all joints.
- Step from current values to target values proportionally so all joints finish together.
- Use `setServoUsImmediate()` for each channel inside each ramp sample.

Main trade-off:
- A blocking `moveAllJointsRamp()` is acceptable as a simple first step and fixes sequential joint motion.
- A later version should use a non-blocking `millis()`-based trajectory update if the robot needs responsiveness to commands, sensors, or emergency stop handling during motion.
