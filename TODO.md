# TODO

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
