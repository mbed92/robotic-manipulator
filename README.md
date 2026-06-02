# robotic-manipulator

Robotic manipulator.

## Hardware

The project's microcontroller is an Arduino Nano.

## Validation

Use Arduino CLI to validate firmware changes before testing on hardware:

```sh
arduino-cli compile --fqbn arduino:avr:nano arduino/robot_arm
```

For throwaway local builds, an explicit build path keeps generated files out of
the repository:

```sh
arduino-cli compile --fqbn arduino:avr:nano --build-path /tmp/robot_arm_build arduino/robot_arm
```
