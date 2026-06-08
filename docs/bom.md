# Bill of Materials

This document lists the main components used in the V1 desktop robotic manipulator build.

The goal of this BOM is not to define a production-ready design, but to document the exact hardware used for the first working prototype.

## Main hardware

| Category         | Component                |                        Model / specification |   Qty | Role in project                        | Notes                                                                                                   |
| ---------------- | ------------------------ | -------------------------------------------: | ----: | -------------------------------------- | ------------------------------------------------------------------------------------------------------- |
| Mechanical kit   | 6DOF robotic arm kit     |                  Metal 6DOF arm with gripper |     1 | Main robot structure                   | Includes metal frame, gripper, servo horns, screws, bearings, cable sleeving and zip ties.              |
| Servo            | High-torque servo        |                                       MG996R |     6 | Arm joints and gripper actuation       | Hobby servos used for all actuated joints. Real range and zero position must be calibrated per joint.   |
| Controller       | Arduino-compatible board | DFRduino Nano V4.0 / Arduino Nano compatible |     1 | Main microcontroller                   | Runs FK/IK, calibration mapping and servo commands.                                                     |
| Servo PWM driver | PCA9685 PWM servo driver |                               I2C PWM driver |     1 | Generates PWM signals for servos       | Driven over I2C from Arduino. Servo power must be provided separately through the servo power rail.     |
| Power supply     | AC/DC switching PSU      |               POS-50-5-C2, 5 V / 10 A / 50 W |     1 | Main 5 V power supply for servos       | Provides power for the servo rail. Do not power the servos from Arduino 5 V.                            |
| Servo tester     | 3-channel servo tester   |                     Generic 3-channel tester |     1 | Manual servo testing and centering     | Used before connecting servos to PCA9685. Useful for checking servo health and approximate range.       |
| Multimeter       | Digital multimeter       |                                 UNI-T UT33A+ |     1 | Voltage and continuity checks          | Used to verify PSU output, common GND and basic wiring.                                                 |
| Capacitor        | Electrolytic capacitor   |                          1000 µF / 25 V, THT |     5 | Servo rail decoupling                  | Used near the servo power rail to reduce voltage dips caused by servo load transients.                  |
| Switch           | Rocker switch            |                  IRS-101-8C/D, 12 VDC / 20 A |     1 | Manual power switch                    | Used as a convenient switch in the prototype. Check ratings carefully before using in a final build.    |
| Cable            | USB cable                |                          microUSB B to USB-A |     1 | Arduino programming and serial monitor | Used for firmware upload and debug output.                                                              |
| Wires            | Jumper wires             |                      20 cm M-M, F-F, M-F set | 1 set | Low-current signal wiring              | Useful for I2C and prototype wiring. Do not use thin jumpers for high-current servo power distribution. |

## Servo details

| Parameter                  |           Value | Notes                                                                                           |
| -------------------------- | --------------: | ----------------------------------------------------------------------------------------------- |
| Servo model                |          MG996R | Metal gear hobby servo.                                                                         |
| Nominal operating voltage  |       4.8–7.2 V | The V1 build uses a 5 V supply.                                                                 |
| Recommended kit voltage    |           5–6 V | Matches the mechanical kit recommendation.                                                      |
| Approximate rotation range |           ~120° | Treat this as approximate. Actual usable range depends on mechanical mounting and joint limits. |
| Stall current              |   ~2.5 A at 6 V | Important for power sizing. Multiple servos can create large transient current draw.            |
| Signal period              |   20 ms / 50 Hz | Standard hobby servo PWM.                                                                       |
| Signal wire                | Yellow / orange | PWM signal.                                                                                     |
| Power wire                 |             Red | Servo VCC.                                                                                      |
| Ground wire                |           Brown | Servo GND.                                                                                      |

## Power architecture

The prototype uses separated logic and servo power:

| Rail      | Source                         | Consumers                     | Notes                                   |
| --------- | ------------------------------ | ----------------------------- | --------------------------------------- |
| Logic 5 V | Arduino USB / Arduino 5 V rail | Arduino, PCA9685 logic side   | Used only for logic-level electronics.  |
| Servo 5 V | External 5 V / 10 A PSU        | PCA9685 servo V+ rail, servos | Used for servo power.                   |
| GND       | Common ground                  | Arduino, PCA9685, PSU, servos | All grounds must be connected together. |

Important wiring rule:

```text
Arduino GND  --------+
PCA9685 GND ---------+---- common GND
PSU GND ------------+
Servo GND ----------+
```

The servos must not be powered directly from the Arduino 5 V pin. The Arduino controls the PWM driver, but the servo current comes from the external PSU.

## Wiring summary

| Connection    | From                | To                      | Notes                              |
| ------------- | ------------------- | ----------------------- | ---------------------------------- |
| I2C SDA       | Arduino SDA / A4    | PCA9685 SDA             | Logic signal.                      |
| I2C SCL       | Arduino SCL / A5    | PCA9685 SCL             | Logic signal.                      |
| Logic power   | Arduino 5 V         | PCA9685 VCC / VDD       | Logic power only.                  |
| Common ground | Arduino GND         | PCA9685 GND, PSU GND    | Required for PWM signal reference. |
| Servo power   | PSU +5 V            | PCA9685 V+              | High-current servo rail.           |
| Servo ground  | PSU GND             | PCA9685 GND / servo GND | Shared return path.                |
| Servo PWM     | PCA9685 channel PWM | Servo signal wire       | One channel per servo.             |

## Tools and consumables

| Component             | Purpose                                 | Notes                                   |
| --------------------- | --------------------------------------- | --------------------------------------- |
| Multimeter            | Voltage, polarity and continuity checks | Verify 5 V before connecting servos.    |
| Servo tester          | Manual servo range and centering tests  | Useful before writing firmware.         |
| Jumper wires          | Prototype signal wiring                 | Use only for low-current paths.         |
| Thicker power wires   | Servo power distribution                | Recommended for PSU-to-servo power bus. |
| Zip ties / sleeving   | Cable management                        | Cables must not restrict joint motion.  |
| Small screwdriver set | Mechanical assembly                     | Required for frame and servo horns.     |

## Known substitutions

| Original component | Possible substitute                 | Requirements                                                               |
| ------------------ | ----------------------------------- | -------------------------------------------------------------------------- |
| DFRduino Nano V4.0 | Arduino Nano / Uno compatible       | Must support I2C and Arduino Servo/PWM-related libraries.                  |
| PCA9685 PWM board  | Any PCA9685 16-channel servo driver | Must expose VCC/VDD, V+, GND, SDA, SCL and servo headers.                  |
| 5 V / 10 A PSU     | Other 5–6 V DC PSU                  | Must provide enough current for multiple servos. Use current margin.       |
| MG996R servos      | Similar metal gear hobby servos     | Requires recalibration of PWM limits, zero positions and joint directions. |

## Safety notes

* Verify PSU output voltage before connecting the servo rail.
* Do not power the servos from the Arduino.
* Keep all grounds common.
* Start with one servo connected during early tests.
* Keep PWM limits conservative until each joint is calibrated.
* Check that cables do not restrict motion at any joint.
* Keep a physical way to cut servo power during first motion tests.

## V1 limitations

This BOM describes the first working prototype. It is not a production-ready electrical design.

Known limitations:

* No current sensing on individual servos.
* No emergency stop circuit.
* No fuse selected specifically for the final DC servo rail.
* No dedicated high-current PCB or power distribution board.
* Servo power distribution is prototype-grade.
* Cable strain relief should be improved before extended operation.
* Servo calibration is specific to this mechanical build.
