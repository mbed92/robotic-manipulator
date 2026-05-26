# PCB Wiring

## Connections

- Laptop USB -> Arduino USB
- Arduino +5V -> PCA9685 VCC/VDD
- Arduino GND -> PCA9685 GND/VSS
- Arduino SDA (pin A4) -> PCA9685 SDA
- Arduino SCL (pin A5) -> PCA9685 SCL
- PSU +5V -> PCA9685 V+
- PSU GND -> PCA9685 GND
- SERVO1..6 PWM -> PCA9685 CH1..6 PWM
- SERVO1..6 5V -> PCA9685 CH1..6 5V
- SERVO1..6 GND -> PCA9685 CH1..6 GND

## Important Notes

- Arduino does NOT power the servos.
- Servos are powered directly from PSU.
- All grounds must be connected together.
- PCA9685 generates hardware PWM signals.
