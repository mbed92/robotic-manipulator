/**
 * Home position for the robot arm.
 * This program sets all servos to their configured HOME positions.
 * Adjust joint_calibration.h to change PWM and angle calibration values.
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#include "joint_calibration.h"

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver();

constexpr uint8_t PCA9685_CHANNEL_COUNT = 16;
constexpr uint16_t SERVO_PWM_PERIOD_US = 20000;
constexpr uint16_t PCA9685_TICKS_PER_PERIOD = 4096;
constexpr uint16_t SERVO_RAMP_STEP_US = 10;
constexpr uint16_t SERVO_RAMP_STEP_DELAY_MS = 20;

// These arrays track the last PWM command known to this sketch.
// They do not represent measured servo position; hobby servos used here have no feedback line.
uint16_t current_pwm_us[PCA9685_CHANNEL_COUNT] = {};
bool current_pwm_known[PCA9685_CHANNEL_COUNT] = {};

void setServoUs(uint8_t channel, uint16_t us);
void printJointState(uint8_t joint_index, const JointCalibration& joint);
uint16_t pwmUsToTicks(uint16_t us);
uint16_t ticksToPwmUs(uint16_t ticks);

void setup()
{
    Serial.begin(115200);

    pca.begin();
    pca.setPWMFreq(50); // servo frequency

    delay(100);

    for (uint8_t i = 0; i < ROBOT_ARM_JOINT_COUNT; ++i) {
        const JointCalibration& joint = JOINT_CALIBRATIONS[i];
        setServoUs(joint.channel, joint.pwm_home_us);
        printJointState(i, joint);
    }

    Serial.println("All servos set to HOME positions");
}

void setServoUs(uint8_t channel, uint16_t us)
{
    if (channel >= PCA9685_CHANNEL_COUNT) {
        return;
    }

    if (!current_pwm_known[channel]) {
        // PCA9685 can report the PWM register value it currently outputs.
        // This is only the last stored command in the driver chip, not measured joint position.
        const uint16_t current_ticks = pca.getPWM(channel, true);
        if (current_ticks > 0 && current_ticks < PCA9685_TICKS_PER_PERIOD) {
            current_pwm_us[channel] = ticksToPwmUs(current_ticks);
            current_pwm_known[channel] = true;
        } else {
            // No useful previous command is available, so avoid inventing a fake start point.
            // The first command after this unknown state is sent directly.
            current_pwm_us[channel] = us;
            current_pwm_known[channel] = true;
            pca.setPWM(channel, 0, pwmUsToTicks(us));
            return;
        }
    }

    int32_t current_us = current_pwm_us[channel];
    const int32_t target_us = us;
    const int32_t direction = (target_us >= current_us) ? 1 : -1;

    while (current_us != target_us) {
        const int32_t next_us = current_us + direction * SERVO_RAMP_STEP_US;

        if ((direction > 0 && next_us > target_us) || (direction < 0 && next_us < target_us)) {
            current_us = target_us;
        } else {
            current_us = next_us;
        }

        pca.setPWM(channel, 0, pwmUsToTicks(static_cast<uint16_t>(current_us)));
        current_pwm_us[channel] = static_cast<uint16_t>(current_us);
        // Keep the ramp slow enough for basic bringup. Tune this with load removed first.
        delay(SERVO_RAMP_STEP_DELAY_MS);
    }
}

void printJointState(uint8_t joint_index, const JointCalibration& joint)
{
    Serial.print("Joint ");
    Serial.print(joint_index + 1);
    Serial.print(" channel=");
    Serial.print(joint.channel);
    Serial.print(" pwm_home_us=");
    Serial.print(joint.pwm_home_us);
    Serial.print(" angle_home_rad=");
    Serial.println(joint.angle_home_rad, 6);
}

uint16_t pwmUsToTicks(uint16_t us)
{
    uint32_t ticks = (static_cast<uint32_t>(us) * PCA9685_TICKS_PER_PERIOD) / SERVO_PWM_PERIOD_US;
    if (ticks >= PCA9685_TICKS_PER_PERIOD) {
        ticks = PCA9685_TICKS_PER_PERIOD - 1;
    }
    return static_cast<uint16_t>(ticks);
}

uint16_t ticksToPwmUs(uint16_t ticks)
{
    return static_cast<uint16_t>(
        (static_cast<uint32_t>(ticks) * SERVO_PWM_PERIOD_US) / PCA9685_TICKS_PER_PERIOD);
}

void loop()
{
}
