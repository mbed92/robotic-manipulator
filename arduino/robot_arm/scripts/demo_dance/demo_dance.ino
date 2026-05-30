/**
 * Demo dance for the robot arm.
 * This program starts from HOME and moves joints in small PWM ranges around HOME.
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
constexpr uint16_t DANCE_HOLD_MS = 350;

const int16_t DANCE_OFFSETS_US[][ROBOT_ARM_JOINT_COUNT] = {
    {40, -60, 30, -30, 50, -45},
    {-40, 60, -30, 30, -50, 45},
    {30, 45, -20, -20, 60, -35},
    {-30, -45, 20, 20, -60, 35},
    {0, 0, 0, 0, 0, 0},
};

constexpr uint8_t DANCE_STEP_COUNT = sizeof(DANCE_OFFSETS_US) / sizeof(DANCE_OFFSETS_US[0]);

// These arrays track the last PWM command known to this sketch.
// They do not represent measured servo position; hobby servos used here have no feedback line.
uint16_t current_pwm_us[PCA9685_CHANNEL_COUNT] = {};
bool current_pwm_known[PCA9685_CHANNEL_COUNT] = {};

void setServoUs(uint8_t channel, uint16_t us);
void printJointState(uint8_t joint_index, const JointCalibration& joint);
uint16_t pwmUsToTicks(uint16_t us);
uint16_t ticksToPwmUs(uint16_t ticks);
uint16_t clampPwmUs(const JointCalibration& joint, int32_t pwm_us);
void moveToHome();
void moveToDanceStep(uint8_t step_index);

void setup()
{
    Serial.begin(115200);

    pca.begin();
    pca.setPWMFreq(50); // servo frequency

    delay(100);

    moveToHome();

    Serial.println("Demo dance started");
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

uint16_t clampPwmUs(const JointCalibration& joint, int32_t pwm_us)
{
    if (pwm_us < joint.pwm_min_us) {
        return joint.pwm_min_us;
    }
    if (pwm_us > joint.pwm_max_us) {
        return joint.pwm_max_us;
    }
    return static_cast<uint16_t>(pwm_us);
}

void moveToHome()
{
    for (uint8_t i = 0; i < ROBOT_ARM_JOINT_COUNT; ++i) {
        const JointCalibration& joint = JOINT_CALIBRATIONS[i];
        setServoUs(joint.channel, joint.pwm_home_us);
        printJointState(i, joint);
    }

    Serial.println("All servos set to HOME positions");
}

void moveToDanceStep(uint8_t step_index)
{
    Serial.print("Dance step ");
    Serial.println(step_index + 1);

    for (uint8_t i = 0; i < ROBOT_ARM_JOINT_COUNT; ++i) {
        const JointCalibration& joint = JOINT_CALIBRATIONS[i];
        const uint16_t target_pwm_us = clampPwmUs(
            joint, static_cast<int32_t>(joint.pwm_home_us) + DANCE_OFFSETS_US[step_index][i]);
        setServoUs(joint.channel, target_pwm_us);

        Serial.print("Joint ");
        Serial.print(i + 1);
        Serial.print(" channel=");
        Serial.print(joint.channel);
        Serial.print(" target_pwm_us=");
        Serial.println(target_pwm_us);
    }
}

void loop()
{
    for (uint8_t step = 0; step < DANCE_STEP_COUNT; ++step) {
        moveToDanceStep(step);
        delay(DANCE_HOLD_MS);
    }
}
