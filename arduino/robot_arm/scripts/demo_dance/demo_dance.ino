/**
 * Demo dance for the robot arm.
 * This program starts from zero and moves joints in small PWM ranges around zero.
 * Adjust joint_calibration.h to change PWM and angle calibration values.
 */

#include <Wire.h>

#include "joint_calibration.h"
#include "pca9685_servo_driver.h"

constexpr uint16_t DANCE_HOLD_MS = 350;

const int16_t DANCE_OFFSETS_US[][ROBOT_ARM_JOINT_COUNT] = {
    {40, -60, 30, -30, 50, -45},
    {-40, 60, -30, 30, -50, 45},
    {30, 45, -20, -20, 60, -35},
    {-30, -45, 20, 20, -60, 35},
    {0, 0, 0, 0, 0, 0},
};

constexpr uint8_t DANCE_STEP_COUNT = sizeof(DANCE_OFFSETS_US) / sizeof(DANCE_OFFSETS_US[0]);

Pca9685ServoDriver servo_driver;

void printJointState(uint8_t joint_index, const JointCalibration& joint);
uint16_t clampPwmUs(const JointCalibration& joint, int32_t pwm_us);
void moveToZero();
void moveToDanceStep(uint8_t step_index);

void setup()
{
    Serial.begin(115200);

    if (!servo_driver.begin()) {
        Serial.println("PCA9685 init failed");
        return;
    }

    delay(100);

    moveToZero();

    Serial.println("Demo dance started");
}

void printJointState(uint8_t joint_index, const JointCalibration& joint)
{
    Serial.print("Joint ");
    Serial.print(joint_index + 1);
    Serial.print(" channel=");
    Serial.print(joint.channel);
    Serial.print(" pwm_zero_us=");
    Serial.print(joint.pwm_zero_us);
    Serial.print(" angle_zero_rad=");
    Serial.println(joint.angle_zero_rad, 6);
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

void moveToZero()
{
    for (uint8_t i = 0; i < ROBOT_ARM_JOINT_COUNT; ++i) {
        const JointCalibration& joint = JOINT_CALIBRATIONS[i];
        servo_driver.setServoUs(joint.channel, joint.pwm_zero_us);
        printJointState(i, joint);
    }

    Serial.println("All servos set to zero positions");
}

void moveToDanceStep(uint8_t step_index)
{
    Serial.print("Dance step ");
    Serial.println(step_index + 1);

    for (uint8_t i = 0; i < ROBOT_ARM_JOINT_COUNT; ++i) {
        const JointCalibration& joint = JOINT_CALIBRATIONS[i];
        const uint16_t target_pwm_us = clampPwmUs(
            joint, static_cast<int32_t>(joint.pwm_zero_us) + DANCE_OFFSETS_US[step_index][i]);
        servo_driver.setServoUs(joint.channel, target_pwm_us);

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
