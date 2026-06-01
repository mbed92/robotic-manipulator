/**
 * Zero position for the robot arm.
 * This program sets all servos to their configured zero positions.
 * Adjust robot_calibration.h to change PWM and angle calibration values.
 */

#include <Wire.h>

#include "robot_calibration.h"
#include "pca9685_servo_driver.h"

Pca9685ServoDriver servo_driver;

void printJointState(uint8_t joint_index, const JointCalibration &joint);

void setup() {
    Serial.begin(115200);

    if (!servo_driver.begin()) {
        Serial.println("PCA9685 init failed");
        return;
    }

    delay(100);

    for (uint8_t i = 0; i < ROBOT_ARM_JOINT_COUNT; ++i) {
        const JointCalibration &joint = JOINT_CALIBRATIONS[i];
        servo_driver.setServoUs(joint.channel, joint.pwm_zero_us);
        printJointState(i, joint);
    }

    Serial.println("All servos set to zero positions");
}

void printJointState(uint8_t joint_index, const JointCalibration &joint) {
    Serial.print("Joint ");
    Serial.print(joint_index + 1);
    Serial.print(" channel=");
    Serial.print(joint.channel);
    Serial.print(" pwm_zero_us=");
    Serial.print(joint.pwm_zero_us);
    Serial.print(" angle_zero_rad=");
    Serial.println(joint.angle_zero_rad, 6);
}

void loop() {
}
