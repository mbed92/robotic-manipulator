#include <Wire.h>

#include "pca9685_servo_driver.h"
#include "robot_calibration.h"

Pca9685ServoDriver servo_driver;

void setup() {
    if (!servo_driver.begin()) {
        return;
    }

    for (uint8_t i = 0; i < ROBOT_TOTAL_JOINT_COUNT; ++i) {
        const JointCalibration &joint = JOINT_CALIBRATIONS[i];
        servo_driver.setServoUs(joint.channel, joint.pwm_zero_us);
    }
}

void loop() {
}
