/**
 * Zero position for the robot arm.
 * This program sets all servos to their configured zero positions.
 * Adjust robot_calibration.h to change PWM and angle calibration values.
 */

#include <Wire.h>

#include "kinematics.h"
#include "pca9685_servo_driver.h"
#include "robot_calibration.h"

Pca9685ServoDriver servo_driver;

void printJointState(uint8_t joint_index, const JointCalibration &joint);
void printTcpPose(const TcpPose &pose);

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

    const ArmJointAngles target_angles = {
        JOINT_CALIBRATIONS[0].angle_zero_rad,
        JOINT_CALIBRATIONS[1].angle_zero_rad,
        JOINT_CALIBRATIONS[2].angle_zero_rad,
        JOINT_CALIBRATIONS[3].angle_zero_rad,
        JOINT_CALIBRATIONS[4].angle_zero_rad,
    };
    const KinematicsResult<TcpPose> fk_result = forwardKinematics(target_angles);
    if (fk_result.status == KinematicsStatus::Ok) {
        printTcpPose(fk_result.value);
    } else {
        Serial.print("FK failed status=");
        Serial.println(static_cast<int>(fk_result.status));
    }
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

void printTcpPose(const TcpPose &pose) {
    Serial.print("FK TCP x_m=");
    Serial.print(pose.position.x_m, 6);
    Serial.print(" y_m=");
    Serial.print(pose.position.y_m, 6);
    Serial.print(" z_m=");
    Serial.print(pose.position.z_m, 6);
    Serial.print(" yaw_rad=");
    Serial.println(pose.yaw_rad, 6);
}

void loop() {
}
