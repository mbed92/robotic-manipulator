/**
 * Zero position for the robot arm.
 * This program sets all servos to their configured zero positions.
 * Adjust robot_calibration.h to change PWM and angle calibration values.
 */

#include <Wire.h>

#include "kinematics.h"
#include "pca9685_servo_driver.h"
#include "robot_calibration.h"

constexpr float ZERO_ANGLE_TOLERANCE_RAD = 0.000001f;
constexpr float ZERO_FK_POSITION_TOLERANCE_M = 0.000001f;
constexpr float ZERO_FK_TOOL_ROLL_TOLERANCE_RAD = 0.000001f;

Pca9685ServoDriver servo_driver;

void printJointState(uint8_t joint_index, const JointCalibration &joint);
void printArmJointAngles(const ArmJointAngles &angles);
void printTcpPose(const TcpPose &pose);
bool verifyZeroArmAngles(const ArmJointAngles &angles);
bool verifyZeroFkPose(const TcpPose &pose, const JointOffsets &offsets = JOINT_OFFSETS);
float absFloat(float value);

void setup() {
    Serial.begin(115200);

    if (!servo_driver.begin()) {
        Serial.println("PCA9685 init failed");
        return;
    }

    delay(100);

    for (uint8_t i = 0; i < ROBOT_TOTAL_JOINT_COUNT; ++i) {
        const JointCalibration &joint = JOINT_CALIBRATIONS[i];
        servo_driver.setServoUs(joint.channel, joint.pwm_zero_us);
        printJointState(i, joint);
    }

    Serial.println("All servos set to zero positions");

    // J1-J5 define the arm TCP pose. J6 is the gripper and is not part of FK/IK.
    const ArmJointAngles target_angles = {
        JOINT_CALIBRATIONS[0].angle_zero_rad,
        JOINT_CALIBRATIONS[1].angle_zero_rad,
        JOINT_CALIBRATIONS[2].angle_zero_rad,
        JOINT_CALIBRATIONS[3].angle_zero_rad,
        JOINT_CALIBRATIONS[4].angle_zero_rad,
    };

    if (verifyZeroArmAngles(target_angles)) {
        Serial.println("Arm joint zero-angle verification OK");
    } else {
        Serial.println("Arm joint zero-angle verification FAILED");
    }

    const KinematicsResult<TcpPose> fk_result = forwardKinematics(target_angles);
    if (fk_result.status == KinematicsStatus::Ok) {
        printTcpPose(fk_result.value);
        if (verifyZeroFkPose(fk_result.value)) {
            Serial.println("FK zero-pose verification OK");
        } else {
            Serial.println("FK zero-pose verification FAILED");
        }

        const KinematicsResult<ArmJointAngles> ik_result =
            inverseKinematicsPositionToolRoll(fk_result.value);
        if (ik_result.status == KinematicsStatus::Ok) {
            printArmJointAngles(ik_result.value);
            if (verifyZeroArmAngles(ik_result.value)) {
                Serial.println("FK-to-IK zero-joint verification OK");
            } else {
                Serial.println("FK-to-IK zero-joint verification FAILED");
            }
        } else {
            Serial.print("FK-to-IK verification failed status=");
            Serial.println(static_cast<int>(ik_result.status));
        }
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
    Serial.print(joint.angle_zero_rad, 6);
    if (joint_index == ROBOT_GRIPPER_JOINT_INDEX) {
        Serial.print(" role=gripper");
    } else {
        Serial.print(" role=arm_fk_ik");
    }
    Serial.println();
}

void printArmJointAngles(const ArmJointAngles &angles) {
    Serial.print("IK arm joints j1_rad=");
    Serial.print(angles.j1_rad, 6);
    Serial.print(" j2_rad=");
    Serial.print(angles.j2_rad, 6);
    Serial.print(" j3_rad=");
    Serial.print(angles.j3_rad, 6);
    Serial.print(" j4_rad=");
    Serial.print(angles.j4_rad, 6);
    Serial.print(" j5_rad=");
    Serial.println(angles.j5_rad, 6);
}

void printTcpPose(const TcpPose &pose) {
    Serial.print("FK TCP x_m=");
    Serial.print(pose.position.x_m, 6);
    Serial.print(" y_m=");
    Serial.print(pose.position.y_m, 6);
    Serial.print(" z_m=");
    Serial.print(pose.position.z_m, 6);
    Serial.print(" tool_roll_rad=");
    Serial.println(pose.tool_roll_rad, 6);
}

bool verifyZeroArmAngles(const ArmJointAngles &angles) {
    return absFloat(angles.j1_rad) <= ZERO_ANGLE_TOLERANCE_RAD &&
           absFloat(angles.j2_rad) <= ZERO_ANGLE_TOLERANCE_RAD &&
           absFloat(angles.j3_rad) <= ZERO_ANGLE_TOLERANCE_RAD &&
           absFloat(angles.j4_rad) <= ZERO_ANGLE_TOLERANCE_RAD &&
           absFloat(angles.j5_rad) <= ZERO_ANGLE_TOLERANCE_RAD;
}

bool verifyZeroFkPose(const TcpPose &pose, const JointOffsets &offsets) {
    const float expected_z_m =
        offsets.l1_m + offsets.l2_m + offsets.l3_m + offsets.l4_m + offsets.l5_m;

    return absFloat(pose.position.x_m) <= ZERO_FK_POSITION_TOLERANCE_M &&
           absFloat(pose.position.y_m) <= ZERO_FK_POSITION_TOLERANCE_M &&
           absFloat(pose.position.z_m - expected_z_m) <= ZERO_FK_POSITION_TOLERANCE_M &&
           absFloat(pose.tool_roll_rad) <= ZERO_FK_TOOL_ROLL_TOLERANCE_RAD;
}

float absFloat(float value) {
    return value < 0.0f ? -value : value;
}

void loop() {
}
