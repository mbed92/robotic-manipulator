/**
 * Zero position for the robot arm.
 * This program sets all servos to their configured zero positions.
 * Adjust robot_calibration.h to change PWM and angle calibration values.
 */

#include <Wire.h>
#include <math.h>

#include "kinematics.h"
#include "pca9685_servo_driver.h"
#include "robot_calibration.h"

constexpr float ZERO_ANGLE_TOLERANCE_RAD = 0.000001f;
constexpr float ZERO_FK_POSITION_TOLERANCE_M = 0.000001f;
constexpr float ZERO_FK_ORIENTATION_TOLERANCE_RAD = 0.000001f;
constexpr float HOME_ROUND_TRIP_TOLERANCE_RAD = 0.03f;

Pca9685ServoDriver servo_driver;

void printJointState(uint8_t joint_index, const JointCalibration &joint);
void printArmJointAngles(const ArmJointAngles &angles);
void printTcpPose(const TcpPose &pose);
bool verifyZeroArmAngles(const ArmJointAngles &angles);
bool verifyHomeArmAngles(const ArmJointAngles &angles);
bool verifyHomeFkPose(
    const TcpPose &pose,
    const ArmJointAngles &home_angles,
    const JointOffsets &offsets = JOINT_OFFSETS);
float absFloat(float value);
float angleDistanceRad(float a_rad, float b_rad);

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

    // J0-J3 define the arm TCP pose. J4 is independent wrist rotation.
    // J5 is the gripper and is not part of FK/IK.
    const ArmJointAngles zero_angles = {
        JOINT_CALIBRATIONS[0].angle_zero_rad,
        JOINT_CALIBRATIONS[1].angle_zero_rad,
        JOINT_CALIBRATIONS[2].angle_zero_rad,
        JOINT_CALIBRATIONS[3].angle_zero_rad,
        JOINT_CALIBRATIONS[4].angle_zero_rad,
    };
    const ArmJointAngles home_angles = robotHomeJointAngles();

    Serial.print("Calibration zero ");
    printArmJointAngles(zero_angles);
    Serial.print("Configured HOME ");
    printArmJointAngles(home_angles);

    if (verifyZeroArmAngles(zero_angles)) {
        Serial.println("Arm joint zero-angle verification OK");
    } else {
        Serial.println("Arm joint zero-angle verification FAILED");
    }

    const KinematicsResult<TcpPose> fk_result = forwardKinematics(home_angles);
    if (fk_result.status == KinematicsStatus::Ok) {
        printTcpPose(fk_result.value);
        if (verifyHomeFkPose(fk_result.value, home_angles)) {
            Serial.println("FK HOME-pose verification OK");
        } else {
            Serial.println("FK HOME-pose verification FAILED");
        }

        const KinematicsResult<ArmJointAngles> ik_result =
            inverseKinematicsPositionYaw(fk_result.value);
        if (ik_result.status == KinematicsStatus::Ok) {
            printArmJointAngles(ik_result.value);
            if (verifyHomeArmAngles(ik_result.value)) {
                Serial.println("FK-to-IK HOME-joint verification OK");
            } else {
                Serial.println("FK-to-IK HOME-joint verification FAILED");
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
    Serial.print("IK arm joints j0_rad=");
    Serial.print(angles.j0_rad, 6);
    Serial.print(" j1_rad=");
    Serial.print(angles.j1_rad, 6);
    Serial.print(" j2_rad=");
    Serial.print(angles.j2_rad, 6);
    Serial.print(" j3_rad=");
    Serial.print(angles.j3_rad, 6);
    Serial.print(" j4_rad=");
    Serial.println(angles.j4_rad, 6);
}

void printTcpPose(const TcpPose &pose) {
    Serial.print("FK TCP reach_x_m=");
    Serial.print(pose.reach_x_m, 6);
    Serial.print(" reach_z_m=");
    Serial.print(pose.reach_z_m, 6);
    Serial.print(" yaw_rad=");
    Serial.println(pose.yaw_rad, 6);
}

bool verifyZeroArmAngles(const ArmJointAngles &angles) {
    return absFloat(angles.j0_rad) <= ZERO_ANGLE_TOLERANCE_RAD &&
           absFloat(angles.j1_rad) <= ZERO_ANGLE_TOLERANCE_RAD &&
           absFloat(angles.j2_rad) <= ZERO_ANGLE_TOLERANCE_RAD &&
           absFloat(angles.j3_rad) <= ZERO_ANGLE_TOLERANCE_RAD &&
           absFloat(angles.j4_rad) <= ZERO_ANGLE_TOLERANCE_RAD;
}

bool verifyHomeArmAngles(const ArmJointAngles &angles) {
    const ArmJointAngles home_angles = robotHomeJointAngles();
    return absFloat(angleDistanceRad(angles.j0_rad, home_angles.j0_rad)) <= HOME_ROUND_TRIP_TOLERANCE_RAD &&
           absFloat(angleDistanceRad(angles.j1_rad, home_angles.j1_rad)) <= HOME_ROUND_TRIP_TOLERANCE_RAD &&
           absFloat(angleDistanceRad(angles.j2_rad, home_angles.j2_rad)) <= HOME_ROUND_TRIP_TOLERANCE_RAD &&
           absFloat(angleDistanceRad(angles.j3_rad, home_angles.j3_rad)) <= HOME_ROUND_TRIP_TOLERANCE_RAD &&
           absFloat(angleDistanceRad(angles.j4_rad, home_angles.j4_rad)) <= HOME_ROUND_TRIP_TOLERANCE_RAD;
}

bool verifyHomeFkPose(
    const TcpPose &pose,
    const ArmJointAngles &home_angles,
    const JointOffsets &offsets) {
    const float pitch_1 = home_angles.j1_rad;
    const float pitch_2 = pitch_1 + home_angles.j2_rad;
    const float pitch_3 = pitch_2 + home_angles.j3_rad;
    const float expected_reach_m =
        offsets.l2_m * sin(pitch_1) +
        offsets.l3_m * sin(pitch_2) +
        offsets.l4_m * sin(pitch_3) +
        offsets.l5_m * sin(pitch_3);
    const float expected_z_m =
        offsets.l1_m +
        offsets.l2_m * cos(pitch_1) +
        offsets.l3_m * cos(pitch_2) +
        offsets.l4_m * cos(pitch_3) +
        offsets.l5_m * cos(pitch_3);

    return absFloat(pose.reach_x_m - expected_reach_m) <= ZERO_FK_POSITION_TOLERANCE_M &&
           absFloat(pose.reach_z_m - expected_z_m) <= ZERO_FK_POSITION_TOLERANCE_M &&
           absFloat(angleDistanceRad(pose.yaw_rad, home_angles.j0_rad)) <= ZERO_FK_ORIENTATION_TOLERANCE_RAD;
}

float absFloat(float value) {
    return value < 0.0f ? -value : value;
}

float angleDistanceRad(float a_rad, float b_rad) {
    float delta_rad = a_rad - b_rad;
    while (delta_rad > PI) {
        delta_rad -= 2.0f * PI;
    }
    while (delta_rad < -PI) {
        delta_rad += 2.0f * PI;
    }
    return delta_rad;
}

void loop() {
}
