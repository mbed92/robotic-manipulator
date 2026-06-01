#ifndef ROBOT_ARM_KINEMATICS_H
#define ROBOT_ARM_KINEMATICS_H

#include <Arduino.h>

#include "robot_calibration.h"

struct Vector3 {
    float x_m;
    float y_m;
    float z_m;
};

struct TcpPose {
    Vector3 position;
    float tool_roll_rad;
};

struct ArmJointAngles {
    float j1_rad;
    float j2_rad;
    float j3_rad;
    float j4_rad;
    float j5_rad;
};

struct ArmJointPwmUs {
    uint16_t j1_us;
    uint16_t j2_us;
    uint16_t j3_us;
    uint16_t j4_us;
    uint16_t j5_us;
};

enum class KinematicsStatus {
    Ok,
    OutOfReach,
    JointLimitViolation,
    NotImplemented,
    InvalidInput,
};

template <typename T>
struct KinematicsResult {
    KinematicsStatus status;
    T value;
};

KinematicsResult<TcpPose> forwardKinematics(
    const ArmJointAngles &angles,
    const JointOffsets &offsets = JOINT_OFFSETS);

KinematicsResult<ArmJointAngles> inverseKinematicsPositionToolRoll(
    const TcpPose &target_pose,
    const JointOffsets &offsets = JOINT_OFFSETS);

KinematicsResult<ArmJointAngles> inverseKinematicsPositionToolRollSeeded(
    const TcpPose &target_pose,
    const ArmJointAngles &seed_angles,
    const JointOffsets &offsets = JOINT_OFFSETS);

KinematicsResult<uint16_t> jointAngleToPwmUs(
    uint8_t joint_index,
    float angle_rad,
    const JointCalibration calibrations[ROBOT_TOTAL_JOINT_COUNT] = JOINT_CALIBRATIONS);

KinematicsResult<ArmJointPwmUs> jointAnglesToPwmUs(
    const ArmJointAngles &angles,
    const JointCalibration calibrations[ROBOT_TOTAL_JOINT_COUNT] = JOINT_CALIBRATIONS);

#endif
