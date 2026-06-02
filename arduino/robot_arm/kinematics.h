#ifndef ROBOT_ARM_KINEMATICS_H
#define ROBOT_ARM_KINEMATICS_H

#include <Arduino.h>

#include "robot_calibration.h"

struct TcpPose {
    // Planar arm target in the local J0 frame. Lateral motion is commanded by yaw_rad.
    float reach_x_m;
    float reach_z_m;
    float yaw_rad;
};

struct ArmJointAngles {
    float j0_rad;
    float j1_rad;
    float j2_rad;
    float j3_rad;
    float j4_rad;
};

struct ArmJointPwmUs {
    uint16_t j0_us;
    uint16_t j1_us;
    uint16_t j2_us;
    uint16_t j3_us;
    uint16_t j4_us;
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

ArmJointAngles robotHomeJointAngles();

KinematicsResult<TcpPose> forwardKinematics(
    const ArmJointAngles &angles,
    const JointOffsets &offsets = JOINT_OFFSETS);

KinematicsResult<ArmJointAngles> inverseKinematicsPositionYaw(
    const TcpPose &target_pose,
    const JointOffsets &offsets = JOINT_OFFSETS);

KinematicsResult<ArmJointAngles> inverseKinematicsPositionYawPitch(
    const TcpPose &target_pose,
    float j3_rad,
    const JointOffsets &offsets = JOINT_OFFSETS);

KinematicsResult<uint16_t> jointAngleToPwmUs(
    uint8_t joint_index,
    float angle_rad,
    const JointCalibration calibrations[ROBOT_TOTAL_JOINT_COUNT] = JOINT_CALIBRATIONS);

KinematicsResult<ArmJointPwmUs> jointAnglesToPwmUs(
    const ArmJointAngles &angles,
    const JointCalibration calibrations[ROBOT_TOTAL_JOINT_COUNT] = JOINT_CALIBRATIONS);

#endif
