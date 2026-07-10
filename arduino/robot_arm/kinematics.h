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
    JointLimitBlocked,
    NoProgress,
    NotImplemented,
    InvalidInput,
};

const char *kinematicsStatusName(KinematicsStatus status);

template <typename T>
struct KinematicsResult {
    KinematicsStatus status;
    T value;
};

ArmJointAngles robotHomeJointAngles();

struct JacobianIkTarget {
    TcpPose tcp_pose;
    float j3_rad;
    float j4_rad;
};

struct JacobianIkConfig {
    float dt_s;
    uint16_t max_iterations;
    float damping_lambda;
    float reach_weight_per_m;
    float z_weight_per_m;
    float j3_weight_per_rad;
    float position_gain_per_s;
    float angle_gain_per_s;
    float yaw_gain_per_s;
    float j4_gain_per_s;
    float joint_velocity_limit_rad_s[ROBOT_KINEMATIC_JOINT_COUNT];
    float position_tolerance_m;
    float angle_tolerance_rad;
    float min_step_alpha;
    float min_error_progress;
    uint8_t no_progress_iteration_limit;
};

struct JacobianIkStepResult {
    KinematicsStatus status;
    ArmJointAngles next_angles;
    float q_dot_rad_s[ROBOT_KINEMATIC_JOINT_COUNT];
    float alpha;
    float reach_error_m;
    float z_error_m;
    float yaw_error_rad;
    float j3_error_rad;
    float j4_error_rad;
    bool converged;
};

JacobianIkConfig defaultJacobianIkConfig();

KinematicsResult<TcpPose> forwardKinematics(
    const ArmJointAngles &angles,
    const JointOffsets &offsets = JOINT_OFFSETS);

JacobianIkStepResult computeJacobianIkStep(
    const ArmJointAngles &current,
    const JacobianIkTarget &target,
    const JacobianIkConfig &config,
    const JointOffsets &offsets = JOINT_OFFSETS);

KinematicsResult<uint16_t> jointAngleToPwmUs(
    uint8_t joint_index,
    float angle_rad,
    const JointCalibration calibrations[ROBOT_TOTAL_JOINT_COUNT] = JOINT_CALIBRATIONS);

KinematicsResult<ArmJointPwmUs> jointAnglesToPwmUs(
    const ArmJointAngles &angles,
    const JointCalibration calibrations[ROBOT_TOTAL_JOINT_COUNT] = JOINT_CALIBRATIONS);

#endif
