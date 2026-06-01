#include "kinematics.h"

#include <math.h>

namespace {

constexpr float FULL_TURN_RAD = 6.28318530718f;

bool isFiniteFloat(float value) {
    return !isnan(value) && !isinf(value);
}

bool isValidOffsets(const JointOffsets &offsets) {
    return isFiniteFloat(offsets.l1_m) && offsets.l1_m >= 0.0f &&
           isFiniteFloat(offsets.l2_m) && offsets.l2_m >= 0.0f &&
           isFiniteFloat(offsets.l3_m) && offsets.l3_m >= 0.0f &&
           isFiniteFloat(offsets.l4_m) && offsets.l4_m >= 0.0f &&
           isFiniteFloat(offsets.l5_m) && offsets.l5_m >= 0.0f;
}

bool isValidAngles(const ArmJointAngles &angles) {
    return isFiniteFloat(angles.j1_rad) &&
           isFiniteFloat(angles.j2_rad) &&
           isFiniteFloat(angles.j3_rad) &&
           isFiniteFloat(angles.j4_rad) &&
           isFiniteFloat(angles.j5_rad);
}

bool isValidPose(const TcpPose &pose) {
    return isFiniteFloat(pose.position.x_m) &&
           isFiniteFloat(pose.position.y_m) &&
           isFiniteFloat(pose.position.z_m) &&
           isFiniteFloat(pose.yaw_rad);
}

float normalizeAngleRad(float angle_rad) {
    while (angle_rad > PI) {
        angle_rad -= FULL_TURN_RAD;
    }
    while (angle_rad < -PI) {
        angle_rad += FULL_TURN_RAD;
    }
    return angle_rad;
}

float interpolatePwmUs(const JointCalibration &calibration, float angle_rad) {
    if (angle_rad >= calibration.angle_zero_rad) {
        const float angle_span = calibration.angle_max_rad - calibration.angle_zero_rad;
        const float pwm_span = static_cast<float>(
            calibration.pwm_max_us - calibration.pwm_zero_us);
        return static_cast<float>(calibration.pwm_zero_us) +
               ((angle_rad - calibration.angle_zero_rad) * pwm_span) / angle_span;
    }

    const float angle_span = calibration.angle_zero_rad - calibration.angle_min_rad;
    const float pwm_span = static_cast<float>(
        calibration.pwm_zero_us - calibration.pwm_min_us);
    return static_cast<float>(calibration.pwm_zero_us) -
           ((calibration.angle_zero_rad - angle_rad) * pwm_span) / angle_span;
}

uint16_t roundToUint16(float value) {
    return static_cast<uint16_t>(value + 0.5f);
}

ArmJointPwmUs emptyPwm() {
    return {0, 0, 0, 0, 0};
}

ArmJointAngles emptyAngles() {
    return {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
}

TcpPose emptyPose() {
    return {{0.0f, 0.0f, 0.0f}, 0.0f};
}

}  // namespace

KinematicsResult<TcpPose> forwardKinematics(
    const ArmJointAngles &angles,
    const JointOffsets &offsets) {
    if (!isValidAngles(angles) || !isValidOffsets(offsets)) {
        return {KinematicsStatus::InvalidInput, emptyPose()};
    }

    const float pitch_2 = angles.j2_rad;
    const float pitch_3 = pitch_2 + angles.j3_rad;
    const float pitch_4 = pitch_3 + angles.j4_rad;

    const float reach_m =
        offsets.l2_m * sin(pitch_2) +
        offsets.l3_m * sin(pitch_3) +
        offsets.l4_m * sin(pitch_4) +
        offsets.l5_m * sin(pitch_4);

    const float z_m =
        offsets.l1_m +
        offsets.l2_m * cos(pitch_2) +
        offsets.l3_m * cos(pitch_3) +
        offsets.l4_m * cos(pitch_4) +
        offsets.l5_m * cos(pitch_4);

    TcpPose pose = {
        {
            reach_m * cos(angles.j1_rad),
            reach_m * sin(angles.j1_rad),
            z_m,
        },
        normalizeAngleRad(angles.j1_rad + angles.j5_rad),
    };

    return {KinematicsStatus::Ok, pose};
}

KinematicsResult<ArmJointAngles> inverseKinematicsPositionYaw(
    const TcpPose &target_pose,
    const JointOffsets &offsets) {
    if (!isValidPose(target_pose) || !isValidOffsets(offsets)) {
        return {KinematicsStatus::InvalidInput, emptyAngles()};
    }

    return {KinematicsStatus::NotImplemented, emptyAngles()};
}

KinematicsResult<uint16_t> jointAngleToPwmUs(
    uint8_t joint_index,
    float angle_rad,
    const JointCalibration calibrations[ROBOT_ARM_JOINT_COUNT]) {
    if (joint_index >= ROBOT_ARM_JOINT_COUNT || !isFiniteFloat(angle_rad)) {
        return {KinematicsStatus::InvalidInput, 0};
    }

    const JointCalibration &calibration = calibrations[joint_index];
    if (angle_rad < calibration.angle_min_rad || angle_rad > calibration.angle_max_rad) {
        return {KinematicsStatus::JointLimitViolation, 0};
    }

    const float pwm_us = interpolatePwmUs(calibration, angle_rad);
    if (!isFiniteFloat(pwm_us) ||
        pwm_us < static_cast<float>(calibration.pwm_min_us) ||
        pwm_us > static_cast<float>(calibration.pwm_max_us)) {
        return {KinematicsStatus::JointLimitViolation, 0};
    }

    return {KinematicsStatus::Ok, roundToUint16(pwm_us)};
}

KinematicsResult<ArmJointPwmUs> jointAnglesToPwmUs(
    const ArmJointAngles &angles,
    const JointCalibration calibrations[ROBOT_ARM_JOINT_COUNT]) {
    if (!isValidAngles(angles)) {
        return {KinematicsStatus::InvalidInput, emptyPwm()};
    }

    const float angle_values[] = {
        angles.j1_rad,
        angles.j2_rad,
        angles.j3_rad,
        angles.j4_rad,
        angles.j5_rad,
    };

    uint16_t pwm_values[] = {0, 0, 0, 0, 0};
    for (uint8_t i = 0; i < 5; ++i) {
        const KinematicsResult<uint16_t> result =
            jointAngleToPwmUs(i, angle_values[i], calibrations);
        if (result.status != KinematicsStatus::Ok) {
            return {result.status, emptyPwm()};
        }
        pwm_values[i] = result.value;
    }

    return {
        KinematicsStatus::Ok,
        {
            pwm_values[0],
            pwm_values[1],
            pwm_values[2],
            pwm_values[3],
            pwm_values[4],
        },
    };
}
