#include "kinematics.h"

#include <math.h>

namespace {

constexpr float FULL_TURN_RAD = 6.28318530718f;
constexpr float IK_EPSILON_M = 0.000001f;
constexpr uint8_t IK_TOOL_PITCH_SAMPLE_COUNT = 181;

ArmJointAngles emptyAngles();

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
           isFiniteFloat(pose.tool_roll_rad);
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

float angleDistanceRad(float a_rad, float b_rad) {
    return normalizeAngleRad(a_rad - b_rad);
}

float jointLimitMargin(uint8_t joint_index, float angle_rad) {
    const JointCalibration &calibration = JOINT_CALIBRATIONS[joint_index];
    const float lower_margin = angle_rad - calibration.angle_min_rad;
    const float upper_margin = calibration.angle_max_rad - angle_rad;
    return lower_margin < upper_margin ? lower_margin : upper_margin;
}

bool isWithinJointLimit(uint8_t joint_index, float angle_rad) {
    const JointCalibration &calibration = JOINT_CALIBRATIONS[joint_index];
    return angle_rad >= calibration.angle_min_rad &&
           angle_rad <= calibration.angle_max_rad;
}

bool areWithinJointLimits(const ArmJointAngles &angles) {
    return isWithinJointLimit(0, angles.j1_rad) &&
           isWithinJointLimit(1, angles.j2_rad) &&
           isWithinJointLimit(2, angles.j3_rad) &&
           isWithinJointLimit(3, angles.j4_rad) &&
           isWithinJointLimit(4, angles.j5_rad);
}

float minJointLimitMargin(const ArmJointAngles &angles) {
    float margin = jointLimitMargin(0, angles.j1_rad);
    const float j2_margin = jointLimitMargin(1, angles.j2_rad);
    const float j3_margin = jointLimitMargin(2, angles.j3_rad);
    const float j4_margin = jointLimitMargin(3, angles.j4_rad);
    const float j5_margin = jointLimitMargin(4, angles.j5_rad);

    if (j2_margin < margin) {
        margin = j2_margin;
    }
    if (j3_margin < margin) {
        margin = j3_margin;
    }
    if (j4_margin < margin) {
        margin = j4_margin;
    }
    if (j5_margin < margin) {
        margin = j5_margin;
    }
    return margin;
}

float jointDistanceSquared(const ArmJointAngles &a, const ArmJointAngles &b) {
    const float d1 = angleDistanceRad(a.j1_rad, b.j1_rad);
    const float d2 = angleDistanceRad(a.j2_rad, b.j2_rad);
    const float d3 = angleDistanceRad(a.j3_rad, b.j3_rad);
    const float d4 = angleDistanceRad(a.j4_rad, b.j4_rad);
    const float d5 = angleDistanceRad(a.j5_rad, b.j5_rad);
    return d1 * d1 + d2 * d2 + d3 * d3 + d4 * d4 + d5 * d5;
}

float jointMagnitudeSquared(const ArmJointAngles &angles) {
    return angles.j1_rad * angles.j1_rad +
           angles.j2_rad * angles.j2_rad +
           angles.j3_rad * angles.j3_rad +
           angles.j4_rad * angles.j4_rad +
           angles.j5_rad * angles.j5_rad;
}

float candidateScore(const ArmJointAngles &angles) {
    return minJointLimitMargin(angles) - 0.001f * jointMagnitudeSquared(angles);
}

float seededCandidateScore(const ArmJointAngles &angles, const ArmJointAngles &seed_angles) {
    return -jointDistanceSquared(angles, seed_angles);
}

bool solvePlanarTwoLink(
    float wrist_r_m,
    float wrist_z_m,
    float l2_m,
    float l3_m,
    bool elbow_positive,
    float &j2_rad,
    float &j3_rad) {
    const float distance_sq_m =
        wrist_r_m * wrist_r_m + wrist_z_m * wrist_z_m;
    const float distance_m = sqrt(distance_sq_m);
    if (distance_m < IK_EPSILON_M) {
        return false;
    }

    const float max_reach_m = l2_m + l3_m;
    const float min_reach_m = fabs(l2_m - l3_m);
    if (distance_m > max_reach_m + IK_EPSILON_M ||
        distance_m < min_reach_m - IK_EPSILON_M) {
        return false;
    }

    float cos_beta =
        (l2_m * l2_m + distance_sq_m - l3_m * l3_m) /
        (2.0f * l2_m * distance_m);
    if (cos_beta > 1.0f) {
        cos_beta = 1.0f;
    } else if (cos_beta < -1.0f) {
        cos_beta = -1.0f;
    }

    const float target_pitch_rad = atan2(wrist_r_m, wrist_z_m);
    const float beta_rad = acos(cos_beta);
    const float link2_pitch_rad =
        elbow_positive ? target_pitch_rad + beta_rad : target_pitch_rad - beta_rad;

    const float link3_r_m = wrist_r_m - l2_m * sin(link2_pitch_rad);
    const float link3_z_m = wrist_z_m - l2_m * cos(link2_pitch_rad);
    const float link3_pitch_rad = atan2(link3_r_m, link3_z_m);

    j2_rad = normalizeAngleRad(link2_pitch_rad);
    j3_rad = normalizeAngleRad(link3_pitch_rad - link2_pitch_rad);
    return true;
}

bool buildIkCandidate(
    const TcpPose &target_pose,
    const JointOffsets &offsets,
    float tool_pitch_rad,
    bool elbow_positive,
    ArmJointAngles &candidate) {
    const float l45_m = offsets.l4_m + offsets.l5_m;
    const float target_r_m = sqrt(
        target_pose.position.x_m * target_pose.position.x_m +
        target_pose.position.y_m * target_pose.position.y_m);
    const float target_z_m = target_pose.position.z_m - offsets.l1_m;
    const float wrist_r_m = target_r_m - l45_m * sin(tool_pitch_rad);
    const float wrist_z_m = target_z_m - l45_m * cos(tool_pitch_rad);

    float j2_rad = 0.0f;
    float j3_rad = 0.0f;
    if (!solvePlanarTwoLink(
            wrist_r_m,
            wrist_z_m,
            offsets.l2_m,
            offsets.l3_m,
            elbow_positive,
            j2_rad,
            j3_rad)) {
        return false;
    }

    const float j1_rad =
        target_r_m <= IK_EPSILON_M ? 0.0f : atan2(target_pose.position.y_m, target_pose.position.x_m);
    const float j4_rad = normalizeAngleRad(tool_pitch_rad - j2_rad - j3_rad);
    const float j5_rad = normalizeAngleRad(target_pose.tool_roll_rad);

    candidate = {j1_rad, j2_rad, j3_rad, j4_rad, j5_rad};
    return isValidAngles(candidate);
}

KinematicsResult<ArmJointAngles> inverseKinematicsPositionToolRollInternal(
    const TcpPose &target_pose,
    const ArmJointAngles *seed_angles,
    const JointOffsets &offsets) {
    if (!isValidPose(target_pose) || !isValidOffsets(offsets)) {
        return {KinematicsStatus::InvalidInput, emptyAngles()};
    }

    const float target_r_m = sqrt(
        target_pose.position.x_m * target_pose.position.x_m +
        target_pose.position.y_m * target_pose.position.y_m);
    const float target_z_m = target_pose.position.z_m - offsets.l1_m;
    const float max_reach_m = offsets.l2_m + offsets.l3_m + offsets.l4_m + offsets.l5_m;
    if (sqrt(target_r_m * target_r_m + target_z_m * target_z_m) > max_reach_m + IK_EPSILON_M) {
        return {KinematicsStatus::OutOfReach, emptyAngles()};
    }

    ArmJointAngles best_angles = emptyAngles();
    float best_score = -3.402823466e+38f;
    bool has_candidate = false;
    bool has_geometric_candidate = false;

    const float tool_pitch_min_rad =
        JOINT_CALIBRATIONS[1].angle_min_rad +
        JOINT_CALIBRATIONS[2].angle_min_rad +
        JOINT_CALIBRATIONS[3].angle_min_rad;
    const float tool_pitch_max_rad =
        JOINT_CALIBRATIONS[1].angle_max_rad +
        JOINT_CALIBRATIONS[2].angle_max_rad +
        JOINT_CALIBRATIONS[3].angle_max_rad;

    for (uint8_t i = 0; i < IK_TOOL_PITCH_SAMPLE_COUNT; ++i) {
        const float alpha =
            static_cast<float>(i) / static_cast<float>(IK_TOOL_PITCH_SAMPLE_COUNT - 1);
        const float tool_pitch_rad =
            tool_pitch_min_rad + alpha * (tool_pitch_max_rad - tool_pitch_min_rad);

        for (uint8_t elbow = 0; elbow < 2; ++elbow) {
            ArmJointAngles candidate = emptyAngles();
            if (!buildIkCandidate(
                    target_pose,
                    offsets,
                    tool_pitch_rad,
                    elbow == 1,
                    candidate)) {
                continue;
            }

            has_geometric_candidate = true;
            if (!areWithinJointLimits(candidate)) {
                continue;
            }

            const float score = seed_angles == nullptr
                                    ? candidateScore(candidate)
                                    : seededCandidateScore(candidate, *seed_angles);
            if (!has_candidate || score > best_score) {
                best_angles = candidate;
                best_score = score;
                has_candidate = true;
            }
        }
    }

    if (!has_candidate) {
        return {
            has_geometric_candidate ? KinematicsStatus::JointLimitViolation : KinematicsStatus::OutOfReach,
            emptyAngles(),
        };
    }

    return {KinematicsStatus::Ok, best_angles};
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
        normalizeAngleRad(angles.j5_rad),
    };

    return {KinematicsStatus::Ok, pose};
}

KinematicsResult<ArmJointAngles> inverseKinematicsPositionToolRoll(
    const TcpPose &target_pose,
    const JointOffsets &offsets) {
    return inverseKinematicsPositionToolRollInternal(target_pose, nullptr, offsets);
}

KinematicsResult<ArmJointAngles> inverseKinematicsPositionToolRollSeeded(
    const TcpPose &target_pose,
    const ArmJointAngles &seed_angles,
    const JointOffsets &offsets) {
    if (!isValidAngles(seed_angles)) {
        return {KinematicsStatus::InvalidInput, emptyAngles()};
    }

    return inverseKinematicsPositionToolRollInternal(target_pose, &seed_angles, offsets);
}

KinematicsResult<uint16_t> jointAngleToPwmUs(
    uint8_t joint_index,
    float angle_rad,
    const JointCalibration calibrations[ROBOT_TOTAL_JOINT_COUNT]) {
    if (joint_index >= ROBOT_TOTAL_JOINT_COUNT || !isFiniteFloat(angle_rad)) {
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
    const JointCalibration calibrations[ROBOT_TOTAL_JOINT_COUNT]) {
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
    for (uint8_t i = 0; i < ROBOT_KINEMATIC_JOINT_COUNT; ++i) {
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
