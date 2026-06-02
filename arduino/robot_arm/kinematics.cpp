#include "kinematics.h"

#include <math.h>

constexpr float FULL_TURN_RAD = 6.28318530718f;
constexpr float IK_EPSILON_M = 0.000001f;
constexpr float TCP_YAW_EPSILON_RAD = 0.0001f;

static bool isFiniteFloat(float value) {
    return !isnan(value) && !isinf(value);
}

static bool isValidOffsets(const JointOffsets &offsets) {
    return isFiniteFloat(offsets.l1_m) && offsets.l1_m >= 0.0f &&
           isFiniteFloat(offsets.l2_m) && offsets.l2_m >= 0.0f &&
           isFiniteFloat(offsets.l3_m) && offsets.l3_m >= 0.0f &&
           isFiniteFloat(offsets.l4_m) && offsets.l4_m >= 0.0f &&
           isFiniteFloat(offsets.l5_m) && offsets.l5_m >= 0.0f;
}

static bool isValidAngles(const ArmJointAngles &angles) {
    return isFiniteFloat(angles.j0_rad) &&
           isFiniteFloat(angles.j1_rad) &&
           isFiniteFloat(angles.j2_rad) &&
           isFiniteFloat(angles.j3_rad) &&
           isFiniteFloat(angles.j4_rad);
}

static bool isValidPose(const TcpPose &pose) {
    return isFiniteFloat(pose.position.x_m) &&
           isFiniteFloat(pose.position.y_m) &&
           isFiniteFloat(pose.position.z_m) &&
           isFiniteFloat(pose.yaw_rad);
}

static float tcpYawFromPositionRad(const Vector3 &position) {
    const float target_r_m = sqrt(
        position.x_m * position.x_m +
        position.y_m * position.y_m);
    return target_r_m <= IK_EPSILON_M ? 0.0f : atan2(position.y_m, position.x_m);
}

static float normalizeAngleRad(float angle_rad) {
    while (angle_rad > PI) {
        angle_rad -= FULL_TURN_RAD;
    }
    while (angle_rad < -PI) {
        angle_rad += FULL_TURN_RAD;
    }
    return angle_rad;
}

static float angleDistanceRad(float a_rad, float b_rad) {
    return normalizeAngleRad(a_rad - b_rad);
}

static bool isWithinJointLimit(uint8_t joint_index, float angle_rad) {
    const JointCalibration &calibration = JOINT_CALIBRATIONS[joint_index];
    return angle_rad >= calibration.angle_min_rad &&
           angle_rad <= calibration.angle_max_rad;
}

static bool areWithinJointLimits(const ArmJointAngles &angles) {
    return isWithinJointLimit(0, angles.j0_rad) &&
           isWithinJointLimit(1, angles.j1_rad) &&
           isWithinJointLimit(2, angles.j2_rad) &&
           isWithinJointLimit(3, angles.j3_rad) &&
           isWithinJointLimit(4, angles.j4_rad);
}

static float jointDistanceSquared(const ArmJointAngles &a, const ArmJointAngles &b) {
    const float d0 = angleDistanceRad(a.j0_rad, b.j0_rad);
    const float d1 = angleDistanceRad(a.j1_rad, b.j1_rad);
    const float d2 = angleDistanceRad(a.j2_rad, b.j2_rad);
    const float d3 = angleDistanceRad(a.j3_rad, b.j3_rad);
    const float d4 = angleDistanceRad(a.j4_rad, b.j4_rad);
    return d0 * d0 + d1 * d1 + d2 * d2 + d3 * d3 + d4 * d4;
}

ArmJointAngles robotHomeJointAngles() {
    return {
        ROBOT_HOME_J0_RAD,
        ROBOT_HOME_J1_RAD,
        ROBOT_HOME_J2_RAD,
        ROBOT_HOME_J3_RAD,
        ROBOT_HOME_J4_RAD,
    };
}

static float interpolatePwmUs(const JointCalibration &calibration, float angle_rad) {
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

static uint16_t lowerPwmBoundUs(const JointCalibration &calibration) {
    uint16_t lower_us = calibration.pwm_min_us;
    if (calibration.pwm_zero_us < lower_us) {
        lower_us = calibration.pwm_zero_us;
    }
    if (calibration.pwm_max_us < lower_us) {
        lower_us = calibration.pwm_max_us;
    }
    return lower_us;
}

static uint16_t upperPwmBoundUs(const JointCalibration &calibration) {
    uint16_t upper_us = calibration.pwm_min_us;
    if (calibration.pwm_zero_us > upper_us) {
        upper_us = calibration.pwm_zero_us;
    }
    if (calibration.pwm_max_us > upper_us) {
        upper_us = calibration.pwm_max_us;
    }
    return upper_us;
}

static uint16_t roundToUint16(float value) {
    return static_cast<uint16_t>(value + 0.5f);
}

static ArmJointPwmUs emptyPwm() {
    return {0, 0, 0, 0, 0};
}

static ArmJointAngles emptyAngles() {
    return {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
}

static TcpPose emptyPose() {
    return {{0.0f, 0.0f, 0.0f}, 0.0f};
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
        pwm_us < static_cast<float>(lowerPwmBoundUs(calibration)) ||
        pwm_us > static_cast<float>(upperPwmBoundUs(calibration))) {
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
        angles.j0_rad,
        angles.j1_rad,
        angles.j2_rad,
        angles.j3_rad,
        angles.j4_rad,
    };

    uint16_t pwm_values[] = {0, 0, 0, 0, 0};
    for (uint8_t i = 0; i < ROBOT_KINEMATIC_JOINT_COUNT; ++i) {
        const KinematicsResult<uint16_t> result = jointAngleToPwmUs(i, angle_values[i], calibrations);
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

/*
 * Forward kinematics implementation.
 *
 * J0 is the base rotation,
 * J1-J3 are the arm pitch joints,
 * J4 is independent local wrist rotation.
 *
 * The FK convention is that all joints at 0 radians point straight up, so the arm
 * extends in the positive Z direction when all angles are zero. J0 rotates the arm
 * around the vertical axis, and J4 rotates the wrist around the local vertical axis at
 * the end of the arm.
 *
 * The reported TCP pose is represented as XYZ position and yaw resulting from J0.
 * J4 is independent local wrist rotation and does not change the reported TCP pose.
 */
KinematicsResult<TcpPose> forwardKinematics(
    const ArmJointAngles &angles,
    const JointOffsets &offsets) {
    if (!isValidAngles(angles) || !isValidOffsets(offsets)) {
        return {KinematicsStatus::InvalidInput, emptyPose()};
    }

    // J1-J3 are planar pitch joints measured from vertical: 0 rad points up,
    // so horizontal reach uses sin() and vertical height uses cos().
    const float pitch_1 = angles.j1_rad;
    const float pitch_2 = pitch_1 + angles.j2_rad;
    const float pitch_3 = pitch_2 + angles.j3_rad;

    const float reach_m =
        offsets.l2_m * sin(pitch_1) +
        offsets.l3_m * sin(pitch_2) +
        offsets.l4_m * sin(pitch_3) +
        offsets.l5_m * sin(pitch_3);

    const float z_m =
        offsets.l1_m +
        offsets.l2_m * cos(pitch_1) +
        offsets.l3_m * cos(pitch_2) +
        offsets.l4_m * cos(pitch_3) +
        offsets.l5_m * cos(pitch_3);

    TcpPose pose = {
        {
            reach_m * cos(angles.j0_rad),
            reach_m * sin(angles.j0_rad),
            z_m,
        },
        normalizeAngleRad(angles.j0_rad),
    };

    return {KinematicsStatus::Ok, pose};
}

/*
 * Inverse kinematics implementation.
 *
 * The IK solution is computed using a geometric approach. The wrist position is
 * computed from the target TCP pose and the final TCP offset length, and then the shoulder
 * and elbow angles are computed using the planar 2-link IK solution for the arm.
 * Finally, the base angle is computed from the target TCP position and the
 * J4 angle is kept at the seed or HOME value because it is independent of
 * the TCP pose.
 *
 * Both elbow configurations are computed when valid, and the one closest to the
 * provided seed or HOME configuration is selected. Joint limits are checked for
 * both configurations, and if both are valid, the nearest one to the seed or HOME
 * is returned. If only one is valid, it is returned. If neither is valid, an error
 * status is returned indicating whether the target was out of reach or if there
 * was a joint limit violation.
 *
 * The IK convention is that all joints at 0 radians point straight up, so the arm
 * extends in the positive Z direction when all angles are zero. J0 rotates the arm
 * around the vertical axis, and J4 rotates the wrist around the local vertical axis at
 * the end of the arm. The TCP pose convention is XYZ position and yaw following
 * J0.
 *
 * The provided seed angles must be within joint limits. If a seed is provided but
 * invalid, an error status is returned. If no seed is provided, HOME configuration
 * is used as the reference for selecting between multiple IK solutions.
 *
 * The IK solution does not infer J4 from the TCP pose. Multiple J4 angles can
 * result in the same TCP pose, so J4 follows the seed or HOME value while
 * respecting joint limits.
 */

static bool solvePlanarTwoLink(
    float wrist_r_m,
    float wrist_z_m,
    float l2_m,
    float l3_m,
    bool elbow_positive,
    float &j1_rad,
    float &j2_rad) {

    // Check reachability before computing IK to avoid unnecessary calculations and potential numerical issues.
    const float distance_sq_m = wrist_r_m * wrist_r_m + wrist_z_m * wrist_z_m;
    const float distance_m = sqrt(distance_sq_m);
    if (distance_m < IK_EPSILON_M) {
        return false;
    }

    // The wrist position must be reachable by the planar 2-link arm formed by l2 and l3.
    // The reachable workspace is an annular region with inner radius |l2 - l3| and outer radius l2 + l3.
    const float max_reach_m = l2_m + l3_m;
    const float min_reach_m = fabs(l2_m - l3_m);
    if (distance_m > max_reach_m + IK_EPSILON_M ||
        distance_m < min_reach_m - IK_EPSILON_M) {
        return false;
    }

    // Compute the shoulder and elbow angles using the law of cosines and trigonometry.
    // The shoulder angle is computed from the target wrist position and the elbow angle,
    // which is computed from the triangle formed by the shoulder, elbow, and wrist.
    float cos_beta = (l2_m * l2_m + distance_sq_m - l3_m * l3_m) / (2.0f * l2_m * distance_m);
    if (cos_beta > 1.0f) {
        cos_beta = 1.0f;
    } else if (cos_beta < -1.0f) {
        cos_beta = -1.0f;
    }

    // The target pitch is the angle from vertical to the line connecting the shoulder to the wrist.
    const float target_pitch_rad = atan2(wrist_r_m, wrist_z_m);
    const float beta_rad = acos(cos_beta);
    const float link2_pitch_rad = elbow_positive ? target_pitch_rad + beta_rad : target_pitch_rad - beta_rad;
    const float link3_r_m = wrist_r_m - l2_m * sin(link2_pitch_rad);
    const float link3_z_m = wrist_z_m - l2_m * cos(link2_pitch_rad);
    const float link3_pitch_rad = atan2(link3_r_m, link3_z_m);

    j1_rad = normalizeAngleRad(link2_pitch_rad);
    j2_rad = normalizeAngleRad(link3_pitch_rad - link2_pitch_rad);
    return true;
}

static bool buildIkCandidate(
    const TcpPose &target_pose,
    const JointOffsets &offsets,
    float tcp_fixed_pitch_rad,
    float j4_rad_from_seed,
    bool elbow_positive,
    ArmJointAngles &candidate) {

    // Compute the wrist position from the target TCP pose and the final TCP offset length.
    // The wrist position is the position of the joint before the final TCP offset,
    // using the configured fixed TCP pitch convention.
    const float target_r_m = sqrt(target_pose.position.x_m * target_pose.position.x_m + target_pose.position.y_m * target_pose.position.y_m);
    const float target_z_m = target_pose.position.z_m - offsets.l1_m;
    const float tcp_offset_length_m = offsets.l4_m + offsets.l5_m;
    const float wrist_r_m = target_r_m - tcp_offset_length_m * sin(tcp_fixed_pitch_rad);
    const float wrist_z_m = target_z_m - tcp_offset_length_m * cos(tcp_fixed_pitch_rad);

    // Compute the shoulder and elbow angles using the planar 2-link IK solution for the arm.
    float j1_rad = 0.0f;
    float j2_rad = 0.0f;
    if (!solvePlanarTwoLink(
            wrist_r_m,
            wrist_z_m,
            offsets.l2_m,
            offsets.l3_m,
            elbow_positive,
            j1_rad,
            j2_rad)) {
        return false;
    }

    // Compute the base angle from the target TCP position. The TCP yaw is reported
    // from J0, so it is validated by FK rather than used as an independent command.
    const float j0_rad = target_r_m <= IK_EPSILON_M ? 0.0f : atan2(target_pose.position.y_m, target_pose.position.x_m);
    const float j3_rad = normalizeAngleRad(tcp_fixed_pitch_rad - j1_rad - j2_rad);
    const float j4_rad = normalizeAngleRad(j4_rad_from_seed);

    candidate = {j0_rad, j1_rad, j2_rad, j3_rad, j4_rad};
    return isValidAngles(candidate);
}

static KinematicsResult<ArmJointAngles> inverseKinematicsPositionYawInternal(
    const TcpPose &target_pose,
    const ArmJointAngles *seed_angles,
    const JointOffsets &offsets) {
    const ArmJointAngles home_angles = robotHomeJointAngles();
    if (!isValidPose(target_pose) ||
        !isValidOffsets(offsets) ||
        !isValidAngles(home_angles) ||
        !isFiniteFloat(FIXED_TCP_OFFSET_PITCH_RAD)) {
        return {KinematicsStatus::InvalidInput, emptyAngles()};
    }

    // Check reachability before computing IK candidates to avoid unnecessary calculations.
    const float target_r_m = sqrt(
        target_pose.position.x_m * target_pose.position.x_m +
        target_pose.position.y_m * target_pose.position.y_m);
    if (fabs(angleDistanceRad(target_pose.yaw_rad, tcpYawFromPositionRad(target_pose.position))) > TCP_YAW_EPSILON_RAD) {
        return {KinematicsStatus::InvalidInput, emptyAngles()};
    }

    const float target_z_m = target_pose.position.z_m - offsets.l1_m;
    const float max_reach_m = offsets.l2_m + offsets.l3_m + offsets.l4_m + offsets.l5_m;
    if (sqrt(target_r_m * target_r_m + target_z_m * target_z_m) > max_reach_m + IK_EPSILON_M) {
        return {KinematicsStatus::OutOfReach, emptyAngles()};
    }

    // Compute both elbow configurations and select the nearest valid candidate
    // to the seed when provided, otherwise to the configured HOME pose.
    ArmJointAngles best_angles = emptyAngles();
    float best_distance = 3.402823466e+38f;
    bool has_candidate = false;
    bool has_geometric_candidate = false;
    const ArmJointAngles reference_angles = seed_angles == nullptr ? home_angles : *seed_angles;
    for (uint8_t elbow = 0; elbow < 2; ++elbow) {
        ArmJointAngles candidate = emptyAngles();
        if (!buildIkCandidate(
                target_pose,
                offsets,
                FIXED_TCP_OFFSET_PITCH_RAD,
                reference_angles.j4_rad,
                elbow == 1,
                candidate)) {
            continue;
        }

        // A geometric IK solution was found for this elbow configuration, even if it may violate joint limits.
        has_geometric_candidate = true;
        if (!areWithinJointLimits(candidate)) {
            continue;
        }

        // This candidate is valid, check if it's the best one we've found so far.
        const float distance = jointDistanceSquared(candidate, reference_angles);
        if (!has_candidate || distance < best_distance) {
            best_angles = candidate;
            best_distance = distance;
            has_candidate = true;
        }
    }

    // If neither elbow configuration yields a valid solution,
    // return an error status indicating whether the target was out of reach or if there was a joint limit violation.
    if (!has_candidate) {
        return {
            has_geometric_candidate ? KinematicsStatus::JointLimitViolation : KinematicsStatus::OutOfReach,
            emptyAngles(),
        };
    }

    return {KinematicsStatus::Ok, best_angles};
}

KinematicsResult<ArmJointAngles> inverseKinematicsPositionYaw(
    const TcpPose &target_pose,
    const JointOffsets &offsets) {
    return inverseKinematicsPositionYawInternal(target_pose, nullptr, offsets);
}

KinematicsResult<ArmJointAngles> inverseKinematicsPositionYawSeeded(
    const TcpPose &target_pose,
    const ArmJointAngles &seed_angles,
    const JointOffsets &offsets) {
    if (!isValidAngles(seed_angles)) {
        return {KinematicsStatus::InvalidInput, emptyAngles()};
    }

    return inverseKinematicsPositionYawInternal(target_pose, &seed_angles, offsets);
}
