#include "kinematics.h"

#include <math.h>

constexpr float FULL_TURN_RAD = 6.28318530718f;
constexpr float IK_EPSILON = 0.000001f;

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
    return isFiniteFloat(pose.reach_x_m) &&
           isFiniteFloat(pose.reach_z_m) &&
           isFiniteFloat(pose.yaw_rad);
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
        const float pwm_span = static_cast<float>(calibration.pwm_max_us) -
                               static_cast<float>(calibration.pwm_zero_us);
        return static_cast<float>(calibration.pwm_zero_us) +
               ((angle_rad - calibration.angle_zero_rad) * pwm_span) / angle_span;
    }

    const float angle_span = calibration.angle_zero_rad - calibration.angle_min_rad;
    const float pwm_span = static_cast<float>(calibration.pwm_zero_us) -
                           static_cast<float>(calibration.pwm_min_us);
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
    return {0.0f, 0.0f, 0.0f};
}

const char *kinematicsStatusName(KinematicsStatus status) {
    switch (status) {
        case KinematicsStatus::Ok:
            return "Ok";
        case KinematicsStatus::OutOfReach:
            return "OutOfReach";
        case KinematicsStatus::JointLimitViolation:
            return "JointLimitViolation";
        case KinematicsStatus::JointLimitBlocked:
            return "JointLimitBlocked";
        case KinematicsStatus::NoProgress:
            return "NoProgress";
        case KinematicsStatus::NotImplemented:
            return "NotImplemented";
        case KinematicsStatus::InvalidInput:
            return "InvalidInput";
    }

    return "Unknown";
}

JacobianIkConfig defaultJacobianIkConfig() {
    return {
        0.02f,
        500,
        0.08f,
        20.0f,
        20.0f,
        1.0f,
        1.5f,
        1.5f,
        1.5f,
        1.5f,
        {0.5f, 0.5f, 0.5f, 0.5f, 0.5f},
        0.003f,
        0.02f,
        0.02f,
        0.0005f,
        25,
    };
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
 * The reported TCP pose is represented as planar reach, height, and yaw
 * resulting from J0. J4 is independent local wrist rotation and does not
 * change the reported TCP pose.
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

    TcpPose pose = {reach_m, z_m, normalizeAngleRad(angles.j0_rad)};

    return {KinematicsStatus::Ok, pose};
}

static bool isValidJacobianIkTarget(const JacobianIkTarget &target) {
    return isValidPose(target.tcp_pose) &&
           isFiniteFloat(target.j3_rad) &&
           isFiniteFloat(target.j4_rad);
}

static bool isValidJacobianIkConfig(const JacobianIkConfig &config) {
    if (!isFiniteFloat(config.dt_s) || config.dt_s <= 0.0f ||
        config.max_iterations == 0 ||
        !isFiniteFloat(config.damping_lambda) || config.damping_lambda < 0.0f ||
        !isFiniteFloat(config.reach_weight_per_m) || config.reach_weight_per_m <= 0.0f ||
        !isFiniteFloat(config.z_weight_per_m) || config.z_weight_per_m <= 0.0f ||
        !isFiniteFloat(config.j3_weight_per_rad) || config.j3_weight_per_rad <= 0.0f ||
        !isFiniteFloat(config.position_gain_per_s) || config.position_gain_per_s < 0.0f ||
        !isFiniteFloat(config.angle_gain_per_s) || config.angle_gain_per_s < 0.0f ||
        !isFiniteFloat(config.yaw_gain_per_s) || config.yaw_gain_per_s < 0.0f ||
        !isFiniteFloat(config.j4_gain_per_s) || config.j4_gain_per_s < 0.0f ||
        !isFiniteFloat(config.position_tolerance_m) || config.position_tolerance_m < 0.0f ||
        !isFiniteFloat(config.angle_tolerance_rad) || config.angle_tolerance_rad < 0.0f ||
        !isFiniteFloat(config.min_step_alpha) || config.min_step_alpha < 0.0f ||
        !isFiniteFloat(config.min_error_progress) || config.min_error_progress < 0.0f ||
        config.no_progress_iteration_limit == 0) {
        return false;
    }

    for (uint8_t i = 0; i < ROBOT_KINEMATIC_JOINT_COUNT; ++i) {
        if (!isFiniteFloat(config.joint_velocity_limit_rad_s[i]) ||
            config.joint_velocity_limit_rad_s[i] <= 0.0f) {
            return false;
        }
    }

    return true;
}

static JacobianIkStepResult emptyStepResult(KinematicsStatus status, const ArmJointAngles &current) {
    return {
        status,
        current,
        {0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        false,
    };
}

static float maxAbs3(float a, float b, float c) {
    float result = fabs(a);
    if (fabs(b) > result) {
        result = fabs(b);
    }
    if (fabs(c) > result) {
        result = fabs(c);
    }
    return result;
}

static bool solveLinear3x3(float a[3][3], float b[3], float x[3]) {
    for (uint8_t pivot = 0; pivot < 3; ++pivot) {
        uint8_t best_row = pivot;
        float best_abs = fabs(a[pivot][pivot]);
        for (uint8_t row = pivot + 1; row < 3; ++row) {
            const float candidate_abs = fabs(a[row][pivot]);
            if (candidate_abs > best_abs) {
                best_abs = candidate_abs;
                best_row = row;
            }
        }

        if (best_abs < IK_EPSILON) {
            return false;
        }

        if (best_row != pivot) {
            for (uint8_t col = pivot; col < 3; ++col) {
                const float tmp = a[pivot][col];
                a[pivot][col] = a[best_row][col];
                a[best_row][col] = tmp;
            }
            const float tmp_b = b[pivot];
            b[pivot] = b[best_row];
            b[best_row] = tmp_b;
        }

        const float pivot_value = a[pivot][pivot];
        for (uint8_t row = pivot + 1; row < 3; ++row) {
            const float factor = a[row][pivot] / pivot_value;
            a[row][pivot] = 0.0f;
            for (uint8_t col = pivot + 1; col < 3; ++col) {
                a[row][col] -= factor * a[pivot][col];
            }
            b[row] -= factor * b[pivot];
        }
    }

    for (int8_t row = 2; row >= 0; --row) {
        float sum = b[row];
        for (uint8_t col = row + 1; col < 3; ++col) {
            sum -= a[row][col] * x[col];
        }
        if (fabs(a[row][row]) < IK_EPSILON) {
            return false;
        }
        x[row] = sum / a[row][row];
    }

    return isFiniteFloat(x[0]) && isFiniteFloat(x[1]) && isFiniteFloat(x[2]);
}

static float limitAlphaForJointVelocity(float q_dot_rad_s, float limit_rad_s) {
    const float abs_q_dot = fabs(q_dot_rad_s);
    if (abs_q_dot <= limit_rad_s) {
        return 1.0f;
    }
    return limit_rad_s / abs_q_dot;
}

static float limitAlphaForJointPosition(uint8_t joint_index, float current_rad, float delta_rad) {
    const JointCalibration &calibration = JOINT_CALIBRATIONS[joint_index];
    if (delta_rad > 0.0f) {
        return (calibration.angle_max_rad - current_rad) / delta_rad;
    }
    if (delta_rad < 0.0f) {
        return (calibration.angle_min_rad - current_rad) / delta_rad;
    }
    return 1.0f;
}

static void stopOutwardVelocityAtJointLimit(uint8_t joint_index, float current_rad, float &q_dot_rad_s) {
    const JointCalibration &calibration = JOINT_CALIBRATIONS[joint_index];
    if (current_rad >= calibration.angle_max_rad - IK_EPSILON && q_dot_rad_s > 0.0f) {
        q_dot_rad_s = 0.0f;
    } else if (current_rad <= calibration.angle_min_rad + IK_EPSILON && q_dot_rad_s < 0.0f) {
        q_dot_rad_s = 0.0f;
    }
}

static void clampAlpha(float &alpha, float candidate) {
    if (candidate < alpha) {
        alpha = candidate;
    }
    if (alpha < 0.0f) {
        alpha = 0.0f;
    }
    if (alpha > 1.0f) {
        alpha = 1.0f;
    }
}

JacobianIkStepResult computeJacobianIkStep(
    const ArmJointAngles &current,
    const JacobianIkTarget &target,
    const JacobianIkConfig &config,
    const JointOffsets &offsets) {
    if (!isValidAngles(current) ||
        !isValidJacobianIkTarget(target) ||
        !isValidJacobianIkConfig(config) ||
        !isValidOffsets(offsets)) {
        return emptyStepResult(KinematicsStatus::InvalidInput, current);
    }

    if (!areWithinJointLimits(current)) {
        return emptyStepResult(KinematicsStatus::JointLimitViolation, current);
    }

    const KinematicsResult<TcpPose> fk_result = forwardKinematics(current, offsets);
    if (fk_result.status != KinematicsStatus::Ok) {
        return emptyStepResult(fk_result.status, current);
    }

    const float reach_error_m = target.tcp_pose.reach_x_m - fk_result.value.reach_x_m;
    const float z_error_m = target.tcp_pose.reach_z_m - fk_result.value.reach_z_m;
    const float yaw_error_rad = angleDistanceRad(target.tcp_pose.yaw_rad, current.j0_rad);
    const float j3_error_rad = angleDistanceRad(target.j3_rad, current.j3_rad);
    const float j4_error_rad = angleDistanceRad(target.j4_rad, current.j4_rad);

    const bool converged =
        fabs(reach_error_m) <= config.position_tolerance_m &&
        fabs(z_error_m) <= config.position_tolerance_m &&
        fabs(yaw_error_rad) <= config.angle_tolerance_rad &&
        fabs(j3_error_rad) <= config.angle_tolerance_rad &&
        fabs(j4_error_rad) <= config.angle_tolerance_rad;

    JacobianIkStepResult result = {
        KinematicsStatus::Ok,
        current,
        {0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
        1.0f,
        reach_error_m,
        z_error_m,
        yaw_error_rad,
        j3_error_rad,
        j4_error_rad,
        converged,
    };

    if (converged) {
        return result;
    }

    const float pitch_1 = current.j1_rad;
    const float pitch_2 = pitch_1 + current.j2_rad;
    const float pitch_3 = pitch_2 + current.j3_rad;
    const float tcp_offset_m = offsets.l4_m + offsets.l5_m;

    float jacobian[3][3] = {
        {
            offsets.l2_m * cos(pitch_1) + offsets.l3_m * cos(pitch_2) + tcp_offset_m * cos(pitch_3),
            offsets.l3_m * cos(pitch_2) + tcp_offset_m * cos(pitch_3),
            tcp_offset_m * cos(pitch_3),
        },
        {
            -offsets.l2_m * sin(pitch_1) - offsets.l3_m * sin(pitch_2) - tcp_offset_m * sin(pitch_3),
            -offsets.l3_m * sin(pitch_2) - tcp_offset_m * sin(pitch_3),
            -tcp_offset_m * sin(pitch_3),
        },
        {0.0f, 0.0f, 1.0f},
    };
    const float weights[3] = {
        config.reach_weight_per_m,
        config.z_weight_per_m,
        config.j3_weight_per_rad,
    };
    const float task_velocity[3] = {
        config.position_gain_per_s * reach_error_m,
        config.position_gain_per_s * z_error_m,
        config.angle_gain_per_s * j3_error_rad,
    };

    float normal[3][3] = {
        {0.0f, 0.0f, 0.0f},
        {0.0f, 0.0f, 0.0f},
        {0.0f, 0.0f, 0.0f},
    };
    float rhs[3] = {0.0f, 0.0f, 0.0f};

    for (uint8_t row = 0; row < 3; ++row) {
        const float weighted_task = weights[row] * task_velocity[row];
        for (uint8_t col = 0; col < 3; ++col) {
            const float weighted_j = weights[row] * jacobian[row][col];
            rhs[col] += weighted_j * weighted_task;
            for (uint8_t other = 0; other < 3; ++other) {
                normal[col][other] += weighted_j * weights[row] * jacobian[row][other];
            }
        }
    }

    const float damping_sq = config.damping_lambda * config.damping_lambda;
    for (uint8_t i = 0; i < 3; ++i) {
        normal[i][i] += damping_sq;
    }

    float planar_q_dot[3] = {0.0f, 0.0f, 0.0f};
    if (!solveLinear3x3(normal, rhs, planar_q_dot)) {
        return emptyStepResult(KinematicsStatus::NoProgress, current);
    }

    result.q_dot_rad_s[0] = config.yaw_gain_per_s * yaw_error_rad;
    result.q_dot_rad_s[1] = planar_q_dot[0];
    result.q_dot_rad_s[2] = planar_q_dot[1];
    result.q_dot_rad_s[3] = planar_q_dot[2];
    result.q_dot_rad_s[4] = config.j4_gain_per_s * j4_error_rad;

    const float current_values[ROBOT_KINEMATIC_JOINT_COUNT] = {
        current.j0_rad,
        current.j1_rad,
        current.j2_rad,
        current.j3_rad,
        current.j4_rad,
    };
    for (uint8_t i = 0; i < ROBOT_KINEMATIC_JOINT_COUNT; ++i) {
        stopOutwardVelocityAtJointLimit(i, current_values[i], result.q_dot_rad_s[i]);
    }

    float alpha = 1.0f;
    for (uint8_t i = 0; i < ROBOT_KINEMATIC_JOINT_COUNT; ++i) {
        clampAlpha(alpha, limitAlphaForJointVelocity(
                           result.q_dot_rad_s[i],
                           config.joint_velocity_limit_rad_s[i]));
    }

    for (uint8_t i = 0; i < ROBOT_KINEMATIC_JOINT_COUNT; ++i) {
        const float delta_rad = result.q_dot_rad_s[i] * config.dt_s;
        clampAlpha(alpha, limitAlphaForJointPosition(i, current_values[i], delta_rad));
    }

    result.alpha = alpha;
    result.next_angles = {
        normalizeAngleRad(current.j0_rad + alpha * result.q_dot_rad_s[0] * config.dt_s),
        normalizeAngleRad(current.j1_rad + alpha * result.q_dot_rad_s[1] * config.dt_s),
        normalizeAngleRad(current.j2_rad + alpha * result.q_dot_rad_s[2] * config.dt_s),
        normalizeAngleRad(current.j3_rad + alpha * result.q_dot_rad_s[3] * config.dt_s),
        normalizeAngleRad(current.j4_rad + alpha * result.q_dot_rad_s[4] * config.dt_s),
    };

    if (!areWithinJointLimits(result.next_angles)) {
        return emptyStepResult(KinematicsStatus::JointLimitViolation, current);
    }

    if (alpha <= config.min_step_alpha &&
        maxAbs3(reach_error_m, z_error_m, 0.0f) > config.position_tolerance_m) {
        result.status = KinematicsStatus::JointLimitBlocked;
    } else if (alpha <= config.min_step_alpha &&
               maxAbs3(yaw_error_rad, j3_error_rad, j4_error_rad) > config.angle_tolerance_rad) {
        result.status = KinematicsStatus::JointLimitBlocked;
    }

    return result;
}
