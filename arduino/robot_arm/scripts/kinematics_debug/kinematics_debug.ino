/**
 * Kinematics debug sketch for the robot arm.
 * This program validates FK, IK, Cartesian trajectory math, and angle-to-PWM conversion.
 */

#include <math.h>

#include "kinematics.h"
#include "robot_calibration.h"

void printStatus(const char *label, KinematicsStatus status);
void printPose(const TcpPose &pose);
void printAngles(const ArmJointAngles &angles);
void printPwm(const ArmJointPwmUs &pwm_us);
void printFkZeroTest(const TcpPose &pose);
void runPwmInterpolationTests();
void printPwmInterpolationTest(
    const char *label,
    uint8_t joint_index,
    float angle_rad,
    uint16_t expected_pwm_us,
    KinematicsStatus expected_status);
void runIkStaticTests();
void runIkRoundTripTest(
    const char *label,
    const ArmJointAngles &source_angles,
    bool expected_reachable);
void runIkPoseTest(
    const char *label,
    const TcpPose &target_pose,
    bool expected_reachable,
    const ArmJointAngles *seed_angles);
void runTrajectoryTest();
TcpPose interpolatePose(const TcpPose &start_pose, const TcpPose &end_pose, float alpha);
float angleDistanceRad(float a_rad, float b_rad);
float positionErrorM(const Vector3 &a, const Vector3 &b);
float maxJointDeltaRad(const ArmJointAngles &a, const ArmJointAngles &b);

void setup() {
    Serial.begin(115200);
    delay(100);

    const ArmJointAngles zero_angles = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    const KinematicsResult<TcpPose> fk_zero = forwardKinematics(zero_angles);
    printStatus("FK zero", fk_zero.status);
    if (fk_zero.status == KinematicsStatus::Ok) {
        printPose(fk_zero.value);
        printFkZeroTest(fk_zero.value);
    }

    const ArmJointAngles tool_yaw_angles = {0.25f, 0.0f, 0.0f, 0.0f, 0.50f};
    const KinematicsResult<TcpPose> fk_tool_yaw = forwardKinematics(tool_yaw_angles);
    printStatus("FK tool yaw", fk_tool_yaw.status);
    if (fk_tool_yaw.status == KinematicsStatus::Ok) {
        printPose(fk_tool_yaw.value);
    }

    const KinematicsResult<ArmJointPwmUs> zero_pwm = jointAnglesToPwmUs(zero_angles);
    printStatus("PWM zero", zero_pwm.status);
    if (zero_pwm.status == KinematicsStatus::Ok) {
        printPwm(zero_pwm.value);
    }

    runPwmInterpolationTests();

    const ArmJointAngles out_of_limit_angles = {2.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    const KinematicsResult<ArmJointPwmUs> out_of_limit_pwm =
        jointAnglesToPwmUs(out_of_limit_angles);
    printStatus("PWM limit", out_of_limit_pwm.status);

    const KinematicsResult<ArmJointAngles> ik_result =
        inverseKinematicsPositionPitchYaw(fk_zero.value);
    printStatus("IK position pitch yaw", ik_result.status);

    runIkStaticTests();
    runTrajectoryTest();
}

void loop() {
}

void printStatus(const char *label, KinematicsStatus status) {
    Serial.print(label);
    Serial.print(" status=");
    Serial.println(static_cast<int>(status));
}

void printPose(const TcpPose &pose) {
    Serial.print("pose x_m=");
    Serial.print(pose.position.x_m, 6);
    Serial.print(" y_m=");
    Serial.print(pose.position.y_m, 6);
    Serial.print(" z_m=");
    Serial.print(pose.position.z_m, 6);
    Serial.print(" tool_pitch_rad=");
    Serial.print(pose.tool_pitch_rad, 6);
    Serial.print(" tool_yaw_rad=");
    Serial.println(pose.tool_yaw_rad, 6);
}

void printAngles(const ArmJointAngles &angles) {
    Serial.print("angles j0=");
    Serial.print(angles.j0_rad, 6);
    Serial.print(" j1=");
    Serial.print(angles.j1_rad, 6);
    Serial.print(" j2=");
    Serial.print(angles.j2_rad, 6);
    Serial.print(" j3=");
    Serial.print(angles.j3_rad, 6);
    Serial.print(" j4=");
    Serial.println(angles.j4_rad, 6);
}

void printPwm(const ArmJointPwmUs &pwm_us) {
    Serial.print("pwm j0=");
    Serial.print(pwm_us.j0_us);
    Serial.print(" j1=");
    Serial.print(pwm_us.j1_us);
    Serial.print(" j2=");
    Serial.print(pwm_us.j2_us);
    Serial.print(" j3=");
    Serial.print(pwm_us.j3_us);
    Serial.print(" j4=");
    Serial.println(pwm_us.j4_us);
}

void runIkStaticTests() {
    Serial.println("IK static tests started");

    runIkRoundTripTest(
        "IK reachable +X",
        {0.0f, 0.35f, -0.45f, 0.25f, 0.0f},
        true);
    runIkRoundTripTest(
        "IK reachable nonzero Y",
        {0.35f, 0.25f, -0.35f, 0.20f, 0.10f},
        true);
    runIkRoundTripTest(
        "IK tool yaw reconstruction",
        {0.20f, 0.20f, -0.30f, 0.15f, 0.45f},
        true);

    const TcpPose out_of_reach_pose = {
        {JOINT_OFFSETS.l2_m + JOINT_OFFSETS.l3_m + JOINT_OFFSETS.l4_m + JOINT_OFFSETS.l5_m + 0.10f,
         0.0f,
         JOINT_OFFSETS.l1_m},
        0.0f,
        0.0f,
    };
    runIkPoseTest("IK out of reach", out_of_reach_pose, false, nullptr);

    const TcpPose invalid_pose = {{NAN, 0.0f, JOINT_OFFSETS.l1_m}, 0.0f, 0.0f};
    runIkPoseTest("IK invalid input", invalid_pose, false, nullptr);
}

void runIkRoundTripTest(
    const char *label,
    const ArmJointAngles &source_angles,
    bool expected_reachable) {
    const KinematicsResult<TcpPose> fk_result = forwardKinematics(source_angles);
    printStatus(label, fk_result.status);
    if (fk_result.status != KinematicsStatus::Ok) {
        Serial.print(label);
        Serial.println(" result=FAIL FK source");
        return;
    }

    runIkPoseTest(label, fk_result.value, expected_reachable, nullptr);
}

void runIkPoseTest(
    const char *label,
    const TcpPose &target_pose,
    bool expected_reachable,
    const ArmJointAngles *seed_angles) {
    const KinematicsResult<ArmJointAngles> ik_result =
        seed_angles == nullptr
            ? inverseKinematicsPositionPitchYaw(target_pose)
            : inverseKinematicsPositionPitchYawSeeded(target_pose, *seed_angles);

    Serial.print(label);
    Serial.print(" ik_status=");
    Serial.print(static_cast<int>(ik_result.status));

    if (!expected_reachable) {
        Serial.print(" result=");
        Serial.println(ik_result.status == KinematicsStatus::Ok ? "FAIL" : "PASS");
        return;
    }

    if (ik_result.status != KinematicsStatus::Ok) {
        Serial.println(" result=FAIL");
        return;
    }

    const KinematicsResult<TcpPose> fk_result = forwardKinematics(ik_result.value);
    if (fk_result.status != KinematicsStatus::Ok) {
        Serial.print(" fk_status=");
        Serial.print(static_cast<int>(fk_result.status));
        Serial.println(" result=FAIL");
        return;
    }

    const float pos_error_m =
        positionErrorM(fk_result.value.position, target_pose.position);
    const float tool_pitch_error_rad =
        fabs(angleDistanceRad(fk_result.value.tool_pitch_rad, target_pose.tool_pitch_rad));
    const float tool_yaw_error_rad =
        fabs(angleDistanceRad(fk_result.value.tool_yaw_rad, target_pose.tool_yaw_rad));
    const bool passed =
        pos_error_m <= 0.005f &&
        tool_pitch_error_rad <= 0.03f &&
        tool_yaw_error_rad <= 0.03f;

    Serial.print(" pos_error_m=");
    Serial.print(pos_error_m, 6);
    Serial.print(" tool_pitch_error_rad=");
    Serial.print(tool_pitch_error_rad, 6);
    Serial.print(" tool_yaw_error_rad=");
    Serial.print(tool_yaw_error_rad, 6);
    Serial.print(" result=");
    Serial.println(passed ? "PASS" : "FAIL");
    printPose(target_pose);
    printAngles(ik_result.value);
}

void runTrajectoryTest() {
    Serial.println("Cartesian trajectory test started");

    const ArmJointAngles start_angles = {0.05f, 0.22f, -0.30f, 0.16f, 0.02f};
    const ArmJointAngles end_angles = {0.30f, 0.30f, -0.36f, 0.18f, -0.08f};
    const KinematicsResult<TcpPose> start_fk = forwardKinematics(start_angles);
    const KinematicsResult<TcpPose> end_fk = forwardKinematics(end_angles);
    if (start_fk.status != KinematicsStatus::Ok || end_fk.status != KinematicsStatus::Ok) {
        Serial.println("Cartesian trajectory result=FAIL FK endpoints");
        return;
    }

    constexpr uint8_t SAMPLE_COUNT = 10;
    ArmJointAngles previous_angles = start_angles;
    bool has_previous = false;
    bool passed = true;
    float max_delta_rad = 0.0f;

    for (uint8_t i = 0; i <= SAMPLE_COUNT; ++i) {
        const float alpha = static_cast<float>(i) / static_cast<float>(SAMPLE_COUNT);
        const TcpPose target_pose = interpolatePose(start_fk.value, end_fk.value, alpha);
        const KinematicsResult<ArmJointAngles> ik_result =
            has_previous
                ? inverseKinematicsPositionPitchYawSeeded(target_pose, previous_angles)
                : inverseKinematicsPositionPitchYaw(target_pose);

        Serial.print("Trajectory sample ");
        Serial.print(i);
        Serial.print(" ik_status=");
        Serial.print(static_cast<int>(ik_result.status));

        if (ik_result.status != KinematicsStatus::Ok) {
            Serial.println(" result=FAIL");
            passed = false;
            continue;
        }

        const KinematicsResult<TcpPose> fk_result = forwardKinematics(ik_result.value);
        const float pos_error_m =
            fk_result.status == KinematicsStatus::Ok
                ? positionErrorM(fk_result.value.position, target_pose.position)
                : 999.0f;
        const float tool_pitch_error_rad =
            fk_result.status == KinematicsStatus::Ok
                ? fabs(angleDistanceRad(fk_result.value.tool_pitch_rad, target_pose.tool_pitch_rad))
                : 999.0f;
        const float tool_yaw_error_rad =
            fk_result.status == KinematicsStatus::Ok
                ? fabs(angleDistanceRad(fk_result.value.tool_yaw_rad, target_pose.tool_yaw_rad))
                : 999.0f;
        const float delta_rad =
            has_previous ? maxJointDeltaRad(ik_result.value, previous_angles) : 0.0f;

        if (delta_rad > max_delta_rad) {
            max_delta_rad = delta_rad;
        }

        const bool sample_passed =
            fk_result.status == KinematicsStatus::Ok &&
            pos_error_m <= 0.005f &&
            tool_pitch_error_rad <= 0.03f &&
            tool_yaw_error_rad <= 0.03f &&
            (!has_previous || delta_rad <= 0.35f);
        passed = passed && sample_passed;

        Serial.print(" pos_error_m=");
        Serial.print(pos_error_m, 6);
        Serial.print(" tool_pitch_error_rad=");
        Serial.print(tool_pitch_error_rad, 6);
        Serial.print(" tool_yaw_error_rad=");
        Serial.print(tool_yaw_error_rad, 6);
        Serial.print(" max_joint_delta_rad=");
        Serial.print(delta_rad, 6);
        Serial.print(" result=");
        Serial.println(sample_passed ? "PASS" : "FAIL");

        previous_angles = ik_result.value;
        has_previous = true;
    }

    Serial.print("Cartesian trajectory max_joint_delta_rad=");
    Serial.print(max_delta_rad, 6);
    Serial.print(" result=");
    Serial.println(passed ? "PASS" : "FAIL");
}

TcpPose interpolatePose(const TcpPose &start_pose, const TcpPose &end_pose, float alpha) {
    const float tool_pitch_delta_rad = angleDistanceRad(end_pose.tool_pitch_rad, start_pose.tool_pitch_rad);
    const float tool_yaw_delta_rad = angleDistanceRad(end_pose.tool_yaw_rad, start_pose.tool_yaw_rad);
    return {
        {
            start_pose.position.x_m +
                alpha * (end_pose.position.x_m - start_pose.position.x_m),
            start_pose.position.y_m +
                alpha * (end_pose.position.y_m - start_pose.position.y_m),
            start_pose.position.z_m +
                alpha * (end_pose.position.z_m - start_pose.position.z_m),
        },
        start_pose.tool_pitch_rad + alpha * tool_pitch_delta_rad,
        start_pose.tool_yaw_rad + alpha * tool_yaw_delta_rad,
    };
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

float positionErrorM(const Vector3 &a, const Vector3 &b) {
    const float dx_m = a.x_m - b.x_m;
    const float dy_m = a.y_m - b.y_m;
    const float dz_m = a.z_m - b.z_m;
    return sqrt(dx_m * dx_m + dy_m * dy_m + dz_m * dz_m);
}

float maxJointDeltaRad(const ArmJointAngles &a, const ArmJointAngles &b) {
    float max_delta_rad = fabs(angleDistanceRad(a.j0_rad, b.j0_rad));
    const float j1_delta_rad = fabs(angleDistanceRad(a.j1_rad, b.j1_rad));
    const float j2_delta_rad = fabs(angleDistanceRad(a.j2_rad, b.j2_rad));
    const float j3_delta_rad = fabs(angleDistanceRad(a.j3_rad, b.j3_rad));
    const float j4_delta_rad = fabs(angleDistanceRad(a.j4_rad, b.j4_rad));

    if (j1_delta_rad > max_delta_rad) {
        max_delta_rad = j1_delta_rad;
    }
    if (j2_delta_rad > max_delta_rad) {
        max_delta_rad = j2_delta_rad;
    }
    if (j3_delta_rad > max_delta_rad) {
        max_delta_rad = j3_delta_rad;
    }
    if (j4_delta_rad > max_delta_rad) {
        max_delta_rad = j4_delta_rad;
    }
    return max_delta_rad;
}

void printFkZeroTest(const TcpPose &pose) {
    const float expected_z_m =
        JOINT_OFFSETS.l1_m +
        JOINT_OFFSETS.l2_m +
        JOINT_OFFSETS.l3_m +
        JOINT_OFFSETS.l4_m +
        JOINT_OFFSETS.l5_m;
    const float tolerance_m = 0.0001f;
    const bool passed =
        fabs(pose.position.x_m) <= tolerance_m &&
        fabs(pose.position.y_m) <= tolerance_m &&
        fabs(pose.position.z_m - expected_z_m) <= tolerance_m &&
        fabs(pose.tool_pitch_rad) <= tolerance_m &&
        fabs(pose.tool_yaw_rad) <= tolerance_m;

    Serial.print("FK zero expected x_m=0.000000 y_m=0.000000 z_m=");
    Serial.print(expected_z_m, 6);
    Serial.print(" tool_pitch_rad=0.000000 tool_yaw_rad=0.000000 result=");
    Serial.println(passed ? "PASS" : "FAIL");
}

void runPwmInterpolationTests() {
    const JointCalibration &joint = JOINT_CALIBRATIONS[0];
    const float positive_half_angle_rad =
        (joint.angle_zero_rad + joint.angle_max_rad) * 0.5f;
    const float negative_half_angle_rad =
        (joint.angle_zero_rad + joint.angle_min_rad) * 0.5f;
    const uint16_t positive_half_pwm_us =
        joint.pwm_zero_us + ((joint.pwm_max_us - joint.pwm_zero_us) / 2);
    const uint16_t negative_half_pwm_us =
        joint.pwm_zero_us - ((joint.pwm_zero_us - joint.pwm_min_us) / 2);

    printPwmInterpolationTest(
        "PWM interp min",
        0,
        joint.angle_min_rad,
        joint.pwm_min_us,
        KinematicsStatus::Ok);
    printPwmInterpolationTest(
        "PWM interp zero",
        0,
        joint.angle_zero_rad,
        joint.pwm_zero_us,
        KinematicsStatus::Ok);
    printPwmInterpolationTest(
        "PWM interp max",
        0,
        joint.angle_max_rad,
        joint.pwm_max_us,
        KinematicsStatus::Ok);
    printPwmInterpolationTest(
        "PWM interp +half",
        0,
        positive_half_angle_rad,
        positive_half_pwm_us,
        KinematicsStatus::Ok);
    printPwmInterpolationTest(
        "PWM interp -half",
        0,
        negative_half_angle_rad,
        negative_half_pwm_us,
        KinematicsStatus::Ok);
    printPwmInterpolationTest(
        "PWM interp above max",
        0,
        joint.angle_max_rad + 0.01f,
        0,
        KinematicsStatus::JointLimitViolation);
}

void printPwmInterpolationTest(
    const char *label,
    uint8_t joint_index,
    float angle_rad,
    uint16_t expected_pwm_us,
    KinematicsStatus expected_status) {
    const KinematicsResult<uint16_t> result =
        jointAngleToPwmUs(joint_index, angle_rad);
    const bool passed =
        result.status == expected_status &&
        (result.status != KinematicsStatus::Ok || result.value == expected_pwm_us);

    Serial.print(label);
    Serial.print(" status=");
    Serial.print(static_cast<int>(result.status));
    Serial.print(" pwm_us=");
    Serial.print(result.value);
    Serial.print(" expected_status=");
    Serial.print(static_cast<int>(expected_status));
    Serial.print(" expected_pwm_us=");
    Serial.print(expected_pwm_us);
    Serial.print(" result=");
    Serial.println(passed ? "PASS" : "FAIL");
}
