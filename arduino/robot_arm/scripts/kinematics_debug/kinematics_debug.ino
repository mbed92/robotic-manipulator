/**
 * Kinematics debug sketch for the robot arm.
 * This program validates FK, angle-to-PWM conversion, and the current IK stub.
 */

#include <math.h>

#include "kinematics.h"
#include "robot_calibration.h"

void printStatus(const char *label, KinematicsStatus status);
void printPose(const TcpPose &pose);
void printPwm(const ArmJointPwmUs &pwm_us);
void printFkZeroTest(const TcpPose &pose);
void runPwmInterpolationTests();
void printPwmInterpolationTest(
    const char *label,
    uint8_t joint_index,
    float angle_rad,
    uint16_t expected_pwm_us,
    KinematicsStatus expected_status);

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

    const ArmJointAngles yaw_angles = {0.25f, 0.0f, 0.0f, 0.0f, 0.50f};
    const KinematicsResult<TcpPose> fk_yaw = forwardKinematics(yaw_angles);
    printStatus("FK yaw", fk_yaw.status);
    if (fk_yaw.status == KinematicsStatus::Ok) {
        printPose(fk_yaw.value);
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
        inverseKinematicsPositionYaw(fk_zero.value);
    printStatus("IK position yaw", ik_result.status);
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
    Serial.print(" yaw_rad=");
    Serial.println(pose.yaw_rad, 6);
}

void printPwm(const ArmJointPwmUs &pwm_us) {
    Serial.print("pwm j1=");
    Serial.print(pwm_us.j1_us);
    Serial.print(" j2=");
    Serial.print(pwm_us.j2_us);
    Serial.print(" j3=");
    Serial.print(pwm_us.j3_us);
    Serial.print(" j4=");
    Serial.print(pwm_us.j4_us);
    Serial.print(" j5=");
    Serial.println(pwm_us.j5_us);
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
        fabs(pose.yaw_rad) <= tolerance_m;

    Serial.print("FK zero expected x_m=0.000000 y_m=0.000000 z_m=");
    Serial.print(expected_z_m, 6);
    Serial.print(" yaw_rad=0.000000 result=");
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
