/**
 * Simple single Cartesian target command.
 *
 * setup() computes one IK solution for TARGET. loop() sends the resulting
 * servo commands once and then stays idle. IK solves J0-J3 from the target
 * TCP position, yaw, and explicit J3 pitch. J4 and J5 are explicit user commands.
 */

#include <Wire.h>
#include <math.h>

#include "kinematics.h"
#include "pca9685_servo_driver.h"
#include "robot_calibration.h"

enum class GripperCommand {
    Closed,
    Open,
};

struct ArmTarget {
    TcpPose tcp_pose;
    float j3_rad;
    float j4_rad;
    GripperCommand gripper;
};

const ArmTarget TARGET = {
    {0.2f, 0.4f, 0.0f},
    0.0f,
    0.0f,
    GripperCommand::Closed,
};

Pca9685ServoDriver servo_driver;
ArmJointAngles target_angles;
ArmJointPwmUs target_pwm;
uint16_t target_gripper_pwm;
bool target_ready = false;
bool target_sent = false;

bool computeTargetCommand(const ArmTarget &target);
void executeTargetCommand();
void printTargetDiagnostics(const ArmTarget &target, const ArmJointAngles &angles, const ArmJointPwmUs &pwm);
void printArmJointAngles(const ArmJointAngles &angles);
void printJointPwmFailure(uint8_t joint_index, float angle_rad, KinematicsStatus status);
uint16_t gripperCommandToPwmUs(GripperCommand command);

void setup() {
    Serial.begin(115200);

    if (!servo_driver.begin()) {
        Serial.println("PCA9685 init failed");
        return;
    }

    delay(100);

    target_ready = computeTargetCommand(TARGET);
    if (!target_ready) {
        Serial.println("Target command computation failed");
        return;
    }

    Serial.println("Target command computation OK");
}

void loop() {
    if (!target_ready || target_sent) {
        return;
    }

    executeTargetCommand();
    target_sent = true;
    Serial.println("Target command sent");
}

bool computeTargetCommand(const ArmTarget &target) {
    // First compute the IK solution for the target TCP position, yaw, and J3 pitch.
    const KinematicsResult<ArmJointAngles> ik_result = inverseKinematicsPositionYawPitch(target.tcp_pose, target.j3_rad);
    if (ik_result.status != KinematicsStatus::Ok) {
        Serial.print("Target IK failed status=");
        Serial.println(static_cast<int>(ik_result.status));
        return false;
    }

    // Incorporate the target J4 and gripper commands into the final servo command.
    ArmJointAngles commanded_angles = ik_result.value;
    commanded_angles.j4_rad = target.j4_rad;
    printArmJointAngles(commanded_angles);

    // Then convert the full joint angle command to PWM values for the servo driver.
    const float angle_values[ROBOT_KINEMATIC_JOINT_COUNT] = {
        commanded_angles.j0_rad,
        commanded_angles.j1_rad,
        commanded_angles.j2_rad,
        commanded_angles.j3_rad,
        commanded_angles.j4_rad,
    };
    for (uint8_t i = 0; i < ROBOT_KINEMATIC_JOINT_COUNT; ++i) {
        const KinematicsResult<uint16_t> joint_pwm_result = jointAngleToPwmUs(i, angle_values[i], JOINT_CALIBRATIONS);
        if (joint_pwm_result.status != KinematicsStatus::Ok) {
            printJointPwmFailure(i, angle_values[i], joint_pwm_result.status);
        }
    }

    const KinematicsResult<ArmJointPwmUs> pwm_result = jointAnglesToPwmUs(commanded_angles, JOINT_CALIBRATIONS);
    if (pwm_result.status != KinematicsStatus::Ok) {
        Serial.print("Target PWM failed status=");
        Serial.println(static_cast<int>(pwm_result.status));
        return false;
    }

    // If we reach this point, the target command is valid and can be sent to the servo driver.
    target_angles = commanded_angles;
    target_pwm = pwm_result.value;
    target_gripper_pwm = gripperCommandToPwmUs(target.gripper);
    printTargetDiagnostics(target, target_angles, target_pwm);
    return true;
}

void executeTargetCommand() {
    Serial.println("Executing target command");

    const uint16_t pwm_values[ROBOT_KINEMATIC_JOINT_COUNT] = {
        target_pwm.j0_us,
        target_pwm.j1_us,
        target_pwm.j2_us,
        target_pwm.j3_us,
        target_pwm.j4_us,
    };

    for (uint8_t i = 0; i < ROBOT_KINEMATIC_JOINT_COUNT; ++i) {
        servo_driver.setServoUs(
            JOINT_CALIBRATIONS[i].channel,
            pwm_values[i]);
    }

    const JointCalibration &gripper = JOINT_CALIBRATIONS[ROBOT_GRIPPER_JOINT_INDEX];
    servo_driver.setServoUs(gripper.channel, target_gripper_pwm);
}

void printTargetDiagnostics(const ArmTarget &target, const ArmJointAngles &angles, const ArmJointPwmUs &pwm) {
    Serial.print("Target TCP reach_x_m=");
    Serial.print(target.tcp_pose.reach_x_m, 6);
    Serial.print(" reach_z_m=");
    Serial.print(target.tcp_pose.reach_z_m, 6);
    Serial.print(" yaw_rad=");
    Serial.print(target.tcp_pose.yaw_rad, 6);
    Serial.print(" requested_j3_rad=");
    Serial.print(target.j3_rad, 6);
    Serial.print(" requested_j4_rad=");
    Serial.println(target.j4_rad, 6);

    printArmJointAngles(angles);

    const KinematicsResult<TcpPose> fk_result = forwardKinematics(angles);
    if (fk_result.status == KinematicsStatus::Ok) {
        Serial.print("Model FK reach_x_m=");
        Serial.print(fk_result.value.reach_x_m, 6);
        Serial.print(" reach_z_m=");
        Serial.print(fk_result.value.reach_z_m, 6);
        Serial.print(" yaw_rad=");
        Serial.println(fk_result.value.yaw_rad, 6);
    } else {
        Serial.print("Model FK failed status=");
        Serial.println(static_cast<int>(fk_result.status));
    }

    Serial.print("Commanded PWM j0_us=");
    Serial.print(pwm.j0_us);
    Serial.print(" j1_us=");
    Serial.print(pwm.j1_us);
    Serial.print(" j2_us=");
    Serial.print(pwm.j2_us);
    Serial.print(" j3_us=");
    Serial.print(pwm.j3_us);
    Serial.print(" j4_us=");
    Serial.println(pwm.j4_us);
}

void printArmJointAngles(const ArmJointAngles &angles) {
    Serial.print("Commanded angles j0_rad=");
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

void printJointPwmFailure(uint8_t joint_index, float angle_rad, KinematicsStatus status) {
    Serial.print("Joint PWM failed joint=");
    Serial.print(joint_index);
    Serial.print(" angle_rad=");
    Serial.print(angle_rad, 6);
    Serial.print(" status=");
    Serial.println(static_cast<int>(status));
}

uint16_t gripperCommandToPwmUs(GripperCommand command) {
    const JointCalibration &gripper = JOINT_CALIBRATIONS[ROBOT_GRIPPER_JOINT_INDEX];
    return command == GripperCommand::Open ? gripper.pwm_min_us : gripper.pwm_max_us;
}
