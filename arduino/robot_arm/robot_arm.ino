/**
 * Simple single Cartesian target command.
 *
 * setup() computes one IK solution for TARGET. loop() sends the resulting
 * servo commands once and then stays idle. IK solves J0-J3 from the target
 * TCP position and yaw. J4 and J5 are explicit user commands.
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
    float j4_rad;
    GripperCommand gripper;
};

const ArmTarget TARGET = {
    {0.05f, 0.45f, 0.7f},
    -1.0f,
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
    // First compute the IK solution for the target TCP position and yaw.
    const KinematicsResult<ArmJointAngles> ik_result = inverseKinematicsPositionYaw(target.tcp_pose);
    if (ik_result.status != KinematicsStatus::Ok) {
        Serial.print("Target IK failed status=");
        Serial.println(static_cast<int>(ik_result.status));
        return false;
    }

    // Incorporate the target J4 and gripper commands into the final servo command.
    ArmJointAngles commanded_angles = ik_result.value;
    commanded_angles.j4_rad = target.j4_rad;

    // Then convert the full joint angle command to PWM values for the servo driver.
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

uint16_t gripperCommandToPwmUs(GripperCommand command) {
    const JointCalibration &gripper = JOINT_CALIBRATIONS[ROBOT_GRIPPER_JOINT_INDEX];
    return command == GripperCommand::Open ? gripper.pwm_min_us : gripper.pwm_max_us;
}
