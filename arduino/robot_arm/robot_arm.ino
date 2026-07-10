/**
 * Fixed pick-and-place demo sequence.
 *
 * setup() moves the robot to HOME/ZERO, opens the gripper, executes the
 * TARGETS[] sequence with the open-loop Jacobian IK trajectory generator, then
 * returns to HOME through the same controlled motion path. loop() stays idle.
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

struct JacobianIkResult {
    KinematicsStatus status;
    uint16_t iterations;
    bool missed_deadline;
    JacobianIkStepResult last_step;
    ArmJointPwmUs last_pwm;
};

const ArmTarget TARGET1 = {
    {0.140f, 0.360f, 0.0f},
    0.900f,
    0.0f,
    GripperCommand::Open,
};

const ArmTarget TARGET2 = {
    {0.140f, 0.360f, -0.95f},
    0.900f,
    -0.7f,
    GripperCommand::Open,
};

const ArmTarget TARGET3 = {
    {0.160f, 0.350f, -0.95f},
    0.900f,
    -0.7f,
    GripperCommand::Open,
};

const ArmTarget TARGET4 = {
    {0.160f, 0.350f, -0.95f},
    0.900f,
    -0.7f,
    GripperCommand::Closed,
};

const ArmTarget TARGET5 = {
    {0.140f, 0.360f, -0.95f},
    0.900f,
    -0.7f,
    GripperCommand::Closed,
};

const ArmTarget TARGET6 = {
    {0.150f, 0.360f, 0.0f},
    0.900f,
    0.0f,
    GripperCommand::Closed,
};

const ArmTarget TARGET7 = {
    {0.140f, 0.360f, 0.95f},
    0.900f,
    0.7f,
    GripperCommand::Closed,
};

const ArmTarget TARGET8 = {
    {0.160f, 0.350f, 0.95f},
    0.900f,
    0.7f,
    GripperCommand::Closed,
};

const ArmTarget TARGET9 = {
    {0.160f, 0.350f, 0.95f},
    0.900f,
    0.7f,
    GripperCommand::Open,
};

const ArmTarget TARGET10 = {
    {0.140f, 0.360f, 0.95f},
    0.900f,
    0.7f,
    GripperCommand::Open,
};

const ArmTarget TARGET11 = {
    {0.0f, 0.489f, 0.0f},
    0.0f,
    0.0f,
    GripperCommand::Open,
};

const ArmTarget TARGETS[] = {
    TARGET1,
    TARGET2,
    TARGET3,
    TARGET4,
    TARGET5,
    TARGET6,
    TARGET7,
    TARGET8,
    TARGET9,
    TARGET10,
    TARGET11,
};

constexpr unsigned long STEP_DELAY_MS = 500;

Pca9685ServoDriver servo_driver;
ArmJointAngles commanded_angles;

bool moveToArmTarget(const char *name, const ArmTarget &target, const JacobianIkConfig &config);
JacobianIkResult moveToTargetJacobianIk(
    ArmJointAngles &commanded_angles,
    const JacobianIkTarget &target,
    const JacobianIkConfig &config);
JacobianIkTarget toJacobianIkTarget(const ArmTarget &target);
void sendZeroCommand();
bool sendKinematicJointCommand(const ArmJointAngles &angles, ArmJointPwmUs &pwm);
void sendGripperCommand(GripperCommand command);
void waitUntilNextPeriod(unsigned long period_start_ms, unsigned long period_ms, bool &missed_deadline);
bool moveExplicitJointTargets(
    ArmJointAngles &commanded_angles,
    const JacobianIkTarget &target,
    const JacobianIkConfig &config,
    JacobianIkResult &result);
float limitedAngleStep(float error_rad, float velocity_limit_rad_s, float dt_s);
float normalizedTaskError(const JacobianIkStepResult &step, const JacobianIkConfig &config);
void printTargetDiagnostics(
    const ArmTarget &target,
    const JacobianIkResult &result,
    const ArmJointAngles &angles);
void printArmJointAngles(const ArmJointAngles &angles);
uint16_t gripperCommandToPwmUs(GripperCommand command);

void setup() {
    Serial.begin(115200);

    if (!servo_driver.begin()) {
        Serial.println(F("PCA9685 init failed"));
        return;
    }

    delay(100);
    commanded_angles = robotHomeJointAngles();
    const JacobianIkConfig ik_config = defaultJacobianIkConfig();

    sendZeroCommand();
    sendGripperCommand(GripperCommand::Open);
    Serial.println(F("ZERO command sent"));
    delay(STEP_DELAY_MS);

    for (uint8_t i = 0; i < sizeof(TARGETS) / sizeof(TARGETS[0]); ++i) {
        if (!moveToArmTarget("TARGET", TARGETS[i], ik_config)) {
            Serial.println(F("Target command failed; sequence stopped"));
            return;
        }
        delay(STEP_DELAY_MS);
    }

    Serial.println(F("Target sequence complete"));
}

void loop() {
}

bool moveToArmTarget(const char *name, const ArmTarget &target, const JacobianIkConfig &config) {
    Serial.print(F("Moving to "));
    Serial.println(name);

    const JacobianIkResult result = moveToTargetJacobianIk(
        commanded_angles,
        toJacobianIkTarget(target),
        config);
    printTargetDiagnostics(target, result, commanded_angles);
    if (result.status != KinematicsStatus::Ok) {
        return false;
    }

    sendGripperCommand(target.gripper);
    Serial.print(name);
    Serial.println(F(" motion complete"));
    return true;
}

JacobianIkResult moveToTargetJacobianIk(
    ArmJointAngles &commanded_angles,
    const JacobianIkTarget &target,
    const JacobianIkConfig &config) {
    JacobianIkResult result = {
        KinematicsStatus::OutOfReach,
        0,
        false,
        {},
        {0, 0, 0, 0, 0},
    };

    const unsigned long period_ms = max(1UL, static_cast<unsigned long>(config.dt_s * 1000.0f + 0.5f));
    float best_error = 3.402823466e+38f;
    uint8_t no_progress_count = 0;

    if (!moveExplicitJointTargets(commanded_angles, target, config, result)) {
        return result;
    }

    for (uint16_t iteration = result.iterations; iteration < config.max_iterations; ++iteration) {
        const unsigned long period_start_ms = millis();
        const JacobianIkStepResult step = computeJacobianIkStep(
            commanded_angles,
            target,
            config);

        result.iterations = iteration + 1;
        result.last_step = step;

        if (step.status != KinematicsStatus::Ok) {
            result.status = step.status;
            return result;
        }

        if (step.converged) {
            result.status = KinematicsStatus::Ok;
            return result;
        }

        ArmJointPwmUs pwm = {0, 0, 0, 0, 0};
        if (!sendKinematicJointCommand(step.next_angles, pwm)) {
            result.status = KinematicsStatus::JointLimitViolation;
            return result;
        }

        commanded_angles = step.next_angles;
        result.last_pwm = pwm;

        const float error = normalizedTaskError(step, config);
        if (best_error - error > config.min_error_progress) {
            best_error = error;
            no_progress_count = 0;
        } else if (++no_progress_count >= config.no_progress_iteration_limit) {
            result.status = KinematicsStatus::NoProgress;
            waitUntilNextPeriod(period_start_ms, period_ms, result.missed_deadline);
            return result;
        }

        waitUntilNextPeriod(period_start_ms, period_ms, result.missed_deadline);
    }

    result.status = KinematicsStatus::OutOfReach;
    return result;
}

JacobianIkTarget toJacobianIkTarget(const ArmTarget &target) {
    return {
        target.tcp_pose,
        target.j3_rad,
        target.j4_rad,
    };
}

void sendZeroCommand() {
    Serial.println(F("Sending ZERO command"));

    for (uint8_t i = 0; i < ROBOT_TOTAL_JOINT_COUNT; ++i) {
        const JointCalibration &joint = JOINT_CALIBRATIONS[i];
        servo_driver.setServoUs(joint.channel, joint.pwm_zero_us);
    }
    commanded_angles = robotHomeJointAngles();
}

bool sendKinematicJointCommand(const ArmJointAngles &angles, ArmJointPwmUs &pwm) {
    const KinematicsResult<ArmJointPwmUs> pwm_result = jointAnglesToPwmUs(angles, JOINT_CALIBRATIONS);
    if (pwm_result.status != KinematicsStatus::Ok) {
        Serial.print(F("Target PWM failed status="));
        Serial.println(kinematicsStatusName(pwm_result.status));
        return false;
    }

    pwm = pwm_result.value;
    const uint16_t pwm_values[ROBOT_KINEMATIC_JOINT_COUNT] = {
        pwm.j0_us,
        pwm.j1_us,
        pwm.j2_us,
        pwm.j3_us,
        pwm.j4_us,
    };

    for (uint8_t i = 0; i < ROBOT_KINEMATIC_JOINT_COUNT; ++i) {
        servo_driver.setServoUs(
            JOINT_CALIBRATIONS[i].channel,
            pwm_values[i]);
    }

    return true;
}

void sendGripperCommand(GripperCommand command) {
    const JointCalibration &gripper = JOINT_CALIBRATIONS[ROBOT_GRIPPER_JOINT_INDEX];
    servo_driver.setServoUs(gripper.channel, gripperCommandToPwmUs(command));
}

void waitUntilNextPeriod(unsigned long period_start_ms, unsigned long period_ms, bool &missed_deadline) {
    const unsigned long elapsed_ms = millis() - period_start_ms;
    if (elapsed_ms < period_ms) {
        delay(period_ms - elapsed_ms);
    } else {
        missed_deadline = true;
    }
}

bool moveExplicitJointTargets(
    ArmJointAngles &commanded_angles,
    const JacobianIkTarget &target,
    const JacobianIkConfig &config,
    JacobianIkResult &result) {
    const unsigned long period_ms = max(1UL, static_cast<unsigned long>(config.dt_s * 1000.0f + 0.5f));

    while (result.iterations < config.max_iterations) {
        const float yaw_error_rad = target.tcp_pose.yaw_rad - commanded_angles.j0_rad;
        const float j3_error_rad = target.j3_rad - commanded_angles.j3_rad;
        const float j4_error_rad = target.j4_rad - commanded_angles.j4_rad;

        if (fabs(yaw_error_rad) <= config.angle_tolerance_rad &&
            fabs(j3_error_rad) <= config.angle_tolerance_rad &&
            fabs(j4_error_rad) <= config.angle_tolerance_rad) {
            return true;
        }

        const unsigned long period_start_ms = millis();
        ArmJointAngles next_angles = commanded_angles;
        next_angles.j0_rad += limitedAngleStep(
            yaw_error_rad,
            config.joint_velocity_limit_rad_s[0],
            config.dt_s);
        next_angles.j3_rad += limitedAngleStep(
            j3_error_rad,
            config.joint_velocity_limit_rad_s[3],
            config.dt_s);
        next_angles.j4_rad += limitedAngleStep(
            j4_error_rad,
            config.joint_velocity_limit_rad_s[4],
            config.dt_s);

        ArmJointPwmUs pwm = {0, 0, 0, 0, 0};
        if (!sendKinematicJointCommand(next_angles, pwm)) {
            result.status = KinematicsStatus::JointLimitViolation;
            return false;
        }

        commanded_angles = next_angles;
        result.last_pwm = pwm;
        result.iterations++;
        waitUntilNextPeriod(period_start_ms, period_ms, result.missed_deadline);
    }

    result.status = KinematicsStatus::OutOfReach;
    return false;
}

float limitedAngleStep(float error_rad, float velocity_limit_rad_s, float dt_s) {
    const float max_step_rad = velocity_limit_rad_s * dt_s;
    if (error_rad > max_step_rad) {
        return max_step_rad;
    }
    if (error_rad < -max_step_rad) {
        return -max_step_rad;
    }
    return error_rad;
}

float normalizedTaskError(const JacobianIkStepResult &step, const JacobianIkConfig &config) {
    float result = fabs(step.reach_error_m) / config.position_tolerance_m;
    const float z_error = fabs(step.z_error_m) / config.position_tolerance_m;
    if (z_error > result) {
        result = z_error;
    }
    const float yaw_error = fabs(step.yaw_error_rad) / config.angle_tolerance_rad;
    if (yaw_error > result) {
        result = yaw_error;
    }
    const float j3_error = fabs(step.j3_error_rad) / config.angle_tolerance_rad;
    if (j3_error > result) {
        result = j3_error;
    }
    const float j4_error = fabs(step.j4_error_rad) / config.angle_tolerance_rad;
    if (j4_error > result) {
        result = j4_error;
    }
    return result;
}

void printTargetDiagnostics(
    const ArmTarget &target,
    const JacobianIkResult &result,
    const ArmJointAngles &angles) {
    Serial.print(F("Target TCP reach_x_m="));
    Serial.print(target.tcp_pose.reach_x_m, 6);
    Serial.print(F(" reach_z_m="));
    Serial.print(target.tcp_pose.reach_z_m, 6);
    Serial.print(F(" yaw_rad="));
    Serial.print(target.tcp_pose.yaw_rad, 6);
    Serial.print(F(" requested_j3_rad="));
    Serial.print(target.j3_rad, 6);
    Serial.print(F(" requested_j4_rad="));
    Serial.println(target.j4_rad, 6);

    Serial.print(F("Jacobian IK status="));
    Serial.print(kinematicsStatusName(result.status));
    Serial.print(F(" iterations="));
    Serial.print(result.iterations);
    Serial.print(F(" missed_deadline="));
    Serial.print(result.missed_deadline ? F("yes") : F("no"));
    Serial.print(F(" alpha="));
    Serial.println(result.last_step.alpha, 6);

    Serial.print(F("Final errors reach_m="));
    Serial.print(result.last_step.reach_error_m, 6);
    Serial.print(F(" z_m="));
    Serial.print(result.last_step.z_error_m, 6);
    Serial.print(F(" yaw_rad="));
    Serial.print(result.last_step.yaw_error_rad, 6);
    Serial.print(F(" j3_rad="));
    Serial.print(result.last_step.j3_error_rad, 6);
    Serial.print(F(" j4_rad="));
    Serial.println(result.last_step.j4_error_rad, 6);

    printArmJointAngles(angles);

    const KinematicsResult<TcpPose> fk_result = forwardKinematics(angles);
    if (fk_result.status == KinematicsStatus::Ok) {
        Serial.print(F("Model FK reach_x_m="));
        Serial.print(fk_result.value.reach_x_m, 6);
        Serial.print(F(" reach_z_m="));
        Serial.print(fk_result.value.reach_z_m, 6);
        Serial.print(F(" yaw_rad="));
        Serial.println(fk_result.value.yaw_rad, 6);
    } else {
        Serial.print(F("Model FK failed status="));
        Serial.println(kinematicsStatusName(fk_result.status));
    }

    Serial.print(F("Last commanded PWM j0_us="));
    Serial.print(result.last_pwm.j0_us);
    Serial.print(F(" j1_us="));
    Serial.print(result.last_pwm.j1_us);
    Serial.print(F(" j2_us="));
    Serial.print(result.last_pwm.j2_us);
    Serial.print(F(" j3_us="));
    Serial.print(result.last_pwm.j3_us);
    Serial.print(F(" j4_us="));
    Serial.println(result.last_pwm.j4_us);
}

void printArmJointAngles(const ArmJointAngles &angles) {
    Serial.print(F("Commanded angles j0_rad="));
    Serial.print(angles.j0_rad, 6);
    Serial.print(F(" j1_rad="));
    Serial.print(angles.j1_rad, 6);
    Serial.print(F(" j2_rad="));
    Serial.print(angles.j2_rad, 6);
    Serial.print(F(" j3_rad="));
    Serial.print(angles.j3_rad, 6);
    Serial.print(F(" j4_rad="));
    Serial.println(angles.j4_rad, 6);
}

uint16_t gripperCommandToPwmUs(GripperCommand command) {
    const JointCalibration &gripper = JOINT_CALIBRATIONS[ROBOT_GRIPPER_JOINT_INDEX];
    return command == GripperCommand::Open ? gripper.pwm_min_us : gripper.pwm_max_us;
}
