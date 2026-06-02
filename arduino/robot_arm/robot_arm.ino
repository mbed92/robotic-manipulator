/**
 * Simple front-facing Cartesian line trajectory demo.
 *
 * setup() computes Cartesian trajectory segments between two TCP positions.
 * loop() executes the precomputed waypoints one by one with a fixed delay.
 * The path stays in the front plane (Y=0). IK solves J0-J3 from the target
 * TCP pose and local wrist pitch. J4 and J5 are explicit user commands.
 */

#include <Wire.h>
#include <math.h>

#include "kinematics.h"
#include "pca9685_servo_driver.h"
#include "robot_calibration.h"

constexpr uint8_t TRAJECTORY_SEGMENT_COUNT = 5;
constexpr uint8_t TRAJECTORY_WAYPOINT_COUNT = TRAJECTORY_SEGMENT_COUNT + 1;
constexpr uint16_t TRAJECTORY_STEP_DELAY_MS = 2000;

enum class GripperCommand {
    Closed,
    Open,
};

struct ArmTarget {
    TcpPose tcp_pose;
    float local_wrist_pitch_rad;
    float j4_rad;
    GripperCommand gripper;
};

const ArmTarget START_TARGET = {
    {{0.08f, 0.0f, 0.4425f}, 0.0f},
    0.0f,
    1.57f,
    GripperCommand::Open,
};

const ArmTarget END_TARGET = {
    {{0.08f, 0.0f, 0.4025f}, 0.0f},
    0.0f,
    0.0f,
    GripperCommand::Closed,
};

Pca9685ServoDriver servo_driver;
ArmTarget trajectory_targets[TRAJECTORY_WAYPOINT_COUNT];
ArmJointAngles trajectory_angles[TRAJECTORY_WAYPOINT_COUNT];
ArmJointPwmUs trajectory_pwm[TRAJECTORY_WAYPOINT_COUNT];
uint16_t trajectory_gripper_pwm[TRAJECTORY_WAYPOINT_COUNT];
uint8_t next_waypoint_index = 0;
bool trajectory_ready = false;
bool trajectory_finished = false;

ArmTarget interpolateTarget(const ArmTarget &start_target, const ArmTarget &end_target, float alpha);
bool computeTrajectory();
bool computeWaypoint(uint8_t waypoint_index, const ArmTarget &target, ArmJointAngles &seed_angles);
void executeWaypoint(uint8_t waypoint_index);
uint16_t gripperCommandToPwmUs(GripperCommand command);

void setup() {
    Serial.begin(115200);

    if (!servo_driver.begin()) {
        Serial.println("PCA9685 init failed");
        return;
    }

    delay(100);

    trajectory_ready = computeTrajectory();
    if (!trajectory_ready) {
        Serial.println("Trajectory precompute failed");
        return;
    }

    Serial.println("Trajectory precompute OK");
}

void loop() {
    if (!trajectory_ready || trajectory_finished) {
        return;
    }

    executeWaypoint(next_waypoint_index);
    ++next_waypoint_index;

    if (next_waypoint_index >= TRAJECTORY_WAYPOINT_COUNT) {
        trajectory_finished = true;
        Serial.println("Cartesian trajectory finished");
        return;
    }

    delay(TRAJECTORY_STEP_DELAY_MS);
}

bool computeTrajectory() {
    KinematicsResult<ArmJointAngles> start_ik =
        inverseKinematicsPositionYawTcpOffsetPitch(
            START_TARGET.tcp_pose,
            START_TARGET.local_wrist_pitch_rad);
    if (start_ik.status != KinematicsStatus::Ok) {
        Serial.print("Start target IK failed status=");
        Serial.println(static_cast<int>(start_ik.status));
        return false;
    }

    // Seed the first waypoint IK with the start pose solution, then seed each
    // subsequent waypoint with the previous one to improve the chances of a
    // valid solution and a smooth trajectory.
    ArmJointAngles seed_angles = start_ik.value;
    for (uint8_t i = 0; i < TRAJECTORY_WAYPOINT_COUNT; ++i) {
        const float alpha = static_cast<float>(i) / static_cast<float>(TRAJECTORY_SEGMENT_COUNT);
        const ArmTarget target = interpolateTarget(START_TARGET, END_TARGET, alpha);

        if (!computeWaypoint(i, target, seed_angles)) {
            Serial.print("Waypoint computation failed index=");
            Serial.println(i);
            return false;
        }
    }

    return true;
}

bool computeWaypoint(uint8_t waypoint_index, const ArmTarget &target, ArmJointAngles &seed_angles) {
    const KinematicsResult<ArmJointAngles> ik_result =
        inverseKinematicsPositionYawTcpOffsetPitchSeeded(
            target.tcp_pose,
            target.local_wrist_pitch_rad,
            seed_angles);
    if (ik_result.status != KinematicsStatus::Ok) {
        return false;
    }

    ArmJointAngles commanded_angles = ik_result.value;
    commanded_angles.j4_rad = target.j4_rad;

    const KinematicsResult<ArmJointPwmUs> pwm_result = jointAnglesToPwmUs(commanded_angles, JOINT_CALIBRATIONS);
    if (pwm_result.status != KinematicsStatus::Ok) {
        return false;
    }

    trajectory_targets[waypoint_index] = target;
    trajectory_angles[waypoint_index] = commanded_angles;
    trajectory_pwm[waypoint_index] = pwm_result.value;
    trajectory_gripper_pwm[waypoint_index] = gripperCommandToPwmUs(target.gripper);
    seed_angles = commanded_angles;
    return true;
}

void executeWaypoint(uint8_t waypoint_index) {
    Serial.print("Executing waypoint ");
    Serial.print(waypoint_index);
    Serial.print("/");
    Serial.println(TRAJECTORY_SEGMENT_COUNT);

    const uint16_t pwm_values[ROBOT_KINEMATIC_JOINT_COUNT] = {
        trajectory_pwm[waypoint_index].j0_us,
        trajectory_pwm[waypoint_index].j1_us,
        trajectory_pwm[waypoint_index].j2_us,
        trajectory_pwm[waypoint_index].j3_us,
        trajectory_pwm[waypoint_index].j4_us,
    };

    for (uint8_t i = 0; i < ROBOT_KINEMATIC_JOINT_COUNT; ++i) {
        servo_driver.setServoUs(
            JOINT_CALIBRATIONS[i].channel,
            pwm_values[i]);
    }

    const JointCalibration &gripper = JOINT_CALIBRATIONS[ROBOT_GRIPPER_JOINT_INDEX];
    servo_driver.setServoUs(gripper.channel, trajectory_gripper_pwm[waypoint_index]);
}

ArmTarget interpolateTarget(const ArmTarget &start_target, const ArmTarget &end_target, float alpha) {
    const Vector3 position = {
        start_target.tcp_pose.position.x_m +
            alpha * (end_target.tcp_pose.position.x_m - start_target.tcp_pose.position.x_m),
        start_target.tcp_pose.position.y_m +
            alpha * (end_target.tcp_pose.position.y_m - start_target.tcp_pose.position.y_m),
        start_target.tcp_pose.position.z_m +
            alpha * (end_target.tcp_pose.position.z_m - start_target.tcp_pose.position.z_m),
    };

    const float target_r_m = sqrt(position.x_m * position.x_m + position.y_m * position.y_m);
    const float target_yaw_rad = target_r_m <= 0.000001f ? 0.0f : atan2(position.y_m, position.x_m);
    return {
        {position, target_yaw_rad},
        start_target.local_wrist_pitch_rad +
            alpha * (end_target.local_wrist_pitch_rad - start_target.local_wrist_pitch_rad),
        start_target.j4_rad + alpha * (end_target.j4_rad - start_target.j4_rad),
        alpha < 1.0f ? start_target.gripper : end_target.gripper,
    };
}

uint16_t gripperCommandToPwmUs(GripperCommand command) {
    const JointCalibration &gripper = JOINT_CALIBRATIONS[ROBOT_GRIPPER_JOINT_INDEX];
    return command == GripperCommand::Open ? gripper.pwm_max_us : gripper.pwm_min_us;
}
