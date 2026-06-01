/**
 * Simple Cartesian trajectory demo.
 *
 * setup() computes 10 Cartesian trajectory segments between two TCP poses.
 * loop() executes the precomputed waypoints one by one with a 1000 ms delay.
 * J6 is the gripper and is left at its configured zero PWM.
 */

#include <Wire.h>
#include <math.h>

#include "kinematics.h"
#include "pca9685_servo_driver.h"
#include "robot_calibration.h"

constexpr uint8_t TRAJECTORY_SEGMENT_COUNT = 5;
constexpr uint8_t TRAJECTORY_WAYPOINT_COUNT = TRAJECTORY_SEGMENT_COUNT + 1;
constexpr uint16_t TRAJECTORY_STEP_DELAY_MS = 2000;

const TcpPose START_POSE = {
    {0.15f, 0.000f, 0.355f},
    0.0f,
};

const TcpPose END_POSE = {
    {0.15f, 0.000f, 0.155f},
    1.57f,
};

Pca9685ServoDriver servo_driver;
TcpPose trajectory_poses[TRAJECTORY_WAYPOINT_COUNT];
ArmJointAngles trajectory_angles[TRAJECTORY_WAYPOINT_COUNT];
ArmJointPwmUs trajectory_pwm[TRAJECTORY_WAYPOINT_COUNT];
uint8_t next_waypoint_index = 0;
bool trajectory_ready = false;
bool trajectory_finished = false;

TcpPose interpolatePose(const TcpPose &start_pose, const TcpPose &end_pose, float alpha);
float normalizeDemoAngleRad(float angle_rad);
bool computeTrajectory();
bool computeWaypoint(uint8_t waypoint_index, const TcpPose &pose, ArmJointAngles &seed_angles);
void executeWaypoint(uint8_t waypoint_index);
void setArmPwm(const ArmJointPwmUs &pwm_us);
void setGripperZero();
void printPose(const TcpPose &pose);
void printAngles(const ArmJointAngles &angles);

void setup() {
    Serial.begin(115200);

    if (!servo_driver.begin()) {
        Serial.println("PCA9685 init failed");
        return;
    }

    delay(100);
    setGripperZero();

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
        inverseKinematicsPositionToolRoll(START_POSE);
    if (start_ik.status != KinematicsStatus::Ok) {
        Serial.print("Start pose IK failed status=");
        Serial.println(static_cast<int>(start_ik.status));
        return false;
    }

    ArmJointAngles seed_angles = start_ik.value;

    for (uint8_t i = 0; i < TRAJECTORY_WAYPOINT_COUNT; ++i) {
        const float alpha =
            static_cast<float>(i) / static_cast<float>(TRAJECTORY_SEGMENT_COUNT);
        const TcpPose pose = interpolatePose(START_POSE, END_POSE, alpha);

        if (!computeWaypoint(i, pose, seed_angles)) {
            return false;
        }
    }

    return true;
}

bool computeWaypoint(uint8_t waypoint_index, const TcpPose &pose, ArmJointAngles &seed_angles) {
    const KinematicsResult<ArmJointAngles> ik_result =
        inverseKinematicsPositionToolRollSeeded(pose, seed_angles);
    if (ik_result.status != KinematicsStatus::Ok) {
        Serial.print("Waypoint IK failed index=");
        Serial.print(waypoint_index);
        Serial.print(" status=");
        Serial.println(static_cast<int>(ik_result.status));
        printPose(pose);
        return false;
    }

    const KinematicsResult<ArmJointPwmUs> pwm_result =
        jointAnglesToPwmUs(ik_result.value);
    if (pwm_result.status != KinematicsStatus::Ok) {
        Serial.print("Waypoint PWM conversion failed index=");
        Serial.print(waypoint_index);
        Serial.print(" status=");
        Serial.println(static_cast<int>(pwm_result.status));
        printAngles(ik_result.value);
        return false;
    }

    trajectory_poses[waypoint_index] = pose;
    trajectory_angles[waypoint_index] = ik_result.value;
    trajectory_pwm[waypoint_index] = pwm_result.value;
    seed_angles = ik_result.value;
    return true;
}

void executeWaypoint(uint8_t waypoint_index) {
    Serial.print("Executing waypoint ");
    Serial.print(waypoint_index);
    Serial.print("/");
    Serial.println(TRAJECTORY_SEGMENT_COUNT);

    printPose(trajectory_poses[waypoint_index]);
    printAngles(trajectory_angles[waypoint_index]);
    setArmPwm(trajectory_pwm[waypoint_index]);
}

TcpPose interpolatePose(const TcpPose &start_pose, const TcpPose &end_pose, float alpha) {
    const float tool_roll_delta_rad = normalizeDemoAngleRad(end_pose.tool_roll_rad - start_pose.tool_roll_rad);
    return {
        {
            start_pose.position.x_m +
                alpha * (end_pose.position.x_m - start_pose.position.x_m),
            start_pose.position.y_m +
                alpha * (end_pose.position.y_m - start_pose.position.y_m),
            start_pose.position.z_m +
                alpha * (end_pose.position.z_m - start_pose.position.z_m),
        },
        normalizeDemoAngleRad(start_pose.tool_roll_rad + alpha * tool_roll_delta_rad),
    };
}

float normalizeDemoAngleRad(float angle_rad) {
    while (angle_rad > PI) {
        angle_rad -= 2.0f * PI;
    }
    while (angle_rad < -PI) {
        angle_rad += 2.0f * PI;
    }
    return angle_rad;
}

void setArmPwm(const ArmJointPwmUs &pwm_us) {
    const uint16_t pwm_values[ROBOT_KINEMATIC_JOINT_COUNT] = {
        pwm_us.j1_us,
        pwm_us.j2_us,
        pwm_us.j3_us,
        pwm_us.j4_us,
        pwm_us.j5_us,
    };

    for (uint8_t i = 0; i < ROBOT_KINEMATIC_JOINT_COUNT; ++i) {
        servo_driver.setServoUs(JOINT_CALIBRATIONS[i].channel, pwm_values[i]);
        Serial.print("J");
        Serial.print(i + 1);
        Serial.print(" pwm_us=");
        Serial.println(pwm_values[i]);
    }
}

void setGripperZero() {
    const JointCalibration &gripper = JOINT_CALIBRATIONS[ROBOT_GRIPPER_JOINT_INDEX];
    servo_driver.setServoUs(gripper.channel, gripper.pwm_zero_us);
    Serial.print("J6 gripper pwm_us=");
    Serial.println(gripper.pwm_zero_us);
}

void printPose(const TcpPose &pose) {
    Serial.print("target x_m=");
    Serial.print(pose.position.x_m, 6);
    Serial.print(" y_m=");
    Serial.print(pose.position.y_m, 6);
    Serial.print(" z_m=");
    Serial.print(pose.position.z_m, 6);
    Serial.print(" tool_roll_rad=");
    Serial.println(pose.tool_roll_rad, 6);
}

void printAngles(const ArmJointAngles &angles) {
    Serial.print("angles j1=");
    Serial.print(angles.j1_rad, 6);
    Serial.print(" j2=");
    Serial.print(angles.j2_rad, 6);
    Serial.print(" j3=");
    Serial.print(angles.j3_rad, 6);
    Serial.print(" j4=");
    Serial.print(angles.j4_rad, 6);
    Serial.print(" j5=");
    Serial.println(angles.j5_rad, 6);
}
