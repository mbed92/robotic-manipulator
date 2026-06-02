#ifndef ROBOT_ARM_ROBOT_CALIBRATION_H
#define ROBOT_ARM_ROBOT_CALIBRATION_H

#include <Arduino.h>

constexpr uint8_t ROBOT_KINEMATIC_JOINT_COUNT = 5;
constexpr uint8_t ROBOT_GRIPPER_JOINT_INDEX = 5;
constexpr uint8_t ROBOT_TOTAL_JOINT_COUNT = 6;
constexpr uint8_t ROBOT_ARM_JOINT_COUNT = ROBOT_TOTAL_JOINT_COUNT;

struct JointCalibration {
    uint8_t channel;
    uint16_t pwm_min_us;
    uint16_t pwm_zero_us;
    uint16_t pwm_max_us;
    float angle_min_rad;
    float angle_zero_rad;
    float angle_max_rad;
};

struct JointOffsets {
    float l1_m;
    float l2_m;
    float l3_m;
    float l4_m;
    float l5_m;
};

constexpr float JOINT_ANGLE_MIN_RAD = -1.0472f;
constexpr float JOINT_ANGLE_ZERO_RAD = 0.0f;
constexpr float JOINT_ANGLE_MAX_RAD = 1.0472f;

constexpr float ROBOT_HOME_J0_RAD = 0.0f;
constexpr float ROBOT_HOME_J1_RAD = 0.0f;
constexpr float ROBOT_HOME_J2_RAD = 0.0f;
constexpr float ROBOT_HOME_J3_RAD = 0.0f;
constexpr float ROBOT_HOME_J4_RAD = 0.0f;

// 2400 -> 60 deg, 1500 -> 0 deg, 600 -> -60 deg
// (2400 - 1500) / 60 deg = 15 pwm_us per degree = 15 * 180 / pi = 859.4366 pwm_us per radian
const JointCalibration JOINT_CALIBRATIONS[ROBOT_TOTAL_JOINT_COUNT] = {
    {0, 600, 1500, 2400, JOINT_ANGLE_MIN_RAD, JOINT_ANGLE_ZERO_RAD, JOINT_ANGLE_MAX_RAD},
    {1, 600, 1500, 2400, JOINT_ANGLE_MIN_RAD, JOINT_ANGLE_ZERO_RAD, JOINT_ANGLE_MAX_RAD},
    {2, 600, 1400, 2400, JOINT_ANGLE_MIN_RAD, JOINT_ANGLE_ZERO_RAD, JOINT_ANGLE_MAX_RAD},
    {3, 2400, 1500, 600, JOINT_ANGLE_MIN_RAD, JOINT_ANGLE_ZERO_RAD, JOINT_ANGLE_MAX_RAD},
    {4, 600, 1500, 2400, JOINT_ANGLE_MIN_RAD, JOINT_ANGLE_ZERO_RAD, JOINT_ANGLE_MAX_RAD},
    {5, 600, 1500, 2400, JOINT_ANGLE_MIN_RAD, JOINT_ANGLE_ZERO_RAD, JOINT_ANGLE_MAX_RAD},
};

// Joint indexing follows the physical robot: J0 base, J1-J3 planar arm,
// J4 local TCP rotation, J5 gripper.
// L1 is the vertical distance from J0/base to J1/shoulder. J0 is at 0,0,0.
// L2-L3-L4 are the planar arm links. L5 is the final flange-to-TCP offset length.
// Units: meters.
const JointOffsets JOINT_OFFSETS = {
    0.0625f,
    0.1035f,
    0.1480f,
    0.0700f,
    0.1050f,
};

#endif
