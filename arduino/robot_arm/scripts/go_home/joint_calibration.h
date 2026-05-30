#ifndef ROBOT_ARM_JOINT_CALIBRATION_H
#define ROBOT_ARM_JOINT_CALIBRATION_H

#include <Arduino.h>

constexpr uint8_t ROBOT_ARM_JOINT_COUNT = 6;

struct JointCalibration
{
    uint8_t channel;
    uint16_t pwm_min_us;
    uint16_t pwm_home_us;
    uint16_t pwm_max_us;
    float angle_min_rad;
    float angle_home_rad;
    float angle_max_rad;
};

constexpr float JOINT_ANGLE_MIN_RAD = -1.04719755120f;
constexpr float JOINT_ANGLE_HOME_RAD = 0.0f;
constexpr float JOINT_ANGLE_MAX_RAD = 1.04719755120f;

const JointCalibration JOINT_CALIBRATIONS[ROBOT_ARM_JOINT_COUNT] = {
    {0, 600, 1700, 2400, JOINT_ANGLE_MIN_RAD, JOINT_ANGLE_HOME_RAD, JOINT_ANGLE_MAX_RAD},
    {1, 600, 2200, 2400, JOINT_ANGLE_MIN_RAD, JOINT_ANGLE_HOME_RAD, JOINT_ANGLE_MAX_RAD},
    {2, 600, 1200, 2400, JOINT_ANGLE_MIN_RAD, JOINT_ANGLE_HOME_RAD, JOINT_ANGLE_MAX_RAD},
    {3, 600, 2200, 2400, JOINT_ANGLE_MIN_RAD, JOINT_ANGLE_HOME_RAD, JOINT_ANGLE_MAX_RAD},
    {4, 600, 1200, 2400, JOINT_ANGLE_MIN_RAD, JOINT_ANGLE_HOME_RAD, JOINT_ANGLE_MAX_RAD},
    {5, 600, 1200, 2400, JOINT_ANGLE_MIN_RAD, JOINT_ANGLE_HOME_RAD, JOINT_ANGLE_MAX_RAD},
};

#endif
