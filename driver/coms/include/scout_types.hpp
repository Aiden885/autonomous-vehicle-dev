/*
 * scout_state.hpp
 *
 * Created on: Jun 11, 2019 08:48
 * Description:
 *
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef SCOUT_STATE_HPP
#define SCOUT_STATE_HPP

#include <cstdint>
#include <iostream>
#include <math.h>

namespace westonrobot
{
struct ScoutState
{
    // motion state
    double velocity = 0;
    double steering_angle = 0;
    uint32_t mode = 0;
};

template <typename T> struct ScoutMotionCmd
{
    ScoutMotionCmd(int8_t linear = 0, int8_t angle = 0) : linear_velocity(linear), steering_angle(angle) {}

    T linear_velocity;
    T steering_angle;
    uint8_t speed_gear = 1;
    uint8_t along_enable = 1;
    uint8_t cross_enable = 1;
    uint8_t gear_shift = 1;

    static constexpr double max_linear_velocity = 3; // 6km/h
    static constexpr double min_linear_velocity = 0;
    static constexpr double max_steering_angle = 720;  // 720°
    static constexpr double min_steering_angle = -720; // -720°
};
} // namespace westonrobot

#endif /* SCOUT_STATE_HPP */
