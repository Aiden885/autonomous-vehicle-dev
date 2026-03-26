#pragma once

#ifndef __INTERPOLATE_M_H__
#define __INTERPOLATE_M_H__

/* Interpolate speed along the path */

#include <cmath>
#include <vector>

/**
 * \namespace interp_m
 * \brief 这是一个名为interp_m的namespace。
 */
// namespace interp_m {
    using std::size_t;

    struct interpLinearParam{
    };

    struct interpLinearInput{
        double start_speed;
        double end_speed;
        size_t num;
    };

    struct interpLinearOutput{
        std::vector<double> result;
    };

    void linear(const interpLinearParam &param, const interpLinearInput &input, interpLinearOutput &output);

    struct interpConstAccParam{
    };

    struct interpConstAccInput{
        double start_speed;
        double end_speed;
        size_t num;
        double acceleration;
    };

    struct interpConstAccOutput{
        std::vector<double> result;
    };

    void constant_acceleration(const interpConstAccParam &param, const interpConstAccInput &input, interpConstAccOutput &output);
// }

#endif 
