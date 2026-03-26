#pragma once

#ifndef __INTERPOLATE_M_H__
#define __INTERPOLATE_M_H__

/* Interpolate speed along the path */

#include <cmath>
#include <vector>

/**
 * @brief 插值函数生成相关函数
 * @file interpolate_m.h
 * @version 0.0.1
 * @author Zehang Zhu (zzh22@mails.tsinghua.edu.cn)
 * @date 2023-11-25
 */

namespace planning {
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
}

#endif 
