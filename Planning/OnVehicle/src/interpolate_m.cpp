/* Interpolate speed along the path */

#include "interpolate_m.h"

/**
 * @brief 插值函数
 * @file interpolate_m.cpp
 * @version 0.0.1
 * @author Zehang Zhu (zzh22@mails.tsinghua.edu.cn)
 * @date 2023-11-25
 */

namespace planning {
    using std::size_t;

    /**
     * @brief 根据初值、终值和项数，进行线性插值
     * @param[IN] param 无
     * @param[IN] input 初值、终值、项数
     * @param[OUT] output 储存插值的向量数组
     
     * @cn_name: 线性插值
     
     * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
     
     * @tag: planning
     */
    void linear(const interpLinearParam &param, const interpLinearInput &input, interpLinearOutput &output){
        double start_speed = input.start_speed;
        double end_speed = input.end_speed;
        size_t num = input.num;
        std::vector<double> result;
        result.reserve(num);
        double step = (end_speed - start_speed) / (num - 1);
        for (size_t i = 0; i < num; ++i) {
            result.push_back(start_speed + step * i);
        }
        output.result = result;
        return;
    }

    /**
     * @brief 在等间隔的情况下，进行匀加速度插值
     * @param[IN] param 无
     * @param[IN] input 初值、终值、项数、加速度
     * @param[OUT] output 储存插值的向量数组
     
     * @cn_name: 匀加速度插值
     
     * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
     
     * @tag: planning
     */
    void constant_acceleration(const interpConstAccParam &param, const interpConstAccInput &input, interpConstAccOutput &output){
        // Speed at equal distance intervals, with constant acceleration
        double start_speed = input.start_speed;
        double end_speed = input.end_speed;
        size_t num = input.num;
        double acceleration = input.acceleration;

        std::vector<double> result;
        result.reserve(num);
        for (size_t i = 0; i < num; ++i) {
            double t = (double)i / (num - 1);
            double v = sqrt(start_speed * start_speed + 2 * acceleration * 15.0 * t);
            if (acceleration > 0 && v > end_speed) {
                v = end_speed;
            } else if (acceleration < 0 && v < end_speed) {
                v = end_speed;
            }
            v = end_speed;
            result.push_back(v);
        }
        output.result = result;
        return;
    }
}