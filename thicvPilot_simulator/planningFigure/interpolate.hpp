#ifndef __INTERPOLATE_HPP__
#define __INTERPOLATE_HPP__

/* Interpolate speed along the path */

#include <cmath>
#include <vector>

namespace interp {
    using std::size_t;

    std::vector<double> linear(double start_speed, double end_speed, size_t num) {
        std::vector<double> result;
        result.reserve(num);
        double step = (end_speed - start_speed) / (num - 1);
        for (size_t i = 0; i < num; ++i) {
            result.push_back(start_speed + step * i);
        }
        return result;
    }

    std::vector<double> constant_acceleration(double start_speed, double end_speed, size_t num) {
        // Speed at equal distance intervals, with constant acceleration
        std::vector<double> result;
        result.reserve(num);
        for (size_t i = 0; i < num; ++i) {
            double t = (double)i / (num - 1);
            double v = sqrt(t * end_speed * end_speed + (1 - t) * start_speed * start_speed);
            result.push_back(v);
        }
        return result;
    }

    std::vector<double> constant_acceleration(double start_speed, double end_speed, size_t num, double acceleration) {
        // Speed at equal distance intervals, with constant acceleration
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
        return result;
    }
}

#endif // __INTERPOLATE_HPP__
