#pragma once




#include "base_types.hpp"


namespace control {


/**
 * @brief GlobalParams 结构体
 */
struct GlobalParams {
    Real lks_l0;
    Real lks_rt;
    Real lks_rAlpha;
    Real lks_curvatureThreshold;
    Real lks_nearPreviewDistance;
    Real lks_w1;
    Real lks_w2;
    Real lks_w3;
    Real lks_Kp;
    Real lks_steerScale;
    Real lks_vMin;
    Real lks_driverSteerThreshold;
    Real lks_ayMax;
    Real lks_wheelBase;
    Real lks_frontWheelMaxRad;
};



} // namespace control