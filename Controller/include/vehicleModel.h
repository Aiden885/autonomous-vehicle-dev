#ifndef VEHICLE_MODEL_H
#define VEHICLE_MODEL_H


/**
 * @brief 车辆状态结构体
 * @en_name VehicleState
 * @cn_name 车辆状态结构体
 * @type module
 * @tag STRUCT
 *
 * @field name=x,   type=double, unit=m,     desc="车辆在全局坐标系下的纵向 x 位置"
 * @field name=v,   type=double, unit=m/s,   desc="车辆对应的车辆速度"
 * @field name=a, type=double, unit=m/s^2, desc="车辆对应的纵向加速度"
 *
 * @version 1.0
 * @date 2026-03-10
 * @author lupeng
 */
typedef struct
{
    double x;   // 车辆位置
    double v;   // 车辆速度
    double a;   // 当前加速度
} VehicleState;

#ifdef __cplusplus
extern "C" {
#endif


void vehicleModelUpdate(
    double accel,
    double dt,
    VehicleState* state
);

#ifdef __cplusplus
}
#endif

#endif