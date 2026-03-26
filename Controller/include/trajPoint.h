#ifndef TRAJ_POINT_H
#define TRAJ_POINT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>

/**
 * @brief 轨迹点结构体
 * @en_name TrajPoint
 * @cn_name 轨迹点结构体
 * @type module
 * @tag STRUCT
 *
 * @field name=x,   type=double, unit=m,     desc="轨迹点在全局坐标系下的 x 位置"
 * @field name=y,   type=double, unit=m,     desc="轨迹点在全局坐标系下的 y 位置"
 * @field name=yaw, type=double, unit=rad,   desc="车辆航向角（以全局坐标系 X 轴为参考顺时针为正，范围[0-360））"
 * @field name=v,   type=double, unit=m/s,   desc="轨迹点对应的车辆速度"
 * @field name=k,   type=double, unit=1/m,   desc="轨迹点曲率，用于横向控制与转向前馈"
 * @field name=s,   type=double, unit=m,     desc="从轨迹起点到当前轨迹点的累计弧长"
 * @field name=acc, type=double, unit=m/s^2, desc="轨迹点对应的纵向加速度"
 *
 * @version 1.0
 * @date 2026-01-15
 * @author lupeng
 */
typedef struct
{
    double x;
    double y;
    double yaw;
    double v;
    double k;
    double s;
    double acc;
} TrajPoint;

/**
 * @brief 轨迹结构体
 * @en_name Traj
 * @cn_name 轨迹
 * @type module
 * @tag STRUCT
 *
 * @field name=points, type=TrajPoint*, desc="轨迹点数组"
 * @field name=size,   type=int, unit=1, desc="轨迹点数量"
 *
 * @version 1.0
 * @date 2026-01-15
 * @author lupeng
 */
typedef struct {
    TrajPoint* points;
    size_t size;
} Traj;

#ifdef __cplusplus
}
#endif

#endif /* TRAJ_POINT_H */
