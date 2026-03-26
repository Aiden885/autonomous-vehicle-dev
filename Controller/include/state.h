#ifndef STATE_H
#define STATE_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 车辆状态结构体
 * @en_name State
 * @cn_name 车辆状态结构体
 * @type module
 * @tag STRUCT
 *
 * @field name=x,        type=double, unit=m,     desc="车辆在全局/地图坐标系下的 X 位置"
 * @field name=y,        type=double, unit=m,     desc="车辆在全局/地图坐标系下的 Y 位置"
 * @field name=yaw,      type=double, unit=rad,   desc="车辆航向角（以全局坐标系 X 轴为参考顺时针为正，范围[0-360））"
 * @field name=v,        type=double, unit=m/s,   desc="车辆当前纵向速度"
 * @field name=accX,     type=double, unit=m/s^2, desc="车辆纵向加速度（车体坐标系 X 轴方向）"
 * @field name=rtkMode,  type=double, unit=-,     desc="RTK 定位模式/状态标志"
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
    double accX;
    double rtkMode;
} State;


#ifdef __cplusplus
}
#endif

#endif /* STATE_H */
