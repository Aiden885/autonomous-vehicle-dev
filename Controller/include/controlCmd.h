#ifndef CONTROL_CMD_H
#define CONTROL_CMD_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 控制指令结构体
 * @en_name TrajPoint
 * @cn_name 控制指令结构体
 * @type module
 * @tag STRUCT
 *
 * @field name=steer,   type=double, unit=rad,     desc="控制算法计算输出的前轮转角"
 * @field name=speed,   type=double, unit=m/s,   desc="控制计算输出的车辆速度"
 * @field name=acceleration, type=double, unit=m/s^2, desc="控制计算输出的纵向加速度"
 *
 * @version 1.0
 * @date 2026-01-15
 * @author lupeng
 */
typedef struct
{
    double steer;        // 转向角 (rad)
    double speed;        // 车速 (m/s)
    double acceleration; // 加速度 (m/s^2)

} ControlCmd;

#ifdef __cplusplus
}
#endif

#endif // CONTROL_CMD_H