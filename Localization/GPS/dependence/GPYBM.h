#ifndef NMEA_GPYBM_H
#define NMEA_GPYBM_H

#include "NMEABase.h"

class NMEAGPYBM : public NMEABase
{
public:
    /**
     * @brief Construct a new NMEAGPYBM object
     *
     */
    NMEAGPYBM();
    /**
     * @brief 解析数据包
     *
     * @param strBuff 接收的一条NMEA语句
     * @return true  解析成功
     * @return false 解析失败
     */
    bool Parse(string strBuff);

    string strSerialNO;     ///< serial no 设备序列号SNxxxxxxxx,x=0-9
    double dTod;                ///< UTC 时间（时/分/秒/小数秒） hhmmss.ss
    double dLatitude;      ///< Lat 纬度，度 +：北纬，-：南纬 dd.ddddddddd
    double dLongtitude;     ///< 5 Lon 经度，度 +：东经，-：西经 ddd.ddddddddd
    double dElpHeight;       ///< 椭球高.xxx（m）
    double dHeading;         ///< 航向角0-360°.xxx（度）
    double dPitch;            ///<俯仰角 -90～90° .xxx（度）
    double dVn;             ///< Vel N 北方向速度.xxx（m/s）
    double dVe;             ///< Vel E 东方向速度.xxx（m/s）
    double dVu;             ///< Vel D 地向速度.xxx（m/s）
    double dVelocity;       ///< Vel G 地面速度.xxx（m/s）
    double dNorthing;       ///< Coordinate Northing 高斯投影坐标X 轴 参考PTNL 和PJL .xxx（m）
    double dEasting;        ///< Coordinate Easting 高斯投影坐标Y 轴 参考PTNL 和PJK .xxx（m）
    double dNorthDis;       ///< North Distance 基站坐标系下的移动站X 坐标（基站坐标为原点）+：北，-：南 .xxx（m）
    double dEastDis;        ///< 16 East Distance 基站坐标系下的移动站Y 轴 坐标（基站坐标为原点） +：东，-：西 .xxx（m）
    int nPositionIndicator; ///<定位状态 Position Indicator 定位解状态：0=未定位或无效解 1=单点定位 4=定位RTK 固定解 5=定位RTK 浮点解
    int nHeadingIndicator;  ///<定向解状态:0=未定位 1=单点定位 4=定向RTK 固定解 5=定向RTK 浮点解
    int nSVMaster;          ///< 主站天线收星数
    double dAge;            ///< Diff Age 差分延迟
    string strRefStationID; ///< Station ID 基准站ID 0000
    double dBaselineLength; ///<主站和从站内之间的距离（双天线基线长）.xxx（米）
    int nSVRover;           ///<Solution sv 从站参与解算的卫星数
    double dRoll;           ///<横滚角（仅带有惯导模块的板卡或整机支持）.xxx（度）

    int nQuality; ///<定位状态
    int nNumber;  ///<卫星数
};
#endif