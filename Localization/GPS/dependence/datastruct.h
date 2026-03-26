#ifndef DATASTRUCT_H
#define DATASTRUCT_H
#include <list>
#include <vector>
#include <QWidget>
using namespace std;
class datastruct
{
public:
    datastruct();
};

typedef struct
{
    double     longitude;//经度 度
    double     latitude;//纬度 度
    double      altitude;//高度 m
    double      yaw;//航向角 rad  (-Pi,+Pi] 正北0
    double      roll; //翻滚角 rad
    double      pitch;//俯仰角 rad
    double      velocityNorth; //北向车速 m/s
    double      velocityEast; //东向车速 m/s
    double      velocityUp;//垂直地面速度 m/s
    double      velocity; //车速 m/s
    QString      locationStatus;//定位模式  50 NARROW_INT//XLGPS里面没有
    short      yawStatus;//定向模式//XLGPS里面没有
    char      calibrationStatus;//bit0: 位置 bit1: 速度  bit2: 姿态 bit3: 航向角 //XLGPS里面没有
    int16_t    satelliteNumber;//卫星数量
    int64_t    utime; //by syp 20220720 时间，与XLGPS的定义一致

///////////////////////////////////////////
    double gaussX;// = 3;
    double gaussY;// = 4;
    int gpsValid;// = 5;

//add for test
    int64_t ltime;// = 9;
    double gyroX;// = 13;
    double gyroY;// = 14;
    double gyroZ;// = 15;
    double accX;// = 16;
    double accY;// = 17;
    double accZ;// = 18;
}GPS;

typedef struct
{
    double x = 0;
    double y = 0;
}PositionXY;

typedef struct {
    QString PortName = "";//
    QString PortNameRTK = "";
    QString usr_pwd = "";
    QString NTripCasterIP = "";
    int NTripCasterPort = 0;
    
}INSDGPSConfigureStruct;

typedef struct
{
    int64_t    utime; //时间
    double     longitude;//经度 度
    double     latitude;//纬度 度
    float      altitude;//高度 m
    float      yaw;//航向角 rad  (-Pi,+Pi] 正北0
    float      roll; //翻滚角 rad
    float      pitch;//俯仰角 rad
    float      yawRate;//横摆角速度 rad/s  左转负，右转正
    float      velocityNorth; //北向车速 m/s
    float      velocityEast; //东向车速 m/s
    float      velocityUp;//垂直地面速度 m/s
    float      accelerationLongitudinal;//纵向加速度 m/s2
    float      accelerationLateral;//横向加速度 m/s2
    float      acceleration;//向地加速度 m/s2
    float      velocity; //车速 m/s
    float      locationStatus;//定位模式0 invalid;1 GPS fix (SPS);2 DGPS fix;3 PPS fix;4 Real Time Kinematic;5 Float RTK;6 estimated (dead reckoning);7 Manual input mode;8 Simulation mode; 9其他
    float      confidenceLevel;//可信度
    float      yawEstimate; //航向角估计值
    double     longitudeEstimate;//经度估计值
    double     latitudeEstimate; //纬度估计值
    int16_t    satelliteNumber;//卫星数量
    QString    rktStatusString;//
}XLGPS;
#endif // DATASTRUCT_H
