#pragma once

#include "ivics/ivics.h"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <zmq.h>
#include <zmq.hpp>
#include <cmath>
#include <chrono>
#include "../proto/roadmsg.pb.h"
#include "../proto/imu.pb.h"
#include "../proto/RoutingMsg.pb.h"
#include "../proto/traffic_light.pb.h"
#include "coorconv.h"
#include "ztgeographycoordinatetransform.h"
#include "../proto/replymsg.pb.h"
#include "../proto/rsmsg.pb.h"
#include <mutex>


class Object
{
public:
    Object() : Type(0), ObjectID(0), Lat(0), Lon(0), X(0), Y(0), Yaw(0), Velocity(0), Power(100), Finish(0), Len(0), Width(0), Height(0), Timestamp(0), status(0), vctype(0), sensorid(0) {}
    Object(const Object &_other) : Type(_other.Type), ObjectID(_other.ObjectID), Lat(_other.Lat), Lon(_other.Lon), X(_other.X), Y(_other.Y), Yaw(_other.Yaw), Velocity(_other.Velocity), Power(_other.Power), Finish(_other.Finish), Len(_other.Len), Width(_other.Width), Height(_other.Height), Timestamp(_other.Timestamp), status(_other.status), vctype(_other.vctype), sensorid(_other.sensorid) {}
    ~Object() {}

public:
    int Type;
    long ObjectID;
    double Lat, Lon;
    double X, Y;
    double Yaw;
    double Velocity;
    double Power;
    bool Finish;
    int Len;
    int Width;
    int Height;
    long long Timestamp;
    int status;
    int vctype;
    int sensorid;
};

class SelfVehicle
{
public:
    double gaussX;
    double gaussY;
    double lat;
    double lon;
    double yaw;
};

extern std::map<int, Object> hisMap;
extern std::map<int,std::chrono::seconds> timeMap;

// write file struct
struct ParameSct
{
    int ssmWriteFlag;          // ssm data write to file flag
    int spartWriteFlag;        // spart data write to file flag
    std::string ssmFilePath;   // ssm write file path
    std::string spartFilePath; // spart write file path
    void *pupbssm;             // ssm pub
    void *pubspart;            // spart pub

    std::string serverReqAddress;//server ip & port
    ObjectsVec * ptrMsgvec;  //server send data
    bool * ptrBRenewMsgvec ;//server send data ready
    std::mutex * ptrMutexMsgvec;//server send data lock
    SelfVehicle *  mySelfVehicle;
};



class V2x
{
public:
    V2x();
    ~V2x();
    bool init();
    void SSMCallback(const Ivics::SSM &sSM, ParameSct &fileSct);
    void SPATCallback(const Ivics::SPAT &sPAT, ParameSct &fileSct);
    bool writeGlobalPlanning2File(Planning::RoutingPointVec *routePointVec, std::string filePath);
    

private:
    void SSM2Probuf(const Ivics::SSM &msg_SSM, RoadsideMsgVec *vec,ParameSct &ps);       // ssm data proto
    void excludeOwnSelfVehicle(RoadsideMsgVec* roadSideVec,SelfVehicle &selfVehicle)  ;
     void CoordTran2DForNew0INOld(double &dX, double &dY, double &dPhi, double dX0, double dY0, double dPhi0);
    int convertType(int type);                                             // type convert
    bool writeSSm2File(RoadsideMsgVec *msgvec, std::string filePath);      // roadside ssm data write to file
    bool publishSSM(RoadsideMsgVec *msgvec, void *publisher);              // publish ssm data to vehicle
    void Light2Probuf(const Ivics::SPAT &msg_sPAT, infopack::SPAT *spart); // light data proto
    bool publishLight(infopack::SPAT *spart, void *publisher);             // publish light data to vehicle
    bool writeLight2File(infopack::SPAT *spart, std::string filePath);     // light data write to file
    double to_radians(double degrees);
    double calculate_distance(double lat1, double lon1, double lat2, double lon2);

private:
    double offset[2] = {275000.02, 3479281.5}; // 偏移量
    const double PI = 3.14159265358979323846;
    int time_threshold = 5;
    double disT = 0.2;
    int degT = 10;
    double dis_thre=2.5;
    double radius_earth = 6371; // 地球半径（单位：公里）
    double pre_max = 0.2;//最大感知范围
};