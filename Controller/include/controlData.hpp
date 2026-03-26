
#ifndef _CONTROL_DATA_H
#define _CONTROL_DATA_H


#include <vector>


namespace Control{

struct State
{
    double x;
    double y;
    double yaw;
    double v;
    double accX;
    double rtkMode;
};


struct  TrajPoint
{
    double x;
    double y;
    double yaw;
    double v;
    double k;
    double s;
    double acc;
};

using Traj = std::vector<TrajPoint>;

struct ControlCMD
{
    double steer;
    double speed;
    double acceleration;
    int driveMode;
};

}

#endif