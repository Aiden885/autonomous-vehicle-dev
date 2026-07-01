#pragma once

#include <algorithm>
#include <cmath>

#include "FuncModule.hpp"

extern "C" int carla_adapter_read_ego_state(double *egoV,
                                            double *egoX,
                                            double *egoY,
                                            double *egoYawRad,
                                            double *egoAcc,
                                            int *valid);
extern "C" int carla_adapter_read_lead_vehicle(double *leadV,
                                               double *distance,
                                               double *relativeSpeed,
                                               double *ttc,
                                               int *valid);
extern "C" int carla_adapter_read_driver_command(int *commandType, int *valid);
extern "C" int carla_adapter_publish_longitudinal_cmd(double targetSpeed,
                                                      int enable);

namespace control {

inline int truth_table_6957bd9f_a281_4c91_a9b5_5b2c83757f58(int u, int v)
{
    int y = 8;

    if ((u == 0) && (v == 1)) {
        y = 1;
    } else if ((u == 0) && (v == 2)) {
        y = 2;
    } else if ((u == 0) && (v == 3)) {
        y = 3;
    } else if ((u == 0) && (v == 4)) {
        y = 4;
    } else if ((u == 0) && (v == 5)) {
        y = 7;
    } else if ((u == 1) && (v == 1)) {
        y = 5;
    } else if ((u == 2) && (v == 1)) {
        y = 5;
    } else if ((u == 1) && (v == 2)) {
        y = 6;
    } else if ((u == 0) && ((v == 6) || (v == 7))) {
        y = 8;
    }

    return y;
}

inline int CARLAACCDriverCommand()
{
    int commandType = 0;
    int valid = 0;
    int rc = 0;
    int result = 0;

    rc = carla_adapter_read_driver_command(&commandType, &valid);
    if ((rc == 0) && (valid != 0)) {
        result = commandType;
    }

    return result;
}

inline Real CARLAACCLeadSpeed()
{
    Real leadV = 0.0;
    Real distance = 1000000.0;
    Real relativeSpeed = 0.0;
    Real ttc = 1000000.0;
    int valid = 0;
    int rc = 0;
    Real result = 0.0;

    rc = carla_adapter_read_lead_vehicle(&leadV, &distance, &relativeSpeed, &ttc, &valid);
    if ((rc == 0) && (valid != 0)) {
        result = leadV;
    }

    return result;
}

inline Real CARLAACCEgoSpeed()
{
    Real egoV = 0.0;
    Real egoX = 0.0;
    Real egoY = 0.0;
    Real egoYawRad = 0.0;
    Real egoAcc = 0.0;
    int valid = 0;
    int rc = 0;
    Real result = 0.0;

    rc = carla_adapter_read_ego_state(&egoV, &egoX, &egoY, &egoYawRad, &egoAcc, &valid);
    if ((rc == 0) && (valid != 0)) {
        result = egoV;
    }

    return result;
}

inline Real CARLAACCLeadDistance()
{
    Real leadV = 0.0;
    Real distance = 1000000.0;
    Real relativeSpeed = 0.0;
    Real ttc = 1000000.0;
    int valid = 0;
    int rc = 0;
    Real result = 1000000.0;

    rc = carla_adapter_read_lead_vehicle(&leadV, &distance, &relativeSpeed, &ttc, &valid);
    if ((rc == 0) && (valid != 0)) {
        result = distance;
    }

    return result;
}

inline void CARLAACCLongitudinalCmd(Real speed, int enable)
{
    int rc = 0;

    rc = carla_adapter_publish_longitudinal_cmd(speed, enable);
    (void)rc;
}

} // namespace control
