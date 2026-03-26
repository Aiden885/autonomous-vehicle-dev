#ifndef GTIME_H
#define GTIME_H

#include <QObject>
#include "nmeadata.h"

class GTIME : public NMEAData
{
public:
    GTIME();
    bool parse(QString strSource);

//raw data
    int nGPSWeek;
    int nGPSTime_ms;
    double dGyroX;
    double dGyroY;
    double dGyroZ;
    double dAccX;
    double dAccY;
    double dAccZ;
    double dTpr;

//easy used data
    int64_t lutctime;

};

#endif // GTIME_H
