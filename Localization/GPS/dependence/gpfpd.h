#ifndef GPFPD_H
#define GPFPD_H


#include <QObject>
#include "nmeadata.h"

class GPFPD : public NMEAData
{
public:
    GPFPD();
    bool parse(QString strSource);
    int transGNSSStatus(QString strSource);

//raw data
    int nGPSWeek;
    int nGPSTime_ms;
    double dHeading;
    double dPitch;
    double dRoll;
    double dLattitude;
    double dLongitude;
    double dAltitude;
    double dVe;
    double dVn;
    double dVu;
    double dBaseline;
    int nNSV1;
    int nNSV2;
    QString strStatus;

//easy used data
    int64_t lutctime;
    int nGNSSStatus;
    double dVelocityEN;

};

#endif // GPFPD_H
