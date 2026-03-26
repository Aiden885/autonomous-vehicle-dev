#ifndef GPGGA_H
#define GPGGA_H

#include <QObject>
#include "nmeadata.h"

class GPGGA : public NMEAData
{
public:
    GPGGA();
    bool parse(QString strSource);
    
//raw data
//$GPGGA,044744.00,3122.4658,N,12025.2791,E,1,10,3.00,12.575,M,7.100,M,00,0000*5F    
    QString strTime;
    double dLattitude;
    double dLongitude;
    int nGNSSStatus;
    int nNSV;
    double dDop;
    double dAltitude;
    double dGeoidSeparation;
    double dAge;
    QString strRefID;

//easy used data
    QTime nUtctime;
  
};

#endif // GPGGA_H

