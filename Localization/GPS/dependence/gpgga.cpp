#include "gpgga.h"
#include <QDebug>

GPGGA::GPGGA()
{
    strTalker = "$GPGGA";
    nFieldSize = 16;

}

//$GPGGA,044744.00,3122.4658,N,12025.2791,E,1,10,3.00,12.575,M,7.100,M,00,0000*5F
bool GPGGA::parse(QString strSource)
{

    //qDebug() << "parse data" << strSource;

    strSource = strSource.simplified();
    strSource.replace("*", ",");             //

    QStringList args = strSource.split(","); //用，拆分
    if(args.size() != nFieldSize)
        return false;

   if (args[1] == "")
        return false;
    strTime = args[1]; // time
    nUtctime.fromString(strTime,"HHmmSS.zz");


    if (args[2] == "")
        return false;
    dLattitude = args[2].toDouble()/100.; //lat
    if (dLattitude >54.0 || dLattitude < 3.0)
        return false;

    if (args[4] == "")
        return false;
    dLongitude = args[4].toDouble()/100.; //Lon
    if (dLongitude >136.0 || dLongitude < 73.0)
        return false;

    if (args[6] == "")
        return false;
    nGNSSStatus = args[6].toInt(); //

    if (args[7] == "")
        return false;
    nNSV = args[7].toInt(); //

    if (args[8] == "")
        return false;
    dDop = args[8].toDouble(); //

    if (args[9] == "")
        return false;
    dAltitude = args[9].toDouble(); //

    if (args[11] == "")
        return false;
    dGeoidSeparation = args[11].toDouble(); //

    if (args[13] == "")
        dAge = -1;
    else
        dAge = args[13].toInt();

    //if (args[14] == "")
    //    return false;
    strRefID = args[14];

    if (args[15] == "")
        return false;
    n_cs = args[15].toUInt(NULL, 16);

    //计算奇偶校验位,并进行检核，
    QByteArray byteSource = strSource.toUtf8();
    int nCRCLength = strSource.length() - 3;
    unsigned int nTemp = GetCRC(byteSource, 1, nCRCLength);

    if (n_cs != nTemp) // CRC校验失败
    {
        qDebug() << "$GPGGA CRC verfication failed " << strSource;
        return false;
    }

    return true;
}
