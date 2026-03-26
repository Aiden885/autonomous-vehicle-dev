#include "gpfpd.h"
#include <math.h>
#include <QDebug>

GPFPD::GPFPD()
{
    strTalker = "$GPFPD";
    nFieldSize = 17;
    lutctime = 0;
}

//strSource = "$GPFPD,2218,194068.100,90.291,-0.433,-1.023,39.99989628,116.33022512,33.661,-0.002,0.000,-0.001,1.507,14,13,04*46\r\n";
bool GPFPD::parse(QString strSource)
{

    //qDebug() << "parse data" << strSource;
    
    strSource = strSource.simplified();
    strSource.replace("*", ",");             //

    QStringList args = strSource.split(","); //用，拆分
    if(args.size() != nFieldSize)
        return false;

   if (args[1] == "")
        return false;
    nGPSWeek = args[1].toInt(); // GNSS周数
    
    if (args[2] == "")
        return false;
    
    QStringList listSecond;                    //用.拆分秒
    listSecond = args[2].split(".");
    if(listSecond.size() != 2)
        return false;
    
    nGPSTime_ms = listSecond[0].toDouble() * 1000 + listSecond[1].toDouble(); // GNSS周内ms
    lutctime = GetUTCFromGNSSWeekMs(nGPSWeek,nGPSTime_ms);
    
    if (args[3] == "")
        return false;
    dHeading = args[3].toDouble(); //航向角
    if (dHeading >= 360.0 || dHeading < 0.0)
        return false;

    if (args[4] == "")
        return false;
    dPitch = args[4].toDouble(); //
    if (dPitch >90.0 || dPitch < -90.0)
        return false;

    if (args[5] == "")
        return false;
    dRoll = args[5].toDouble(); //
    if (dRoll >180.0 || dRoll < -180.0)
        return false;

    if (args[6] == "")
        return false;
    dLattitude = args[6].toDouble(); // Lat
    if (dLattitude >54.0 || dLattitude < 3.0)
        return false;

    if (args[7] == "")
        return false;
    dLongitude = args[7].toDouble(); // Lon
    if (dLongitude >136.0 || dLongitude < 73.0)
        return false;

    if (args[8] == "")
        return false;
    dAltitude = args[8].toDouble(); //

    if (args[9] == "")
        return false;
    dVe = args[9].toDouble(); //

    if (args[10] == "")
        return false;
    dVn = args[10].toDouble(); //

    dVelocityEN = sqrt(dVe * dVe + dVn * dVn); //速度

    if (args[11] == "")
        return false;
    dVu = args[11].toDouble(); //

    if (args[12] == "")
        return false;
    dBaseline = args[12].toDouble(); //

    if (args[13] == "")
        return false;
    nNSV1 = args[13].toInt(); //

    if (args[14] == "")
        return false;
    nNSV2 = args[14].toInt(); //卫星数目

    if (args[15] == "")
        return false;
    strStatus = args[15];
    nGNSSStatus = transGNSSStatus(strStatus);

    if (args[16] == "")
        return false;
    n_cs = args[16].toUInt(NULL, 16);

    //计算奇偶校验位,并进行检核，
    QByteArray byteSource = strSource.toUtf8();
    int nCRCLength = strSource.length() - 3;
    unsigned int nTemp = GetCRC(byteSource, 1, nCRCLength);

    if (n_cs != nTemp) // CRC校验失败
    {
        qDebug() << "$GPFPD CRC verfication failed " << strSource;
        return false;
    }

    return true;
}

int GPFPD::transGNSSStatus(QString strSource)
{
    if(strSource.isEmpty())
        return false;

    int dgpsvalid;

    if(strSource == "4B")
    {
        dgpsvalid =4;

    }
    else if(strSource == "5B")
    {
        dgpsvalid =5;

    }
    else if(strSource == "00" ||
            strSource == "01" ||
            strSource == "02" ||
            strSource == "07" ||
            strSource == "0c" ||
            strSource == "0F")
    {
        dgpsvalid =0;

    }
    else
    {
        dgpsvalid =6;

    }

    return dgpsvalid;
}
