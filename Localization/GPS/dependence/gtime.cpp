#include "gtime.h"
#include <QDebug>

GTIME::GTIME()
{
    strTalker = "$GTIMU";
    nFieldSize = 11;
    lutctime = 0;
}

bool GTIME::parse(QString strSource)
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
    dGyroX = args[3].toDouble(); //

    if (args[4] == "")
        return false;
    dGyroY = args[4].toDouble(); //

    if (args[5] == "")
        return false;
    dGyroZ = args[5].toDouble(); //

    if (args[6] == "")
        return false;
    dAccX = args[6].toDouble(); //

    if (args[7] == "")
        return false;
    dAccY = args[7].toDouble(); //

    if (args[8] == "")
        return false;
    dAccZ = args[8].toDouble(); //

    if (args[9] == "")
        return false;
    dTpr = args[9].toDouble(); //


    if (args[10] == "")
        return false;
    n_cs = args[10].toUInt(NULL, 16);

    //计算奇偶校验位,并进行检核，
    QByteArray byteSource = strSource.toUtf8();
    int nCRCLength = strSource.length() - 3;
    unsigned int nTemp = GetCRC(byteSource, 1, nCRCLength);

    if (n_cs != nTemp) // CRC校验失败
    {
        qDebug() << "$GTIMU CRC verfication failed " << strSource;
        return false;
    }

    return true;
}
