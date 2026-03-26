#include "nmeadata.h"

NMEAData::NMEAData()
{

}

int64_t NMEAData::GetUTCFromGNSSWeekMs(int nWeek, int nMsInWeek)
{
    int64_t difFromGNSSBegin = (int64_t)nWeek * 604800 * 1000 + nMsInWeek; // 604800 = 7*24*60*60 (一周的秒数)
    QDateTime gnssBeginTime(QDate(1980, 1, 6), QTime(0, 0, 0));
    QDateTime utcTime = gnssBeginTime.addMSecs(difFromGNSSBegin - 18); //跳秒 18
    return utcTime.toMSecsSinceEpoch(); //  转换成毫秒的长整型
}

unsigned int NMEAData::GetCRC(QByteArray byteSource,int nStart, int nEnd)
{
    unsigned char chCRC = 0;
    for(int i=nStart; i<nEnd;i++)
    {
        chCRC = chCRC ^ byteSource[i];
    }

    return (unsigned int)chCRC;

}

