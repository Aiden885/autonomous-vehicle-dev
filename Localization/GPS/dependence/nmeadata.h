#ifndef NMEADATA_H
#define NMEADATA_H

#include <QObject>
#include <QDateTime>

class NMEAData
{
public:
    NMEAData();
    unsigned int GetCRC(QByteArray byteSource,int nStart, int nEnd);
    int64_t GetUTCFromGNSSWeekMs(int nWeek, int nMsInWeek);
    virtual bool parse(QString strSource){return false;};

//raw data
    QString strTalker;
    unsigned int n_cs;
    QString strEnd;

//easy used data

    int nFieldSize;
};

#endif // NMEADATA_H
