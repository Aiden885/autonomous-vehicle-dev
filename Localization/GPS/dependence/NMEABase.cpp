#include "NMEABase.h"
#include <math.h>

NMEABase::NMEABase()
{
    strStart = "$";
    strEnd = {0x0D, 0x0A};
    nFiledNum = 0;
}

bool NMEABase::Parse(string strBuff)
{
    return false;
}

long NMEABase::CalcCheckSum(string strBuff)
{
    char chCRC = 0x00;

    for (int i = 0; i < strBuff.size(); i++) 
    {
        chCRC = chCRC ^ strBuff[i];
    }

    return (long ) chCRC;
}

/* convert ddmm.mm in nmea format to deg -------------------------------------*/
double NMEABase::dmm2deg(double dmm)
{
    return floor(dmm / 100.0) + fmod(dmm, 100.0) / 60.0;
}