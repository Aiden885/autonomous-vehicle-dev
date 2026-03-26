#include "GGA.h"
#include <iostream>
#include <string>
#include <vector>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

using namespace std;

NMEAGGA::NMEAGGA() : NMEABase()
{
    strTalkerID = "GGA";
    nFiledNum = 16;
}

//解析数据包
bool NMEAGGA::Parse(string strBuff)
{
     //去掉最后的回车换行
    strBuff = strBuff.substr(0,strBuff.length()-2);

    vector<string> vStrTemp;
    boost::split(vStrTemp, strBuff, boost::is_any_of(",*"), boost::token_compress_off);

    if (vStrTemp.size() != nFiledNum)
    {
        cout << "field num is error ,fieldNum = " << vStrTemp.size() << endl;
        return false;
    }

    // for (vector<string>::iterator it = vStrTemp.begin(); it != vStrTemp.end(); ++it)
    //     cout << *it << endl;
        
    dTod = atof(vStrTemp[1].c_str());        /* UTC of position (hhmmss) */
    dLatitude = atof(vStrTemp[2].c_str());   /* Latitude (ddmm.mmm) */
    chNs = vStrTemp[3].c_str()[0];           /* N=north,S=south */
    dLongtitude = atof(vStrTemp[4].c_str()); /* Longitude (dddmm.mmm) */
    chEw = vStrTemp[5].c_str()[0];           /* E=east,W=west */
    nQuality = atoi(vStrTemp[6].c_str());    /* GPS quality indicator */
    nNumber = atoi(vStrTemp[7].c_str());     /* # of satellites in use */
    dHdop = atof(vStrTemp[8].c_str());       /* HDOP */
    dAltitude = atof(vStrTemp[9].c_str());   /* Altitude MSL */
    chUa = vStrTemp[10].c_str()[0];          /* unit (M) */
    dMsl = atof(vStrTemp[11].c_str());       /* Geoid separation */
    chUm = vStrTemp[12].c_str()[0];          /* unit (M) */
    dAge = atof(vStrTemp[13].c_str());       /* Age of differential */
    strRefStationID = vStrTemp[14];          /* reference station ID */
    strCheckSum = vStrTemp[15];              /* check sum  */

    //数据单位转换和合理性检查
    if ((chNs!='N'&&chNs!='S')||(chEw!='E'&&chEw!='W')) 
    {
        cout  << "invalid nmea gga format" << endl;
        return false;
    }

    dLatitude = (chNs=='N'?1.0:-1.0)*dmm2deg(dLatitude);
    if(dLatitude >54 || dLatitude<3)
    {
        cout  << "wrong  dLatitude = " << dLatitude << endl;
        return false;
    }

    dLongtitude = (chEw=='E'?1.0:-1.0)*dmm2deg(dLongtitude);
    if(dLongtitude >136 || dLongtitude<73)
    {
        cout  << "wrong  dLongtitude = " << dLongtitude << endl;
        return false;
    }

    //去掉$ 和 * 之后的字符
    strBuff = strBuff.substr(1, strBuff.length() - 4);
    long nTemp = CalcCheckSum(strBuff);

    long  nCheckSum = strtol(("0X" + strCheckSum).c_str(), NULL, 16);
    if(nTemp !=nCheckSum )
    {
        cout  << "wrong  CheckSum = "  << endl;
        return false;        
    }

    
    
    return true;
}