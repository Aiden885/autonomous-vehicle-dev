#ifndef  NMEA_GGA_H
#define NMEA_GGA_H

#include "NMEABase.h"

class NMEAGGA:public NMEABase
{
    public:
        /**
         * @brief Construct a new NMEAGGA object
         * 
         */
        NMEAGGA();
        /**
         * @brief 解析数据包
         * 
         * @param strBuff 接收的一条NMEA语句
         * @return true  解析成功
         * @return false 解析失败
         */
        bool Parse(string strBuff);
           
    
        double dTod; ///< UTC of position (hhmmss) 待定
        double dLatitude; ///<  Latitude  
        char chNs; ///<  N=north,S=south 
        double dLongtitude; ///<  Longitude
        char chEw; ///<  E=east,W=west 
        int nQuality; ///<  GPS quality indicator 
        int nNumber; ///<   Number of satellites in use 
        double dHdop; ///<  HDOP  Horizontal dilution of position
        double dAltitude; ///<  Antenna altitude above/below mean sea level (geoid) 
        char chUa; ///<  unit (M) 
        double dMsl; ///<  Geoid separation 
        char chUm; ///<  unit (M) 
        double dAge; ///<  Age of differential 
        string strRefStationID;///<  reference station ID 
   

};
#endif