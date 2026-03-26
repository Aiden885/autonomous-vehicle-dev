#ifndef NMEA_BASE_H
#define NMEA_BASE_H

#include <string>
using namespace std;

class NMEABase
{
public:
    /**
     * @brief Construct a new NMEABase object
     *
     */
    NMEABase();
        /**
         * @brief 解析数据包
         * 
         * @param strBuff 接收的一条NMEA语句
         * @return true  解析成功
         * @return false 解析失败
         */
    bool Parse(string strBuff);        
    /**
     * @brief 计算校验位
     * 
     * @param strBuff 计算校验位字符串
     * @return long 校验位对应的16进制数，用于进行比较
     */
    long CalcCheckSum(string strBuff); //计算校验位
    /**
     * @brief  将NMEA中 ddmm.mm 格式转换为小数度，用于经纬度转换
     * 
     * @param dmm ddmm.mm格式经纬度
     * @return double 小数度格式的经纬度 
     */
    double dmm2deg(double dmm);        

protected:
    string strStart; ///<起始符
    string strTalkerID;///<talker ID
    string strCheckSum;///<校验位
    string strEnd;///<截止符
    int nFiledNum; //字段数量
};

#endif
