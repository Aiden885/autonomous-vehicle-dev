#include "GPYBM.h"
#include <iostream>
#include <string>
#include <vector>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <cmath>

using namespace std;

NMEAGPYBM::NMEAGPYBM() : NMEABase()
{
    strTalkerID = "GPYBM";
    nFiledNum = 25;
}

//解析数据包
bool NMEAGPYBM::Parse(string strBuff)
{
    // CalcCheckSum(strBuff);

    //去掉最后的回车换行
    strBuff = strBuff.substr(0, strBuff.length() - 2);

    vector<string> vStrTemp;
    boost::split(vStrTemp, strBuff, boost::is_any_of(",*"), boost::token_compress_off);

    if (vStrTemp.size() != nFiledNum)
    {
        cout << "field num is error ,fieldNum = " << vStrTemp.size() << endl;
        return false;
    }

    // for (vector<string>::iterator it = vStrTemp.begin(); it != vStrTemp.end(); ++it)
    //     cout << *it << endl;

    strSerialNO = vStrTemp[1];                       // 设备序列号SNxxxxxxxx,x=0-9
    dTod = atof(vStrTemp[2].c_str());                // UTC 时间（时/分/秒/小数秒） hhmmss.ss
    dLatitude = atof(vStrTemp[3].c_str());           // Lat 纬度，度 +：北纬，-：南纬 dd.ddddddddd
    dLongtitude = atof(vStrTemp[4].c_str());         // Lon 经度，度 +：东经，-：西经 ddd.ddddddddd
    dElpHeight = atof(vStrTemp[5].c_str());          // 椭球高.xxx（m）
    dHeading = atof(vStrTemp[6].c_str());            // 航向角0-360°.xxx（度）
    dPitch = atof(vStrTemp[7].c_str());              //俯仰角 -90～90° .xxx（度）
    dVn = atof(vStrTemp[8].c_str());                 // Vel N 北方向速度.xxx（m/s）
    dVe = atof(vStrTemp[9].c_str());                 // Vel E 东方向速度.xxx（m/s）
    dVu = atof(vStrTemp[10].c_str());                // Vel D 地向速度.xxx（m/s）
    dVelocity = atof(vStrTemp[11].c_str());          // Vel G 地面速度.xxx（m/s）
    dNorthing = atof(vStrTemp[12].c_str());          // Coordinate Northing 高斯投影坐标X 轴 参考PTNL 和PJL .xxx（m）
    dEasting = atof(vStrTemp[13].c_str());           // Coordinate Easting 高斯投影坐标Y 轴 参考PTNL 和PJK .xxx（m）
    dNorthDis = atof(vStrTemp[14].c_str());          // North Distance 基站坐标系下的移动站X 坐标（基站坐标为原点）+：北，-：南 .xxx（m）
    dEastDis = atof(vStrTemp[15].c_str());           // 16 East Distance 基站坐标系下的移动站Y 轴 坐标（基站坐标为原点） +：东，-：西 .xxx（m）
    nPositionIndicator = atoi(vStrTemp[16].c_str()); //定位状态 Position Indicator 定位解状态：0=未定位或无效解 1=单点定位 4=定位RTK 固定解 5=定位RTK 浮点解
    nHeadingIndicator = atoi(vStrTemp[17].c_str());  //定向解状态:0=未定位 1=单点定位 4=定向RTK 固定解 5=定向RTK 浮点解
    nSVMaster = atoi(vStrTemp[18].c_str());          // 主站天线收星数
    dAge = atof(vStrTemp[19].c_str());               // Diff Age 差分延迟
    strRefStationID = vStrTemp[20];                  // Station ID 基准站ID 0000
    dBaselineLength = atof(vStrTemp[21].c_str());    //主站和从站内之间的距离（双天线基线长）.xxx（米）
    nSVRover = atoi(vStrTemp[22].c_str());           // Solution sv 从站参与解算的卫星数
    dRoll = atof(vStrTemp[23].c_str());              //横滚角（仅带有惯导模块的板卡或整机支持）.xxx（度）
    strCheckSum = vStrTemp[24];                      //*xx 校验值*hh

    //数据单位转换和合理性检查
    if (dLatitude > 54 || dLatitude < 3)
    {
        cout << "wrong  dLatitude = " << dLatitude << endl;
        return false;
    }

    if (dLongtitude > 136 || dLongtitude < 73)
    {
        cout << "wrong  dLongtitude = " << dLongtitude << endl;
        return false;
    }

    if (dHeading > 360 || dHeading < 0)
    {
        cout << "wrong  dHeading = " << dHeading << endl;
        return false;
    }

    if (dPitch > 90 || dPitch < -90)
    {
        cout << "wrong  dPitch = " << dPitch << endl;
        return false;
    }

    if (nPositionIndicator == 4 && nHeadingIndicator == 4)
    {
        nQuality = 4; //定位状态
    }
    else if ((nPositionIndicator == 4 && nHeadingIndicator == 5) ||
             (nPositionIndicator == 5 && nHeadingIndicator == 4) ||
             (nPositionIndicator == 5 && nHeadingIndicator == 5))
    {
        nQuality = 5;
    }
    else
    {
        nQuality = min(nPositionIndicator, nHeadingIndicator);
    }

    nNumber = min(nSVMaster, nSVRover); //卫星数

    if (dRoll > 180 || dRoll < -180)
    {
        cout << "wrong  dRoll = " << dRoll << endl;
        return false;
    }

    //去掉$ 和 * 之后的字符
    strBuff = strBuff.substr(1, strBuff.length() - 4);
    long nTemp = CalcCheckSum(strBuff);

    long nCheckSum = strtol(("0X" + strCheckSum).c_str(), NULL, 16);
    if (nTemp != nCheckSum)
    {
        cout << "wrong  CheckSum = " << endl;
        return false;
    }

    return true;
}