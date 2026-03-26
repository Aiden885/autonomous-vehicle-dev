/**
 * @file main.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-12-13
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include"gpsFunctional.h"
using namespace std;

void printGGA(outputGGA d)
{
    cout<<"printGGA"<<endl;
    cout<<"dTod:"<<d.data.dTod<<endl
    <<"dLatitude:"<<d.data.dLatitude<<endl
    <<"chNs:"<<d.data.chNs<<endl
    <<"dLongtitude:"<<d.data.dLongtitude<<endl
    <<"nQuality:"<<d.data.nQuality<<endl
    <<"nNumber:"<<d.data.nNumber<<endl
    <<"dHdop:"<<d.data.dHdop<<endl
    <<"dAltitude"<<d.data.dAltitude<<endl
    <<"chUa:"<<d.data.chUa<<endl
    <<"dMsl:"<<d.data.dMsl<<endl
    <<"chUm:"<<d.data.chUm<<endl
    <<"dAge:"<<d.data.dAge<<endl
    <<"strRefStationID"<<d.data.strRefStationID<<endl;
}

void printGPFPD(outputGPFPD d)
{
    // cout<<"printGPFP"<<endl;
    // cout<<"nGPSWeek:"<<d.data.nGPSWeek<<endl
    // <<"dGPSTime:"<<d.data.nGPSTime_ms<<endl
    // <<"dHeading:"<<d.data.dHeading<<endl
    // <<"dPitch:"<<d.data.dPitch<<endl
    // <<"dRoll:"<<d.data.dRoll<<endl
    // <<"dLatitude:"<<d.data.dLattitude<<endl
    // <<"dLongtitude:"<<d.data.dLongitude<<endl
    // <<"dAltitudel"<<d.data.dAltitude<<endl
    // <<"dVe:"<<d.data.dVe<<endl
    // <<"dVn:"<<d.data.dVn<<endl
    // <<"dVu:"<<d.data.dVu<<endl
    // <<"dBaseline:"<<d.data.dBaseline<<endl
    // <<"nNSV1"<<d.data.nNSV1<<endl
    // <<"nNSV2"<<d.data.nNSV2<<endl
    // <<"strStatus"<<d.data.strStatus<<endl
    // <<"dVelocity"<<d.data.dVelocityEN<<endl;
    // <<"nNumber"<<d.data.nNumber<<endl
    // <<"nQuality"<<d.data.nQuality<<endl
    // <<"strCheckSum"<<d.data.strCheckSum<<endl;
}

void printGPYBM(outputGPYBM d)
{
    cout<<"printGPYBM"<<endl;
    cout<<"strSerialNO:"<<d.data.strSerialNO<<endl
    <<"dTod:"<<d.data.dTod<<endl
    <<"dLatitude:"<<d.data.dLatitude<<endl
    <<"dLongtitude:"<<d.data.dLongtitude<<endl
    <<"dElpHeight:"<<d.data.dElpHeight<<endl
    <<"dHeading:"<<d.data.dHeading<<endl
    <<"dPitch:"<<d.data.dPitch<<endl
    <<"dVn"<<d.data.dVn<<endl
    <<"dVe:"<<d.data.dVe<<endl
    <<"dVu:"<<d.data.dVu<<endl
    <<"dVelocity:"<<d.data.dVelocity<<endl
    <<"dNorthing:"<<d.data.dNorthing<<endl
    <<"dEasting"<<d.data.dEasting<<endl
    <<"dNorthDis"<<d.data.dNorthDis<<endl
    <<"dEastDis"<<d.data.dEastDis<<endl
    <<"nPositionIndicator"<<d.data.nPositionIndicator<<endl
    <<"nHeadingIndicator"<<d.data.nHeadingIndicator<<endl
    <<"nSVMaster"<<d.data.nSVMaster<<endl
    <<"dAge"<<d.data.dAge<<endl
    <<"strRefStationID"<<d.data.strRefStationID<<endl
    <<"dBaselineLength"<<d.data.dBaselineLength<<endl
    <<"nSVRover"<<d.data.nSVRover<<endl
    <<"dRoll"<<d.data.dRoll<<endl
    <<"nQuality"<<d.data.nQuality<<endl
    <<"nNumber"<<d.data.nNumber<<endl;
}



int main(int argc,char **argv)
{
    // paraAPP apppara;
    // inputAPP appin{argc,argv};
    // outputAPP appout;
    // paraGGA ggapara;
    // inputGGA ggain{"$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47"};
    // outputGGA ggaout;
    // paraGPFPD gpfpdpara;
    // inputGPFPD gpfpdin{"$GPFPD,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,4B*3F"};
    // outputGPFPD gpfpdout;
    // paraGPYBM gpybmpara;
    // inputGPYBM gpybmin{"$GPYBM,123519,45.2,12.5,30.8*2E"};
    // outputGPYBM gpybmout;
    // getGGA(ggapara,ggain,ggaout);
    // getGPFPD(gpfpdpara,gpfpdin,gpfpdout);
    // getGPYBM(gpybmpara,gpybmin,gpybmout);
    // printGGA(ggaout);
    // printGPFPD(gpfpdout);
    // printGPYBM(gpybmout);
    // startGPSAPP(apppara,appin,appout);
    calcCheckPara para1;
    calcCheckInput input1{"$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47"};
    calcCheckOutput output1;
    calcCheckSum(para1,input1,output1);
    cout<<hex<<output1.check<<endl;
    dmm2degPara para2;
    dmm2degInput input2{0.5};
    dmm2degOutput output2;
    dmm2deg(para2,input2,output2);
    cout<<output2.data<<endl;

}