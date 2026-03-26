/**
 * @brief GPS源文件
 * @file gpsFunctional.cpp
 * @version 0.0.1
 * @author Zihan Xie (770178863@qq.com)
 * @date 2023-11-27
 */
#include"gpsFunctional.h"
using namespace std;

namespace Localization{

int openQTWindow(int argc,char **argv)
{
    QApplication a(argc,argv);
    Configure BottomConfigure;//读配置文件
    MainWindow w;//开启画图+串口

    //    //开启zmq通信线程
    //ZmqCommunication *ZMQ = new ZmqCommunication();
    //ZMQ->start();

    w.show();
    return a.exec();
}


/**
 * @brief 打开GPS窗口
 * @param[IN] paraAPP 空、占位
 * @param[IN] inputAPP main输入参数
 * @param[OUT] outputAPP 空、占位
 
 * @cn_name: 打开GPS窗口
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: localization
 */
void startGPSAPP(const paraAPP &para,const inputAPP &input,outputAPP &output)
{
    int start = openQTWindow(input.argc,input.argv);
}
/**
 * @brief 解析GGA数据
 * @param[IN] paraGGA 空、占位
 * @param[IN] inputGGA 输入字符串
 * @param[OUT] outputGGA NMEAGGA对象
 
 * @cn_name: 解析GGA数据
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: localization
 */
void getGGA(const paraGGA &para,const inputGGA &input,outputGGA &output)
{
    output.data.Parse(input.data);
}

/**
 * @brief 解析GPFPD数据
 * @param[IN] paraGPFPD 空、占位
 * @param[IN] inputGPFPD 输入字符串
 * @param[OUT] outputGPFPD NMEAGPFPD对象
 
 * @cn_name: 解析GPFPD数据
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: localization
 */
void getGPFPD(const paraGPFPD &para,const inputGPFPD &input,outputGPFPD &output)
{
    output.data.parse(QString::fromStdString(input.data));
}

/**
 * @brief 解析GPYBM数据
 * @param[IN] paraGPYBM 空、占位
 * @param[IN] inputGPYBM 输入字符串
 * @param[OUT] outputGPYBM NMEAGPYBM对象
 
 * @cn_name: 解析GPYBM数据
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: localization
 */
void getGPYBM(const paraGPYBM &para,const inputGPYBM &input,outputGPYBM &output)
{
    output.data.Parse(input.data);
}
/**
 * @brief 计算NEMA数据校验位
 * @param[IN] calcCheckPara 空、占位
 * @param[IN] calcCheckInput 计算校验位字符串
 * @param[OUT] calcCheckOutput 校验位对应的16进制数
 
 * @cn_name: 计算校验位
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: localization
 */

void calcCheckSum(calcCheckPara &para,calcCheckInput &input,calcCheckOutput &output)
{
    NMEABase base1;
    output.check = base1.CalcCheckSum(input.data);
}
/**
 * @brief 将NMEA中 ddmm.mm 格式转换为小数度
 * @param[IN] dmm2degPara 空、占位
 * @param[IN] dmm2degInput ddmm.mm格式经纬度
 * @param[OUT] dmm2degOutput 小数度格式的经纬度
 
 * @cn_name: 将NMEA中 ddmm.mm 格式转换为小数度
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: localization
 */

void dmm2deg(dmm2degPara &para,dmm2degInput &input,dmm2degOutput &output)
{
    NMEABase base1;
    output.data = base1.dmm2deg(input.dmm);
}

}