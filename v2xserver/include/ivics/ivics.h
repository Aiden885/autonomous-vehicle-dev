#ifndef ___IVICS_H__
#define ___IVICS_H__

#include <string>
#include <vector>
#include <functional>
#include <memory>
#include "bsm.h"
#include "rsi.h"
#include "spat.h"
#include "ssm.h"
#include "vir.h"

namespace Ivics
{
// 点的定义
struct Vector3D
{
    double x;
    double y;
    double z;
    Vector3D() :x(0), y(0), z(0) {};
    Vector3D(double x, double y) :x(x), y(y), z(0) {}
    Vector3D(double x, double y, double z) :x(x), y(y), z(z) {}
    bool operator==(const Vector3D& p)const;
};

// 个人安全信息。弱势交通参与者发布的消息，用于广播有关各种易受伤害道路使用者（VRU）运动状态的安全数据，如行人、骑自行车者或道路工人。暂不支持
// struct PSM
// {

// };


/**
 * @brief 手动输入mac地址，连接到中心网关, 并完成鉴权.
 *
 * @return 0： 完成连接及鉴权。 
 * @return -1：连接失败。
 * @return -2：鉴权失败。
 * @return -3：获取配置失败。
 * @return -4：授权失败。
 * @return -5：返回信息解析失败。
 */
int login();

/**
 * @brief 关闭连接
 *
 */
void close();

/**
 * @brief 设置VIR。目前主要用来超视距感知。设置以后长期有效，为了避免浪费资源，不需要超视距感知时额外调用一次该函数，将path设置为空数组。
 *
 * @param pointList
 */
void setVIR(const std::shared_ptr<VIR> pVir);

/**
 * @brief 设置VIR。目前主要用来超视距感知。设置以后长期有效，为了避免浪费资源，不需要超视距感知时额外调用一次该函数，将path设置为空数组。
 *
 * @param path 车辆规划行驶路线。
 * 路线上的感知结果都会发给车辆，所以path不宜过长，path上两点间距离不宜过小（程序会自动抽稀）。
 * 目前最多获取（暂定4个）Group的数据
 */
void setVIR(const std::vector<Vector3D>& path);

/**
 * @brief 设置接收数据的频率。暂不支持
 *
 * @param frequency 频率，范围[1,10]
 */
void setFrequency(int frequency);

/**
 * @brief 发送bsm数据。测试
 *
 * @param pDagta
 */
void sendBSM(const std::shared_ptr<BSM> pData);

/**
 * @brief 发送Allride标准的bsm数据
 *
 * @param p 类型为 allride::proto::planning::PlanningState。不会 delete 指针，会拷贝指针内容。
 */
void sendABSM(void* p);

/**
 * @brief 发送Allride标准的bsm数据
 *
 * @param x
 * @param y
 * @param z
 */
void sendABSM(double x, double y, double z);

// /**
//  * @brief 发送planningState数据
//  *
//  * @param pDagta
//  */
// void sendBsm(const std::shared_ptr<CarStatus> pDagta);

/**
 * @brief 发送Allride标准的path数据
 *
 * @param p类型为 allride::proto::config::Path。 不会 delete 指针，会拷贝指针内容。
 */
void sendAPath(void* p);

/**
 * @brief 通过http发送Allride标准的Routes数据
 *
 * @param pDagta allride::proto::map::HDMapRoutes。 不会 delete 指针，会拷贝指针内容。
 */
void sendARoutes(void* pData);

/**
 * @brief 设置车辆行驶路径
 *
 * @param points 规划行驶路线上的点。
 */
void sendRoutes(std::vector<Vector3D>& points);

/**
 * @brief 订阅感知数据 行标
 *
 * @param fp 回调函数
 */
void subscribeSSM(std::function<void(const SSM&)>);

/**
 * @brief 订阅感知数据 allride标准
 *
 * @param fp 回调函数 参数 类型为allride::proto::perception::DetectedObjectsDetectedObject 注意指针由调用着负责delete
 */
void subscribeASSM(std::function<void(void*)>);

/**
 * @brief 订阅超视距感知数据 行标
 *
 * @param fp 回调函数
 * TODO 实现
 */
void subscribeRemoteSSM(std::function<void(const SSM&)>);

/**
 * @brief 订阅超视距感知数据
 *
 * @param fp 回调函数 参数 类型为allride::proto::perception::DetectedObjectsDetectedObject 注意指针由调用着负责delete
 * TODO 实现
 */
void subscribeRemoteASSM(std::function<void(void*)> fp);

/**
 * @brief 停止订阅感知数据
 *
 */
void unsubscribeSSM();

/**
 * @brief 订阅事件，例如路侧突发情况。行标
 *
 * @param fp 参数类型为allride::proto::rsi::MSG_RSI
 */
void subscribeARSI(std::function<void(void*)>);

/**
 * @brief 订阅事件，例如路侧突发情况。暂不支持
 *
 * @param fp
 */
void subscribeRSI(std::function<void(const RSI&)>);

/**
 * @brief 取消订阅事件数据
 *
 */
void unsubscribeRSI();

/**
 * @brief 订阅信号灯消息（Signal Phase and Timing Message）。暂不支持
 *
 * @param fp
 */
void subscribeSPAT(std::function<void(const SPAT&)>);

/**
 * @brief 订阅信号灯消息（Signal Phase and Timing Message）。暂不支持
 *
 * @param fp 参数为 allride::proto::perception::SignalResponse
 */
void subscribeASPAT(std::function<void(void*)>);

/**
 * @brief 取消订阅 ASPAT。内用
 *
 */
void unsubscribeASPAT();

/**
 * @brief 取消订阅 SPAT。外用
 *
 */
void unsubscribeSPAT();

/**
 * @brief 查询 SDK version
 *
 */
std::string version();

/**
 * @brief create crash demo
 *
 */
void crash();

/**
 * @brief config dumpfile path when program crash
 *
 * @param dumpFilePath dump file path, e.g. /tmp/dump/, the folder must exist
 */
void configCrashDump(std::string dumpFilePath);

/**
 * @brief config SDK log level and log file path
 *
 * @param level one of trace, debug, info, warn, error
 * @param path the file path & file name, e.g. ./logs/sdk.log.
 * @param maxFiles the max log file count, maxFiles==0 will cause log to stdout
 * @param maxSize the max size in bytes of each log file
 */
void configLog(std::string level, std::string path, int maxSize, int maxFiles);

void configInputAndOutput(std::string path, int maxFiles);
}

#endif