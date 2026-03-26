/**
 * @brief 地图分析功能
 * @file localizationDeclaration.h
 * @version 0.0.1
 * @author Wenliang Xu (xuwenliang2000@163.com)
 * @date 2023-11-22
 */
#include "rapidxml.hpp"
#include "rapidxml_print.hpp"
#include "localization_MapConstruction.h"
#include <iostream> 


/**
 * \namespace Localization
 * \brief Localization namespace
 */

namespace Localization{

struct MapCon {
	MapConstruction df;
};
struct Path {
	std::string path;
};
struct IntStru{
	int i;
};
struct LaneStru{
	Lane l;
};
struct RoadStru{
	Road r;
};

struct Empty {

};
/**
 * @brief 对路径id进行更新
 * @param[IN] lane Lane类
 * @param[IN] id int类型，路径的id
 * @param[OUT] output 空
 * @cn_name: 车道更新道路ID
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 * @tag: localization
 */
void laneUpdateId(LaneStru& lane, IntStru& id, Empty output);
/**
 * @brief 将原始经纬度路点进行漂移点删除、坐标转换、插值等过程得到处理后的高斯坐标路点
 * @param[IN] lane Lane类
 * @param[IN] in 空
 * @param[OUT] output 空
 * @cn_name: 车道计算高斯坐标
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 * @tag: localization
 */
void laneGetGaussRoadPoint(LaneStru& lane, Empty in, Empty output);
/**
 * @brief 根据每个路点的高斯坐标计算每个路点的航向角yaw和曲率curvature
 * @param[IN] lane Lane类
 * @param[IN] in 空
 * @param[OUT] output 空
 * @cn_name: 车道计算航向曲率
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 * @tag: localization
 */
void laneGetYawCruvature(LaneStru& lane, Empty in, Empty output);
/**
 * @brief 高斯坐标修正，包括加上x_min和y_min进行还原，以及考虑车载天线和车辆中心线的偏移
 * @param[IN] lane Lane类
 * @param[IN] in 空
 * @param[OUT] output 空
 * @cn_name: 车道高斯坐标修正
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 * @tag: localization
 */
void laneCorectGaussXY(LaneStru& lane, Empty in, Empty output);
/**
 * @brief 根据曲率信息计算限速，单位m/s
 * @param[IN] lane Lane类
 * @param[IN] in 空
 * @param[OUT] output 空
 * @cn_name: 车道计算限速
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 * @tag: localization
 */
void laneCalcSpeedProfile(LaneStru& lane, Empty in, Empty output);
/**
 * @brief 对路径id进行更新
 * @param[IN] road Road类
 * @param[IN] id int类型，路径的id
 * @param[OUT] output 空
 * @cn_name: 道路更新信息
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 * @tag: localization
 */
void roadUpdateId(RoadStru& road, IntStru& id, Empty output);
/**
 * @brief 获取所有路点txt文件的文件名，path为所有txt文件所在的文件夹路径
 * @param[IN] map MapConstruction类
 * @param[IN] pa path为所有txt文件所在的文件夹路径
 * @param[OUT] output 空
 * @cn_name: 地图构造获取文件路径
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 * @tag: localization
 */ 
void mapconstGetAllFileNames(MapCon& map, Path& pa,Empty output);
/**
 * @brief 加载所有原始经纬度路点数据并储存
 * @param[IN] in1 MapConstruction类
 * @param[IN] in2 path为所有txt文件所在的文件夹路径
 * @param[OUT] output 空
 * @cn_name: 地图构造加载路点数据
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 * @tag: localization
 */ 
void mapconstLoadRawRoadPoint(MapCon& in1,Path& in2,Empty output);
/**
 * @brief 对原始经纬度路点数据进行处理
 * @param[IN] in1 MapConstruction类
 * @param[IN] in2 空
 * @param[OUT] output 空
 * @cn_name: 地图构造处理路点数据
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 * @tag: localization
 */ 
void mapconstProcessRawRoadPoint(MapCon& in1,Empty in2,Empty output);
/**
 * @brief 加载路段后继路段id的txt文件并储存内容
 * @param[IN] in1 MapConstruction类
 * @param[IN] in2 path为该文件的路径
 * @param[OUT] output 空
 * @cn_name: 地图构造加载后继路段
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 * @tag: localization
 */ 
void mapconstLoadSuccessor(MapCon& in1,Path& in2,Empty output);
/**
 * @brief 加载车道相邻车道的id的txt文件并储存内容
 * @param[IN] in1 MapConstruction类
 * @param[IN] in2 path为该文件的路径
 * @param[OUT] output 空
 * @cn_name: 地图构造加载相邻车道
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 * @tag: localization
 */ 
void mapconstLoadLaneNeighborId(MapCon& in1,Path& in2,Empty output);
/**
 * @brief 加载车道后继车道的id的txt文件并储存内容
 * @param[IN] in1 MapConstruction类
 * @param[IN] in2 path为该文件的路径
 * @param[OUT] output 空
 * @cn_name: 地图构造加载后继车道
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 * @tag: localization
 */ 
void mapconstLoadLaneSuccessorId(MapCon& in1,Path& in2,Empty output);
/**
 * @brief 加载不能换道的路段id的txt文件并储存内容
 * @param[IN] in1 MapConstruction类
 * @param[IN] in2 path为该文件的路径
 * @param[OUT] output 空
 * @cn_name: 地图构造加载不能换道的路段
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 * @tag: localization
 */ 
void mapconstLoadLaneChange(MapCon& in1,Path& in2,Empty output);
/**
 * @brief 将处理好的地图数据转化成xml语言能够储存的string类型的数据
 * @param[IN] in1 MapConstruction类
 * @param[IN] in2 空
 * @param[OUT] output 空
 * @cn_name: 地图构造转化数据类型
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 * @tag: localization
 */ 
void mapconstGetXMLInfo(MapCon& in1,Empty in2,Empty output);
/**
 * @brief 将处理好的地图数据储存进xodr文件
 * @param[IN] in1 MapConstruction类
 * @param[IN] in2 path是输出的xodr文件的路径
 * @param[OUT] output 空
 * @cn_name: 地图构造储存数据
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 * @tag: localization
 */ 
void mapconstSaveMapToXML(MapCon in1,Path in2,Empty output);
/**
 * @brief 自我验证函数，检查本模块计算输出是否满足数据格式、范围要求
 * @param[IN] in1 MapConstruction类
 * @param[IN] in2 空
 * @param[OUT] output 空
 * @cn_name: 地图构造信息自检
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 * @tag: localization
 */ 
void mapconstModuleSelfCheck(MapCon& in1,Empty in2,Empty output);
/**
 * @brief 输出打印显示函数，打印本模块的关键参数及输出
 * @param[IN] in1 MapConstruction类
 * @param[IN] in2 空
 * @param[OUT] output 空
 * @cn_name: 地图构造打印输出
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 * @tag: localization
 */ 
void mapconstModuleSelfCheckPrint(MapCon& in1,Empty in2,Empty output);
}