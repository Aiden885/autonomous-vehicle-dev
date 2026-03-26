/**
 * @brief 地图分析功能
 * @file mapAnalysisAdd.cpp
 * @version 0.0.1
 * @author Wenliang Xu (xuwenliang2000@163.com)
 * @date 2023-11-22
 */
#include"../src/mapAnalysisAdd.h"
#include <iostream>


namespace Localization{

/**
 * @brief 获取路网地图
 * @param[IN] in1 std::string字符串，是xodr文件位置
 * @param[IN] in2 空
 * @param[OUT] output 空
 
 * @cn_name: 获取路网地图
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: localization
 */
void roadMapGet(Path in1, Empty in2, Empty output){
    RoadMap m(in1.path);
};

/**
 * @brief 路段解析，数据解析到Map类的roads
 * @param[IN] rm RoadMap类
 * @param[IN] p std::string字符串，是xodr文件位置
 * @param[OUT] output 空
 
 * @cn_name: 解析地图路段
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: localization
 */
void roadmapMapAnalysis(RoadM& rm, Path& p, Empty& output){
    rm.m.mapAnalysis(p.path);
    std::cout << "RoadMap成员函数mapAnalysis调用成功" << std::endl;
};
/**
 * @brief neighborLaneSort成员函数用来给相邻车道id排序,离本车道越近在数组中排的越前
 * @param[IN] rm RoadMap类
 * @param[IN] in2 空
 * @param[OUT] output 空
 
 * @cn_name: 地图车道排序
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: localization
 */
void roadmapNeighborLaneSort(RoadM& rm, Empty& in2, Empty& output){
    rm.m.neighborLaneSort();
    std::cout << "RoadMap成员函数neighborLaneSort调用成功" << std::endl;
};
/**
 * @brief 自我验证函数，检查本模块计算输出是否满足数据格式、范围要求
 * @param[IN] rm RoadMap类
 * @param[IN] in2 空
 * @param[OUT] output 空
 
 * @cn_name: 地图读取自检
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: localization
 */
void roadmapModuleSelfCheck(RoadM& rm, Empty& in2, Empty& output){
    rm.m.moduleSelfCheck();
    std::cout << "RoadMap成员函数moduleSelfCheck调用成功" << std::endl;
};
/**
 * @brief 输出打印显示函数，打印本模块的关键参数及输出
 * @param[IN] rm RoadMap类
 * @param[IN] in2 空
 * @param[OUT] output 空
 
 * @cn_name: 地图打印输出
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: localization
 */
void roadmapModuleSelfCheckPrint(RoadM& rm, Empty& in2, Empty& output){
    rm.m.moduleSelfCheckPrint();
    std::cout << "RoadMap成员函数moduleSelfCheckPrint调用成功" << std::endl;
};
MapAnlDemo
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: localization
 */
void osmformatFormatConversion(OSMFormatStru& of, Path& p, Empty& output){
    of.osmf.formatConversion(p.path);
    std::cout << "OSMFormat成员函数formatConversion调用成功" << std::endl;
};
/**
 * @brief 将转换得来的osm文件内容储存进osm文件
 * @param[IN] of OSMFormat类
 * @param[IN] p std::string字符串，osm文件位置roadMap.osm
 * @param[OUT] output 空
 
 * @cn_name: 储存osm文件
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: localization
 */
void osmformatSaveMapToOSM(OSMFormatStru& of, Path& p, Empty& output){
    of.osmf.saveMapToOSM(p.path);
    std::cout << "OSMFormat成员函数saveMapToOSM调用成功" << std::endl;
};
}