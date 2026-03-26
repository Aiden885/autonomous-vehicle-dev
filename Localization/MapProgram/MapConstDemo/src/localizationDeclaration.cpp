/**
 * @brief 地图分析功能
 * @file localizationDeclaration.cpp
 * @version 0.0.1
 * @author Wenliang Xu (xuwenliang2000@163.com)
 * @date 2023-11-22
 */
#include "localizationDeclaration.h"


namespace Localization{
/**
 * @brief 对路径id进行更新
 * @param[IN] lane Lane类
 * @param[IN] id int类型，路径的id
 * @param[OUT] output 空

 * @cn_name: 车道更新道路ID

 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）

 * @tag: localization
 */
void laneUpdateId(LaneStru& lane, IntStru& id, Empty output){
    lane.l.updateId(id.i);
    std::cout << "Lane成员函数updateId调用成功" << std::endl;
    std::cout << "更新后的ID为: " << lane.l.id << std::endl;
} 
/**
 * @brief 将原始经纬度路点进行漂移点删除、坐标转换、插值等过程得到处理后的高斯坐标路点
 * @param[IN] lane Lane类
 * @param[IN] in 空
 * @param[OUT] output 空

 * @cn_name: 车道计算高斯坐标

 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）

 * @tag: localization
 */
void laneGetGaussRoadPoint(LaneStru& lane, Empty in, Empty output){
    lane.l.getGaussRoadPoint();
    std::cout << "Lane成员函数getGaussRoadPoint调用成功" << std::endl;
};
/**
 * @brief 根据每个路点的高斯坐标计算每个路点的航向角yaw和曲率curvature
 * @param[IN] lane Lane类
 * @param[IN] in 空
 * @param[OUT] output 空

 * @cn_name: 车道计算航向曲率

 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）

 * @tag: localization
 */
void laneGetYawCruvature(LaneStru& lane, Empty in, Empty output){
    lane.l.getYawCruvature();
    std::cout << "Lane成员函数getYawCruvature调用成功" << std::endl;
}
/**
 * @brief 高斯坐标修正，包括加上x_min和y_min进行还原，以及考虑车载天线和车辆中心线的偏移
 * @param[IN] lane Lane类
 * @param[IN] in 空
 * @param[OUT] output 空

 * @cn_name: 车道高斯坐标修正

 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）

 * @tag: localization
 */
void laneCorectGaussXY(LaneStru& lane, Empty in, Empty output){
    lane.l.corectGaussXY();
    std::cout << "Lane成员函数corectGaussXY调用成功" << std::endl;
};
/**
 * @brief 根据曲率信息计算限速，单位m/s
 * @param[IN] lane Lane类
 * @param[IN] in 空
 * @param[OUT] output 空

 * @cn_name: 车道计算限速

 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）

 * @tag: localization
 */
void laneCalcSpeedProfile(LaneStru& lane, Empty in, Empty output){
    lane.l.calcSpeedProfile();
    std::cout << "Lane成员函数calcSpeedProfile调用成功" << std::endl;
};

/**
 * @brief 对路径id进行更新
 * @param[IN] road Road类
 * @param[IN] id int类型，路径的id
 * @param[OUT] output 空

 * @cn_name: 道路更新信息

 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）

 * @tag: localization
 */
void roadUpdateId(RoadStru& road, IntStru& id, Empty output){
    road.r.updateId(id.i);
    std::cout << "Road成员函数updateId调用成功" << std::endl;
    std::cout << "更新后的ID为: " << road.r.id << std::endl;
} 

/**
 * @brief 获取所有路点txt文件的文件名，path为所有txt文件所在的文件夹路径
 * @param[IN] map MapConstruction类
 * @param[IN] pa path为所有txt文件所在的文件夹路径
 * @param[OUT] output 空

 * @cn_name: 地图构造获取文件路径

 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）

 * @tag: localization
 */ 
void mapConstructGetAllFileNames(MapCon& map, Path& pa,Empty output){
    map.df.getAllFileNames(pa.path);
    std::cout << "MapConstruction成员函数getAllFileNames调用成功" << std::endl;
};
/**
 * @brief 加载所有原始经纬度路点数据并储存
 * @param[IN] in1 MapConstruction类
 * @param[IN] in2 path为所有txt文件所在的文件夹路径
 * @param[OUT] output 空

 * @cn_name: 地图构造加载路点数据

 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）

 * @tag: localization
 */ 
void mapConstructLoadRawRoadPoint(MapCon& in1,Path& in2,Empty output){
    in1.df.loadRawRoadPoint(in2.path);
    std::cout << "MapConstruction成员函数loadRawRoadPoint调用成功" << std::endl;
};
/**
 * @brief 对原始经纬度路点数据进行处理
 * @param[IN] in1 MapConstruction类
 * @param[IN] in2 空
 * @param[OUT] output 空

 * @cn_name: 地图构造处理路点数据

 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）

 * @tag: localization
 */ 
void mapConstructProcessRawRoadPoint(MapCon& in1,Empty in2,Empty output){
    in1.df.processRawRoadPoint();
    std::cout << "MapConstruction成员函数processRawRoadPoint调用成功" << std::endl;
};
/**
 * @brief 加载路段后继路段id的txt文件并储存内容
 * @param[IN] in1 MapConstruction类
 * @param[IN] in2 path为该文件的路径
 * @param[OUT] output 空

 * @cn_name: 地图构造加载后继路段

 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）

 * @tag: localization
 */ 
void mapConstructLoadSuccessor(MapCon& in1,Path& in2,Empty output){
	in1.df.loadSuccessor(in2.path);
    std::cout << "MapConstruction成员函数loadSuccessor调用成功" << std::endl;
};
/**
 * @brief 加载车道相邻车道的id的txt文件并储存内容
 * @param[IN] in1 MapConstruction类
 * @param[IN] in2 path为该文件的路径
 * @param[OUT] output 空

 * @cn_name: 地图构造加载相邻车道

 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）

 * @tag: localization
 */ 
void mapConstructLoadLaneNeighborId(MapCon& in1,Path& in2,Empty output){
    in1.df.loadLaneNeighborId(in2.path);
    std::cout << "MapConstruction成员函数loadLaneNeighborId调用成功" << std::endl;
};
/**
 * @brief 加载车道后继车道的id的txt文件并储存内容
 * @param[IN] in1 MapConstruction类
 * @param[IN] in2 path为该文件的路径
 * @param[OUT] output 空

 * @cn_name: 地图构造加载后继车道

 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）

 * @tag: localization
 */ 
void mapConstructLoadLaneSuccessorId(MapCon& in1,Path& in2,Empty output){
	in1.df.loadLaneSuccessorId(in2.path);
    std::cout << "MapConstruction成员函数loadLaneSuccessorId调用成功" << std::endl;
};
/**
 * @brief 加载不能换道的路段id的txt文件并储存内容
 * @param[IN] in1 MapConstruction类
 * @param[IN] in2 path为该文件的路径
 * @param[OUT] output 空

 * @cn_name: 地图构造加载不能换道的路段

 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）

 * @tag: localization
 */ 
void mapConstructLoadLaneChange(MapCon& in1,Path& in2,Empty output){
    in1.df.loadLaneChange(in2.path);
    std::cout << "MapConstruction成员函数loadLaneChange调用成功" << std::endl;
};
/**
 * @brief 将处理好的地图数据转化成xml语言能够储存的string类型的数据
 * @param[IN] in1 MapConstruction类
 * @param[IN] in2 空
 * @param[OUT] output 空

 * @cn_name: 地图构造转化数据类型

 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）

 * @tag: localization
 */ 
void mapConstructGetXMLInfo(MapCon& in1,Empty in2,Empty output){
    in1.df.getXMLInfo();
    std::cout << "MapConstruction成员函数getXMLInfo调用成功" << std::endl;
};
/**
 * @brief 将处理好的地图数据储存进xodr文件
 * @param[IN] in1 MapConstruction类
 * @param[IN] in2 path是输出的xodr文件的路径
 * @param[OUT] output 空

 * @cn_name: 地图构造储存数据

 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）

 * @tag: localization
 */ 
void mapConstructSaveMapToXML(MapCon in1,Path in2,Empty output){
	in1.df.saveMapToXML(in2.path);
    std::cout << "MapConstruction成员函数saveMapToXML调用成功" << std::endl;
};
/**
 * @brief 自我验证函数，检查本模块计算输出是否满足数据格式、范围要求
 * @param[IN] in1 MapConstruction类
 * @param[IN] in2 空
 * @param[OUT] output 空

 * @cn_name: 地图构造信息自检

 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）

 * @tag: localization
 */ 
void mapConstructModuleSelfCheck(MapCon& in1,Empty in2,Empty output){
    in1.df.moduleSelfCheck();
    std::cout << "MapConstruction成员函数moduleSelfCheck调用成功" << std::endl;
};
/**
 * @brief 输出打印显示函数，打印本模块的关键参数及输出
 * @param[IN] in1 MapConstruction类
 * @param[IN] in2 空
 * @param[OUT] output 空

 * @cn_name: 地图构造打印输出

 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）

 * @tag: localization
 */ 
void mapConstructModuleSelfCheckPrint(MapCon& in1,Empty in2,Empty output){
    in1.df.moduleSelfCheckPrint();
    std::cout << "MapConstruction成员函数moduleSelfCheckPrint调用成功" << std::endl;
};
}