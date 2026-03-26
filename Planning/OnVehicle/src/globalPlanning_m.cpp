#include "globalPlanning_m.h"

namespace planning{

/**
 * @brief 全局规划函数
 * @file globalPlanning_m.cpp
 * @version 0.0.1
 * @author Zehang Zhu (zzh22@mails.tsinghua.edu.cn)
 * @date 2023-11-25
 */

/**
 * @brief 使用Map类构建Astar类
 * @param[IN] param 地图，初始化的Astar
 * @param[IN] input 无
 * @param[OUT] output 更新的Astar
 
 * @cn_name: 使用地图构建Astar
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void AstarMapToAstar(const AstarMapToAstarParam &param, const AstarMapToAstarInput &input, AstarMapToAstarOutput &output)
{
    Astar as = param.as;
    RoadMap m = param.m;
    as.mapToAstar(m, &as);
    output.as = as;
}

/**
 * @brief 使用Astar算法获取路径，包括预处理、后处理等部分
 * @param[IN] param Astar
 * @param[IN] input 起点、终点、起点Id、终点Id
 * @param[OUT] output 更新的Astar，起点到终点的路径
 
 * @cn_name: Astar获取全局路径
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void AstarGetPath(const AstarGetPathParam &param, const AstarGetPathInput &input, AstarGetPathOutput &output)
{
    Astar as = param.as;
    int origin = origin;
    int destination = destination;
    int originPointId = originPointId;
    int destinationPointId = destinationPointId;
    std::list<int> path = as.getPath(origin, destination, originPointId, destinationPointId);
    output.as = as;
    output.path = path;
}

/**
 * @brief Astar道路初始化
 * @param[IN] param Astar
 * @param[IN] input 道路Id，起点XY，终点XY，长度
 * @param[OUT] output 更新的Astar
 
 * @cn_name: Astar添加道路
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void AstarInitRoad(const AstarInitRoadParam &param, const AstarInitRoadInput &input, AstarInitRoadOutput &output){
	Astar as = param.as;
	int number = input.number;
	double xStart = input.xStart;
	double yStart = input.yStart;
	double xEnd = input.xEnd;
	double yEnd = input.yEnd;
	double length = input.length;
	as.initRoad(number, xStart, yStart, xEnd, yEnd, length);
	output.as = as;
}

/**
 * @brief Astar道路连接关系初始化
 * @param[IN] param Astar
 * @param[IN] input 起始道路Id，到达道路Id
 * @param[OUT] output 更新的Astar
 
 * @cn_name: Astar添加道路连接关系
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void AstarInitLink(const AstarInitLinkParam &param, const AstarInitLinkInput &input, AstarInitLinkOutput &output){
	Astar as = param.as;
	int from = input.from;
	int to = input.to;
	as.initLink(from, to);
	output.as = as;
}

/**
 * @brief 在保留地图的情况下，初始化Astar参数
 * @param[IN] param Astar
 * @param[IN] input 无
 * @param[OUT] output 更新的Astar
 
 * @cn_name: Astar参数初始化
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void AstarReset(const AstarResetParam &param, const AstarResetInput &input, AstarResetOutput &output){
	Astar as = param.as;
	as.reset();
	output.as = as;
}

/**
 * @brief 检查Astar道路Id
 * @param[IN] param Astar
 * @param[IN] input 无
 * @param[OUT] output 无
 
 * @cn_name: Astar道路Id检查
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void AstarModuleSelfCheck(const AstarModuleSelfCheckParam &param, const AstarModuleSelfCheckInput &input, AstarModuleSelfCheckOutput &output){
	Astar as = param.as;
	std::list<int> path = input.path;
	as.moduleSelfCheck(path);
}

/**
 * @brief 检查Astar道路Id打印
 * @param[IN] param Astar
 * @param[IN] input 无
 * @param[OUT] output 无
 
 * @cn_name: Astar道路Id打印
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void AstarModuleSelfCheckPrint(const AstarModuleSelfCheckPrintParam &param, const AstarModuleSelfCheckPrintInput &input, AstarModuleSelfCheckPrintOutput &output){
	Astar as = param.as;
	vector<pair<int, int>> pathLanes = input.pathLanes;
	as.moduleSelfCheckPrint(pathLanes);
}

/**
 * @brief 根据Astar道路级全局规划结果，获得车道级全局规划结果
 * @param[IN] param Astar
 * @param[IN] input 地图、道路级全局规划结果
 * @param[OUT] output 更新的Astar
 
 * @cn_name: Astar获得全局规划车道结果
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void AstarFindLane(const AstarFindLaneParam &param, const AstarFindLaneInput &input, AstarFindLaneOutput &output){
	Astar as = param.as;
	RoadMap m = input.m;
	list<int> path = input.path;
	as.findLane(m, path);
	output.as = as;
}

/**
 * @brief 根据Astar道路级全局规划结果，使用递归的方法获得车道级全局规划结果
 * @param[IN] param Astar
 * @param[IN] input 道路向量数组、道路级全局规划结果、储存结果的向量数组、终点车道Id
 * @param[OUT] output 更新的Astar
 
 * @cn_name: Astar递归地获得全局规划车道结果
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void AstarFindLaneR(const AstarFindLaneRParam &param, const AstarFindLaneRInput &input, AstarFindLaneROutput &output){
    Astar as = param.as;
    vector<Road> roads_ = input.roads_;
    list<int> path_ = input.path_;
    vector<std::pair<int, int>> pathLanes_ = input.pathLanes_;
    int lane_id_ = input.lane_id_;
    as.findLane(roads_, path_, pathLanes_, lane_id_);
    output.as = as;    
}

/**
 * @brief Astar递归获得车道级全局规划结果主体函数
 * @param[IN] param Astar
 * @param[IN] input 道路、道路级全局规划结果、可递归车道、终点道路Id、终点车道Id
 * @param[OUT] output 更新的Astar
 
 * @cn_name: Astar递归获得车道级全局规划结果主体函数
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void AstarRecursiveFindLane(const AstarRecursiveFindLaneParam &param, const AstarRecursiveFindLaneInput &input, AstarRecursiveFindLaneOutput &output){
    Astar as = param.as;
    std::vector<Road> roads_ = input.roads_;
    std::list<int> path_ = input.path_;
    std::vector<std::tuple<int, int, int>> optional_lanes_ = input.optional_lanes_;
    int road_id_ = input.road_id_;
    int lane_id_ = input.lane_id_;
    as.recursiveFindLane(roads_, path_, optional_lanes_, road_id_, lane_id_);
    output.as = as;
}
}