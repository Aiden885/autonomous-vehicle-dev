/**
 * @file globalPlanning_m.cpp
 * @author 
 * @brief globalPlanning
 * @version 1.0
 * @date 2024-12-04 
 * 
 * @copyright Copyright (c) 2022
 *
 */
#include "globalPlanning_m.h"

// namespace GolbalPlanning
// {

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
	// AstarMapToAstarParam param_map_to_astar{m, as};
	// AstarMapToAstarInput input_map_to_astar{};
	// AstarMapToAstarOutput output_map_to_astar{as};
	// AstarMapToAstar(param_map_to_astar, input_map_to_astar, output_map_to_astar);
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
	// std::cout<<" m-"<<std::endl;
    Astar as = param.as;
    int origin = input.origin;
    int destination = input.destination;
    int originPointId = input.originPointId;
    int destinationPointId = input.destinationPointId;
	std::cout<<"origin "<<origin<<" destination "<<destination<<" originPointId "<<originPointId<<" destinationPointId "<<destinationPointId<<std::endl;
    std::list<int> path = as.getPath(origin, destination, originPointId, destinationPointId);
	
	as.path = path;
    output.as = as;
	output.as.path = path;
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
	// cout << " AstarFindLaneR :"<< endl;
	// for(auto path_ : path_){
	// 	std::cout<<path_<<" ";
	// 		}
	// std::cout<<std::endl;
	// for (auto it = pathLanes_.begin(); it != pathLanes_.end(); it++)
    //     {
    //         cout << " AstarFindLaneR : " << (*it).first << "  " << (*it).second << endl;
    //     }
	// std::cout<<	" lane_id_ "<<lane_id_<<std::endl;
	as.path=path_;
    as.pathLanes=pathLanes_;
    as.findLane(roads_, as.path, as.pathLanes, lane_id_);
	// std::cout<<"AstarFindLaneR.pathLanes.size() "<<as.pathLanes.size()<<std::endl;


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

// } // namespace name