/**
 * @brief DijkstraMap功能源文件
 * @file dijkstraMapFunctional.cpp
 * @version 0.0.1
 * @author Zihan Xie (770178863@qq.com)
 * @date 2023-12-19
 */
#include "dijkstraMapFunctional.h"

namespace planning{

/**
 * @brief Dijkstra读取路网图
 * @param[IN] rmPara 无效占位
 * @param[IN] rmInput 文件路径
 * @param[IN] rmOutput Map对象
 
 * @cn_name: Dijkstra读取路网图
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void DijkstraReadMap(const rmPara &para,const rmInput &input,rmOutput &output)
{
    Map m(input.filename);
    output.map.roads = m.roads;
}

/**
 * @brief Dijkstra寻找最短路径
 * @param[IN] dijPara 无效占位
 * @param[IN] dijInput 起始路段、车道；目标路段、车道
 * @param[IN] dijOutput 路径
 
 * @cn_name: Dijkstra寻找最短路径
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void DijkstraGetPathDijMap(const dijPara &para,const dijInput &input,dijOutput &output)
{
    vector<pair<int,int>>().swap(output.path);
    Astar as;
	as.path = as.getPath(input.originRoad, input.destinationRoad);//这一步确定roads
	as.pathLanes.push_back(make_pair(input.originRoad, input.originLane));//初始化
	as.findLane(input.map,as.path);
	if (as.pathLanes.back().second != input.destinationLane)as.pathLanes.push_back(make_pair(input.destinationRoad, input.destinationLane));//保证终点的lane
    for(auto p:as.pathLanes)
        output.path.push_back(p);
}
/**
 * @brief Dijkstra解析路网图获取数据
 * @param[IN] mapAnalysisPara 无效占位
 * @param[IN] mapAnalysisInput 文件路径
 * @param[IN] mapAnalysisOutput Map对象
 
 * @cn_name: Dijkstra解析路网图获取数据
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void DijkstraMapAnalysis(mapAnalysisPara &para,mapAnalysisInput &input,mapAnalysisOutput &output)
{
    output.m.mapAnalysis(input.path);
}
/**
 * @brief Dijkstra打印路网图信息
 * @param[IN] mapPrintPara 无效占位
 * @param[IN] mapPrintInput Map对象
 * @param[IN] mapPrintOutput 无效占位
 
 * @cn_name: Dijkstra打印路网图信息
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void DijkstraMapPrint(mapPrintPara &para,mapPrintInput &input,mapPrintOutput &output)
{
    input.m.moduleSelfCheckPrint();
}
/**
 * @brief Dijkstra相邻车道按序号排序
 * @param[IN] neighborLaneSortPara 无效占位
 * @param[IN] neighborLaneSortInput Map对象
 * @param[IN] neighborLaneSortOutput 无效占位
 
 * @cn_name: Dijkstra相邻车道按序号排序
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void DijkstraNeighborLaneSort(neighborLaneSortPara &para,neighborLaneSortInput &input,neighborLaneSortOutput &output)
{
    output.m.neighborLaneSort();
}
/**
 * @brief Dijkstra路网图格式检查
 * @param[IN] moduleSelfCheckParaMap 无效占位
 * @param[IN] moduleSelfCheckInputMap Map对象
 * @param[IN] moduleSelfCheckOutputMap 无效占位
 
 * @cn_name: Dijkstra路网图格式检查
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void DijkstraModuleSelfCheckMap(moduleSelfCheckParaMap &para,moduleSelfCheckInputMap &input,moduleSelfCheckOutputMap &output)
{
    input.m.moduleSelfCheck();
}
/**
 * @brief Dijkstra初始化道路
 * @param[IN] initRoadPara 无效占位
 * @param[IN] initRoadInput 编号、起始坐标、结束坐标、长度
 * @param[IN] initRoadOutput Astar对象
 
 * @cn_name: Dijkstra初始化道路
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void DijkstraInitRoad(initRoadPara &para,initRoadInput &input,initRoadOutput &output)
{
    output.A.initRoad(input.number,input.xStart,input.yStart,input.xEnd,input.yEnd,input.length);
}
/**
 * @brief Dijkstra初始化相连道路
 * @param[IN] initLinkPara 无效占位
 * @param[IN] initLinkInput 起始编号、结束编号
 * @param[IN] initLinkOutput Astar对象
 
 * @cn_name: Dijkstra初始化相连道路
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void DijkstraInitLink(initLinkPara &para,initLinkInput &input,initLinkOutput &output)
{
    output.A.initLink(input.from,input.to);
}
/**
 * @brief 重置Dijkstra模型
 * @param[IN] resetPara 无效占位
 * @param[IN] resetInput 无效占位
 * @param[IN] resetOutput Astar对象
 
 * @cn_name: 重置Dijkstra模型
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void DijkstraReset(resetPara &para,resetInput &input,resetOutput &output)
{
    output.A.reset();
}
/**
 * @brief Dijkstra模型自测
 * @param[IN] moduleSelfCheckPara 无效占位
 * @param[IN] moduleSelfCheckInput Astar对象、路径
 * @param[IN] moduleSelfCheckOutput 无效占位
 
 * @cn_name: Dijkstra模型自测
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void DijkstraModuleSelfCheck(moduleSelfCheckPara &para,moduleSelfCheckInput &input,moduleSelfCheckOutput &output)
{
    input.A.moduleSelfCheck(input.path);
}
/**
 * @brief Dijkstra模型自测并打印
 * @param[IN] moduleSelfCheckPrintPara 无效占位
 * @param[IN] moduleSelfCheckPrintInput Astar对象、车道路径
 * @param[IN] moduleSelfCheckPrintOutput 无效占位
 
 * @cn_name: Dijkstra模型自测并打印
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void DijkstraModuleSelfCheckPrint(moduleSelfCheckPrintPara &para,moduleSelfCheckPrintInput &input,moduleSelfCheckPrintOutput &output)
{
    input.A.moduleSelfCheckPrint(input.pathLanes);
}
/**
 * @brief 由路网图构建Dijstra模型
 * @param[IN] mapToAstarPara 无效占位
 * @param[IN] mapToAstarInput Map对象
 * @param[IN] mapToAstarOutput Astar对象
 
 * @cn_name: 由路网图构建Dijstra模型
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void DijkstraMapToAstar(mapToAstarPara &para,mapToAstarInput &input,mapToAstarOutput &output)
{
    output.A.mapToAstar(input.m,&(output.A));
}
/**
 * @brief Dijkstra寻找车道路线
 * @param[IN] findLanePara 无效占位
 * @param[IN] findLaneInput Map对象、路径、起始路段、车道、目标路段、车道
 * @param[IN] findLaneOutput Astar对象
 
 * @cn_name: Dijkstra寻找车道路线
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void DijkstraFindLane(findLanePara &para,findLaneInput &input,findLaneOutput &output)
{
    output.A.pathLanes.push_back(make_pair(input.originRoad, input.originLane));
    output.A.findLane(input.m,input.path);
	if (output.A.pathLanes.back().second != input.destinationLane)output.A.pathLanes.push_back(make_pair(input.destinationRoad, input.destinationLane));

}
/**
 * @brief Dijkstra寻找最短路径
 * @param[IN] getPathPara 无效占位
 * @param[IN] getPathInput Astar对象、起始路段、目标路段三
 * @param[IN] getPathOutput 路径
 
 * @cn_name: Dijkstra寻找最短路径
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void DijkstraGetPath(getPathPara &para,getPathInput &input,getPathOutput &output)
{
    output.path = input.A.getPath(input.origin,input.destination);
}

}