#pragma once
#ifndef GLOBALPLANNING_M_H
#define GLOBALPLANNING_M_H

#include "localization_MapAnalysis.h"
#include "globalPlanning.h"
#include <iostream>
#include <list>
#include <queue>
#include <utility>
#include <vector>
#include <math.h>
#include <tuple>
using namespace std;

/**
 * \namespace GolbalPlanning
 * \brief 这是一个名为GolbalPlanning的namespace。
 */
// namespace GolbalPlanning{

// typedef struct
// {
//     int number;            // 编号
//     double xBegin, yBegin; // 起点和终点坐标
//     double xEnd, yEnd;
//     double length;
//     // std::vector<int> from;
//     std::vector<int> to;
//     int father;
//     int isInList = -2; // 代表边在以下的四个状态之一：
//     // 未初始化（-2），不在list中（0），在openList中（1），在closeList中（-1）
//     double F, G, H;
//     int numLanes; // 车道数量
// } road;

// class node
// {
// public:
//     int number;
//     double F;
//     int father;
//     node(int _number, double _F, int _father) : number(_number), F(_F), father(_father) {}
// };

// class cmp
// {
// public:
//     bool operator()(node a, node b) const
//     {
//         return a.F > b.F;
//     }
// };

// class Astar
// {
// public:
// 	Astar();
//     ~Astar();
// 	road *roadList;
// 	list<int> path;                   // 中间成果，road的id序列
//     vector<pair<int, int>> pathLanes; // 最后要输出的结果，road的id和lane的id
// 	int size = 10000;                                           // 边的编号上限，可以修改
//     std::priority_queue<node, std::vector<node>, cmp> openList; // priority_queue形式的openList可以提高效率
//     // 因为有isInList，不专门设置closeList了
// };


//modular function
struct AstarMapToAstarParam
{
    RoadMap m;
    Astar as;
};

struct AstarMapToAstarInput
{
};

struct AstarMapToAstarOutput
{
    Astar as;
};
// AstarMapToAstarParam param_map2astar{m, as};
// AstarMapToAstarInput input_map2astar{};
// AstarMapToAstarOutput output_map2astar{as};
void AstarMapToAstar(const AstarMapToAstarParam &param, const AstarMapToAstarInput &input, AstarMapToAstarOutput &output);

struct AstarGetPathParam
{
    Astar as;
};

struct AstarGetPathInput
{
    int origin;
    int destination;
    int originPointId;
    int destinationPointId;
};

struct AstarGetPathOutput
{
    std::list<int> &path;
    Astar &as;
};
// AstarGetPathParam param_get_path{as};
// AstarGetPathInput input_get_path{origin, destination, originPointId, destinationPointId};
// AstarGetPathOutput output_get_path{path, as};
void AstarGetPath(const AstarGetPathParam &param, const AstarGetPathInput &input, AstarGetPathOutput &output);

struct AstarInitRoadParam{
	Astar as;
};

struct AstarInitRoadInput{
	int number;
	double xStart;
	double yStart;
	double xEnd;
	double yEnd;
	double length;
};

struct AstarInitRoadOutput{
	Astar as;
};
// AstarInitRoadParam param_init_road{as};
// AstarInitRoadInput input_init_road{number, xStart, yStart, xEnd, yEnd, length};
// AstarInitRoadOutput output_init_road{as};
void AstarInitRoad(const AstarInitRoadParam &param, const AstarInitRoadInput &input, AstarInitRoadOutput &output);

struct AstarInitLinkParam{
	Astar as;
};

struct AstarInitLinkInput{
	int from;
	int to;
};

struct AstarInitLinkOutput{
	Astar as;
};
// AstarInitLinkParam param_init_link{as};
// AstarInitLinkInput input_init_link{from, to};
// AstarInitLinkOutput output_init_link{as};
void AstarInitLink(const AstarInitLinkParam &param, const AstarInitLinkInput &input, AstarInitLinkOutput &output);

struct AstarResetParam{
	Astar as;
};

struct AstarResetInput{
};

struct AstarResetOutput{
	Astar as;
};
// AstarResetParam param_reset{as};
// AstarResetInput input_reset{};
// AstarResetOutput output_reset{as};
void AstarReset(const AstarResetParam &param, const AstarResetInput &input, AstarResetOutput &output);

struct AstarModuleSelfCheckParam{
	Astar as;
};

struct AstarModuleSelfCheckInput{
	std::list<int> path;
};

struct AstarModuleSelfCheckOutput{
};
// AstarModuleSelfCheckParam param_module_selfcheck{as};
// AstarModuleSelfCheckInput input_module_selfcheck{path};
// AstarModuleSelfCheckOutput output_module_selfcheck{};
void AstarModuleSelfCheck(const AstarModuleSelfCheckParam &param, const AstarModuleSelfCheckInput &input, AstarModuleSelfCheckOutput &output);

struct AstarModuleSelfCheckPrintParam{
	Astar &as;
};

struct AstarModuleSelfCheckPrintInput{
	vector<pair<int, int>> &pathLanes;
};

struct AstarModuleSelfCheckPrintOutput{
};
// AstarModuleSelfCheckPrintParam param_module_selfcheck_print{as};
// AstarModuleSelfCheckPrintInput input_module_selfcheck_print{pathLanes};
// AstarModuleSelfCheckPrintOutput output_module_selfcheck_print{};
void AstarModuleSelfCheckPrint(const AstarModuleSelfCheckPrintParam &param, const AstarModuleSelfCheckPrintInput &input, AstarModuleSelfCheckPrintOutput &output);

struct AstarFindLaneParam{
	Astar as;
};

struct AstarFindLaneInput{
	RoadMap m;
	list<int> path;
};

struct AstarFindLaneOutput{
	Astar as;
};
// AstarFindLaneParam param_find_lane{as};
// AstarFindLaneInput input_find_lane{m, path};
// AstarFindLaneOutput output_find_lane{as};
void AstarFindLane(const AstarFindLaneParam &param, const AstarFindLaneInput &input, AstarFindLaneOutput &output);

struct AstarFindLaneRParam{
	Astar &as;
};

struct AstarFindLaneRInput{
    vector<Road> &roads_;
    list<int> &path_;
    vector<std::pair<int, int>> &pathLanes_;
    int &lane_id_;
};

struct AstarFindLaneROutput{
	Astar &as;
};
// AstarFindLaneRParam param_find_lane_r{as};
// AstarFindLaneRInput input_find_lane_r{roads, path, pathLanes, lane_id};
// AstarFindLaneROutput output_find_lane_r{as};
void AstarFindLaneR(const AstarFindLaneRParam &param, const AstarFindLaneRInput &input, AstarFindLaneROutput &output);

struct AstarRecursiveFindLaneParam{
	Astar as;
};

struct AstarRecursiveFindLaneInput{
    std::vector<Road> roads_;
    std::list<int> path_;
    std::vector<std::tuple<int, int, int>> optional_lanes_;
    int road_id_;
    int lane_id_;
};

struct AstarRecursiveFindLaneOutput{
	Astar as;
};
// AstarRecursiveFindLaneParam param_recursive_find_lane{as};
// AstarRecursiveFindLaneInput input_recursive_find_lane{roads, path, optional_lanes, road_id, lane_id};
// AstarRecursiveFindLaneOutput output_recursive_find_lane{as};
void AstarRecursiveFindLane(const AstarRecursiveFindLaneParam &param, const AstarRecursiveFindLaneInput &input, AstarRecursiveFindLaneOutput &output);



struct AstarFindPathParam{
	Astar as;
};

struct AstarFindPathInput{
    int origin;
	int destination;
};

struct AstarFindPathOutput{
	road *result;
};
// AstarFindPathParam param_recursive_find_lane{as};
// AstarFindPathInput input_recursive_find_lane{origin, destination};
// AstarFindPathOutput output_recursive_find_lane{result};
void AstarFindPath(const AstarFindPathParam &param, const AstarFindPathInput &input, AstarFindPathOutput &output);

struct AstarCalcGParam{
	Astar as;
};

struct AstarCalcGInput{
    int tmp;
	int from;
};

struct AstarCalcGOutput{
	double result;
};

void AstarCalcG(const AstarCalcGParam &param, const AstarCalcGInput &input, AstarCalcGOutput &output);

struct AstarCalcHParam{
	Astar as;
};

struct AstarCalcHInput{
    int tmp;
	int destination;
};

struct AstarCalcHOutput{
	double result;
};
void AstarCalcH(const AstarCalcHParam &param, const AstarCalcHInput &input, AstarCalcHOutput &output);

struct AstarCalcFParam{
	Astar as;
};

struct AstarCalcFInput{
    int tmp;
};

struct AstarCalcFOutput{
	double result;
};
void AstarCalcF(const AstarCalcFParam &param, const AstarCalcFInput &input, AstarCalcFOutput &output);

struct AstarGetLeastFParam{
	Astar as;
};

struct AstarGetLeastFInput{
};

struct AstarGetLeastFOutput{
	int result;
};
void AstarGetLeastF(const AstarGetLeastFParam &param, const AstarGetLeastFInput &input, AstarGetLeastFOutput &output);


struct AstarGetNextRoadParam{
	Astar as;
};

struct AstarGetNextRoadInput{
	int temp;
};

struct AstarGetNextRoadOutput{
	std::vector<int>  result;
};
void AstarGetNextRoad(const AstarGetNextRoadParam &param, const AstarGetNextRoadInput &input, AstarGetNextRoadOutput &output); // 返回F值最小的边的编号

// } // namespace GolbalPlanning

#endif