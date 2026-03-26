#pragma once
#ifndef GLOBALPLANNING_M_H
#define GLOBALPLANNING_M_H

#include "globalPlanning.h"

/**
 * \namespace planning
 * \brief planning namespace
 */

namespace planning{

/**
 * @brief 全局规划函数生成相关声明
 * @file globalPlanning_m.h
 * @version 0.0.1
 * @author Zehang Zhu (zzh22@mails.tsinghua.edu.cn)
 * @date 2023-11-25
 */

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
    std::list<int> path;
    Astar as;
};

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

void AstarInitLink(const AstarInitLinkParam &param, const AstarInitLinkInput &input, AstarInitLinkOutput &output);

struct AstarResetParam{
	Astar as;
};

struct AstarResetInput{
};

struct AstarResetOutput{
	Astar as;
};

void AstarReset(const AstarResetParam &param, const AstarResetInput &input, AstarResetOutput &output);

struct AstarModuleSelfCheckParam{
	Astar as;
};

struct AstarModuleSelfCheckInput{
	std::list<int> path;
};

struct AstarModuleSelfCheckOutput{
};

void AstarModuleSelfCheck(const AstarModuleSelfCheckParam &param, const AstarModuleSelfCheckInput &input, AstarModuleSelfCheckOutput &output);

struct AstarModuleSelfCheckPrintParam{
	Astar as;
};

struct AstarModuleSelfCheckPrintInput{
	vector<pair<int, int>> pathLanes;
};

struct AstarModuleSelfCheckPrintOutput{
};

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

void AstarFindLane(const AstarFindLaneParam &param, const AstarFindLaneInput &input, AstarFindLaneOutput &output);

struct AstarFindLaneRParam{
	Astar as;
};

struct AstarFindLaneRInput{
    vector<Road> roads_;
    list<int> path_;
    vector<std::pair<int, int>> pathLanes_;
    int lane_id_;
};

struct AstarFindLaneROutput{
	Astar as;
};

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

void AstarRecursiveFindLane(const AstarRecursiveFindLaneParam &param, const AstarRecursiveFindLaneInput &input, AstarRecursiveFindLaneOutput &output);

}

#endif