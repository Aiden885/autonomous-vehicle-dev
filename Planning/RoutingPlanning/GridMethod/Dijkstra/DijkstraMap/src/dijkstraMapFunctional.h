/**
 * @brief DijkstraMap功能头文件
 * @file dijkstraMapFunctional.h
 * @version 0.0.1
 * @author ZihanXie (770178863@qq.com)
 * @date 2023-12-19
 */
#include "dijkstra_topologyMap.h"
#include "localization_MapAnalysis.h"

/**
 * \namespace planning
 * \brief planning namespace
 */

namespace planning{

struct rmPara{};

struct rmInput{string filename;};

struct rmOutput{Map map;};

struct  dijPara{};

struct dijInput
{
	int originRoad;
	int originLane;
	int destinationRoad;
	int destinationLane;
    Map map;
};

struct dijOutput
{
    vector<pair<int,int>> path;
};

struct mapAnalysisPara{};

struct mapAnalysisInput{std::string path;};

struct mapAnalysisOutput{Map m;};

struct mapPrintPara{};

struct mapPrintInput{Map m;};

struct mapPrintOutput{};

struct neighborLaneSortPara{};

struct neighborLaneSortInput{};

struct neighborLaneSortOutput{Map m;};

struct moduleSelfCheckParaMap{};

struct moduleSelfCheckInputMap{Map m;};

struct moduleSelfCheckOutputMap{};

struct initRoadPara{};

struct initRoadInput
{
	int number;
	double xStart;
	double yStart;
	double xEnd;
	double yEnd;
	double length;
};

struct initRoadOutput{Astar &A;};


struct initLinkPara{};

struct initLinkInput
{
	int from;
	int to;
};

struct initLinkOutput{Astar &A;};

struct resetPara{};

struct resetInput{};

struct resetOutput{Astar &A;};

struct moduleSelfCheckPara{};

struct moduleSelfCheckInput
{
	Astar &A;
	list<int> path;
};

struct moduleSelfCheckOutput{};

struct moduleSelfCheckPrintPara{};

struct moduleSelfCheckPrintInput
{
	Astar &A;
	vector <pair<int, int>> pathLanes;
};

struct moduleSelfCheckPrintOutput{};

struct mapToAstarPara{};

struct mapToAstarInput{Map m;};

struct mapToAstarOutput{Astar A;};

struct findLanePara{};

struct findLaneInput
{
	Map m;
	list<int> path;
	int originRoad;
	int originLane;
	int destinationRoad;
	int destinationLane;
};

struct findLaneOutput{	Astar &A;};

struct getPathPara{};

struct getPathInput
{
	Astar &A;
	int origin;
	int destination;
};

struct getPathOutput{list<int> path;};

void DijkstraReadMap(const rmPara &para,const rmInput &input,rmOutput &output);

void DijkstraGetPathDijMap(const dijPara &para,const dijInput &input,dijOutput &output);

void DijkstraMapAnalysis(mapAnalysisPara &para,mapAnalysisInput &input,mapAnalysisOutput &output);

void DijkstraNeighborLaneSort(neighborLaneSortPara &para,neighborLaneSortInput &input,neighborLaneSortOutput &output);

void DijkstraModuleSelfCheckMap(moduleSelfCheckParaMap &para,moduleSelfCheckInputMap &input,moduleSelfCheckOutputMap &output);

void DijkstraMapPrint(mapPrintPara &para,mapPrintInput &input,mapPrintOutput &output);

void DijkstraInitRoad(initRoadPara &para,initRoadInput &input,initRoadOutput &output);

void DijkstraInitLink(initLinkPara &para,initLinkInput &input,initLinkOutput &output);

void DijkstraReset(resetPara &para,resetInput &input,resetOutput &output);

void DijkstraModuleSelfCheck(moduleSelfCheckPara &para,moduleSelfCheckInput &input,moduleSelfCheckOutput &output);

void DijkstraModuleSelfCheckPrint(moduleSelfCheckPrintPara &para,moduleSelfCheckPrintInput &input,moduleSelfCheckPrintOutput &output);

void DijkstraMapToAstar(mapToAstarPara &para,mapToAstarInput &input,mapToAstarOutput &output);

void DijkstraFindLane(findLanePara &para,findLaneInput &input,findLaneOutput &output);

void DijkstraGetPath(getPathPara &para,getPathInput &input,getPathOutput &output);

}