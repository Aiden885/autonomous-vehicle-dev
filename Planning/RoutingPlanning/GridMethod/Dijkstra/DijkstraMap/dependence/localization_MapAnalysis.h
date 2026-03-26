#ifndef _LOCALIZATION_MAPANALYSIS_H
#define _LOCALIZATION_MAPANALYSIS_H


#include "rapidxml.hpp"
#include "rapidxml_utils.hpp"
#include<vector>
#include<string>


//车道后继车道所属路段的id和车道本身的id
struct LaneSuccessorId {
	int sucRoadID;
	int sucLaneID;
};


//路点的高斯坐标和航向角和曲率
struct GaussRoadPoint {
	double GaussX;
	double GaussY;
	double yaw;
	double curvature;
};


//车道类，包含车道id，后继id，左相邻车道id，右相邻车道id和路点信息
class Lane {
public:
	int id;
	std::vector<LaneSuccessorId>successorId;
	std::vector<int>leftLaneId;
	std::vector<int>rightLaneId;
	std::vector<GaussRoadPoint>gaussRoadPoints;
public:
	Lane() {}
	~Lane() {}
};



//路段类，每个路段有一个id，若干后继路段id，和若干车道
class Road {
public:
	int id;
	std::vector<Lane>lanes;
	std::vector<int>successorId;
public:
	Road() {}
	~Road() {}
};



//地图类，一个地图由多条路段组成
class Map {
public:
	std::vector<Road>roads;
public:
	Map(){}
	Map(std::string path);
	~Map(){}
	void mapAnalysis(std::string path);//路段解析，给出xodr文件位置，解析数据到roads
	void neighborLaneSort();//相邻车道id排序，越临近本车道id排的越前
	void moduleSelfCheck();//自我验证函数
	void moduleSelfCheckPrint();//自我打印函数
};


#endif