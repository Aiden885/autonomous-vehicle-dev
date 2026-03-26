#ifndef _LOCALIZATION_MAPANALYSIS_H
#define _LOCALIZATION_MAPANALYSIS_H


#include "rapidxml.hpp"
#include "rapidxml_utils.hpp"
#include "rapidxml_print.hpp"
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
	double s;
	double speedMax;
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
	bool isLaneChange;
	std::vector<Lane>lanes;
	std::vector<int>successorId;
public:
	Road() {}
	~Road() {}
};



//地图类，一个地图由多条路段组成
class RoadMap {
public:
	std::vector<Road>roads;
public:
	RoadMap(){}
	RoadMap(std::string path);
	~RoadMap(){}
	void mapAnalysis(std::string path);//路段解析，给出xodr文件位置，解析数据到roads
	void neighborLaneSort();//相邻车道id排序，越临近本车道id排的越前
	void moduleSelfCheck();//自我验证函数
	void moduleSelfCheckPrint();//自我打印函数
};



//以下四个类OSMNode、OSMWay、OSMRelation、OSMFormat用于将xodr文件转化成osm文件
//OSM文件中的node节点
class OSMNode {
public:
	int id;
	double localX, localY;
	double lon, lat;
	double yaw, curvature;
	std::string idString;
	std::string localXString, localYString;
	std::string lonString, latString;
	std::string yawString, curvatureString;
};

//OSM文件中的way节点
class OSMWay {
public:
	int id;
	std::string idString;
	std::vector<std::string>nodeSet;
};

//OSM文件中的relation节点
class OSMRelation {
public:
	int id;
	std::string idString;
	std::vector<std::string>waySet;
};

//构建OSM文件
class OSMFormat {
public:
	std::vector<OSMNode>allNodes;
	std::vector<OSMWay>allWays;
	std::vector<OSMRelation>allRelations;
public:
	OSMFormat(){}
	OSMFormat(std::string path) { OSMFormat::formatConversion(path); }
	~OSMFormat(){}
	void formatConversion(std::string path);
	void saveMapToOSM(std::string path);
};


#endif