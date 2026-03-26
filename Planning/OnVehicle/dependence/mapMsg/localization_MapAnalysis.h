#ifndef _LOCALIZATION_MAPANALYSIS_H
#define _LOCALIZATION_MAPANALYSIS_H


#include "rapidxml.hpp"
#include "rapidxml_utils.hpp"
#include "rapidxml_print.hpp"
#include<vector>
#include<string>
#include  "TrafficLight.h"

//������̳�������·�ε�id�ͳ���������id
struct LaneSuccessorId {
	int sucRoadID;
	int sucLaneID;
};


//·��ĸ�˹����ͺ���Ǻ�����
struct GaussRoadPoint {
	double GaussX;
	double GaussY;
	double yaw;
	double curvature;
	double s;
	double speedMax;
};


//�����࣬��������id�����id�������ڳ���id�������ڳ���id��·����Ϣ
class Lane {
public:
	int id;
	int BazierCurNUM;//贝塞尔曲线的数量 20230920
	double BazierCurDIS;//贝塞尔曲线的间距
	std::string  turnLSR;//???允许转向的方向，暂时用不上

	std::vector<LaneSuccessorId>successorId;
	std::vector<int>leftLaneId;
	std::vector<int>rightLaneId;
	std::vector<GaussRoadPoint>gaussRoadPoints;
public:
	Lane() {}
	~Lane() {}
};



//·���࣬ÿ��·����һ��id�����ɺ��·��id�������ɳ���
class Road {
public:
	int id;
	bool isLaneChange;
	std::vector<Lane>lanes;
	std::vector<int>successorId;
	double speedMax; //本road的最大速度，20230920新地图增加
public:
	Road() {}
	~Road() {}
};



//��ͼ�࣬һ����ͼ�ɶ���·�����
class RoadMap {
public:
	std::vector<Road>roads;
	
public:
	RoadMap(){}
	RoadMap(std::string path);
	~RoadMap(){}
	void mapAnalysis(std::string path);//·�ν���������xodr�ļ�λ�ã��������ݵ�roads
	void neighborLaneSort();//���ڳ���id����Խ�ٽ�������id�ŵ�Խǰ
	void moduleSelfCheck();//������֤����
	void moduleSelfCheckPrint();//���Ҵ�ӡ����

	bool  GetRoadByRoadID(int roadID,Road & road);//根据roadID获取road对象
	bool  GetLaneByLaneID(int laneID,Road  road, Lane & lane);//根据laneID获取lane对象
	bool GetStartPointByRoadID(int roadID,  double&  GaussX, double & GaussY);//根据roadID获取起点坐标
	bool GetEndPointByRoadID(int roadID,  double&  GaussX, double & GaussY);//根据roadID获取终点坐标
	bool GetLaneByRoadIDLaneID(int roadID, int LaneID,  Lane & lane);//根据roadID 和 laneID 获取lane对象
	bool GetLaneByRoadIDLaneID(int roadID, int LaneID,  int pointID , GaussRoadPoint & guassPoint);///根据roadID , laneID, pointID   获取lpoint对象

	TrafficLightMap trafficeLightMap_ ;//红绿灯信息20230713把红绿灯信息放在地图里面
	double meridianLine_;
};



//�����ĸ���OSMNode��OSMWay��OSMRelation��OSMFormat���ڽ�xodr�ļ�ת����osm�ļ�
//OSM�ļ��е�node�ڵ�
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

//OSM�ļ��е�way�ڵ�
class OSMWay {
public:
	int id;
	std::string idString;
	std::vector<std::string>nodeSet;
};

//OSM�ļ��е�relation�ڵ�
class OSMRelation {
public:
	int id;
	std::string idString;
	std::vector<std::string>waySet;
};

//����OSM�ļ�
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