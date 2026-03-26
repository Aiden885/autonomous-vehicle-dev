#ifndef _LOCALIZATION_MAPANALYSIS_H
#define _LOCALIZATION_MAPANALYSIS_H


#include "rapidxml.hpp"
#include "rapidxml_utils.hpp"
#include "rapidxml_print.hpp"
#include<vector>
#include<string>


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

	struct mapAnalysisParam
	{
	};

	struct mapAnalysisInput
	{
		std::string path;
	};

	struct mapAnalysisOutput
	{
	
	};

	void mapAnalysis(const mapAnalysisParam &param, const mapAnalysisInput &input, const mapAnalysisOutput &output);//·�ν���������xodr�ļ�λ�ã��������ݵ�roads
	void neighborLaneSort();//���ڳ���id����Խ�ٽ�������id�ŵ�Խǰ
	void moduleSelfCheck();//������֤����
	void moduleSelfCheckPrint();//���Ҵ�ӡ����

	bool  GetRoadByRoadID(int roadID,Road & road);//根据roadID获取road对象
	bool  GetLaneByLaneID(int laneID,Road  road, Lane & lane);//根据laneID获取lane对象
	bool GetStartPointByRoadID(int roadID,  double&  GaussX, double & GaussY);//根据roadID获取起点坐标
	bool GetEndPointByRoadID(int roadID,  double&  GaussX, double & GaussY);//根据roadID获取终点坐标
};


struct mapAnalysisParam
{
};

struct mapAnalysisInput
{
	std::string path;
};

struct mapAnalysisOutput
{
	std::vector<Road> roads;
};

void mapAnalysis(const mapAnalysisParam &param, const mapAnalysisInput &input, mapAnalysisOutput &output);//·�ν���������xodr�ļ�λ�ã��������ݵ�roads

struct neighborLaneSortParam
{
};

struct neighborLaneSortInput
{
	std::vector<Road> roads;
};

struct neighborLaneSortOutput
{
};

void neighborLaneSort(const neighborLaneSortParam &param, const neighborLaneSortInput &input, neighborLaneSortOutput &output);//���ڳ���id����Խ�ٽ�������id�ŵ�Խǰ

struct moduleSelfCheckParam
{
};

struct moduleSelfCheckInput
{
	std::vector<Road> roads;
};

struct moduleSelfCheckOutput
{
};

void moduleSelfCheck(const moduleSelfCheckParam &param, const moduleSelfCheckInput &input, moduleSelfCheckOutput &output);

struct moduleSelfCheckPrintParam
{
};

struct moduleSelfCheckPrintInput
{
	std::vector<Road> roads;
};

struct moduleSelfCheckPrintOutput
{
};

void moduleSelfCheckPrint(const moduleSelfCheckPrintParam &param, const moduleSelfCheckPrintInput &input, moduleSelfCheckPrintOutput &output);

struct GetRoadByRoadIDParam
{
};

struct GetRoadByRoadIDInput
{
	std::vector<Road> roads;
	int roadID;
};

struct GetRoadByRoadIDOutput
{
	Road  road ;
	bool isValidRoad;
};

void GetRoadByRoadID(const GetRoadByRoadIDParam &param, const GetRoadByRoadIDInput &input, GetRoadByRoadIDOutput &output);

struct GetLaneByLaneIDParam
{
};

struct GetLaneByLaneIDInput
{
	int laneID;
	Road  road;
};

struct GetLaneByLaneIDOutput
{
	Lane & lane;
	bool isValidLine;
};

void GetLaneByLaneID(const GetLaneByLaneIDParam &param, const GetLaneByLaneIDInput &input, GetLaneByLaneIDOutput &output);
//bool GetStartPointByRoadID(int roadID,  double&  GaussX, double & GaussY);//根据roadID获取起点坐标
struct GetStartPointByRoadIDParam
{
};

struct GetStartPointByRoadIDInput
{
	int roadID;
	std::vector<Road> roads;
};

struct GetStartPointByRoadIDOutput
{
	double GaussX;
	double GaussY;
	bool isValidStartPoint;
};

void GetStartPointByRoadID(const GetStartPointByRoadIDParam &param, const GetStartPointByRoadIDInput &input, GetStartPointByRoadIDOutput &output);
//bool GetEndPointByRoadID(int roadID,  double&  GaussX, double & GaussY);//根据roadID获取终点坐标
struct GetEndPointByRoadIDParam
{
};

struct GetEndPointByRoadIDInput
{
	int roadID;
	std::vector<Road> roads;
};

struct GetEndPointByRoadIDOutput
{
	double GaussX;
	double GaussY;
	bool isValidStartPoint;
};

void GetEndPointByRoadID(const GetEndPointByRoadIDParam &param, const GetEndPointByRoadIDInput &input, GetEndPointByRoadIDOutput &output);


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