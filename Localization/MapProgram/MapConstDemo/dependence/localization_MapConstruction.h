#ifndef _LOCALIZATION_MAPCONSTRUCTION_H
#define _LOCALIZATION_MAPCONSTRUCTION_H
//��ͼ����

#include<vector>
#include<string>



struct RawRoadPoint {
	double longitude;
	double latitude;
	double yaw;
};

struct GaussRoadPoint {
	int id;
	double GaussX;//x��������ָ�򱱷�
	double GaussY;//y��������ָ�򶫷�
	double yaw;
	double curvature;
	double s;
	double speedLimit;
};

struct LaneSuccessorId {
	int sucRoadID;
	int sucLaneID;
	std::string sucRoadIDString;
	std::string sucLaneIDString;
};

struct roadPointInfo {
	std::string GaussX;
	std::string GaussY;
	std::string yaw;
	std::string curvature;
	std::string s;
	std::string speedLimit;
};



class Lane {
public:
	int id;
	std::string idString;
	double globalOffsetX, globalOffsetY;
	std::vector<LaneSuccessorId>successorId;
	std::vector<int>leftLaneId;
	std::vector<int>rightLaneId;
	std::vector<std::string>leftLaneIdString;
	std::vector<std::string>rightLaneIdString;
	std::vector<RawRoadPoint>rawRoadPoints;
	std::vector<GaussRoadPoint>gaussRoadPoints;
	std::vector<roadPointInfo>roadPointString;
public:
	Lane(){}
	~Lane(){}
	void updateId(int i) { id = i; }
	void getGaussRoadPoint();
	void getYawCruvature();
	void corectGaussXY();
	void calcSpeedProfile();
	friend std::istream& operator>>(std::istream&, Lane&);//��������������أ����ڴ��ļ���ȡ��·��Ϣ
};

class Road {
public:
	int id;
	bool isLaneChange;
	std::string idString;
	std::string isLaneChangeString;
	std::vector<Lane>lanes;
	std::vector<int>successorId;
	std::vector<std::string>successorIdString;
public:
	Road(bool laneChange = true):isLaneChange(laneChange){}
	~Road(){}
	void updateId(int i) { id = i; }
};


class MapConstruction {
public:
	std::vector<std::string>filenames;
	std::vector<int>roadId;
	std::vector<int>laneId;
	std::vector<Road>roads;
	
public:
	void getAllFileNames(std::string path);
	void loadRawRoadPoint(std::string path);
	void processRawRoadPoint();
	void loadSuccessor(std::string path);
	void loadLaneNeighborId(std::string path);
	void loadLaneSuccessorId(std::string path);
	void loadLaneChange(std::string path);
	void getXMLInfo();
	void saveMapToXML(std::string path);
	void moduleSelfCheck();
	void moduleSelfCheckPrint();
	
};


#endif