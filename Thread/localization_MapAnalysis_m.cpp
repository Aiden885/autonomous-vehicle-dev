#include"localization_MapAnalysis_m.h"
#include <cstdlib>
#include<algorithm>
#include<cmath>
#include<iostream>
#include<sstream>
#include<iomanip>
RoadMap::RoadMap(std::string path) {
	RoadMap::mapAnalysis(path);
	RoadMap::neighborLaneSort();
	RoadMap::moduleSelfCheck();
	//RoadMap::moduleSelfCheckPrint();
}


//·�ν�����path��xodr�ļ�λ�ã�����path="C:\\Users\\Administrator\\Desktop\\roadMap.xodr"���ļ�·�����ܺ������ģ�����
//���ݽ�����Map���roads
// change by rgy 20241112 
/**
 * @brief 经纬度转全局坐标
 * @param[IN] param 无
 * @param[IN] input path
 * @param[OUT] output 全局坐标值
 
 * @cn_name: 

 * @granularity:  //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: 
 */
void mapAnalysis(const mapAnalysisParam &param, const mapAnalysisInput &input, mapAnalysisOutput &output) {
	std::string path = input.path;
	std::cout << "RoadMap::mapAnalysis(std::string path) {"<<std::endl;
	rapidxml::file<> fdoc(path.c_str());
	rapidxml::xml_document<> doc;
	doc.parse<0>(fdoc.data());
	rapidxml::xml_node<>* xmlMap = doc.first_node();
	rapidxml::xml_node<>* xmlRoad = xmlMap->first_node("road");
	std::cout << "11111111111111111111111111111111111111111"<<std::endl;
	std::vector<Road> roads;
	while (xmlRoad != NULL)
	{
		Road roadTmp;
		rapidxml::xml_attribute<>* attr;

		int roadId = -1;
		attr = xmlRoad->first_attribute("id");
		if (attr != NULL) { roadId = atoi(attr->value()); }
		roadTmp.id = roadId;
		std::cout<<"roadID = "<<roadId<<std::endl;

		bool laneChange = true;
		std::string lc;
		attr = xmlRoad->first_attribute("isLaneChange");
		//std::cout << "333333333333333333333333333333333333333"<<std::endl;
		if (attr != NULL) 
		{ 
			lc = attr->value();
			if (lc == "true") { laneChange = true; }
			else if (lc == "false") { laneChange = false; } 
		}
		roadTmp.isLaneChange = laneChange;
//std::cout << "444444444444444444444444444444444"<<std::endl;
		rapidxml::xml_node<>* xmlSuccessor = xmlRoad->first_node("successor");
		//std::cout << "555555555555555555555555555555"<<std::endl;
		while (xmlSuccessor != NULL)
		{
			attr = xmlSuccessor->first_attribute("successor_roadId");
			//std::cout << "666666666666666666666666666"<<std::endl;
			int successorID = -1;
			if(attr == NULL)
			{
				std::cout<<"attr坑人呀" <<std::endl;
			}
			std::cout<<"attr->value()" << attr->name()<<std::endl;
			successorID = atoi(attr->value());
			std::cout<<"successorID" << successorID<<std::endl;
			roadTmp.successorId.push_back(successorID);
			xmlSuccessor = xmlSuccessor->next_sibling("successor");
		}

		//std::cout << "222222222222222222222222222222222222"<<std::endl;
		rapidxml::xml_node<>* xmlLanes = xmlRoad->first_node("lanes");
		rapidxml::xml_node<>* xmlLane = xmlLanes->first_node("lane");
		while (xmlLane != NULL)
		{
			Lane laneTmp;


			int laneID = -1;
			attr = xmlLane->first_attribute("id");
			if (attr != NULL) { laneID = atoi(attr->value()); }
			laneTmp.id = laneID;

			std::cout<<"laneID" <<std::endl;
		
			rapidxml::xml_node<>* xmlLaneSuccessor = xmlLane->first_node("successor");
			while (xmlLaneSuccessor != NULL)
			{
				LaneSuccessorId laneSucID = { -1,-1 };
				attr = xmlLaneSuccessor->first_attribute("successor_roadId");
				if (attr != NULL) { laneSucID.sucRoadID = atoi(attr->value()); }
				attr = xmlLaneSuccessor->first_attribute("successor_laneId");
				if (attr != NULL) { laneSucID.sucLaneID = atoi(attr->value()); }
				laneTmp.successorId.push_back(laneSucID); 
				xmlLaneSuccessor = xmlLaneSuccessor->next_sibling("successor");
			}


			rapidxml::xml_node<>* xmlLeftLaneID = xmlLane->first_node("leftLaneID");
			while (xmlLeftLaneID != NULL)
			{
				int leftLane = -1;
				attr = xmlLeftLaneID->first_attribute("id");
				if (attr != NULL) { leftLane = atoi(attr->value()); }
				laneTmp.leftLaneId.push_back(leftLane);
				xmlLeftLaneID = xmlLeftLaneID->next_sibling("leftLaneID");
			}


			rapidxml::xml_node<>* xmlrightLaneID = xmlLane->first_node("rightLaneID");
			while (xmlrightLaneID != NULL)
			{
				int rightLane = -1;
				attr = xmlrightLaneID->first_attribute("id");
				if (attr != NULL) { rightLane = atoi(attr->value()); }
				laneTmp.rightLaneId.push_back(rightLane);
				xmlrightLaneID = xmlrightLaneID->next_sibling("rightLaneID");
			}


			rapidxml::xml_node<>* xmlRoadPoints = xmlLane->first_node("roadPoints");
			rapidxml::xml_node<>* xmlRoadPoint = xmlRoadPoints->first_node("roadPoint");
			while (xmlRoadPoint != NULL)
			{
				GaussRoadPoint point = { 0.0,0.0,0.0,0.0 };
				attr = xmlRoadPoint->first_attribute("gaussX");
				if (attr != NULL) { point.GaussX = atof(attr->value()); }
				attr = xmlRoadPoint->first_attribute("gaussY");
				if (attr != NULL) { point.GaussY = atof(attr->value()); }
				attr = xmlRoadPoint->first_attribute("yaw");
				if (attr != NULL) { point.yaw = atof(attr->value()); }
				attr = xmlRoadPoint->first_attribute("curvature");
				if (attr != NULL) { point.curvature = atof(attr->value()); }
				attr = xmlRoadPoint->first_attribute("s");
				if (attr != NULL) { point.s = atof(attr->value()); }
				attr = xmlRoadPoint->first_attribute("speedMax");
				if (attr != NULL) { point.speedMax = atof(attr->value()); }
				laneTmp.gaussRoadPoints.push_back(point);
				xmlRoadPoint = xmlRoadPoint->next_sibling("roadPoint");
			}


			roadTmp.lanes.push_back(laneTmp);
			xmlLane = xmlLane->next_sibling("lane");
		}

		roads.push_back(roadTmp);
		xmlRoad = xmlRoad->next_sibling("road");

	}
	output.roads = roads;
}

//Ŀǰ����id���������Ǵ������󳵵�id����Ϊ0��1��2����
//neighborLaneSort���������������ڳ���id���򣬱���һ������idΪ2���������������3��4���ұ�����������0��1�������leftLaneId[0]=3,leftLaneId[1]=4,rightLaneId[0]=1,rightLaneId[1]=0,�뱾����Խ�����������ŵ�Խǰ
void neighborLaneSort(const neighborLaneSortParam &param, const neighborLaneSortInput &input, neighborLaneSortOutput &output) {
	std::cout << "RoadMap::neighborLaneSort()  {"<<std::endl;
	for (int i = 0; i < input.roads.size(); i++) 
	{
		for (int j = 0; j < input.roads[i].lanes.size(); j++)
		{
			sort(input.roads[i].lanes[j].leftLaneId.begin(), input.roads[i].lanes[j].leftLaneId.end());
			sort(input.roads[i].lanes[j].rightLaneId.rbegin(), input.roads[i].lanes[j].rightLaneId.rend());
		}
	}
}



//������֤��������鱾ģ���������Ƿ��������ݸ�ʽ����ΧҪ��
void moduleSelfCheck(const moduleSelfCheckParam &param, const moduleSelfCheckInput &input, moduleSelfCheckOutput &output) {
	std::cout << "moduleSelfCheck()   {"<<std::endl;
	int flag = 0;
	std::vector<Road> roads = input.roads;
	for (int i = 0; i < roads.size(); i++)
	{
		if (roads[i].id < 0) { std::cout << "��" << i << "��·��idΪ" << roads[i].id << ",idӦ���ڵ���0������" << std::endl; flag++; }
		for (int j = 0; j < roads[i].successorId.size(); j++)
		{
			if (roads[i].successorId[j] < 0) { std::cout << "��" << i << "��·��idΪ" << roads[i].id << ",���idΪ" << roads[i].successorId[j] << ",���·��idӦ���ڵ���0������" << std::endl; flag++; }
		}
		for (int j = 0; j < roads[i].lanes.size(); j++)
		{
			if (roads[i].lanes[j].id < 0) { std::cout << "��·�ε�" << j << "������id:" << roads[i].lanes[j].id << "������idӦ���ڵ���0������" << std::endl; flag++; }
			for (int k = 0; k < roads[i].lanes[j].successorId.size(); k++)
			{
				if (roads[i].lanes[j].successorId[k].sucRoadID < 0 || roads[i].lanes[j].successorId[k].sucLaneID < 0)
				{
					std::cout << "��·�ε�" << j << "��������̳���:" << roads[i].lanes[j].successorId[k].sucRoadID << "��·�Σ�" << roads[i].lanes[j].successorId[k].sucLaneID << "�ų���" << ",·��idӦ����0������idӦ���ڵ���0������" << std::endl;
					flag++;
				}
			}
			for (int k = 0; k < roads[i].lanes[j].leftLaneId.size(); k++)
			{
				if (roads[i].lanes[j].leftLaneId[k] < 0) { std::cout << "��·�ε�" << j << "�����������ڳ���id:" << roads[i].lanes[j].leftLaneId[k] << "������idӦ���ڵ���0������" << std::endl; flag++; }
			}
			for (int k = 0; k < roads[i].lanes[j].rightLaneId.size(); k++)
			{
				if (roads[i].lanes[j].rightLaneId[k] < 0) { std::cout << "��·�ε�" << j << "�����������ڳ���id:" << roads[i].lanes[j].rightLaneId[k] << "������idӦ���ڵ���0������" << std::endl; flag++; }
			}
			if (roads[i].lanes[j].gaussRoadPoints.size() == 0) { std::cout << "��" << i << "��·��idΪ" << roads[i].id << "����·��·������Ϊ0������" << std::endl; flag++; }
		}
	}
	if (flag == 0) { std::cout << "·�������ϣ����ݶ�������" << std::endl; }
}


//�����ӡ��ʾ��������ӡ��ģ��Ĺؼ����������
void moduleSelfCheckPrint(const moduleSelfCheckPrintParam &param, const moduleSelfCheckPrintInput &input, moduleSelfCheckPrintOutput &output) {
	std::vector<Road> roads = input.roads;
	std::cout << "��ͼ����" << roads.size() << "��·��" << std::endl;
	for (int i = 0; i < roads.size(); i++)
	{
		std::cout << "·��id��" << roads[i].id << std::endl;
		std::cout << "�Ƿ�ɻ�����" << roads[i].isLaneChange << std::endl;
		for (int j = 0; j < roads[i].successorId.size(); j++)
		{
			std::cout << "���·��id��" << roads[i].successorId[j] << std::endl;
		}
		std::cout << "��·����" << roads[i].lanes.size()<<"������" << std::endl;
		for (int j = 0; j < roads[i].lanes.size(); j++)
		{
			std::cout << "��·�ε�" << j << "������id:"<< roads[i].lanes[j].id << std::endl;
			for (int k = 0; k < roads[i].lanes[j].successorId.size(); k++)
			{
				std::cout << "��·�ε�" << j << "��������̳���:" << roads[i].lanes[j].successorId[k].sucRoadID<<"��·�Σ�"<< roads[i].lanes[j].successorId[k].sucLaneID<<"�ų���" << std::endl;
			}
			for (int k = 0; k < roads[i].lanes[j].leftLaneId.size(); k++)
			{
				std::cout << "��·�ε�" << j << "�����������ڳ���id:" << roads[i].lanes[j].leftLaneId[k]  << std::endl;
			}
			for (int k = 0; k < roads[i].lanes[j].rightLaneId.size(); k++)
			{
				std::cout << "��·�ε�" << j << "�����������ڳ���id:" << roads[i].lanes[j].rightLaneId[k] << std::endl;
			}
			std::cout << "��·�ε�" << j << "������·������:" << roads[i].lanes[j].gaussRoadPoints.size() << std::endl;
			//��ӡ��ͼ������·�����Ϣ��·����ܱȽ϶�
			for (int k = 0; k < roads[i].lanes[j].gaussRoadPoints.size(); k++)
			{
				std::cout << "gaussX:" << roads[i].lanes[j].gaussRoadPoints[k].GaussX << ";";
				std::cout << "gaussY:" << roads[i].lanes[j].gaussRoadPoints[k].GaussY << ";";
				std::cout << "yaw:" << roads[i].lanes[j].gaussRoadPoints[k].yaw << ";";
				std::cout << "curvature::" << roads[i].lanes[j].gaussRoadPoints[k].curvature;
				std::cout << "s:" << roads[i].lanes[j].gaussRoadPoints[k].s << ";";
				std::cout << "speedMax::" << roads[i].lanes[j].gaussRoadPoints[k].speedMax << std::endl;
			}
			
		}
	}
}

void GetRoadByRoadID(const GetRoadByRoadIDParam &param, const GetRoadByRoadIDInput &input, GetRoadByRoadIDOutput &output)//根据roadID获取road对象
{
	int roadID = input.roadID;
	std::vector<Road> roads = input.roads;
	if(roadID < 0)
	{
		output.isValidRoad = false;
		return ;
	}
		
		
    for (int i = 0; i < roads.size(); i++)
    {
        if (roadID == roads[i].id)
        {
           	output.road =roads[i] ;
			output.isValidRoad = true;
            return;
        }
    }
	output.isValidRoad = false;
	return ;
}

void GetLaneByLaneID(const GetLaneByLaneIDParam &param, const GetLaneByLaneIDInput &input, GetLaneByLaneIDOutput &output)//根据laneID获取lane对象
{
	Road road = input.road;
	int laneID = input.laneID;
	if(road.id < 0 || laneID < 0)
	{
		output.isValidLine = false;
		return ;
	}
		
    for (int i = 0; i < road.lanes.size(); i++)
    {
        if (laneID == road.lanes[i].id)
        {
           	output.lane =road.lanes[i] ;
			output.isValidLine = true;
            return;
        }
    }
	
	output.isValidLine = false;
	return ;
	
}

//根据roadID获取起点坐标
void GetStartPointByRoadID(const GetStartPointByRoadIDParam &param, const GetStartPointByRoadIDInput &input, GetStartPointByRoadIDOutput &output)
{
	int roadID = input.roadID;
	Road  road;
	bool getRoad;
	GetRoadByRoadIDParam paramGRBID{};
	GetRoadByRoadIDInput inputGRBID{input.roads, input.roadID};
	GetRoadByRoadIDOutput outputGRBID{road, getRoad};
	GetRoadByRoadID(paramGRBID, inputGRBID, outputGRBID);
	if(! outputGRBID.isValidRoad)//找路
	{
		output.isValidStartPoint = false;
		return;
	}
	Lane  lane;
	bool getLane;
	GetLaneByLaneIDParam paramGLBLID{};
	GetLaneByLaneIDInput inputGLBLID{0, road};
	GetLaneByLaneIDOutput outputGLBLID{lane, getLane};
	GetLaneByLaneID(paramGLBLID, inputGLBLID, outputGLBLID);
	if(!outputGLBLID.isValidLine)	//找lane
	{
		output.isValidStartPoint = false;
		return;
	}
		

	if(lane.gaussRoadPoints.size() <= 0)
	{
		output.isValidStartPoint = false;
		return;
	}

	output.GaussX = lane.gaussRoadPoints[0].GaussX;
	output.GaussY = lane.gaussRoadPoints[0].GaussY;
	output.isValidStartPoint = true;
	return;
}

//根据roadID获取终点坐标
void GetEndPointByRoadID(const GetEndPointByRoadIDParam &param, const GetEndPointByRoadIDInput &input, GetEndPointByRoadIDOutput &output)
{
	int roadID = input.roadID;
	Road  road;
	bool getRoad;
	GetRoadByRoadIDParam paramGRBID{};
	GetRoadByRoadIDInput inputGRBID{input.roads, input.roadID};
	GetRoadByRoadIDOutput outputGRBID{road, getRoad};
	GetRoadByRoadID(paramGRBID, inputGRBID, outputGRBID);

	if(! outputGRBID.isValidRoad)//找路
	{
		output.isValidStartPoint = false;
		return;
	}

	Lane  lane;
	bool getLane;
	GetLaneByLaneIDParam paramGLBLID{};
	GetLaneByLaneIDInput inputGLBLID{0, road};
	GetLaneByLaneIDOutput outputGLBLID{lane, getLane};
	GetLaneByLaneID(paramGLBLID, inputGLBLID, outputGLBLID);
	if(!outputGLBLID.isValidLine)	//找lane
	{
		output.isValidStartPoint = false;
		return;
	}

	int nPoint = lane.gaussRoadPoints.size();
	if(nPoint  <= 0)	
	{
		output.isValidStartPoint = false;
		return;
	}

	nPoint -- ;
	output.GaussX = lane.gaussRoadPoints[nPoint].GaussX;
	output.GaussY = lane.gaussRoadPoints[nPoint].GaussY;
	output.isValidStartPoint = true;
	return;
}


//�Ӵ��ͼ���ڳ�һ��С��ͼ������t���������ͼ��RoadMap��Ķ���x��y��С��ͼ�������꣬r�ǰ뾶��Χ
//�˺��������ͼ����������������뾶��Χ�ڵ�·�ζ����������¹���һ���µĵ�ͼ�������һ��RoadMap����󣬰�����Χ������·��
// RoadMap getSmallMap(const RoadMap&t,double x,double y, double r) {
// 	RoadMap smallMap;
// 	for (int i = 0; i < t.roads.size(); i++)
// 	{
// 		int flag = 0;
// 		for (int j = 0; flag==0&&j < t.roads[i].lanes.size(); j++)
// 		{
// 			for (int k = 0; flag==0&&k < t.roads[i].lanes[j].gaussRoadPoints.size(); k++)
// 			{
// 				double distanceX = t.roads[i].lanes[j].gaussRoadPoints[k].GaussX - x;
// 				double distanceY = t.roads[i].lanes[j].gaussRoadPoints[k].GaussY - y;
// 				double distance = sqrt(distanceX * distanceX + distanceY * distanceY);
// 				if (distance < r)
// 				{
// 					smallMap.roads.push_back(t.roads[i]);
// 					flag = 1;
// 				}
// 			}
// 		}
// 	}
// 	return smallMap;
// }
struct GetSmallMapParam
{
};

struct GetSmallMapInput
{
	RoadMap t;
	double x;
	double y;
	double r;
};

struct GetSmallMapOutput
{
	RoadMap smallMap;
};
RoadMap getSmallMap(const GetSmallMapParam &param, const GetSmallMapInput &input, GetSmallMapOutput output) {
	RoadMap smallMap;
	RoadMap t = input.t;
	double x = input.x;
	double y = input.y;
	double r = input.r;

	for (int i = 0; i < t.roads.size(); i++)
	{
		int flag = 0;
		for (int j = 0; flag==0&&j < t.roads[i].lanes.size(); j++)
		{
			for (int k = 0; flag==0&&k < t.roads[i].lanes[j].gaussRoadPoints.size(); k++)
			{
				double distanceX = t.roads[i].lanes[j].gaussRoadPoints[k].GaussX - x;
				double distanceY = t.roads[i].lanes[j].gaussRoadPoints[k].GaussY - y;
				double distance = sqrt(distanceX * distanceX + distanceY * distanceY);
				if (distance < r)
				{
					smallMap.roads.push_back(t.roads[i]);
					flag = 1;
				}
			}
		}
	}
	output.smallMap = smallMap;
	return ;
}



//std::vector<double>getLonAndLat(double gaussX, double gaussY) {
//	std::vector<double>LonLat;
//	double iPI = 0.0174532925199433; //3.1415926535898/180.0
//	double PI = 3.1415926535898;
//	double a = 6378245.0;   // ����
//	double f = 1.0 / 298.3; // ����   (a-b)/a
//	double ZoneWide = 6.0;     // ����
//	double e2 = 2 * f - f * f; // eΪ��һƫ���ʣ����������ֱ���ṩ��e2 = e * e
//	int ProjNo = (int)(gaussX / 1000000); // ���Ҵ���
//	double longitude0 = (ProjNo - 1) * ZoneWide + ZoneWide / 2;
//	longitude0 = longitude0 * iPI; // ���뾭��
//
//	double X0 = ProjNo * 1000000 + 500000;
//	double Y0 = 0;
//	double xval = gaussX - X0;
//	double yval = gaussY - Y0; // ���ڴ������
//
//	double e1 = (1.0 - sqrt(1 - e2)) / (1.0 + sqrt(1 - e2));
//	double ee = e2 / (1 - e2);
//	double M = yval;
//	double u = M / (a * (1 - e2 / 4 - 3 * e2 * e2 / 64 - 5 * e2 * e2 * e2 / 256));
//	double fai = u + (3 * e1 / 2 - 27 * e1 * e1 * e1 / 32) * sin(2 * u) + (21 * e1 * e1 / 16 - 55 * e1 * e1 * e1 * e1 / 32) * sin(4 * u) + (151 * e1 * e1 * e1 / 96) * sin(6 * u) + (1097 * e1 * e1 * e1 * e1 / 512) * sin(8 * u);
//	double C = ee * cos(fai) * cos(fai);
//	double T = tan(fai) * tan(fai);
//	double N = a / sqrt(1.0 - e2 * sin(fai) * sin(fai)); // �õ��î��Ȧ���ʰ뾶
//	double R = a * (1 - e2) / sqrt((1 - e2 * sin(fai) * sin(fai)) * (1 - e2 * sin(fai) * sin(fai)) * (1 - e2 * sin(fai) * sin(fai)));
//	double D = xval / N;
//	// ���㾭��(Longitude) γ��(Latitude)
//	double longitude = longitude0 + (D - (1 + 2 * T + C) * D * D * D / 6 + (5 - 2 * C + 28 * T - 3 * C * C + 8 * ee + 24 * T * T) * D * D * D * D * D / 120) / cos(fai);
//	double latitude = fai - (N * tan(fai) / R) * (D * D / 2 - (5 + 3 * T + 10 * C - 4 * C * C - 9 * ee) * D * D * D * D / 24 + (61 + 90 * T + 298 * C + 45 * T * T - 256 * ee - 3 * C * C) * D * D * D * D * D * D / 720);
//	// ת��Ϊ�� DD
//	longitude = longitude / iPI;
//	latitude = latitude / iPI;
//	LonLat.push_back(longitude);
//	LonLat.push_back(latitude);
//	return LonLat;
//}

//int����ת����string���͵�����
std::string intToString(int s) {
	std::stringstream ss;
	std::string st;
	ss << s;
	ss >> st;
	return st;
}

//double����ת����string���͵����ݣ�decimalPlaces�Ǳ�����С��������λ
//����double t=9.32232653213e+2;string m=doubleToString(t,3);��m����"932.233"
std::string doubleToString(double s, int decimalPlaces) {
	std::stringstream ss;
	std::string st;
	ss << std::fixed << std::setprecision(decimalPlaces) << s;
	ss >> st;
	return st;
}

//��ȡoxdr�ļ����ݲ�ת����osm�ļ����ݣ�path��xodr�ļ�λ�ã�����path="C:\\Users\\Administrator\\Desktop\\roadMap.xodr"���ļ�·�����ܺ������ģ�����
void OSMFormat::formatConversion(std::string path) {
	rapidxml::file<> fdoc(path.c_str());
	rapidxml::xml_document<> doc;
	doc.parse<0>(fdoc.data());
	rapidxml::xml_node<>* xmlMap = doc.first_node();
	rapidxml::xml_node<>* xmlRoad = xmlMap->first_node("road");
	rapidxml::xml_attribute<>* attr;
	int nodeNumber = 0, wayNumber = 0, relationNumber = 0;
	while (xmlRoad != NULL)
	{
		rapidxml::xml_node<>* xmlLanes = xmlRoad->first_node("lanes");
		rapidxml::xml_node<>* xmlLane = xmlLanes->first_node("lane");
		while (xmlLane != NULL)
		{
			wayNumber++;
			relationNumber++;
			OSMWay Linestrings;
			Linestrings.id = wayNumber;
			Linestrings.idString = intToString(wayNumber);
			OSMRelation lanelet;
			lanelet.id = relationNumber;
			lanelet.idString = intToString(relationNumber);
			lanelet.waySet.push_back(intToString(wayNumber));
			rapidxml::xml_node<>* xmlRoadPoints = xmlLane->first_node("roadPoints");
			rapidxml::xml_node<>* xmlRoadPoint = xmlRoadPoints->first_node("roadPoint");
			while (xmlRoadPoint != NULL)
			{
				nodeNumber++;
				Linestrings.nodeSet.push_back(intToString(nodeNumber));
				OSMNode point;
				attr = xmlRoadPoint->first_attribute("gaussX");
				if (attr != NULL) { point.localX = atof(attr->value()); }
				attr = xmlRoadPoint->first_attribute("gaussY");
				if (attr != NULL) { point.localY = atof(attr->value()); }
				attr = xmlRoadPoint->first_attribute("yaw");
				if (attr != NULL) { point.yaw = atof(attr->value()); }
				attr = xmlRoadPoint->first_attribute("curvature");
				if (attr != NULL) { point.curvature = atof(attr->value()); }
				point.id = nodeNumber;
				point.idString = intToString(point.id);
				point.localXString = doubleToString(point.localX,3);
				point.localYString = doubleToString(point.localY, 3);
				point.curvatureString = doubleToString(point.curvature, 3);
				point.yawString = doubleToString(point.yaw, 3);
				allNodes.push_back(point);
				xmlRoadPoint = xmlRoadPoint->next_sibling("roadPoint");
			}
			allWays.push_back(Linestrings);
			allRelations.push_back(lanelet);
			xmlLane = xmlLane->next_sibling("lane");
		}
		xmlRoad = xmlRoad->next_sibling("road");
	}
}

//��ת��������osm�ļ����ݴ����osm�ļ���path��osm�ļ�λ�ã�����path="C:\\Users\\Administrator\\Desktop\\roadMap.osm"���ļ�·�����ܺ������ģ�����
void OSMFormat::saveMapToOSM(std::string path) {
	std::cout << allNodes.size() << std::endl;
	rapidxml::xml_document<> doc;  //����һ���յ�xml�ĵ�
	rapidxml::xml_node<>* rot = doc.allocate_node(rapidxml::node_pi, doc.allocate_string("setting.xml version='1.0' encoding='utf-8'"));//allocate_node����һ���ڵ㣬�ýڵ�����Ϊnode_pi����XML�ļ������裬����������allocate_string��
	doc.append_node(rot); //�Ѹýڵ����ӵ�doc��

	rapidxml::xml_node<>* osm = doc.allocate_node(rapidxml::node_element, "OSM", NULL);
	int cnt = 0;
	for (int i = 0; i < allNodes.size(); i++)
	{
		rapidxml::xml_node<>* node = doc.allocate_node(rapidxml::node_element, "node", NULL);
		node->append_attribute(doc.allocate_attribute("id", allNodes[i].idString.c_str()));
		osm->append_node(node);

		rapidxml::xml_node<>* tag1 = doc.allocate_node(rapidxml::node_element, "tag", NULL);
		tag1->append_attribute(doc.allocate_attribute("k", "local_x"));
		tag1->append_attribute(doc.allocate_attribute("v", allNodes[i].localXString.c_str()));
		node->append_node(tag1);
		rapidxml::xml_node<>* tag2 = doc.allocate_node(rapidxml::node_element, "tag", NULL);
		tag2->append_attribute(doc.allocate_attribute("k", "local_y"));
		tag2->append_attribute(doc.allocate_attribute("v", allNodes[i].localYString.c_str()));
		node->append_node(tag2);
		rapidxml::xml_node<>* tag3 = doc.allocate_node(rapidxml::node_element, "tag", NULL);
		tag3->append_attribute(doc.allocate_attribute("k", "yaw"));
		tag3->append_attribute(doc.allocate_attribute("v", allNodes[i].yawString.c_str()));
		node->append_node(tag3);
		rapidxml::xml_node<>* tag4 = doc.allocate_node(rapidxml::node_element, "tag", NULL);
		tag4->append_attribute(doc.allocate_attribute("k", "curvature"));
		tag4->append_attribute(doc.allocate_attribute("v", allNodes[i].curvatureString.c_str()));
		node->append_node(tag4);
	}

	for (int i = 0; i < allWays.size(); i++) 
	{
		rapidxml::xml_node<>* way = doc.allocate_node(rapidxml::node_element, "way", NULL);
		way->append_attribute(doc.allocate_attribute("id", allWays[i].idString.c_str()));
		osm->append_node(way);
		for (int j = 0; j < allWays[i].nodeSet.size(); j++)
		{
			rapidxml::xml_node<>* nd = doc.allocate_node(rapidxml::node_element, "nd", NULL);
			nd->append_attribute(doc.allocate_attribute("ref", allWays[i].nodeSet[j].c_str()));
			way->append_node(nd);
		}
		rapidxml::xml_node<>* tag = doc.allocate_node(rapidxml::node_element, "tag", NULL);
		tag->append_attribute(doc.allocate_attribute("type", "linestrings"));
		way->append_node(tag);
	}

	for (int i = 0; i < allRelations.size(); i++)
	{
		rapidxml::xml_node<>* relation = doc.allocate_node(rapidxml::node_element, "relation", NULL);
		relation->append_attribute(doc.allocate_attribute("id", allRelations[i].idString.c_str()));
		osm->append_node(relation);
		for (int j = 0; j < allRelations[i].waySet.size(); j++)
		{
			rapidxml::xml_node<>* member = doc.allocate_node(rapidxml::node_element, "member", NULL);
			member->append_attribute(doc.allocate_attribute("type", "way"));
			member->append_attribute(doc.allocate_attribute("role", "center"));
			member->append_attribute(doc.allocate_attribute("ref", allRelations[i].waySet[j].c_str()));
			relation->append_node(member);
		}
		rapidxml::xml_node<>* tag = doc.allocate_node(rapidxml::node_element, "tag", NULL);
		tag->append_attribute(doc.allocate_attribute("type", "lanelet"));
		relation->append_node(tag);
	}
	doc.append_node(osm);
	std::ofstream pout(path);
	pout << doc;
}


// int main() {
// 	RoadMap m("C:\\Users\\Administrator\\Desktop\\opendrive\\sampleNew\\roadMap.xodr");
// 	//RoadMap t = getSmallMap(m,0,0,20.0);
// 	//OSMFormat osm("C:\\Users\\Administrator\\Desktop\\opendrive\\sample\\roadMap.xodr");
// 	//osm.saveMapToOSM("C:\\Users\\Administrator\\Desktop\\opendrive\\sample\\test.osm");
// 	return 0;
// }