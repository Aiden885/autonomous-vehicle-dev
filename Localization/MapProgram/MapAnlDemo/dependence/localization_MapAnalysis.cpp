#include"../dependence/localization_MapAnalysis.h"
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


//路段解析，path是xodr文件位置，比如path="C:\\Users\\Administrator\\Desktop\\roadMap.xodr"，文件路径不能含有中文！！！
//数据解析到Map类的roads
void RoadMap::mapAnalysis(std::string path) {
	rapidxml::file<> fdoc(path.c_str());
	rapidxml::xml_document<> doc;
	doc.parse<0>(fdoc.data());
	rapidxml::xml_node<>* xmlMap = doc.first_node();
	rapidxml::xml_node<>* xmlRoad = xmlMap->first_node("road");
	while (xmlRoad != NULL)
	{
		Road roadTmp;
		rapidxml::xml_attribute<>* attr;

		int roadId = -1;
		attr = xmlRoad->first_attribute("id");
		if (attr != NULL) { roadId = atoi(attr->value()); }
		roadTmp.id = roadId;

		bool laneChange = true;
		std::string lc;
		attr = xmlRoad->first_attribute("isLaneChange");
		if (attr != NULL) 
		{ 
			lc = attr->value();
			if (lc == "true") { laneChange = true; }
			else if (lc == "false") { laneChange = false; } 
		}
		roadTmp.isLaneChange = laneChange;

		rapidxml::xml_node<>* xmlSuccessor = xmlRoad->first_node("successor");
		while (xmlSuccessor != NULL)
		{
			attr = xmlSuccessor->first_attribute("successor_roadId");
			int successorID = -1;
			successorID = atoi(attr->value());
			roadTmp.successorId.push_back(successorID);
			xmlSuccessor = xmlSuccessor->next_sibling("successor");
		}

		rapidxml::xml_node<>* xmlLanes = xmlRoad->first_node("lanes");
		rapidxml::xml_node<>* xmlLane = xmlLanes->first_node("lane");
		while (xmlLane != NULL)
		{
			Lane laneTmp;


			int laneID = -1;
			attr = xmlLane->first_attribute("id");
			if (attr != NULL) { laneID = atoi(attr->value()); }
			laneTmp.id = laneID;

		
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
	
}


//目前车道id给出规则是从右往左车道id依次为0，1，2……
//neighborLaneSort函数是用来给相邻车道id排序，比如一条车道id为2，左边有两条车道3和4，右边有两条车道0和1，排序后leftLaneId[0]=3,leftLaneId[1]=4,rightLaneId[0]=1,rightLaneId[1]=0,离本车道越近在数组中排的越前
void RoadMap::neighborLaneSort() {
	for (int i = 0; i < roads.size(); i++) 
	{
		for (int j = 0; j < roads[i].lanes.size(); j++)
		{
			sort(roads[i].lanes[j].leftLaneId.begin(), roads[i].lanes[j].leftLaneId.end());
			sort(roads[i].lanes[j].rightLaneId.rbegin(), roads[i].lanes[j].rightLaneId.rend());
		}
	}
}



//自我验证函数，检查本模块计算输出是否满足数据格式、范围要求
void RoadMap::moduleSelfCheck() {
	int flag = 0;
	for (int i = 0; i < roads.size(); i++)
	{
		if (roads[i].id < 0) { std::cout << "第" << i << "条路段id为" << roads[i].id << ",id应大于等于0，请检查" << std::endl; flag++; }
		for (int j = 0; j < roads[i].successorId.size(); j++)
		{
			if (roads[i].successorId[j] < 0) { std::cout << "第" << i << "条路段id为" << roads[i].id << ",后继id为" << roads[i].successorId[j] << ",后继路段id应大于等于0，请检查" << std::endl; flag++; }
		}
		for (int j = 0; j < roads[i].lanes.size(); j++)
		{
			if (roads[i].lanes[j].id < 0) { std::cout << "该路段第" << j << "条车道id:" << roads[i].lanes[j].id << "，车道id应大于等于0，请检查" << std::endl; flag++; }
			for (int k = 0; k < roads[i].lanes[j].successorId.size(); k++)
			{
				if (roads[i].lanes[j].successorId[k].sucRoadID < 0 || roads[i].lanes[j].successorId[k].sucLaneID < 0)
				{
					std::cout << "该路段第" << j << "条车道后继车道:" << roads[i].lanes[j].successorId[k].sucRoadID << "号路段，" << roads[i].lanes[j].successorId[k].sucLaneID << "号车道" << ",路段id应大于0，车道id应大于等于0，请检查" << std::endl;
					flag++;
				}
			}
			for (int k = 0; k < roads[i].lanes[j].leftLaneId.size(); k++)
			{
				if (roads[i].lanes[j].leftLaneId[k] < 0) { std::cout << "该路段第" << j << "条车道左相邻车道id:" << roads[i].lanes[j].leftLaneId[k] << "，车道id应大于等于0，请检查" << std::endl; flag++; }
			}
			for (int k = 0; k < roads[i].lanes[j].rightLaneId.size(); k++)
			{
				if (roads[i].lanes[j].rightLaneId[k] < 0) { std::cout << "该路段第" << j << "条车道右相邻车道id:" << roads[i].lanes[j].rightLaneId[k] << "，车道id应大于等于0，请检查" << std::endl; flag++; }
			}
			if (roads[i].lanes[j].gaussRoadPoints.size() == 0) { std::cout << "第" << i << "条路段id为" << roads[i].id << "，该路段路点数量为0，请检查" << std::endl; flag++; }
		}
	}
	if (flag == 0) { std::cout << "路点解析完毕，数据读入正常" << std::endl; }
}


//输出打印显示函数，打印本模块的关键参数及输出
void RoadMap::moduleSelfCheckPrint() {
	std::cout << "地图共有" << roads.size() << "条路段" << std::endl;
	for (int i = 0; i < roads.size(); i++)
	{
		std::cout << "路段id：" << roads[i].id << std::endl;
		std::cout << "是否可换道：" << roads[i].isLaneChange << std::endl;
		for (int j = 0; j < roads[i].successorId.size(); j++)
		{
			std::cout << "后继路段id：" << roads[i].successorId[j] << std::endl;
		}
		std::cout << "该路段有" << roads[i].lanes.size()<<"条车道" << std::endl;
		for (int j = 0; j < roads[i].lanes.size(); j++)
		{
			std::cout << "该路段第" << j << "条车道id:"<< roads[i].lanes[j].id << std::endl;
			for (int k = 0; k < roads[i].lanes[j].successorId.size(); k++)
			{
				std::cout << "该路段第" << j << "条车道后继车道:" << roads[i].lanes[j].successorId[k].sucRoadID<<"号路段，"<< roads[i].lanes[j].successorId[k].sucLaneID<<"号车道" << std::endl;
			}
			for (int k = 0; k < roads[i].lanes[j].leftLaneId.size(); k++)
			{
				std::cout << "该路段第" << j << "条车道左相邻车道id:" << roads[i].lanes[j].leftLaneId[k]  << std::endl;
			}
			for (int k = 0; k < roads[i].lanes[j].rightLaneId.size(); k++)
			{
				std::cout << "该路段第" << j << "条车道右相邻车道id:" << roads[i].lanes[j].rightLaneId[k] << std::endl;
			}
			std::cout << "该路段第" << j << "条车道路点数量:" << roads[i].lanes[j].gaussRoadPoints.size() << std::endl;
			//打印地图上所有路点的信息，路点可能比较多
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


//从大地图中挖出一个小地图，输入t是整个大地图，RoadMap类的对象，x和y是小地图中心坐标，r是半径范围
//此函数将大地图里所有在中心坐标半径范围内的路段都保留，重新构建一个新的地图，输出是一个RoadMap类对象，包含范围内所有路段
RoadMap getSmallMap(const RoadMap&t,double x,double y, double r) {
	RoadMap smallMap;
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
	return smallMap;
}




//std::vector<double>getLonAndLat(double gaussX, double gaussY) {
//	std::vector<double>LonLat;
//	double iPI = 0.0174532925199433; //3.1415926535898/180.0
//	double PI = 3.1415926535898;
//	double a = 6378245.0;   // 长轴
//	double f = 1.0 / 298.3; // 扁率   (a-b)/a
//	double ZoneWide = 6.0;     // 带宽
//	double e2 = 2 * f - f * f; // e为第一偏心率，可以算可以直接提供，e2 = e * e
//	int ProjNo = (int)(gaussX / 1000000); // 查找带号
//	double longitude0 = (ProjNo - 1) * ZoneWide + ZoneWide / 2;
//	longitude0 = longitude0 * iPI; // 中央经线
//
//	double X0 = ProjNo * 1000000 + 500000;
//	double Y0 = 0;
//	double xval = gaussX - X0;
//	double yval = gaussY - Y0; // 带内大地坐标
//
//	double e1 = (1.0 - sqrt(1 - e2)) / (1.0 + sqrt(1 - e2));
//	double ee = e2 / (1 - e2);
//	double M = yval;
//	double u = M / (a * (1 - e2 / 4 - 3 * e2 * e2 / 64 - 5 * e2 * e2 * e2 / 256));
//	double fai = u + (3 * e1 / 2 - 27 * e1 * e1 * e1 / 32) * sin(2 * u) + (21 * e1 * e1 / 16 - 55 * e1 * e1 * e1 * e1 / 32) * sin(4 * u) + (151 * e1 * e1 * e1 / 96) * sin(6 * u) + (1097 * e1 * e1 * e1 * e1 / 512) * sin(8 * u);
//	double C = ee * cos(fai) * cos(fai);
//	double T = tan(fai) * tan(fai);
//	double N = a / sqrt(1.0 - e2 * sin(fai) * sin(fai)); // 该点的卯酉圈曲率半径
//	double R = a * (1 - e2) / sqrt((1 - e2 * sin(fai) * sin(fai)) * (1 - e2 * sin(fai) * sin(fai)) * (1 - e2 * sin(fai) * sin(fai)));
//	double D = xval / N;
//	// 计算经度(Longitude) 纬度(Latitude)
//	double longitude = longitude0 + (D - (1 + 2 * T + C) * D * D * D / 6 + (5 - 2 * C + 28 * T - 3 * C * C + 8 * ee + 24 * T * T) * D * D * D * D * D / 120) / cos(fai);
//	double latitude = fai - (N * tan(fai) / R) * (D * D / 2 - (5 + 3 * T + 10 * C - 4 * C * C - 9 * ee) * D * D * D * D / 24 + (61 + 90 * T + 298 * C + 45 * T * T - 256 * ee - 3 * C * C) * D * D * D * D * D * D / 720);
//	// 转换为度 DD
//	longitude = longitude / iPI;
//	latitude = latitude / iPI;
//	LonLat.push_back(longitude);
//	LonLat.push_back(latitude);
//	return LonLat;
//}

//int数据转换成string类型的数据
std::string intToString(int s) {
	std::stringstream ss;
	std::string st;
	ss << s;
	ss >> st;
	return st;
}

//double数据转换成string类型的数据，decimalPlaces是保留到小数点后多少位
//比如double t=9.32232653213e+2;string m=doubleToString(t,3);则m就是"932.233"
std::string doubleToString(double s, int decimalPlaces) {
	std::stringstream ss;
	std::string st;
	ss << std::fixed << std::setprecision(decimalPlaces) << s;
	ss >> st;
	return st;
}

//读取oxdr文件内容并转换成osm文件内容，path是xodr文件位置，比如path="C:\\Users\\Administrator\\Desktop\\roadMap.xodr"，文件路径不能含有中文！！！
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

//将转换得来的osm文件内容储存进osm文件，path是osm文件位置，比如path="C:\\Users\\Administrator\\Desktop\\roadMap.osm"，文件路径不能含有中文！！！
void OSMFormat::saveMapToOSM(std::string path) {
	std::cout << allNodes.size() << std::endl;
	rapidxml::xml_document<> doc;  //构造一个空的xml文档
	rapidxml::xml_node<>* rot = doc.allocate_node(rapidxml::node_pi, doc.allocate_string("setting.xml version='1.0' encoding='utf-8'"));//allocate_node分配一个节点，该节点类型为node_pi，对XML文件进行描，描述内容在allocate_string中
	doc.append_node(rot); //把该节点添加到doc中

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
