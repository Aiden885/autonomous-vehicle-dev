#include"localization_MapAnalysis.h"
#include <cstdlib>
#include<algorithm>
#include<cmath>
#include<iostream>
Map::Map(std::string path) {
	Map::mapAnalysis(path);
	Map::neighborLaneSort();
	Map::moduleSelfCheck();
	//Map::moduleSelfCheckPrint();
}


//路段解析，path是xodr文件位置，比如path="C:\\Users\\Administrator\\Desktop\\roadMap.xodr"，文件路径不能含有中文！！！
//数据解析到Map类的roads
void Map::mapAnalysis(std::string path) {
	rapidxml::file<> fdoc(path.c_str());
	rapidxml::xml_document<> doc;
	doc.parse<0>(fdoc.data());
	rapidxml::xml_node<>* xmlMap = doc.first_node();
	rapidxml::xml_node<>* xmlRoad = xmlMap->first_node("road");
	while (xmlRoad != NULL)
	{
		Road roadTmp;
		rapidxml::xml_attribute<>* attr;

		int roadid = -1;
		attr = xmlRoad->first_attribute("id");
		if (attr != NULL) { roadid = atoi(attr->value()); }
		roadTmp.id = roadid;

		rapidxml::xml_node<>* xmlSuccessor = xmlRoad->first_node("successor");
		attr = xmlSuccessor->first_attribute("successor_Id");
		while (attr != NULL)
		{
			int successorID = -1;
			successorID = atoi(attr->value());
			roadTmp.successorId.push_back(successorID);
			attr = attr->next_attribute("successor_Id");
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
void Map::neighborLaneSort() {
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
void Map::moduleSelfCheck() {
	int flag = 0;
	for (int i = 0; i < roads.size(); i++)
	{
		if (roads[i].id < 0) { std::cout << "第" << i << "条路段id为" << roads[i].id << ",id应大于等于0，请检查" << std::endl; flag++; }
		for (int j = 0; j < roads[i].successorId.size(); j++)
		{
			if (roads[i].successorId[j] < 1) { std::cout << "第" << i << "条路段id为" << roads[i].id << ",后继id为" << roads[i].successorId[j] << ",后继路段id应大于0，请检查" << std::endl; flag++; }
		}
		for (int j = 0; j < roads[i].lanes.size(); j++)
		{
			if (roads[i].lanes[j].id < 0) { std::cout << "该路段第" << j << "条车道id:" << roads[i].lanes[j].id << "，车道id应大于等于0，请检查" << std::endl; flag++; }
			for (int k = 0; k < roads[i].lanes[j].successorId.size(); k++)
			{
				if (roads[i].lanes[j].successorId[k].sucRoadID < 1 || roads[i].lanes[j].successorId[k].sucLaneID < 0)
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
void Map::moduleSelfCheckPrint() {
	std::cout << "地图共有" << roads.size() << "条路段" << std::endl;
	for (int i = 0; i < roads.size(); i++)
	{
		std::cout << "路段id：" << roads[i].id << std::endl;
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
			/*for (int k = 0; k < roads[i].lanes[j].gaussRoadPoints.size(); k++)
			{
				std::cout << "gaussX:" << roads[i].lanes[j].gaussRoadPoints[k].GaussX << ";";
				std::cout << "gaussY:" << roads[i].lanes[j].gaussRoadPoints[k].GaussY << ";";
				std::cout << "yaw:" << roads[i].lanes[j].gaussRoadPoints[k].yaw << ";";
				std::cout << "curvature::" << roads[i].lanes[j].gaussRoadPoints[k].curvature << std::endl;
			}*/
			
		}
	}
}


//从大地图中挖出一个小地图，输入t是整个大地图，Map类的对象，x和y是小地图中心坐标，r是半径范围
//此函数将大地图里所有在中心坐标半径范围内的路段都保留，重新构建一个新的地图，输出是一个Map类对象，包含范围内所有路段
Map getSmallMap(const Map&t,double x,double y, double r) {
	Map smallMap;
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


