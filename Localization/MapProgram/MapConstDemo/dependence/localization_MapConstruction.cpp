//地图构建

#include<iostream>
#include<vector>
#include<string.h>
#include <fstream>
#include<cfloat>
#include <algorithm>
//#include <windows.h>
#include<cmath>
#include<iomanip>
#include<sstream>
//#include<io.h>
#include <dirent.h>
#include "rapidxml.hpp"
#include "rapidxml_print.hpp"
#include "localization_MapConstruction.h"

#define pi 3.14159265358979323846
#define driftingPointDeletionThreshold 25//drifting point deletion threshold
#define interpolationThreshold 0.25// interpolation threshold
#define offset 0

#define MIN(a,b) (a<b)?a:b

//路点采集的txt命名规则为RaLb，其中a为路段id，b为车道id，比如R2L1就是说这些路点数据所在的这条车道在id为2的路段上，车道id为1
//输入t是形式为RaLb的字符串，getRoadId返回a对应的int数据，也就是路段id
int getRoadId(char* t) {
	std::string id;
	int cnt = 1;
	int ant = int(t[cnt]);
	while (ant > 47 && ant < 58)
	{
		id += t[cnt];
		cnt++;
		ant = int(t[cnt]);
	}
	int roadID=atoi(id.c_str());
	return roadID;
}

//输入t是形式为RaLb的字符串，getLaneId返回b对应的int数据，也就是车道id
int getLaneId(char* t) {
	std::string id;
	int cnt = 1;
	while (t[cnt] != 'L') { cnt++; }
	cnt++;
	int ant = int(t[cnt]);
	while (ant > 47 && ant < 58)
	{
		id += t[cnt];
		cnt++;
		ant = int(t[cnt]);
	}
	int laneID=atoi(id.c_str());
	return laneID;
}



//判断一个数是否在vector中，在返回true，不在返回false
bool isInArray(std::vector<int>laneChangeAllowed, int roadId) {
	bool inArray = false;
	for (int i = 0; i < laneChangeAllowed.size(); i++)
	{
		if (laneChangeAllowed[i] == roadId)
		{
			inArray = true;
			break;
		}
	}
	return inArray;
}

//bool数据转换成string类型的数据
std::string boolToString(bool s) {
	std::string boolString;
	if (s == true) { boolString = "true"; }
	else if (s == false) { boolString = "false"; }
	return boolString;
}

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
std::string doubleToString(double s,int decimalPlaces) {
	std::stringstream ss;
	std::string st;
	ss << std::fixed << std::setprecision(decimalPlaces) << s;
	ss >> st;
	return st;
}



//算向量（x，y）的二范数，即sqrt（x*x+y*y）
double norm2(double x, double y) {
	return sqrt(x * x + y * y);
}

//算两向量夹角，返回角度制，比如返回69代表69°
double vectorAngleDeg(double vtr1X, double vtr1Y, double vtr2X, double vtr2Y) {
	double rad = acos((vtr1X * vtr2X + vtr1Y * vtr2Y) / norm2(vtr1X, vtr1Y) / norm2(vtr2X, vtr2Y));
	double deg = rad / pi * 180;
	if (vtr1X * vtr2Y - vtr1Y * vtr2X > 0){deg = -1 * deg;}
	return deg;
}

//将经纬度坐标转换成高斯坐标
std::vector<double>gaussXY(double longitude, double latitude) {
	std::vector<double>XY;
	double a = 6378137.0;
	double e2 = 0.0066943799013;
	double latitude2Rad = (pi / 180.0) * latitude;
	int beltNo = floor((longitude + 1.5) / 3.0);
	int L = beltNo * 3;
	double l0 = longitude - L;
	double tsin = sin(latitude2Rad);
	double tcos = cos(latitude2Rad);
	double t = tan(latitude2Rad);
	double m = (pi / 180.0) * l0 * tcos;
	double et2 = e2 * tcos * tcos;
	double et3 = e2 * tsin * tsin;
	double X = 111132.9558 * latitude - 16038.6496 * sin(2 * latitude2Rad) + 16.8607 * sin(4 * latitude2Rad) - 0.0220 * sin(6 * latitude2Rad);
	double N = a / sqrt(1 - et3);
	double y1 = X + N * t * (0.5 * m * m + (5.0 - t * t + 9.0 * et2 + 4 * et2 * et2) * pow(m,4) / 24.0 + (61.0 - 58.0 * t * t + pow(t,4)) *pow(m,6) / 720.0);
	double x1 = 500000 + N * (m + (1.0 - pow(t,2) + et2) * pow(m, 3) / 6.0 + (5.0 - 18.0 * pow(t, 2) + pow(t, 4) + 14.0 * et2 - 58.0 * et2 * pow(t, 2)) * pow(m, 5) / 120.0);
	XY.push_back(x1);
	XY.push_back(y1);
	return XY;
}

//插值函数，两相邻路点过远则进行插值增加路点
void interpolatePath(std::vector<double>path0X, std::vector<double>path0Y, double threshold, std::vector<GaussRoadPoint>& gaussRoadPoints,int id) {
	GaussRoadPoint t = { id,path0X[0],path0Y[0],0.0,0.0 };
	gaussRoadPoints.push_back(t);
	double vtrX, vtrY;
	int cnt = 0;
	for (int i = 1; i < path0X.size(); i++)
	{
		vtrX = path0X[i] - gaussRoadPoints[cnt].GaussX;
		vtrY = path0Y[i] - gaussRoadPoints[cnt].GaussY;
		double angle = atan2(vtrY, vtrX);
		double dis = norm2(vtrX, vtrY);
		if (dis > threshold)
		{
			double dd = dis / ceil(dis / threshold);
			double ddd = floor(dis / threshold);
			for (int j = 0; j < ddd; j++)
			{
				cnt++;
				double gaussX = gaussRoadPoints[cnt - 1].GaussX + dd * cos(angle);
				double gaussY = gaussRoadPoints[cnt - 1].GaussY + dd * sin(angle);
				GaussRoadPoint tt = { id,gaussX,gaussY,0.0,0.0 };
				gaussRoadPoints.push_back(tt);
			}
		}
		cnt++;
		GaussRoadPoint ttt = { id,path0X[i],path0Y[i],0.0,0.0 };
		gaussRoadPoints.push_back(ttt);
	}
}



//重载输入函数，方便从txt文件读取路点数据到Lane类的对象中
std::istream& operator>>(std::istream& input, Lane& t) {
	int id,type;
	double longitude, latitude, yaw,speed;
	input >> id;
	input >> longitude;
	input >> latitude;
	input >> yaw;
	input >> type;
	input >> speed;
	if (id > 0) {
		RawRoadPoint point = { longitude ,latitude,yaw };
		t.rawRoadPoints.push_back(point);
	}
	return input;
}

//将原始经纬度路点进行漂移点删除、坐标转换、插值等过程得到处理后的高斯坐标路点，此时的高斯路点x，y坐标全部减去x_min和y_min方便进行计算
void Lane::getGaussRoadPoint() {
std::vector<RawRoadPoint>deleteRepeatedPoints;
std::vector<GaussRoadPoint>gaussPath;
std::vector<double>gaussPathXY;
globalOffsetX = DBL_MAX, globalOffsetY = DBL_MAX;
double vrt1X = 0.0, vrt1Y = 0.0, vrt2X = 0.0, vrt2Y = 0.0;
int cnt = 1, st = 0;
if (rawRoadPoints.size() > 1)
{
	//std::cout << " rawRoadPoints.size():" << rawRoadPoints.size() << std::endl;
	for (int i = 1; i < rawRoadPoints.size(); i++)
	{
		if (rawRoadPoints[i].latitude != rawRoadPoints[i - 1].latitude || rawRoadPoints[i].longitude != rawRoadPoints[i - 1].longitude)
		{
			if (rawRoadPoints[i].latitude < 90 && rawRoadPoints[i].longitude < 180)
			{
				RawRoadPoint t = { rawRoadPoints[i].longitude,rawRoadPoints[i].latitude, rawRoadPoints[i].yaw };
				deleteRepeatedPoints.push_back(t);
			}
		}
	}
	//std::cout << " deleteRepeatedPoints.size():" << deleteRepeatedPoints.size() << std::endl;
	for (int i = 0; i < deleteRepeatedPoints.size(); i++)
	{
		//std::cout << "deleteRepeatedPoints[i].longitude:" << deleteRepeatedPoints[i].longitude << "   deleteRepeatedPoints[i].latitude:" << deleteRepeatedPoints[i].latitude << std::endl;
		gaussPathXY = gaussXY(deleteRepeatedPoints[i].longitude, deleteRepeatedPoints[i].latitude);
		//std::cout << "gaussPathX:" << gaussPathXY[0] << "   gaussPathY:" << gaussPathXY[1] << std::endl;
		GaussRoadPoint t = { id ,gaussPathXY[0], gaussPathXY[1], deleteRepeatedPoints[i].yaw,0.0 };
		gaussPath.push_back(t);
		globalOffsetX = MIN(globalOffsetX, gaussPathXY[0]);
		globalOffsetY = MIN(globalOffsetY, gaussPathXY[1]);
	}
	//std::cout << " gaussPath.size():" << gaussPath.size() << std::endl;
	//std::cout << "globalOffsetX:" << globalOffsetX << "   globalOffsetY:" << globalOffsetY << std::endl;
	for (int i = 0; i < gaussPath.size(); i++)
	{
		gaussPath[i].GaussX -= globalOffsetX;
		gaussPath[i].GaussY -= globalOffsetY;
		//std::cout <<"gaussPath[i].GaussX:" << gaussPath[i].GaussX <<"   gaussPath[i].GaussY:" << gaussPath[i].GaussY << std::endl;
	}
	double norm = norm2(gaussPath[st + 1].GaussX - gaussPath[st].GaussX, gaussPath[st + 1].GaussY - gaussPath[st].GaussY);
	//std::cout << "norm:" << norm << std::endl;
	while (norm < 0.08)
	{
		st = st + 1;
		if (st > gaussPath.size() - 2) { st = 1; break; }
		else { norm = norm2(gaussPath[st + 1].GaussX - gaussPath[st].GaussX, gaussPath[st + 1].GaussY - gaussPath[st].GaussY); }
	}
	std::vector<double>path0X, path0Y;
	//std::cout << "st:" << st << std::endl;
	path0X.push_back(gaussPath[st].GaussX);
	path0Y.push_back(gaussPath[st].GaussY);
	path0X.push_back(gaussPath[st + 1].GaussX);
	path0Y.push_back(gaussPath[st + 1].GaussY);
	//std::cout << "path0:" << path0X[0] << "  " << path0Y[0] << "  " << path0X[1] << "  " << path0Y[1] << std::endl;
	for (int i = st + 2; i < gaussPath.size(); i++)
	{
		vrt1X = gaussPath[i].GaussX - path0X[cnt];
		vrt1Y = gaussPath[i].GaussY - path0Y[cnt];
		vrt2X = path0X[cnt] - path0X[cnt - 1];
		vrt2Y = path0Y[cnt] - path0Y[cnt - 1];
		double deg = abs(vectorAngleDeg(vrt1X, vrt1Y, vrt2X, vrt2Y));
		double normVtr1 = norm2(vrt1X, vrt1Y);
		if (deg < driftingPointDeletionThreshold && normVtr1>interpolationThreshold * 0.7)
		{
			cnt++;
			path0X.push_back(gaussPath[i].GaussX);
			path0Y.push_back(gaussPath[i].GaussY);
		}
	}
	//std::cout << " path0X.size():" << path0X.size() << std::endl;
	interpolatePath(path0X, path0Y, interpolationThreshold, gaussRoadPoints, id);
	//std::cout << " gaussRoadPoints.size():" << gaussRoadPoints.size() << std::endl;
}
else { std::cout << "路点少于两个，严重不足！"; }
}

//根据曲率信息计算限速，单位m/s
void Lane::calcSpeedProfile() {
	double topSpeed = 60 / 3.6;
	double straightCurLmt = 0.02;
	double turnDisThreshold = 5;
	double ayLimit = 2;
	double accLimit = 8;
	double dccLimit = 1;
	std::vector<double>refpathCur, turnMaxCur;
	std::vector<int>turnMaxIdx;
	for (int i = 0; i < gaussRoadPoints.size(); i++) { refpathCur.push_back(fabs(gaussRoadPoints[i].curvature)); }
	
	for (int i = 0; i < refpathCur.size(); i++)
	{
		if (refpathCur[i] > straightCurLmt && (i == 0 || refpathCur[i - 1] <= straightCurLmt))
		{
			int j = i;
			while (j < gaussRoadPoints.size())
			{
				if (j == gaussRoadPoints.size() - 1 || refpathCur[j + 1] < straightCurLmt)
				{
					if (gaussRoadPoints[j].s - gaussRoadPoints[i].s < turnDisThreshold) { break; }
					std::vector<double>::iterator maxFront = std::max_element(std::begin(refpathCur) + i, (std::begin(refpathCur) + j + 1));
					int idx = std::distance(std::begin(refpathCur), maxFront);
					turnMaxCur.push_back(*maxFront);
					turnMaxIdx.push_back(idx);
					break;
				}
				else { j++; }
			}
		}
	}

	std::vector<double>spdProf(refpathCur.size(), topSpeed);
	for (int i = 0; i < turnMaxCur.size(); i++)
	{
		int j = turnMaxIdx[i];
		spdProf[j] = sqrt(ayLimit / turnMaxCur[i]);
	}
	for (int i = 0; i < turnMaxCur.size(); i++)
	{
		if (i < turnMaxCur.size() - 1)
		{
			for (int j = turnMaxIdx[i] + 1; j <= turnMaxIdx[i + 1]; j++)
			{
				double ds = gaussRoadPoints[j].s - gaussRoadPoints[j - 1].s;
				spdProf[j] = MIN(spdProf[j], sqrt(2 * accLimit * ds + spdProf[j - 1]* spdProf[j - 1]));
				if(refpathCur[j] >= straightCurLmt){ spdProf[j] = MIN(spdProf[j], sqrt(ayLimit / refpathCur[j])); }
			}
			for (int k = turnMaxIdx[i + 1] - 1; k >= turnMaxIdx[i]; k--)
			{
				double ds = gaussRoadPoints[k + 1].s - gaussRoadPoints[k].s;
				spdProf[k] = MIN(spdProf[k], sqrt(2 * dccLimit * ds + spdProf[k + 1]* spdProf[k + 1]));
				if(refpathCur[k]>= straightCurLmt){ spdProf[k] = MIN(spdProf[k], sqrt(ayLimit / refpathCur[k])); }
			}
		}
		if (i == 0)
		{
			int j = turnMaxIdx[i] - 1;
			while (j >= 0) 
			{
				double ds= gaussRoadPoints[j + 1].s - gaussRoadPoints[j].s;
				spdProf[j] = MIN(spdProf[j], sqrt(2 * dccLimit * ds + spdProf[j + 1]* spdProf[j + 1]));
				if(refpathCur[j]>= straightCurLmt){ spdProf[j] = MIN(spdProf[j], sqrt(ayLimit / refpathCur[j])); }
				j--;
			}
		}
		if (i == turnMaxCur.size() - 1)
		{
			int j = turnMaxIdx[i] + 1;
			while (j < refpathCur.size())
			{
				double ds = gaussRoadPoints[j].s - gaussRoadPoints[j-1].s;
				spdProf[j] = MIN(spdProf[j], sqrt(2 * accLimit * ds + spdProf[j - 1] * spdProf[j - 1]));
				if (refpathCur[j] >= straightCurLmt) { spdProf[j] = MIN(spdProf[j], sqrt(ayLimit / refpathCur[j])); }
				j++;
			}
		}
	}
	//for (int i = 0; i < refpathCur.size(); i++) {std::cout<<spdProf[i]<<" "; }
	for (int i = 0; i < refpathCur.size(); i++) { gaussRoadPoints[i].speedLimit = MIN(spdProf[i], topSpeed); }
}

//根据每个路点的高斯坐标计算每个路点的航向角yaw和曲率curvature
void Lane::getYawCruvature() {
	int size = gaussRoadPoints.size();
	std::vector<double>refpathDis(size, 0), refhdgDlt(size, 0), refhdgDltSum(size, 0);
	for (int i = 0; i < size - 1; i++)
	{
		double x = gaussRoadPoints[i + 1].GaussX - gaussRoadPoints[i].GaussX;
		double y = gaussRoadPoints[i + 1].GaussY - gaussRoadPoints[i].GaussY;
		gaussRoadPoints[i].yaw = (atan2(y, x) * 180 / pi - 90)*(-1);
		if (gaussRoadPoints[i].yaw < 0){gaussRoadPoints[i].yaw += 360;}
		refpathDis[i + 1] = refpathDis[i] + norm2(x, y);
	}
	gaussRoadPoints[size - 1].yaw = gaussRoadPoints[size - 2].yaw;
	for (int i = 0; i < size; i++) { gaussRoadPoints[i].s = refpathDis[i]; }
	for (int i = 1; i < size - 1; i++)
	{
		refhdgDlt[i] = gaussRoadPoints[i].yaw - gaussRoadPoints[i - 1].yaw;
		if(refhdgDlt[i] > 180){ refhdgDlt[i] = refhdgDlt[i] - 360; }
		else if(refhdgDlt[i] < -180){ refhdgDlt[i] = refhdgDlt[i] + 360; }
		refhdgDltSum[i] = refhdgDltSum[i-1] + refhdgDlt[i-1];
	}
	refhdgDltSum[size - 1] = refhdgDltSum[size - 2];

	int num = 20 / 0.25;
	double delayRate = 0.5;
	int st,ed;
	for (int i = 0; i < size - 1; i++)
	{
		st = i;
		ed = MIN(i + num, size-1);
		gaussRoadPoints[i].curvature = (refhdgDltSum[ed] - refhdgDltSum[st]) / (refpathDis[ed] - refpathDis[st]) * pi / 180;
		if (i > 0){ gaussRoadPoints[i].curvature = gaussRoadPoints[i - 1].curvature * (1 - delayRate) + gaussRoadPoints[i].curvature * delayRate;}
	}
	gaussRoadPoints[size - 1].curvature = gaussRoadPoints[size - 2].curvature;
}

//高斯坐标修正，包括加上x_min和y_min进行还原，以及考虑车载天线和车辆中心线的偏移
void Lane::corectGaussXY() {
	double x, y;
	for (int i = 0; i < gaussRoadPoints.size(); i++)
	{
		double yaw = gaussRoadPoints[i].yaw / 180 * pi;
		x = gaussRoadPoints[i].GaussY + globalOffsetY + offset * sin(yaw);
		y = gaussRoadPoints[i].GaussX + globalOffsetX - offset * cos(yaw);
		gaussRoadPoints[i].GaussX = x;
		gaussRoadPoints[i].GaussY = y;
		gaussRoadPoints[i].curvature = gaussRoadPoints[i].curvature * 180 / pi;
	}
}



//获取所有路点txt文件的文件名，path为所有txt文件所在的文件夹路径
void MapConstruction::getAllFileNames(std::string path) {
	DIR* pDir;
	struct dirent* ptr;
	if (!(pDir = opendir(path.c_str()))) {
		std::cout << "Folder doesn't Exist!" << std::endl;
		return;
	}
	while ((ptr = readdir(pDir)) != NULL) {
		if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0) {
			filenames.push_back(path + "/" + ptr->d_name);
			int rID = getRoadId(ptr->d_name);
			roadId.push_back(rID);
			int lID = getLaneId(ptr->d_name);
			laneId.push_back(lID);
		}
	}
	closedir(pDir);
	/*for (int i = 0; i < roadId.size(); i++)
	{
		std::cout <<"roadId[i]:" << roadId[i] << std::endl;
		std::cout << "LaneId[i]:" << laneId[i] << std::endl;
	}*/
	
}

//加载所有原始经纬度路点数据并储存，path为所有txt文件所在的文件夹路径，path输入后作为getAllFileNames函数的输入
void MapConstruction::loadRawRoadPoint(std::string path) {
	MapConstruction::getAllFileNames(path);
	for (int i = 0; i < filenames.size(); i++)
	{
		std::ifstream in(filenames[i].c_str(), std::ios::in);
		if (in) 
		{
			Lane t;
			while (!in.eof()) 
			{
				if (in >> t) {}
			}
			t.updateId(laneId[i]);
			int flag = 0;
			for (int j=0; j < roads.size(); j++)
			{
				if (roadId[i] == roads[j].id) { roads[j].lanes.push_back(t); flag = 1; break; }
			}
			if (flag==0)
			{
				Road tmp;
				tmp.updateId(roadId[i]);
				tmp.lanes.push_back(t);
				roads.push_back(tmp);
			}
		}
		in.close();
	}
}

//对原始经纬度路点数据进行处理，对每条车道执行getGaussRoadPoint，getYawCruvature，corectGaussXY三个函数
void MapConstruction::processRawRoadPoint() {
	for (int i = 0; i < roads.size(); i++)
	{
		for (int j = 0; j < roads[i].lanes.size(); j++)
		{
			roads[i].lanes[j].getGaussRoadPoint();
			roads[i].lanes[j].getYawCruvature();
			roads[i].lanes[j].calcSpeedProfile();
			roads[i].lanes[j].corectGaussXY();
		}
		
	}
}

//加载路段后继路段id的txt文件并储存内容，path为该文件的路径
void MapConstruction::loadSuccessor(std::string path) {
	std::ifstream in(path.c_str(), std::ios::in);
	if (in)
	{
		int id, sucID;
		while (in >> id&&in >> sucID)
		{
			if (id >= 0) {
				for (int i = 0; i < roads.size(); i++)
				{
					if (id == roads[i].id) { roads[i].successorId.push_back(sucID); break; }
				}
			}
		}
	}
	in.close();
}

//加载车道相邻车道的id的txt文件并储存内容，path为该文件的路径
void MapConstruction::loadLaneNeighborId(std::string path) {
	std::ifstream in(path.c_str(), std::ios::in);
	if (in)
	{
		int roadID, laneNum;
		while (in >> roadID && in >> laneNum)
		{
			for (int i = 0; i < roads.size(); i++)
			{
				if (roadID == roads[i].id)
				{
					for (int j = 0; j < roads[i].lanes.size(); j++)
					{
						int leftLaneID = laneNum - 1;
						while (leftLaneID > roads[i].lanes[j].id) 
						{
							roads[i].lanes[j].leftLaneId.push_back(leftLaneID);
						    leftLaneID--; 
						}
						int rightLaneID = 0;
						while(rightLaneID<roads[i].lanes[j].id)
						{
							roads[i].lanes[j].rightLaneId.push_back(rightLaneID);
							rightLaneID++;
						}
					}
					break;
				}
			}
		}
	}
	in.close();
}

//加载车道后继车道的id的txt文件并储存内容，path为该文件的路径
void MapConstruction::loadLaneSuccessorId(std::string path) {
	std::ifstream in(path.c_str(), std::ios::in);
	if (in)
	{
		int roadID, laneID,sucRoadID,sucLaneID;
		while (in >> roadID && in >> laneID && in >> sucRoadID && in >> sucLaneID)
		{
			for (int i = 0; i < roads.size(); i++)
			{
				if (roadID == roads[i].id)
				{
					for (int j = 0; j < roads[i].lanes.size(); j++)
					{
						if(laneID== roads[i].lanes[j].id)
						{
							LaneSuccessorId t = { sucRoadID ,sucLaneID };
							roads[i].lanes[j].successorId.push_back(t);
							break;
						}
					}
					break;
				}
			}
		}
	}
	in.close();
}

//加载不能换道的路段id的txt文件并储存内容，path为该文件的路径
void MapConstruction::loadLaneChange(std::string path) {
	std::ifstream in(path.c_str(), std::ios::in);
	std::vector<int>laneChangeAllowed;
	if (in)
	{
		int roadID;
		while (in >> roadID)
		{
			laneChangeAllowed.push_back(roadID);
		}
	}
	for (int i = 0; i < roads.size(); i++)
	{
		bool inArray = isInArray(laneChangeAllowed, roads[i].id);
		if (inArray == true) { roads[i].isLaneChange = 0; }
		else if (inArray == false) { roads[i].isLaneChange = 1; }
	}
	in.close();
}

//将处理好的地图数据转化成xml语言能够储存的string类型的数据
void MapConstruction::getXMLInfo() {
	for (int i = 0; i < roads.size(); i++)
	{
		roads[i].idString = intToString(roads[i].id);
		roads[i].isLaneChangeString = boolToString(roads[i].isLaneChange);
		for (int k = 0; k < roads[i].successorId.size(); k++)
		{
			std::string t = intToString(roads[i].successorId[k]);
			roads[i].successorIdString.push_back(t);
		}
		for (int j = 0; j < roads[i].lanes.size(); j++) 
		{
			roads[i].lanes[j].idString = intToString(roads[i].lanes[j].id);
			for (int m = 0; m < roads[i].lanes[j].successorId.size(); m++)
			{
				roads[i].lanes[j].successorId[m].sucLaneIDString = intToString(roads[i].lanes[j].successorId[m].sucLaneID);
				roads[i].lanes[j].successorId[m].sucRoadIDString = intToString(roads[i].lanes[j].successorId[m].sucRoadID);
			}
			for (int u = 0; u < roads[i].lanes[j].leftLaneId.size(); u++)
			{
				std::string tt = intToString(roads[i].lanes[j].leftLaneId[u]);
				roads[i].lanes[j].leftLaneIdString.push_back(tt);
			}
			for (int v = 0; v < roads[i].lanes[j].rightLaneId.size(); v++)
			{
				std::string tt = intToString(roads[i].lanes[j].rightLaneId[v]);
				roads[i].lanes[j].rightLaneIdString.push_back(tt);
			}
			for (int n = 0; n < roads[i].lanes[j].gaussRoadPoints.size();n++)
			{
				std::vector<std::string>info(6);
				info[0] = doubleToString(roads[i].lanes[j].gaussRoadPoints[n].GaussX, 3);
				info[1] = doubleToString(roads[i].lanes[j].gaussRoadPoints[n].GaussY, 3);
				info[2] = doubleToString(roads[i].lanes[j].gaussRoadPoints[n].yaw, 2);
				info[3] = doubleToString(roads[i].lanes[j].gaussRoadPoints[n].curvature, 2);
				info[4] = doubleToString(roads[i].lanes[j].gaussRoadPoints[n].s, 2);
				info[5] = doubleToString(roads[i].lanes[j].gaussRoadPoints[n].speedLimit, 2);
				roadPointInfo ttt = { info[0],info[1],info[2],info[3],info[4],info[5] };
				roads[i].lanes[j].roadPointString.push_back(ttt);
			}
		}
	}
}

//将处理好的地图数据储存进xodr文件，path是输出的xodr文件的路径
void MapConstruction::saveMapToXML(std::string path) {
	MapConstruction::getXMLInfo();
	MapConstruction::moduleSelfCheck();
	//MapConstruction::moduleSelfCheckPrint();
	rapidxml::xml_document<> doc;  //构造一个空的xml文档
	rapidxml::xml_node<>* rot = doc.allocate_node(rapidxml::node_pi, doc.allocate_string("setting.xml version='1.0' encoding='utf-8'"));//allocate_node分配一个节点，该节点类型为node_pi，对XML文件进行描，描述内容在allocate_string中
	doc.append_node(rot); //把该节点添加到doc中

	rapidxml::xml_node<>* map = doc.allocate_node(rapidxml::node_element, "MAP", NULL);
	int cnt = 0;
	for (int i = 0; i < roads.size(); i++)
	{
		rapidxml::xml_node<>* road = doc.allocate_node(rapidxml::node_element, "road", NULL);
		road->append_attribute(doc.allocate_attribute("id", roads[i].idString.c_str()));
		road->append_attribute(doc.allocate_attribute("isLaneChange", roads[i].isLaneChangeString.c_str()));
		map->append_node(road);
		

		//rapidxml::xml_node<>* successor = doc.allocate_node(rapidxml::node_element, "successor", NULL);
		//road->append_node(successor);
		for (int k = 0; k < roads[i].successorIdString.size(); k++)
		{
			rapidxml::xml_node<>* successor = doc.allocate_node(rapidxml::node_element, "successor", NULL);
			successor->append_attribute(doc.allocate_attribute("successor_roadId", roads[i].successorIdString[k].c_str()));
			road->append_node(successor);
		}


		rapidxml::xml_node<>* lanes = doc.allocate_node(rapidxml::node_element, "lanes", NULL);
		road->append_node(lanes);
		for (int j = 0; j < roads[i].lanes.size(); j++)
		{
			rapidxml::xml_node<>* lane = doc.allocate_node(rapidxml::node_element, "lane", NULL);
			lanes->append_node(lane);
			lane->append_attribute(doc.allocate_attribute("id", roads[i].lanes[j].idString.c_str()));
			
			for (int u = 0; u < roads[i].lanes[j].successorId.size(); u++)
			{
				rapidxml::xml_node<>* laneSuccessor = doc.allocate_node(rapidxml::node_element, "successor", NULL);
				lane->append_node(laneSuccessor);
				laneSuccessor->append_attribute(doc.allocate_attribute("successor_roadId", roads[i].lanes[j].successorId[u].sucRoadIDString.c_str()));
				laneSuccessor->append_attribute(doc.allocate_attribute("successor_laneId", roads[i].lanes[j].successorId[u].sucLaneIDString.c_str()));
			}
			for (int v = 0; v < roads[i].lanes[j].leftLaneId.size(); v++)
			{
				rapidxml::xml_node<>* leftLaneID = doc.allocate_node(rapidxml::node_element, "leftLaneID", NULL);
				lane->append_node(leftLaneID);
				leftLaneID->append_attribute(doc.allocate_attribute("id", roads[i].lanes[j].leftLaneIdString[v].c_str()));
			}
			for (int v = 0; v < roads[i].lanes[j].rightLaneId.size(); v++)
			{
				rapidxml::xml_node<>* rightLaneID = doc.allocate_node(rapidxml::node_element, "rightLaneID", NULL);
				lane->append_node(rightLaneID);
				rightLaneID->append_attribute(doc.allocate_attribute("id", roads[i].lanes[j].rightLaneIdString[v].c_str()));
			}
			rapidxml::xml_node<>* roadPoints = doc.allocate_node(rapidxml::node_element, "roadPoints", NULL);
			lane->append_node(roadPoints);
			for (int n = 0; n < roads[i].lanes[j].gaussRoadPoints.size(); n++)
			{
				rapidxml::xml_node<>* roadPoint = doc.allocate_node(rapidxml::node_element, "roadPoint", NULL);
				roadPoints->append_node(roadPoint);
				roadPoint->append_attribute(doc.allocate_attribute("gaussX", roads[i].lanes[j].roadPointString[n].GaussX.c_str()));
				roadPoint->append_attribute(doc.allocate_attribute("gaussY", roads[i].lanes[j].roadPointString[n].GaussY.c_str()));
				roadPoint->append_attribute(doc.allocate_attribute("yaw", roads[i].lanes[j].roadPointString[n].yaw.c_str()));
				roadPoint->append_attribute(doc.allocate_attribute("curvature", roads[i].lanes[j].roadPointString[n].curvature.c_str()));
				roadPoint->append_attribute(doc.allocate_attribute("s", roads[i].lanes[j].roadPointString[n].s.c_str()));
				roadPoint->append_attribute(doc.allocate_attribute("speedMax", roads[i].lanes[j].roadPointString[n].speedLimit.c_str()));
			}
		}
	}

	doc.append_node(map);
	std::ofstream pout(path);
	pout << doc;
}



//自我验证函数，检查本模块计算输出是否满足数据格式、范围要求
void MapConstruction::moduleSelfCheck() {
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
					std::cout << "该路段第" << j << "条车道后继车道:" << roads[i].lanes[j].successorId[k].sucRoadID << "号路段，" << roads[i].lanes[j].successorId[k].sucLaneID << "号车道" << ",路段id应大于等于0，车道id应大于等于0，请检查" << std::endl;
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
	if (flag == 0) { std::cout << "地图构建完毕，数据读入正常" << std::endl; }
}

//输出打印显示函数，打印本模块的关键参数及输出
void MapConstruction::moduleSelfCheckPrint() {
	std::cout << "地图共有" << roads.size() << "条路段" << std::endl;
	for (int i = 0; i < roads.size(); i++)
	{
		std::cout << "路段id：" << roads[i].id << std::endl;
		for (int j = 0; j < roads[i].successorId.size(); j++)
		{
			std::cout << "后继路段id：" << roads[i].successorId[j] << std::endl;
		}
		std::cout << "该路段有" << roads[i].lanes.size() << "条车道" << std::endl;
		for (int j = 0; j < roads[i].lanes.size(); j++)
		{
			std::cout << "该路段第" << j << "条车道id:" << roads[i].lanes[j].id << std::endl;
			for (int k = 0; k < roads[i].lanes[j].successorId.size(); k++)
			{
				std::cout << "该路段第" << j << "条车道后继车道:" << roads[i].lanes[j].successorId[k].sucRoadID << "号路段，" << roads[i].lanes[j].successorId[k].sucLaneID << "号车道" << std::endl;
			}
			for (int k = 0; k < roads[i].lanes[j].leftLaneId.size(); k++)
			{
				std::cout << "该路段第" << j << "条车道左相邻车道id:" << roads[i].lanes[j].leftLaneId[k] << std::endl;
			}
			for (int k = 0; k < roads[i].lanes[j].rightLaneId.size(); k++)
			{
				std::cout << "该路段第" << j << "条车道右相邻车道id:" << roads[i].lanes[j].rightLaneId[k] << std::endl;
			}
			std::cout << "该路段第" << j << "条车道路点数量:" << roads[i].lanes[j].gaussRoadPoints.size() << std::endl;
			for (int k = 0; k < roads[i].lanes[j].gaussRoadPoints.size(); k++)
			{
				std::cout << "gaussX:" << roads[i].lanes[j].gaussRoadPoints[k].GaussX << ";";
				std::cout << "gaussY:" << roads[i].lanes[j].gaussRoadPoints[k].GaussY << ";";
				std::cout << "yaw:" << roads[i].lanes[j].gaussRoadPoints[k].yaw << ";";
				std::cout << "curvature::" << roads[i].lanes[j].gaussRoadPoints[k].curvature << std::endl;
			}

		}
	}
}

//void GetFileNames(std::string path, std::vector<std::string>& filenames)
//{
//	DIR* pDir;
//	struct dirent* ptr;
//	if (!(pDir = opendir(path.c_str()))) {
//		std::cout << "Folder doesn't Exist!" << std::endl;
//		return;
//	}
//	while ((ptr = readdir(pDir)) != NULL) {
//		if (strcmp(ptr->d_name, ".") != 0  && strcmp(ptr->d_name, "..") != 0) {
//			filenames.push_back(path + "/" + ptr->d_name);
//		}
//	}
//	closedir(pDir);
//}

//void GetAllFiles(std::string path, std::vector<std::string>& files)
//{
//
//	intptr_t  hFile = 0;
//	//文件信息  
//	struct _finddata_t fileinfo;
//	std::string p;
//	if ((hFile = _findfirst(p.assign(path).append("\\*").c_str(), &fileinfo)) != -1)
//	{
//		do
//		{
//			if ((fileinfo.attrib & _A_SUBDIR))
//			{
//				if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
//				{
//					files.push_back(p.assign(path).append("\\").append(fileinfo.name));
//					GetAllFiles(p.assign(path).append("\\").append(fileinfo.name), files);
//				}
//			}
//			else
//			{
//				files.push_back(p.assign(path).append("\\").append(fileinfo.name));
//			}
//
//		} while (_findnext(hFile, &fileinfo) == 0);
//
//		_findclose(hFile);
//	}
//
//}
