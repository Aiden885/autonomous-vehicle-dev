#include "dijkstra_topologyMap.h"
#include <math.h>
using namespace std;

Astar::Astar() {
	roadList = new road[size];
	for (int i = 0; i < size; i++) {
		roadList[i].number = i;
	}
};


Astar::~Astar() {
	if (roadList != NULL) {
		delete[]roadList;
		roadList = NULL;
		cout << "调用了析构函数\n";
	}
}

std::vector<int> Astar::getNextRoad(int x)
{
	return roadList[x].to;
}

double Astar::calcH(int temp, int destination)
{
	return 0;
}

double Astar::calcG(int temp, int from)
{
	return roadList[from].G + roadList[temp].length;
}

double Astar::calcF(int temp)
{
	return roadList[temp].G + roadList[temp].H;
}

int Astar::getLeastF()
{
	if (!openList.empty())
	{
		auto resPoint = openList.top();
		return resPoint.number;
	}
	cout << "getLeastF : wrong openList为空无法进一步寻找";//
	//因为套在while循环里，!openList.empty()应该必成立
	//如果openList为空，错误应该会被先归类到 "findPath : wrong 无法找到合理的道路" 中
}

road* Astar::findPath(int origin, int destination)//Astar算法的主要部分
{
	roadList[origin].G = roadList[origin].length;
	openList.push(node(origin, roadList[origin].length, origin));//往openList里放入出发点
	while (!openList.empty())
	{
		cout << endl;
		int cur = getLeastF();
		cout << "加入closeList的边为 : " << cur << endl;
		openList.pop();
		roadList[cur].isInList = -1;
		for (auto& it : roadList[cur].to) {
			if (it == destination)//如果搜到了目的地可以直接结束
			{
				cout << "已搜索到目的地" << endl;
				roadList[destination].father = cur;
				return &roadList[destination];
			}

			//分不在list、在openlist、在closelist三种情况讨论
			int inList = roadList[it].isInList;
			cout << "到达 : " << it << "  状态 : " << inList;

			if (inList == 0)
			{
				roadList[it].isInList = 1;
				roadList[it].H = calcH(it, destination);
				roadList[it].G = calcG(it, cur);
				roadList[it].F = calcF(it);
				roadList[it].father = cur;
				openList.push(node(it, roadList[it].F, cur));
				cout << "  更新了F值 : " << roadList[it].F << endl;
				continue;
			}

			if (inList == 1)
			{
				double tempG = calcG(it, cur);
				if (tempG > roadList[it].G)continue;
				roadList[it].G = tempG;
				roadList[it].F = calcF(it);
				roadList[it].father = cur;
				openList.push(node(it, roadList[it].F, cur));
				cout << " 更新了F值 : " << roadList[it].F << endl;
				continue;
			}

			if (inList == -1) {
				cout << endl;
				continue;
			}

			cout << endl << "findPath : wrong isInList出现异常值 : " << inList << endl;
		}
	}
	cout << "findPath : wrong 无法找到合理的道路" << endl;
	return NULL;
}

std::list<int> Astar::getPath(int origin, int destination)
{
	std::list<int> path;
	if (roadList[origin].isInList == -2)
	{
		cout << "getPath wrong : 出发点的边未被初始化";
		return path;
	}
	if (roadList[destination].isInList == -2)
	{
		cout << "getPath wrong : 目的地的边未被初始化";
		return path;
	}
	cout << "使用A*算法寻找从 " << origin << " 到 " << destination << "的最短路" << endl;
	cout << "已经初始化的边有: ";
	for (int i = 0; i < size; i++) {
		if (roadList[i].isInList == 0)cout << i << " ";
	}
	cout << endl;

	if (origin == destination)
	{
		path.push_back(destination);
		return path;
	}

	road* result = findPath(origin, destination);
	int temp = result->father;
	path.push_back(destination);
	path.push_front(temp);
	while (temp != origin)
	{
		temp = roadList[temp].father;
		path.push_front(temp);
	}
	if (!openList.empty())openList.pop();
	return path;
}

void Astar::initRoad(int number, double xStart, double yStart, double xEnd, double yEnd, double length)
{
	if (number > size)
	{
		cout << "initRoad : wrong 编号过大，请修改size";
		return;
	}
	if (sqrt((xStart - xEnd) * (xStart - xEnd) + (yStart - yEnd) * (yStart - yEnd)) - 0.000001 > length)
	{
		cout << "initRoad : wrong 编号为" << number << "的边长度不合理" << endl;
	}

	roadList[number].xBegin = xStart;
	roadList[number].xEnd = xEnd;
	roadList[number].yBegin = yStart;
	roadList[number].yEnd = yEnd;
	roadList[number].length = length;
	roadList[number].isInList = 0;
}

void Astar::initLink(int from, int to)
{
	roadList[from].to.push_back(to);
}

void Astar::reset()
{
	for (int i = 0; i < size; i++) {
		roadList[i].F = 0;
		roadList[i].G = 0;
		roadList[i].H = 0;
		if (roadList[i].isInList != -2)roadList[i].isInList = 0;
	}
}

void Astar::moduleSelfCheck(list<int> path)
{
	for (auto& it : path)
	{
		if (it < 0 || it > size) {
			cout << endl << "moduleSelfCheck : wrong 输出不符合要求" << endl;
			return;
		}
	}
	cout << endl << "moduleSelfCheck : 输出符合要求" << endl;
}

void Astar::moduleSelfCheckPrint(vector<pair<int, int>> pathLane)
{
	for (auto it = pathLane.begin(); it != pathLane.end(); it++)
	{
		cout << (*it).first << "  " << (*it).second << endl;
	}
}

void Astar::mapToAstar(Map m, Astar* as)
{
	for (auto it = m.roads.begin();it != m.roads.end(); it++)
	{
		int number = it->id;
		double xStart = it->lanes.begin()->gaussRoadPoints.begin()->GaussX;
		double yStart = it->lanes.begin()->gaussRoadPoints.begin()->GaussY;
		double xEnd = it->lanes.begin()->gaussRoadPoints.rbegin()->GaussX;
		double yEnd = it->lanes.begin()->gaussRoadPoints.rbegin()->GaussY;
		double length = sqrt((xStart - xEnd) * (xStart - xEnd) + (yStart - yEnd) * (yStart - yEnd));
		(*as).initRoad(number, xStart, yStart, xEnd, yEnd, length);
		for (auto itSuccessor = it->successorId.begin();itSuccessor != it->successorId.end();itSuccessor++)
		{
			(*as).initLink(number, *itSuccessor);
		}
	}
}

void Astar::findLane(Map m, list<int> path)
{
	auto it = path.begin();
	auto itNext = path.begin();//用两个迭代器表示目前的road和下一个road，寻找目前的road中哪条lane能前往下一条道路的某条lane
	//再看看目前road的那条lane是不是已经在pathLanes中（如果不是说明要更换道路）
	itNext++;
	while (itNext != path.end())
	{
		auto itRoads = m.roads.begin();
		while ((*itRoads).id != *it)itRoads++;//找到*it对应id的Map::Road
		for (auto itLanes = (*itRoads).lanes.begin();itLanes != (*itRoads).lanes.end();itLanes++)//遍历Road的各条Lane
		{
			for (auto itSuccessor = (*itLanes).successorId.begin();itSuccessor != (*itLanes).successorId.end();itSuccessor++)
				//遍历Lane的各个Successor
				if ((*itSuccessor).sucRoadID == *itNext) {//如果有Successor的RoadId为下一个要前往的Road
					if ((*itLanes).id != pathLanes.back().second)
					{
						pathLanes.push_back(make_pair(*it, (*itLanes).id));
						//若该Lane不再pathLanes中就将其加入
					}
					pathLanes.push_back(make_pair(*itNext, (*itSuccessor).sucLaneID));
					goto outloop;
					//加入刚才找到的能到达的下一个Lane
				}
		}
	outloop:
		it++;
		itNext++;
	}
}
