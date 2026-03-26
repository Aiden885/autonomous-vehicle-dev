#include <math.h>
#include "dijkstra_latticeMap.h"


namespace planning{

void Astar::InitAstar(const std::vector<std::vector<int>>& _maze, const std::vector<std::vector<int>>& _hvalue)
{
	
	maze = _maze;
			
	hvalue = _hvalue;
}

int Astar::calcG(Point1* temp_start, Point1* point)
{
	
	/*int extraG1;
	if (point->y <= 168 || point->y >= 295 || 120 <= point->x && point->x <= 726) {
		if (point->x == 56 || point->y == 232 || point->x == 768)   extraG1 = 0;
		else extraG1 = 9;
	}
	else   extraG1 = 10;*/
	//if ((point->x - temp_start->x) != 0 && (point->y - end->y) != 0)   extraG1 = 50;
	//int extraG1 = ((point->x - temp_start->x) == 0 || (point->y - end->y) == 0) ? 0 : 10 ;
	int extraG1 = (abs(point->x - temp_start->x) + abs(point->y - temp_start->y)) == 1 ? kCost1 : kCost2;
	/*if ((abs(point->x - temp_start->x) + abs(point->y - temp_start->y)) == 1)    extraG1 = kCost1;
	else   extraG1 = kCost2;*/
		

	//int extraG = (abs(point->x - temp_start->x) + abs(point->y - temp_start->y)) == 1 ? kCost1 : kCost2; 
	int parentG = point->parent == NULL ? 0 : point->parent->G; //����ǳ�ʼ�ڵ㣬���丸�ڵ��ǿ�
	return parentG + extraG1;
}

int Astar::calcH(Point1* point, Point1* end)
{
	//�ü򵥵�ŷ����þ������H�����H�ļ����ǹؼ������кܶ��㷨��û�����о�^_^
	return 0;
  //因为此处是dijkstra算法，直接取0即可
}

int Astar::calcF(Point1 * point)
{
	return point->G + point->H;
}

Point1* Astar::getLeastFpoint()
{
	if (!openList1.empty())
	{
		auto resPoint = openList1.top();
		return resPoint;
	}
}

Point1* Astar::findPath(Point1& startPoint,Point1& endPoint, bool isIgnoreCorner)
{
	openList1.push(&startPoint); //�������,��������һ���ڵ㣬�������
	std::cout << startPoint.x << "  " << startPoint.y << "   ";
	std::cout << endPoint.x << "  " << endPoint.y << "   ";
	while (!openList1.empty())
	{
		Point1* curPoint1 = getLeastFpoint(); //�ҵ�Fֵ��С�ĵ�
		Point1* curPoint = new Point1(curPoint1->x, curPoint1->y);
		*curPoint = *curPoint1;
		openList1.pop();
		//std::cout << curPoint->F << " " << curPoint->H << " " << curPoint->G << " " << curPoint->x << " " << curPoint->y << "  " << std::endl;
		if (curPoint->x == endPoint.x && curPoint->y == endPoint.y)return curPoint;
		closeList.push_back(curPoint); //�ŵ��ر��б�
		hvalue[curPoint->x][curPoint->y] = -1;
		//openList1.pop();
		//1,�ҵ���ǰ��Χ�˸����п���ͨ���ĸ���

		auto surroundPoints = getSurroundPoints(curPoint, isIgnoreCorner);
		for (auto& target : surroundPoints)
		{
			////�ڹر��б�������Ѿ�д��isCanReach����
			////2,��ĳһ�����ӣ���������ڿ����б��У����뵽�����б������õ�ǰ��Ϊ�丸�ڵ㣬����F G H
			//if (!isInList(openList, target))
			//{
			//	target->parent = curPoint;
			//	target->G = calcG(curPoint, target);
			//	target->H = calcH(target, &endPoint);
			//	target->F = calcF(target);

			//	openList.push_back(target);
			//}
			////3����ĳһ�����ӣ����ڿ����б��У�����Gֵ, �����ԭ���Ĵ�, ��ʲô������, �����������ĸ��ڵ�Ϊ��ǰ��,������G��F
			//else
			//{
			//	int tempG = calcG(curPoint, target);
			//	if (tempG < target->G)
			//	{
			//		target->parent = curPoint;
			//		target->G = tempG;
			//		target->F = calcF(target);
			//	}
			//}
			//Point1* resPoint = isInList(openList, &endPoint);
			//if (resPoint)
			//	return resPoint; //�����б���Ľڵ�ָ�룬��Ҫ��ԭ�������endpointָ�룬��Ϊ���������
			target->H = calcH(target, &endPoint);
			if (hvalue[target->x][target->y] != 0 && target->H >= hvalue[target->x][target->y])continue;
			hvalue[target->x][target->y] = target->H;
			target->parent = curPoint;
			target->G = calcG(curPoint, target);
			target->F = calcF(target);
			openList1.push(target);
		}
	}
	return NULL;
}

std::list<Point1*> Astar::GetPath(Point1& startPoint, Point1& endPoint, bool isIgnoreCorner)
{
	Point1* result = findPath(startPoint, endPoint, isIgnoreCorner);
	std::list<Point1*> path;
	//����·�������û�ҵ�·�������ؿ�����
	while (result)
	{
		path.push_front(result);
		result = result->parent;
	}

	// �����ʱ�����б�����ֹ�ظ�ִ��GetPath���½���쳣
	if (!openList1.empty())openList1.pop();
	closeList.clear();
	return path;
}


Point1* Astar::isInList(const std::list<Point1 *> & list, const Point1* point) const
{
	//�ж�ĳ���ڵ��Ƿ����б��У����ﲻ�ܱȽ�ָ�룬��Ϊÿ�μ����б����¿��ٵĽڵ㣬ֻ�ܱȽ�����
	for (auto p : list)
		if (p->x == point->x && p->y == point->y)
			return p;
	return NULL;
}

bool Astar::isCanreach(const Point1* point, const Point1* target, bool isIgnoreCorner) const
{
	    
	if (target->x<0 || target->x > maze[0].size()
		|| target->y<0 || target->y > maze.size()
		|| maze[target->y][target->x] == 1
		|| target->x == point->x && target->y == point->y
		|| hvalue[target->x][target->y] == -1 || maze[target->y -10][(target-> x)-10] == 1) //������뵱ǰ�ڵ��غϡ�������ͼ�����ϰ�������ڹر��б��У�����fals
		return false;
	
	else
	{
		if (abs(point->x - target->x) + abs(point->y - target->y) == 1) //��б�ǿ���
			return true;
		else
		{
			//б�Խ�Ҫ�ж��Ƿ��ס
			if (maze[point->y][target->x] == 0 && maze[target->y][point->x] == 0)
				return true;
			else
				return isIgnoreCorner;
		}
	}
}

std::vector<Point1*> Astar::getSurroundPoints(const Point1* point, bool isIgnoreCorner) const
{
	std::vector<Point1*> surroundPoints;

	for (int x = point->x - 1; x - 1 <= point->x; x++)
	{
		for (int y = point->y - 1; y - 1 <= point->y; y++) 
		{
			if (isCanreach(point, new Point1(x, y), isIgnoreCorner))
				surroundPoints.push_back(new Point1(x, y));
		}
	}
	return surroundPoints;
}

}