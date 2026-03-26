#pragma once
/*
//A*�㷨������
*/
#include <vector>
#include <list>
#include <opencv2/core/core.hpp> 
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/opencv.hpp>
#include <queue>
#include <iostream>

using namespace cv;

namespace planning{

const int kCost1 = 10; //ֱ��һ������
const int kCost2 = 15; //б��һ������

struct Point1
{
	int x, y; //�����꣬����Ϊ�˷��㰴��C++�����������㣬x�������ţ�y��������
	int F, G, H; //F=G+H
	Point1* parent; //parent�����꣬����û����ָ�룬�Ӷ��򻯴���
	Point1(int _x, int _y) :x(_x), y(_y), F(0), G(0), H(0), parent(NULL)  //������ʼ��
	{
	}
	Point1(){F = 0;G = 0;H = 0;};
};

struct cmp
{
	bool operator()(Point1*& a, Point1*& b)const
	{
		return a->F > b->F;
	}
};

class Astar
{
public:
	void InitAstar(const std::vector<std::vector<int>>& _maze, const std::vector<std::vector<int>>& _hvalue);
	std::list<Point1*> GetPath(Point1& startPoint,Point1& endPoint, bool isIgnoreCorner);

private:
	Point1* findPath(Point1& startPoint,Point1& endPoint, bool isIgnoreCorner);
	std::vector<Point1*> getSurroundPoints(const Point1* point, bool isIgnoreCorner) const;
	bool isCanreach(const Point1* point, const Point1* target, bool isIgnoreCorner) const; //�ж�ĳ���Ƿ����������һ���ж�
	Point1* isInList(const std::list<Point1*>& list, const Point1* point) const; //�жϿ���/�ر��б����Ƿ����ĳ��
	Point1* getLeastFpoint(); //�ӿ����б��з���Fֵ��С�Ľڵ�
	//����FGHֵ
	int calcG(Point1* temp_start, Point1* point);
	int calcH(Point1* point, Point1* end);
	int calcF(Point1* point);

private:
	std::vector<std::vector<int>> maze;
	std::vector<std::vector<int>> hvalue;//��Ÿ���Ŀǰ����Сhֵ������openlist��Ϊ0����closelist��Ϊ-1
	std::list<Point1*> openList;  //�����б�
	std::priority_queue<Point1*,std::vector<Point1*>,cmp > openList1; //�����б��޸İ�
	std::list<Point1*> closeList; //�ر��б�

};

}