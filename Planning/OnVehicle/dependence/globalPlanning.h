#pragma once
#include "localization_MapAnalysis.h"
#include <iostream>
#include <list>
#include <queue>
#include <utility>
#include <vector>
using namespace std;

namespace planning{

typedef struct
{
    int number;            // 编号
    double xBegin, yBegin; // 起点和终点坐标
    double xEnd, yEnd;
    double length;
    // std::vector<int> from;
    std::vector<int> to;
    int father;
    int isInList = -2; // 代表边在以下的四个状态之一：
    // 未初始化（-2），不在list中（0），在openList中（1），在closeList中（-1）
    double F, G, H;
    int numLanes; // 车道数量
} road;

class node
{
public:
    int number;
    double F;
    int father;
    node(int _number, double _F, int _father) : number(_number), F(_F), father(_father) {}
};

class cmp
{
public:
    bool operator()(node a, node b) const
    {
        return a.F > b.F;
    }
};

class Astar // 和Map类不一样之处在于Astar类中开辟了一块内存来储存各边信息，这样所占内存会变大，但是搜索速度会变快
{
public:
    std::list<int> getPath(int origin, int destination, int originPointId, int destinationPointId);
    Astar();
    ~Astar();
    road *roadList;
    void initRoad(int number, double xStart, double yStart, double xEnd, double yEnd, double length);
    void initLink(int from, int to);
    void reset(); // 不改变地图时的重置，改变地图直接初始化新类就行
    void moduleSelfCheck(std::list<int> path);
    void moduleSelfCheckPrint(vector<pair<int, int>> pathLanes);
    void mapToAstar(RoadMap m, Astar *as);    // 用Map类来构造Astar类
    void findLane(RoadMap m, list<int> path); // 根据road的顺序来确定所在的lane
    /**
     * @brief overload 'findLane', avoid to take an useless turn in the path.
     * @param roads our road network
     * @param path our road path
     * @param pathLanes our lane path
     */
    void findLane(std::vector<Road> &roads_, std::list<int> &path_, std::vector<std::pair<int, int>> &pathLanes_, int lane_id_);

    /**
     * @brief algorithm 1, this is globally excellent but not stable
     * @param roads_ road network
     * @param path_ road path
     * @param optional_lanes_ lane path for recursion
     * @param road_id_ destination road id
     * @param lane_id_ destination lane id
     */
    void recursiveFindLane(std::vector<Road> &roads_, const std::list<int> &path_, std::vector<std::tuple<int, int, int>> &optional_lanes_, int road_id_, int lane_id_);

    list<int> path;                   // 中间成果，road的id序列
    vector<pair<int, int>> pathLanes; // 最后要输出的结果，road的id和lane的id

private:
    road *findPath(int origin, int destination);
    std::vector<int> getNextRoad(int temp); // 返回F值最小的边的编号
    // 计算FGH值
    double calcG(int temp, int from);
    double calcH(int temp, int destination);
    double calcF(int temp);
    int getLeastF();

private:
    int size = 10000;                                           // 边的编号上限，可以修改
    std::priority_queue<node, std::vector<node>, cmp> openList; // priority_queue形式的openList可以提高效率
    // 因为有isInList，不专门设置closeList了
};
}