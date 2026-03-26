#pragma once
#include "localization_MapAnalysis.h"
#include <iostream>
#include <list>
#include <queue>
#include <utility>
#include <vector>
#include "defineColor.h"

using namespace std;

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
    void mapToAstar(RoadMap& m, Astar *as);    // 用Map类来构造Astar类
    void findLane(RoadMap m, list<int> path); // 根据road的顺序来确定所在的lane

    /**
     * @brief based on Astar algorithm
     * @param roads_ our road network
     * @param path_ our A* - find road result
     * @param pathLanes_ ready to return A* - find lane result
     * @param lane_id_ destination lane id
     * @param start_index_ start point id in the lane
     */
    bool seekLane(const std::vector<Road>& roads_, const std::list<int>& path_, std::vector<std::pair<int, int>>& pathLanes_, const int lane_id_, const int start_index_);

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

class NewLane{
public:
    enum class SeekState{
        YOUNG,  ////<yet not
        MATURE,  ////<has haven
        OLD   ////has left
    };
public:
    NewLane(int number_, double cost_, double cost_change, double anticipatory, std::vector<int>& reach_, bool change_):
             cost_of_entire_lane(cost_), cost_of_change_lane(cost_change), anticipatory_of_current_lane(anticipatory),
             number(number_), reachable_lanes(std::move(reach_)){
        allow_change = change_;
        F=G=H=0;
        father = -1;
        state = SeekState::YOUNG;
        //cost_of_change_lane = 1000;
    }

    friend bool operator<(const NewLane& a, const NewLane& b){
        return a.F > b.F;
    }

    double F, G, H;
    double cost_of_entire_lane;
    double cost_of_change_lane;
    double anticipatory_of_current_lane;
    int number;  //1001, road 10 lane 1
    int father;
    SeekState state;
    bool allow_change;

    std::vector<int> reachable_lanes;

};

