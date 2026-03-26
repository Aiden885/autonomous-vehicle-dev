#include "globalPlanning.h"
#include <math.h>
#include <tuple>
using namespace std;

namespace planning{

Astar::Astar()
{
    roadList = new road[size];
    for (int i = 0; i < size; i++)
    {
        roadList[i].number = i;
    }
};

Astar::~Astar()
{
    if (roadList != NULL)
    {
        delete[] roadList;
        roadList = NULL;
        // cout << "调用了析构函数\n";
    }
}

std::vector<int> Astar::getNextRoad(int x)
{
    return roadList[x].to;
}

double Astar::calcH(int temp, int destination)
{
    double h;
    h = (roadList[temp].xEnd - roadList[destination].xBegin) * (roadList[temp].xEnd - roadList[destination].xBegin) + (roadList[temp].yEnd - roadList[destination].yBegin) * (roadList[temp].yEnd - roadList[destination].yBegin);
    return sqrt(h);
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
    cout << "getLeastF : wrong openList为空无法进一步寻找"; //
                                                            // 因为套在while循环里，!openList.empty()应该必成立
    // 如果openList为空，错误应该会被先归类到 "findPath : wrong 无法找到合理的道路" 中
}

road *Astar::findPath(int origin, int destination) // Astar算法的主要部分
{
    roadList[origin].G = roadList[origin].length;
    openList.push(node(origin, roadList[origin].length, origin)); // 往openList里放入出发点
    while (!openList.empty())
    {
        // cout << endl;
        int cur = getLeastF();
        // cout << "加入closeList的边为 : " << cur << endl;
        openList.pop();
        roadList[cur].isInList = -1;
        for (auto &it : roadList[cur].to)
        {
            if (it == destination) // 如果搜到了目的地可以直接结束
            {
                // cout << "已搜索到目的地" << endl;
                roadList[destination].father = cur;
                return &roadList[destination];
            }

            // 分不在list、在openlist、在closelist三种情况讨论
            int inList = roadList[it].isInList;
            // cout << "到达 : " << it << "  状态 : " << inList ;

            if (inList == 0)
            {
                roadList[it].isInList = 1;
                roadList[it].H = calcH(it, destination);
                roadList[it].G = calcG(it, cur);
                roadList[it].F = calcF(it);
                roadList[it].father = cur;
                openList.push(node(it, roadList[it].F, cur));
                // cout << "  更新了F值 : " << roadList[it].F << endl;
                continue;
            }

            if (inList == 1)
            {
                double tempG = calcG(it, cur);
                if (tempG > roadList[it].G)
                    continue;
                roadList[it].G = tempG;
                roadList[it].F = calcF(it);
                roadList[it].father = cur;
                openList.push(node(it, roadList[it].F, cur));
                // cout << " 更新了F值 : " << roadList[it].F << endl;
                continue;
            }

            if (inList == -1)
            {
                // cout << endl;
                continue;
            }

            cout << endl
                 << "findPath : wrong isInList出现异常值 : " << inList << endl;
        }
    }
    cout << "findPath : wrong 无法找到合理的道路" << endl;
    return NULL;
}

std::list<int> Astar::getPath(int origin, int destination, int originPointId, int destinationPointId)
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
    // cout << "使用A*算法寻找从 " << origin << " 到 " << destination << "的最短路" << endl;
    // cout << "已经初始化的边有: ";
    // for (int i = 0; i < size; i++)
    // {
    //     if (roadList[i].isInList == 0)
    //         cout << i << " ";
    // }
    // cout << endl;

    // cout << "originPointId:" <<originPointId << " destinationPointId:"<<destinationPointId<<endl;

    if (origin == destination && originPointId < destinationPointId)
    {
        path.push_back(destination);
        return path;
    }

    road *result = findPath(origin, destination);
    int temp = result->father;
    path.push_back(destination);
    path.push_front(temp);
    while (temp != origin)
    {
        temp = roadList[temp].father;
        path.push_front(temp);
    }
    if (!openList.empty())
        openList.pop();
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
    for (int i = 0; i < size; i++)
    {
        roadList[i].F = 0;
        roadList[i].G = 0;
        roadList[i].H = 0;
        if (roadList[i].isInList != -2)
            roadList[i].isInList = 0;
    }
}

void Astar::moduleSelfCheck(list<int> path)
{
    for (auto &it : path)
    {
        if (it < 0 || it > size)
        {
            cout << endl
                 << "moduleSelfCheck : wrong 输出不符合要求" << endl;
            return;
        }
    }
    cout << endl
         << "moduleSelfCheck : 输出符合要求" << endl;
}

void Astar::moduleSelfCheckPrint(vector<pair<int, int>> pathLane)
{
    for (auto it = pathLane.begin(); it != pathLane.end(); it++)
    {
        cout << (*it).first << "  " << (*it).second << endl;
    }
}

void Astar::mapToAstar(RoadMap m, Astar *as)
{
    for (auto it = m.roads.begin(); it != m.roads.end(); it++)
    {
        int number = it->id;
        double xStart = it->lanes.begin()->gaussRoadPoints.begin()->GaussX;
        double yStart = it->lanes.begin()->gaussRoadPoints.begin()->GaussY;
        double xEnd = it->lanes.begin()->gaussRoadPoints.rbegin()->GaussX;
        double yEnd = it->lanes.begin()->gaussRoadPoints.rbegin()->GaussY;
        // road的起点和终点直接用了第一条lane的
        double length = sqrt((xStart - xEnd) * (xStart - xEnd) + (yStart - yEnd) * (yStart - yEnd));
        // 长度直接取的是起点和终点的欧氏距离
        (*as).initRoad(number, xStart, yStart, xEnd, yEnd, length);
        for (auto itSuccessor = it->successorId.begin(); itSuccessor != it->successorId.end(); itSuccessor++)
        {
            (*as).initLink(number, *itSuccessor);
        }
    }
}

void Astar::findLane(RoadMap m, list<int> path)
{
    auto it = path.begin();
    auto itNext = path.begin(); // 用两个迭代器表示目前的road和下一个road，寻找目前的road中哪条lane能前往下一条道路的某条lane
    // 再看看目前road的那条lane是不是已经在pathLanes中（如果不是说明要更换道路）
    itNext++;
    while (itNext != path.end())
    {
        auto itRoads = m.roads.begin();
        while ((*itRoads).id != *it)
            itRoads++;                                                                              // 找到*it对应id的Map::Road
        for (auto itLanes = (*itRoads).lanes.begin(); itLanes != (*itRoads).lanes.end(); itLanes++) // 遍历Road的各条Lane
        {
            for (auto itSuccessor = (*itLanes).successorId.begin(); itSuccessor != (*itLanes).successorId.end(); itSuccessor++)
                // 遍历Lane的各个Successor
                if ((*itSuccessor).sucRoadID == *itNext)
                { // 如果有Successor的RoadId为下一个要前往的Road
                    if ((*itLanes).id != pathLanes.back().second)
                    {
                        pathLanes.push_back(make_pair(*it, (*itLanes).id));
                        // 若该Lane不再pathLanes中就将其加入
                    }
                    pathLanes.push_back(make_pair(*itNext, (*itSuccessor).sucLaneID));
                    goto outloop;
                    // 加入刚才找到的能到达的下一个Lane
                }
        }
    outloop:
        it++;
        itNext++;
    }
}

void Astar::findLane(vector<Road> &roads_, list<int> &path_, vector<std::pair<int, int>> &pathLanes_, int lane_id_)
{
    std::vector<std::tuple<int, int, int>> all_road;
    all_road.emplace_back(std::tuple{pathLanes_.at(0).first, pathLanes_.at(0).second, -2});
    recursiveFindLane(roads_, path_, all_road, *(--path_.end()), lane_id_);

    std::vector<std::pair<int, int>>().swap(pathLanes_);
    for (auto &ro : all_road)
    {
        pathLanes_.emplace_back(make_pair(std::get<0>(ro), std::get<1>(ro)));
    }
    vector<std::tuple<int, int, int>>().swap(all_road);
}

void Astar::recursiveFindLane(vector<Road> &roads_, const list<int> &path_, vector<std::tuple<int, int, int>> &optional_lanes_, int road_id_, int lane_id_)
{
    auto current_pointer = path_.begin();
    auto next_pointer = ++path_.begin();
    if (next_pointer == path_.end())
        return;

    auto current_road = roads_.begin();
    while (current_road->id != *current_pointer)
    {
        current_road++;
    }

    std::vector<std::tuple<int, int, int>> lane_per_road;
    int score = 0;
    for (const auto &current_lane : current_road->lanes)
    {
        // quickly find correct lane
        if (current_lane.id != std::get<1>(optional_lanes_.back()))
            continue;
        std::vector<std::tuple<int, int, int>> rec_lane_per_road;
        std::vector<std::tuple<int, int, int>> rec_lane_per_road_1;
        // we need the lane which can lead to next road
        for (const auto &successor : current_lane.successorId)
        {
            if (successor.sucRoadID != *next_pointer)
                continue;
            int this_score = 0;
            this_score = (successor.sucLaneID == lane_id_ && successor.sucRoadID == road_id_) ? 4 : 3;
            if (this_score == 4)
            {
                optional_lanes_.emplace_back(std::tuple{successor.sucRoadID, successor.sucLaneID, this_score});
                return;
            }
            auto rec_path_ = path_;
            rec_path_.erase(rec_path_.begin());
            // first find 3
            if (score != 3)
            {
                rec_lane_per_road.emplace_back(
                    std::tuple{successor.sucRoadID, successor.sucLaneID, this_score});
                recursiveFindLane(roads_, rec_path_, rec_lane_per_road, road_id_, lane_id_);
            }
            else
            { // second find 3, let us compare with it.
                rec_lane_per_road_1.emplace_back(
                    std::tuple{successor.sucRoadID, successor.sucLaneID, this_score});
                recursiveFindLane(roads_, rec_path_, rec_lane_per_road_1, road_id_, lane_id_);
                if (rec_lane_per_road_1.size() > rec_lane_per_road.size())
                {
                    std::vector<std::tuple<int, int, int>>().swap(rec_lane_per_road);
                    rec_lane_per_road = rec_lane_per_road_1;
                }
                else if (rec_lane_per_road_1.size() == rec_lane_per_road.size())
                {
                    int rec_1_score = 0;
                    int rec_score = 0;
                    for (auto &lanes : rec_lane_per_road)
                    {
                        rec_score += std::get<2>(lanes);
                    }
                    for (auto &lanes : rec_lane_per_road_1)
                    {
                        rec_1_score += std::get<2>(lanes);
                    }
                    if (rec_score < rec_1_score)
                    {
                        std::vector<std::tuple<int, int, int>>().swap(rec_lane_per_road);
                        rec_lane_per_road = rec_lane_per_road_1;
                    }
                }
                std::vector<std::tuple<int, int, int>>().swap(rec_lane_per_road_1);
            }
            if (this_score > score)
            {
                score = this_score;
            }
        }

        // deal with head (start point for recursion)
        if (current_lane.successorId.empty())
        {
            for (const auto right_ : current_lane.rightLaneId)
            {
                if (rec_lane_per_road.empty())
                    rec_lane_per_road.emplace_back(std::tuple{current_road->id, right_, 2});
            }
            for (const auto left_ : current_lane.leftLaneId)
            {
                if (rec_lane_per_road.empty())
                    rec_lane_per_road.emplace_back(std::tuple{current_road->id, left_, 2});
            }

            recursiveFindLane(roads_, path_, rec_lane_per_road, road_id_, lane_id_);
        }

        lane_per_road = rec_lane_per_road;
        std::vector<std::tuple<int, int, int>>().swap(rec_lane_per_road);
        std::vector<std::tuple<int, int, int>>().swap(rec_lane_per_road_1);
    }

    // deal with body (middle any point)
    if (lane_per_road.empty())
    {
        std::vector<std::tuple<int, int, int>> rec_lane_per_road_2;
        for (const auto &current_lane : current_road->lanes)
        {
            if (current_lane.id == std::get<1>(optional_lanes_.back()))
                continue;
            bool isOk = false;
            for (auto &sucId : current_lane.successorId)
            {
                if (sucId.sucRoadID == *next_pointer)
                {
                    isOk = true;
                    break;
                }
            }
            if (!isOk)
                continue;
            for (const auto right_ : current_lane.rightLaneId)
            {
                if (rec_lane_per_road_2.empty() && right_ == std::get<1>(optional_lanes_.back()))
                    rec_lane_per_road_2.emplace_back(std::tuple{current_road->id, current_lane.id, 2});
            }
            for (const auto left_ : current_lane.leftLaneId)
            {
                if (rec_lane_per_road_2.empty() && left_ == std::get<1>(optional_lanes_.back()))
                    rec_lane_per_road_2.emplace_back(std::tuple{current_road->id, current_lane.id, 2});
            }
            recursiveFindLane(roads_, path_, rec_lane_per_road_2, road_id_, lane_id_);
            break;
        }
        lane_per_road = rec_lane_per_road_2;
    }

    // deal with tail (end point)
    if (std::get<0>(lane_per_road.back()) == path_.back() && std::get<1>(lane_per_road.back()) != lane_id_)
    {
        int cur_lane_id = std::get<1>(lane_per_road.back());
        int delta = cur_lane_id - lane_id_;
        int signal = delta > 0 ? -1 : 1;
        for (int i = 0; i < std::abs(delta); ++i)
        {
            cur_lane_id += signal;
            lane_per_road.emplace_back(tuple{std::get<0>(lane_per_road.back()), cur_lane_id, -2});
        }
    }
    for (const auto &per_lane : lane_per_road)
    {
        optional_lanes_.emplace_back(per_lane);
    }
    std::vector<std::tuple<int, int, int>>().swap(lane_per_road);
}

}