#include "globalPlanning.h"
#include <math.h>
#include <algorithm>
#include <tuple>
#include <unordered_map>
#include <iomanip>

using namespace std;

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
    if (temp == 1)
    {
        roadList[temp].length -= 500;
    }
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
    //  cout << "已经初始化的边有: ";
    //  for (int i = 0; i < size; i++)
    //  {
    //      if (roadList[i].isInList == 0)
    //          cout << i << " ";
    //  }
    //  cout << endl;

    // cout << "originPointId:" <<originPointId << " destinationPointId:"<<destinationPointId<<endl;

    if (origin == destination)
    {
        if (originPointId <= destinationPointId)
            path.push_back(destination);
        if (originPointId > destinationPointId && !roadList[destination].to.empty())
        {
            path.push_back(destination);
            path.push_back(roadList[destination].to.front());
        }
        return path;
    }

    road *result = findPath(origin, destination);
    // cout << "1111111111111111" << endl;
    if (result == NULL)
    {
        return path;
    }
    int temp = result->father;
    // cout << "2222222222222222" << endl;
    path.push_back(destination);
    path.push_front(temp);
    while (temp != origin)
    {
        temp = roadList[temp].father;
        path.push_front(temp);
    }

    if (!openList.empty())
        openList.pop();

    // cout << "333333333333333" << endl;

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
        cout << (*it).first << "  " << (*it).second << ", ";
    }
    cout << "moduleSelfCheckPrint --------" << endl;
}

void Astar::mapToAstar(RoadMap &m, Astar *as)
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

bool Astar::seekLane(const vector<Road> &roads_, const list<int> &path_, vector<std::pair<int, int>> &pathLanes_, const int lane_id_, const int start_index_)
{

    const int danger_ = 15;      ///< be in the end of lane, don't imagine to change lane
    const double discount = 0.7; ///< middle lane can get some discounts

    std::unordered_map<int, NewLane> newLanes;

    // std::cout << "seekLane start = " << pathLanes_.front().first << "," << pathLanes_.front().second << std::endl;
    int start = pathLanes_.front().first * 100 + pathLanes_.front().second;
    int destination = path_.back() * 100 + lane_id_;

    // std::cout << "path::";
    // for (auto &p : path_)
    // {
    //     std::cout << p << "-->";
    // }
    // std::cout << std::endl;

    // say you goodbye
    if (start == destination)
        return true;

    // std::cout << "path_.size() = " << path_.size() <<  std::endl;
    // 20230830 由于调度发送road list可能不包含当前道路，导致程序异常退出，因此增加当前道路信息
    auto &a = const_cast<list<int> &>(path_);
    if (pathLanes_.front().first != path_.front())
        a.push_back(pathLanes_.front().first);

    // make a fast and efficient table
    double anticipatory = 0;
    for (auto iter = path_.rbegin(); iter != path_.rend(); iter++)
    {
        auto path_iter = (*iter);
        for (const auto &road : roads_)
        {
            if (road.id == path_iter)
            {
                for (const auto &lane : road.lanes)
                {
                    double cost = lane.gaussRoadPoints.back().s; ///< just is the length of lane
                    double change_lane_cost = 100;               ///< relate to the length of lane
                    if (lane.id == 0)
                    {
                        anticipatory += cost;
                    }
                    // n*omega > 2 and omega < 2
                    double omega = 1.5; ///< 1.0~2.0, if less than 1.0, always straight; if more than 2.0, always go to middle
                    // restrict changing lane somewhere
                    bool change_flag = true;
                    if (cost < danger_)
                        change_flag = false;
                    if (road.id == path_.front() && ((1 - double(start_index_) / double(lane.gaussRoadPoints.size())) * cost) < danger_)
                        change_flag = false;
                    if (!road.isLaneChange)
                    {
                        change_flag = false;
                    }
                    // calm down
                    if (road.id == path_.back())
                    {
                        change_lane_cost = 10000;
                    }

                    std::vector<int> relationship; ///< can reach which lanes
                    for (const auto &s : lane.successorId)
                    {
                        if (std::find(path_.begin(), path_.end(), s.sucRoadID) != path_.end())
                            relationship.push_back(s.sucRoadID * 100 + s.sucLaneID);
                    }
                    for (const auto &b : lane.leftLaneId)
                    {
                        if (change_flag)
                            relationship.push_back(road.id * 100 + b);
                    }
                    for (const auto &b : lane.rightLaneId)
                    {
                        if (change_flag)
                            relationship.push_back(road.id * 100 + b);
                    }

                    // use some discounts here
                    if (lane.rightLaneId.empty())
                    {
                        // cost += omega * change_lane_cost;
                    }

                    newLanes.emplace(road.id * 100 + lane.id,
                                     NewLane(road.id * 100 + lane.id,
                                             cost,
                                             change_lane_cost,
                                             anticipatory,
                                             relationship,
                                             change_flag));
                }
                break;
            }
        }
    }

    NewLane *to_lane;   ///< use a pointer just for convenience
    NewLane *from_lane; ///< use a pointer just for convenience
    bool find_flag = false;
    newLanes.find(start)->second.G = newLanes.find(start)->second.cost_of_entire_lane; ///< lazy

    // A star
    std::priority_queue<NewLane> open_list;
    open_list.push(newLanes.find(start)->second);
    while (!open_list.empty())
    {
        int from = open_list.top().number;
        open_list.pop();
        newLanes.find(from)->second.state = NewLane::SeekState::OLD;
        for (const auto &to : newLanes.find(from)->second.reachable_lanes)
        {
            // Never handle incorrect maps
            if (newLanes.find(from) == newLanes.end() || newLanes.find(to) == newLanes.end())
            {
                continue;
                // exit(0);
            }
            //            std::cout<<"==================from lane============"<<from<<std::endl;
            //            std::cout<<"==================to lane============"<<to<<std::endl;
            // real pointer
            from_lane = &newLanes.find(from)->second;
            to_lane = &newLanes.find(to)->second;

            // my code never 'goto' and 'return' unsafely
            if (to == destination)
            {
                to_lane->father = from_lane->number;
                // std::cout<<"how much do you cost?? "<<std::setprecision(10)<<from_lane->F<<std::endl;
                find_flag = true;
                while (!open_list.empty())
                    open_list.pop();
                break;
            }

            // restrict changing many lane in one road
            int who_is_father = from_lane->number;
            int how_many_father = 0;
            while ((newLanes.find(who_is_father)->first / 100) == (to_lane->number / 100))
            {
                how_many_father++;
                who_is_father = newLanes.find(who_is_father)->second.father;
                if (who_is_father == -1)
                {
                    break;
                }
            }
            // the following is how to deal with these cases

            if (to_lane->state == NewLane::SeekState::YOUNG)
            {
                to_lane->state = NewLane::SeekState::MATURE;
                to_lane->G = int(from / 100) == int(to / 100)
                                 ? from_lane->G + std::pow(to_lane->cost_of_change_lane, how_many_father)
                                 : from_lane->G + to_lane->cost_of_entire_lane;
                to_lane->H = to_lane->anticipatory_of_current_lane;
                to_lane->F = to_lane->G + to_lane->H;
                to_lane->father = from_lane->number;
                //                std::cout<<" G::"<<from_lane->G<<" "<<std::pow(to_lane->cost_of_change_lane, how_many_father)<<" "<<to_lane->cost_of_entire_lane<<std::endl;
                //                std::cout<<"to lane G::"<<to_lane->number<<" "<<to_lane->G<<std::endl;
                //                std::cout<<"to lane H::"<<to_lane->number<<" "<<to_lane->H<<std::endl;
                //                std::cout<<"to lane F::"<<to_lane->number<<" "<<to_lane->F<<std::endl;
                open_list.push(*to_lane);
                continue;
            }

            if (to_lane->state == NewLane::SeekState::MATURE)
            {
                double tempG = int(from / 100) == int(to / 100)
                                   ? from_lane->G + std::pow(to_lane->cost_of_change_lane, how_many_father)
                                   : from_lane->G + to_lane->cost_of_entire_lane;
                if (tempG > to_lane->G)
                {
                    //                    std::cout<<" G::"<<from_lane->G<<" "<<std::pow(to_lane->cost_of_change_lane, how_many_father)<<" "<<to_lane->cost_of_entire_lane<<std::endl;
                    //                    std::cout<<"continue:: "<<tempG<<" "<<to_lane->G<<std::endl;
                    continue;
                }
                to_lane->G = tempG;
                to_lane->F = to_lane->G + to_lane->H;
                to_lane->father = from_lane->number;
                //                std::cout<<" G::"<<from_lane->G<<" "<<std::pow(to_lane->cost_of_change_lane, how_many_father)<<" "<<to_lane->cost_of_entire_lane<<std::endl;
                //                std::cout<<"to lane G::"<<to_lane->number<<" "<<to_lane->G<<std::endl;
                //                std::cout<<"to lane H::"<<to_lane->number<<" "<<to_lane->H<<std::endl;
                //                std::cout<<"to lane F::"<<to_lane->number<<" "<<to_lane->F<<std::endl;
                open_list.push(*to_lane);
                continue;
            }

            if (to_lane->state == NewLane::SeekState::OLD)
            {
                //                std::cout<<"real old"<<std::endl;
                continue;
            }
        }
    }

    // check
    if (!find_flag)
    {
        std::cout << "\033[1m\033[31m"
                  << "To User: There is no way to end point, so we have to terminate this progress." << RESET << std::endl;
        return false;
    }

    // backtrace
    int path_step = destination;
    std::vector<int> path_reverse;
    while (newLanes.find(path_step)->second.father != -1)
    {
        path_reverse.push_back(path_step);
        path_step = newLanes.find(path_step)->second.father;
    }

    // reverse and output
    std::reverse(path_reverse.begin(), path_reverse.end());
    for (const auto &p : path_reverse)
    {
        pathLanes_.emplace_back(make_pair<int, int>(int(p / 100), p % 100));
    }

    return true;
}