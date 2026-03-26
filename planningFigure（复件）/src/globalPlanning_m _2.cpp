/**
 * @file globalPlanning_m.cpp
 * @author 
 * @brief globalPlanning
 * @version 1.0
 * @date 2024-12-04 
 * 
 * @copyright Copyright (c) 2022
 *
 */
#include "globalPlanning_m.h"

// namespace GolbalPlanning
// {

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

/**
 * @brief 使用Map类构建Astar类
 * @param[IN] param 地图，初始化的Astar
 * @param[IN] input 无
 * @param[OUT] output 更新的Astar
 
 * @cn_name: 使用地图构建Astar
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void AstarMapToAstar(const AstarMapToAstarParam &param, const AstarMapToAstarInput &input, AstarMapToAstarOutput &output)
{
    Astar as = param.as;
    RoadMap m = param.m;
	// AstarMapToAstarParam param_map_to_astar{m, as};
	// AstarMapToAstarInput input_map_to_astar{};
	// AstarMapToAstarOutput output_map_to_astar{as};
	// AstarMapToAstar(param_map_to_astar, input_map_to_astar, output_map_to_astar);
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
		AstarInitRoadParam param_init_road{as};
		AstarInitRoadInput input_init_road{number, xStart, yStart, xEnd, yEnd, length};
		AstarInitRoadOutput output_init_road{as};
		AstarInitRoad(param_init_road, input_init_road, output_init_road);
        // as.initRoad(number, xStart, yStart, xEnd, yEnd, length);
        for (auto itSuccessor = it->successorId.begin(); itSuccessor != it->successorId.end(); itSuccessor++)
        {
			AstarInitLinkParam param_init_link{as};
			AstarInitLinkInput input_init_link{number, *itSuccessor};
			AstarInitLinkOutput output_init_link{as};
			AstarInitLink(param_init_link, input_init_link, output_init_link);
            // as.initLink(number, *itSuccessor);
        }
    }
    output.as = as;
}

/**
 * @brief 使用Astar算法获取路径，包括预处理、后处理等部分
 * @param[IN] param Astar
 * @param[IN] input 起点、终点、起点Id、终点Id
 * @param[OUT] output 更新的Astar，起点到终点的路径
 
 * @cn_name: Astar获取全局路径
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void AstarGetPath(const AstarGetPathParam &param, const AstarGetPathInput &input, AstarGetPathOutput &output)
{
    std::cout<<" m-2"<<std::endl;
    Astar as = param.as;
    int origin = origin;
    int destination = destination;
    int originPointId = originPointId;
    int destinationPointId = destinationPointId;
    // std::list<int> path = as.getPath(origin, destination, originPointId, destinationPointId);

	std::list<int> path;
    if (as.roadList[origin].isInList == -2)
    {
        cout << "getPath wrong m2: 出发点的边未被初始化";
		output.path = path;
        return;
    }
    if (as.roadList[destination].isInList == -2)
    {
        cout << "getPath wrong m2 : 目的地的边未被初始化";
        output.path = path;
        return;
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
        output.path = path;
        return;
    }
	road *result;
	AstarFindPathParam param_recursive_find_lane{as};
	AstarFindPathInput input_recursive_find_lane{origin, destination};
	AstarFindPathOutput output_recursive_find_lane{result};
	AstarFindPath(param_recursive_find_lane, input_recursive_find_lane, output_recursive_find_lane);
    //  = findPath(origin, destination);
    int temp = result->father;
    path.push_back(destination);
    path.push_front(temp);
    while (temp != origin)
    {
        temp = as.roadList[temp].father;
        path.push_front(temp);
    }
    if (!as.openList.empty())
        as.openList.pop();

    output.as = as;
    output.path = path;
}

/**
 * @brief Astar道路初始化
 * @param[IN] param Astar
 * @param[IN] input 道路Id，起点XY，终点XY，长度
 * @param[OUT] output 更新的Astar
 
 * @cn_name: Astar添加道路
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void AstarInitRoad(const AstarInitRoadParam &param, const AstarInitRoadInput &input, AstarInitRoadOutput &output){
	Astar as = param.as;
	int number = input.number;
	double xStart = input.xStart;
	double yStart = input.yStart;
	double xEnd = input.xEnd;
	double yEnd = input.yEnd;
	double length = input.length;
	// as.initRoad(number, xStart, yStart, xEnd, yEnd, length);

	if (number > as.size)
    {
        cout << "initRoad : wrong 编号过大，请修改size";
        return;
    }
    if (sqrt((xStart - xEnd) * (xStart - xEnd) + (yStart - yEnd) * (yStart - yEnd)) - 0.000001 > length)
    {
        cout << "initRoad : wrong 编号为" << number << "的边长度不合理" << endl;
    }

    as.roadList[number].xBegin = xStart;
    as.roadList[number].xEnd = xEnd;
    as.roadList[number].yBegin = yStart;
    as.roadList[number].yEnd = yEnd;
    as.roadList[number].length = length;
    as.roadList[number].isInList = 0;

	output.as = as;
}

/**
 * @brief Astar道路连接关系初始化
 * @param[IN] param Astar
 * @param[IN] input 起始道路Id，到达道路Id
 * @param[OUT] output 更新的Astar
 
 * @cn_name: Astar添加道路连接关系
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void AstarInitLink(const AstarInitLinkParam &param, const AstarInitLinkInput &input, AstarInitLinkOutput &output){
	Astar as = param.as;
	int from = input.from;
	int to = input.to;
	// as.initLink(from, to);

	as.roadList[from].to.push_back(to);

	output.as = as;
}

/**
 * @brief 在保留地图的情况下，初始化Astar参数
 * @param[IN] param Astar
 * @param[IN] input 无
 * @param[OUT] output 更新的Astar
 
 * @cn_name: Astar参数初始化
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void AstarReset(const AstarResetParam &param, const AstarResetInput &input, AstarResetOutput &output){
	Astar as = param.as;
	// as.reset();

	for (int i = 0; i < as.size; i++)
    {
        as.roadList[i].F = 0;
        as.roadList[i].G = 0;
        as.roadList[i].H = 0;
        if (as.roadList[i].isInList != -2)
            as.roadList[i].isInList = 0;
    }

	output.as = as;
}

/**
 * @brief 检查Astar道路Id
 * @param[IN] param Astar
 * @param[IN] input 无
 * @param[OUT] output 无
 
 * @cn_name: Astar道路Id检查
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void AstarModuleSelfCheck(const AstarModuleSelfCheckParam &param, const AstarModuleSelfCheckInput &input, AstarModuleSelfCheckOutput &output){
	Astar as = param.as;
	std::list<int> path = input.path;
	// as.moduleSelfCheck(path);
	for (auto &it : path)
    {
        if (it < 0 || it > as.size)
        {
            cout << endl
                 << "moduleSelfCheck : wrong 输出不符合要求" << endl;
            return;
        }
    }
    cout << endl
         << "moduleSelfCheck : 输出符合要求" << endl;
}

/**
 * @brief 检查Astar道路Id打印
 * @param[IN] param Astar
 * @param[IN] input 无
 * @param[OUT] output 无
 
 * @cn_name: Astar道路Id打印
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void AstarModuleSelfCheckPrint(const AstarModuleSelfCheckPrintParam &param, const AstarModuleSelfCheckPrintInput &input, AstarModuleSelfCheckPrintOutput &output){
	Astar as = param.as;
	vector<pair<int, int>> pathLane = input.pathLanes;
	// as.moduleSelfCheckPrint(pathLanes);
	for (auto it = pathLane.begin(); it != pathLane.end(); it++)
    {
        cout << (*it).first << "  " << (*it).second << endl;
    }
}

/**
 * @brief 根据Astar道路级全局规划结果，获得车道级全局规划结果
 * @param[IN] param Astar
 * @param[IN] input 地图、道路级全局规划结果
 * @param[OUT] output 更新的Astar
 
 * @cn_name: Astar获得全局规划车道结果
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void AstarFindLane(const AstarFindLaneParam &param, const AstarFindLaneInput &input, AstarFindLaneOutput &output){
	Astar as = param.as;
	RoadMap m = input.m;
	list<int> path = input.path;
	// as.findLane(m, path);

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
                    if ((*itLanes).id != as.pathLanes.back().second)
                    {
                        as.pathLanes.push_back(make_pair(*it, (*itLanes).id));
                        // 若该Lane不再pathLanes中就将其加入
                    }
                    as.pathLanes.push_back(make_pair(*itNext, (*itSuccessor).sucLaneID));
                    goto outloop;
                    // 加入刚才找到的能到达的下一个Lane
                }
        }
    outloop:
        it++;
        itNext++;
    }

	output.as = as;
}

/**
 * @brief 根据Astar道路级全局规划结果，使用递归的方法获得车道级全局规划结果
 * @param[IN] param Astar
 * @param[IN] input 道路向量数组、道路级全局规划结果、储存结果的向量数组、终点车道Id
 * @param[OUT] output 更新的Astar
 
 * @cn_name: Astar递归地获得全局规划车道结果
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void AstarFindLaneR(const AstarFindLaneRParam &param, const AstarFindLaneRInput &input, AstarFindLaneROutput &output){
    Astar as = param.as;
    vector<Road> roads_ = input.roads_;
    list<int> path_ = input.path_;
    vector<std::pair<int, int>> pathLanes_ = input.pathLanes_;
    int lane_id_ = input.lane_id_;
    // as.findLane(roads_, path_, pathLanes_, lane_id_);

	std::vector<std::tuple<int, int, int>> all_road;
    all_road.emplace_back(std::tuple{pathLanes_.at(0).first, pathLanes_.at(0).second, -2});

	AstarRecursiveFindLaneParam param_recursive_find_lane{as};
	AstarRecursiveFindLaneInput input_recursive_find_lane{roads_, path_, all_road, *(--path_.end()), lane_id_};
	AstarRecursiveFindLaneOutput output_recursive_find_lane{as};
	AstarRecursiveFindLane(param_recursive_find_lane, input_recursive_find_lane, output_recursive_find_lane);
    // recursiveFindLane(roads_, path_, all_road, *(--path_.end()), lane_id_);

    std::vector<std::pair<int, int>>().swap(pathLanes_);
    for (auto &ro : all_road)
    {
        pathLanes_.emplace_back(make_pair(std::get<0>(ro), std::get<1>(ro)));
    }
    vector<std::tuple<int, int, int>>().swap(all_road);

    output.as = as; 
}

/**
 * @brief Astar递归获得车道级全局规划结果主体函数
 * @param[IN] param Astar
 * @param[IN] input 道路、道路级全局规划结果、可递归车道、终点道路Id、终点车道Id
 * @param[OUT] output 更新的Astar
 
 * @cn_name: Astar递归获得车道级全局规划结果主体函数
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void AstarRecursiveFindLane(const AstarRecursiveFindLaneParam &param, const AstarRecursiveFindLaneInput &input, AstarRecursiveFindLaneOutput &output){
    Astar as = param.as;
    std::vector<Road> roads_ = input.roads_;
    std::list<int> path_ = input.path_;
    std::vector<std::tuple<int, int, int>> optional_lanes_ = input.optional_lanes_;
    int road_id_ = input.road_id_;
    int lane_id_ = input.lane_id_;
    // as.recursiveFindLane(roads_, path_, optional_lanes_, road_id_, lane_id_);


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

				AstarRecursiveFindLaneParam param_recursive_find_lane{as};
				AstarRecursiveFindLaneInput input_recursive_find_lane{roads_, rec_path_, rec_lane_per_road, road_id_, lane_id_};
				AstarRecursiveFindLaneOutput output_recursive_find_lane{as};
				AstarRecursiveFindLane(param_recursive_find_lane, input_recursive_find_lane, output_recursive_find_lane);
                // recursiveFindLane(roads_, rec_path_, rec_lane_per_road, road_id_, lane_id_);
            }
            else
            { // second find 3, let us compare with it.
                rec_lane_per_road_1.emplace_back(
                    std::tuple{successor.sucRoadID, successor.sucLaneID, this_score});

				AstarRecursiveFindLaneParam param_recursive_find_lane{as};
				AstarRecursiveFindLaneInput input_recursive_find_lane{roads_, rec_path_, rec_lane_per_road_1, road_id_, lane_id_};
				AstarRecursiveFindLaneOutput output_recursive_find_lane{as};
				AstarRecursiveFindLane(param_recursive_find_lane, input_recursive_find_lane, output_recursive_find_lane);
                // recursiveFindLane(roads_, rec_path_, rec_lane_per_road_1, road_id_, lane_id_);
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
			AstarRecursiveFindLaneParam param_recursive_find_lane{as};
			AstarRecursiveFindLaneInput input_recursive_find_lane{roads_, path_, rec_lane_per_road, road_id_, lane_id_};
			AstarRecursiveFindLaneOutput output_recursive_find_lane{as};
			AstarRecursiveFindLane(param_recursive_find_lane, input_recursive_find_lane, output_recursive_find_lane);
            // recursiveFindLane(roads_, path_, rec_lane_per_road, road_id_, lane_id_);
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
			AstarRecursiveFindLaneParam param_recursive_find_lane{as};
			AstarRecursiveFindLaneInput input_recursive_find_lane{roads_, path_, rec_lane_per_road_2, road_id_, lane_id_};
			AstarRecursiveFindLaneOutput output_recursive_find_lane{as};
			AstarRecursiveFindLane(param_recursive_find_lane, input_recursive_find_lane, output_recursive_find_lane);
            // recursiveFindLane(roads_, path_, rec_lane_per_road_2, road_id_, lane_id_);
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
            lane_per_road.emplace_back(std::tuple{std::get<0>(lane_per_road.back()), cur_lane_id, -2});
        }
    }
    for (const auto &per_lane : lane_per_road)
    {
        optional_lanes_.emplace_back(per_lane);
    }
    std::vector<std::tuple<int, int, int>>().swap(lane_per_road);

    output.as = as;
}

/**
 * @brief Astar算法的主要部分
 * @param[IN] param Astar
 * @param[IN] input 出发点  目的地
 * @param[OUT] output 合理的道路
 
 * @cn_name: Astar算法的主要部分
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void AstarFindPath(const AstarFindPathParam &param, const AstarFindPathInput &input, AstarFindPathOutput &output)
{
	Astar as = param.as;
	int origin = input.origin;
	int destination = input.destination;

	as.roadList[origin].G = as.roadList[origin].length;
    as.openList.push(node(origin, as.roadList[origin].length, origin)); // 往openList里放入出发点
    while (!as.openList.empty())
    {
        // cout << endl;
		int cur;
		AstarGetLeastFParam param_get_leatest_f{as};
		AstarGetLeastFInput input_get_leatest_f{};
		AstarGetLeastFOutput output_get_leatest_f{cur};
		AstarGetLeastF(param_get_leatest_f, input_get_leatest_f, output_get_leatest_f);
        // int cur = getLeastF();
        // cout << "加入closeList的边为 : " << cur << endl;
        as.openList.pop();
        as.roadList[cur].isInList = -1;
        for (auto &it : as.roadList[cur].to)
        {
            if (it == destination) // 如果搜到了目的地可以直接结束
            {
                // cout << "已搜索到目的地" << endl;
                as.roadList[destination].father = cur;
				output.result = &as.roadList[destination];
                return ;
            }

            // 分不在list、在openlist、在closelist三种情况讨论
            int inList = as.roadList[it].isInList;
            // cout << "到达 : " << it << "  状态 : " << inList ;

            if (inList == 0)
            {
                as.roadList[it].isInList = 1;
				AstarCalcHParam param_calc_H{as};
				AstarCalcHInput input_calc_H{it, destination};
				AstarCalcHOutput output_calc_H{as.roadList[it].H};
				AstarCalcH(param_calc_H, input_calc_H, output_calc_H);

				AstarCalcGParam param_calc_G{as};
				AstarCalcGInput input_calc_G{it, cur};
				AstarCalcGOutput output_calc_G{as.roadList[it].G};
				AstarCalcG(param_calc_G, input_calc_G, output_calc_G);

				AstarCalcFParam param_calc_F{as};
				AstarCalcFInput input_calc_F{it};
				AstarCalcFOutput output_calc_F{as.roadList[it].F};
				AstarCalcF(param_calc_F, input_calc_F, output_calc_F);

                // as.roadList[it].H = calcH(it, destination);
                // as.roadList[it].G = calcG(it, cur);
                // as.roadList[it].F = calcF(it);
                as.roadList[it].father = cur;
                as.openList.push(node(it, as.roadList[it].F, cur));
                // cout << "  更新了F值 : " << roadList[it].F << endl;
                continue;
            }

            if (inList == 1)
            {
				double tempG;
				AstarCalcGParam param_calc_G{as};
				AstarCalcGInput input_calc_G{it, cur};
				AstarCalcGOutput output_calc_G{tempG};
				AstarCalcG(param_calc_G, input_calc_G, output_calc_G);
                // double tempG = as.calcG(it, cur);
                if (tempG > as.roadList[it].G)
                    continue;
                as.roadList[it].G = tempG;
				AstarCalcFParam param_calc_F{as};
				AstarCalcFInput input_calc_F{it};
				AstarCalcFOutput output_calc_F{as.roadList[it].F};
				AstarCalcF(param_calc_F, input_calc_F, output_calc_F);
                // as.roadList[it].F = as.calcF(it);
                as.roadList[it].father = cur;
                as.openList.push(node(it, as.roadList[it].F, cur));
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
	output.result = NULL;
}

/**
 * @brief Astar算法 计算起点到当前点的已知长度
 * @param[IN] param Astar
 * @param[IN] input 出发点  当前点
 * @param[OUT] output G值
 
 * @cn_name: Astar算法的主要部分
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void AstarCalcG(const AstarCalcGParam &param, const AstarCalcGInput &input, AstarCalcGOutput &output)
{
	int from = input.from;
	int temp = input.tmp;
	Astar as = param.as;
	output.result = as.roadList[from].G + as.roadList[temp].length;
}

/**
 * @brief Astar算法 计算当前点到终点的估计长度
 * @param[IN] param Astar
 * @param[IN] input 当前点 终点
 * @param[OUT] output H值
 
 * @cn_name: Astar算法的主要部分
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void AstarCalcH(const AstarCalcHParam &param, const AstarCalcHInput &input, AstarCalcHOutput &output)
{
	int temp = input.tmp;
	int destination = input.destination;
	Astar as = param.as;
	double h;
    h = (as.roadList[temp].xEnd - as.roadList[destination].xBegin) * (as.roadList[temp].xEnd - as.roadList[destination].xBegin) + (as.roadList[temp].yEnd - as.roadList[destination].yBegin) * (as.roadList[temp].yEnd - as.roadList[destination].yBegin);
    output.result = sqrt(h);
	// return sqrt(h);
}

/**
 * @brief Astar算法 计算当前点估计最终路径长度
 * @param[IN] param Astar
 * @param[IN] input 
 * @param[OUT] output F值
 
 * @cn_name: Astar算法的主要部分
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void AstarCalcF(const AstarCalcFParam &param, const AstarCalcFInput &input, AstarCalcFOutput &output)
{
	int temp = input.tmp;
	Astar as = param.as;
	output.result = as.roadList[temp].G + as.roadList[temp].H;
}

/**
 * @brief Astar算法 找到估计最终路径长度最小的点
 * @param[IN] param Astar
 * @param[IN] input 
 * @param[OUT] output openList下标
 
 * @cn_name: Astar算法的主要部分
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void AstarGetLeastF(const AstarGetLeastFParam &param, const AstarGetLeastFInput &input, AstarGetLeastFOutput &output)
{
	Astar as = param.as;
    if (!as.openList.empty())
    {
        auto resPoint = as.openList.top();
		output.result = resPoint.number;
		return;
        // return resPoint.number;
    }
    cout << "getLeastF : wrong openList为空无法进一步寻找"; //
                                                            // 因为套在while循环里，!openList.empty()应该必成立
    // 如果openList为空，错误应该会被先归类到 "findPath : wrong 无法找到合理的道路" 中
}

/**
 * @brief Astar算法 找到下一条路
 * @param[IN] param Astar
 * @param[IN] input road id
 * @param[OUT] output 下一条路id
 
 * @cn_name: Astar算法的主要部分
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void AstarGetNextRoad(const AstarGetNextRoadParam &param, const AstarGetNextRoadInput &input, AstarGetNextRoadOutput &output) // 返回F值最小的边的编号
{
    Astar as = param.as;

    output.result = as.roadList[input.temp].to;
}

// } // namespace GolbalPlanning