#ifndef  TRAFFIC_LIGHT_H
#define  TRAFFIC_LIGHT_H

#include <iostream>
#include <string>
#include <map>

class TrafficLight
{   
    public:

       enum DirectionOfTravel
       {
        STRAIGHT = 0,
        RIGHT = 1,
        LEFT = 2,
        UTURN = 3,       
        };

        enum DirectionOfLight
        {
            HORIZONTAL = 0, 
            VERTICAL =1
        };


    //从路口和相位描述，spat信息的索引主键
    std::string intersetionID_; //路口ID
    long  phaseID_;//相位ID

    //从道路前后关系，全局规划结果的索引主键，
    //需要注意的是一个相位等可能对应两个后继道路，这个是xodr地图中定义的问题，
    //但是并不定义多条nextRoadID的list，每一个对应单独的nextRoad
    int currentRoadID_;//当前所在道路ID
    int currentLaneID_;//当前所在LaneID
    int nextRoadID_;//下一条道路ID
    int nextLaneID_;//下一条laneID, 我的理解应该不需要定义到lane，但是为了避免出现麻烦，就定义了，只是增加对象的数目
    int stopPoint_;//停车点，这点对于一条路中间有红绿灯是区分的关键，也是车辆停止的位置

    //信号灯的转向，用于实际红绿灯的识别。
    DirectionOfTravel directionofTravel_;

    //红绿灯的安装方向，水平或者竖直，原来说需要这个参数，后续暂时不需要了
    //DirectionOfLight DirectionOfLight ;
    double gaussX_;//红绿灯坐标
    double	gaussY_;//红绿灯坐标
    double	height_;//红绿灯高度
    int groupSize_;//	该组红绿灯总个数
    int	groupIndex_;//子灯索引	
    int lightShapeFlag_;//子灯形状flag
    std::string lightShape_;//子灯形状



    public:
    std::string getRoadLaneKey();
    static std::string CreateRoadLaneKey(int currentRoadID, int currentLaneID,  int nextRoadID, int nextLaneID);//静态的用于其他构造key
       
};

//用于根据全局规划道路对spat红绿灯进行查找的map
class TrafficLightMap
{
    private:
        std::multimap<std::string, TrafficLight> mapForRoadLane_;//用于从道路到traffic类的索引

    public:    
        bool InitMapFromFile(std::string trafficLightFileName );//从文件初始化map
        bool  GetTrafficLightByRoadLane(int currentRoadID, int currentLaneID, int currentPoint, int nextRoadID, int nextLaneID ,TrafficLight & trafficeLight);//根据roadlane信息查找对象

       
};

#endif