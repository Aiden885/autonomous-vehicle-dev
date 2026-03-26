#pragma once

 #include<iostream>
 #include<vector>
  #include <tuple>
  #include <list>

//全局规划输出的road-lane 序列
class RoutingList
  {
    public :
      std::vector<std::tuple<int32_t, int32_t>>  roadlanelist;//road和lane的序列
      std::list<int>    roadlist;//全局规划道路的序列，因为全局规划这个是用list，所以这里也用list；
      bool bNewRoadList = false;  //是否是新更新的roadlist，用于与当前roadlist进行比较，确定是否发送给高铁新城的路测设备
      

      enum LANE_DIRECTION
      {
         MIDDLE,
         LEFT,
         RIGHT
      };

      LANE_DIRECTION leftRightLane;//在当前lane的那个方向
      int nChangeLane;//与规划中心lane的相差几个lane，本中心线是0，
  };
  