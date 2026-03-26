#pragma once

#include <iostream>
#include <sstream>
#include <assert.h>
#include <unistd.h>
#include <mutex>
#include <list>
// for thread and timer:
#include <iostream>
// for zmq:
#include <zmq.h>
#include "localization_MapAnalysis.h"
#include "localPlanning.hpp"

//add by syp
#include "uiMsg/ui.pb.h"
#include "serverMsg/decision.pb.h"
#include "serverMsg/msg2vehicle.pb.h"
#include "serverMsg/objects.pb.h"
#include "serverMsg/objectsvec.pb.h"
#include "serverMsg/traffic_light.pb.h"


//time
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/timer.hpp>
//end of by syp

class Jobs
{
public:
  explicit Jobs(std::vector<void *> &receivingSocketList, std::vector<void *> &sendingSocketList,
                RoadMap &m, std::vector<GaussRoadPoint> &stopPoints, std::tuple<int32_t, int32_t, int32_t> &stopPointRoadLaneId,
                std::vector<std::tuple<int32_t, int32_t>> &rList);
  virtual ~Jobs();

  virtual void subscriber();
  virtual void publisher();
  virtual void processorLocalPlanning();
  virtual void processorGlobalPlanning();
  //add by syp
   virtual void processorGlobalPlanningBaseLocal(); //基于本地信息全局规划
  virtual void processorGlobalPlanningBaseServer();//基于服务器信息全局规划
  void FindPointIDatRoad( GaussRoadPoint roadPoint,int32_t roadID, int32_t &laneID,int32_t & pointID);//根据坐标和路ID，找laneID和pointID
  virtual void request();//用于与服务器通讯
//end of add by syp

private:
  std::mutex mutex;

  int sendRate, recvRate, localPlanningRate, globalPlanningRate;
  std::vector<void *> rSocketList;
  std::vector<void *> sSocketList;

  RoadMap map;
  std::vector<GaussRoadPoint> stopPoints;
  std::tuple<int32_t, int32_t, int32_t> stopPointRoadLanePointId;
  std::vector<std::tuple<int32_t, int32_t>> routingList;

  pc::Imu imu;
  prediction::ObjectList prediction;
  controlData::ChassisInfo chassisInfo;
  DecisionData decisionData;

  // add by syp
 //for pad UI app 交互通讯
  ui::UiPad2VData uiPad2Vdata;  //平板发送到车的数据
  boost::posix_time::ptime lastPacketTime4uiPad2V;//接收到上一条数据的时间
  ui::UiV2PadData uiV2PadData;//车发送到平板的数据

  //for server交互通讯
  infopack::DecisionsListProto decisionListProto; //服务器发送到车的数据
  infopack::TrafficLight trafficLight;
  //infopack::ObjectsProto objectsProto;//车发送到服务器的数据
  infopack::DispatchProto dispatchCmd;//服务器消息中解析到的本车命令
  std::vector<infopack::ObjectsProto> objectsCmd;//服务器消息中解析到的路测信息

  enum  CAR_STATUS//用于服务器消息中的车辆状态
  {
    FREE = 0, 
    GOTO_PASSENGER = 1,
    WAIT_SCAN  = 2,
    PASSENGER_ONBOARD = 3,
    SECOND_FINISH = 4,
    FIRST_FINISH = 5
  };

std::vector<GaussRoadPoint> stopPointsFromLocal;//本地文件读取的停车点坐标
std::tuple<int32_t, int32_t, int32_t> stopPointRoadLanePointIdFromLocal;//本地文件读取的停车点ID
  //end by syp  
};
