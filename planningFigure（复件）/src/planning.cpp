#include <chrono>
#include <thread>
#include "planning.hpp"
#include "globalPlanning.h"
#include "localPlanning.hpp"
#include "defineColor.h"
#include "sys/time.h"
#include "unistd.h"
#include "PlanningMsg.pb.h"

#define STOP_POINT_FILE_NAME "../mapMsg/stopPoint.txt" // TODO

#define RECV_PERIOD 40
#define SEND_PERIOD 100
#define GLOBAL_PLANNING_PERIOD 100 //全局规划执行周期(ms)
#define LOCAL_PLANNING_PERIOD 100  //局部规划执行周期(ms)

//#define MAP_PATH "../mapMsg/roadMap_GCinKCY.xodr" //地图位置TODO
#define MAP_PATH "../mapMsg/roadMap.xodr" 
Jobs::Jobs(std::vector<void *> &receivingSocketList, std::vector<void *> &sendingSocketList, RoadMap &m,
           std::vector<GaussRoadPoint> &sPoints, std::tuple<int32_t, int32_t> &sPointRoadLaneId,
           std::vector<std::tuple<int32_t, int32_t>> &rList) : recvRate(RECV_PERIOD), sendRate(SEND_PERIOD),
                                                               globalPlanningRate(GLOBAL_PLANNING_PERIOD), localPlanningRate(LOCAL_PLANNING_PERIOD),
                                                               rSocketList(receivingSocketList), sSocketList(sendingSocketList),
                                                               map(m), stopPoints(sPoints), stopPointRoadLaneId(sPointRoadLaneId),
                                                               routingList(rList)
{
}

Jobs::~Jobs()
{
  ;
}

void Jobs::subscriber()
{
  while (1)
  {

    auto start = std::chrono::steady_clock::now();

    // =======do your works here======
    mutex.lock();
    zmq_pollitem_t items[3];
    items[0].socket = rSocketList[0];
    items[0].events = ZMQ_POLLIN;
    items[1].socket = rSocketList[1];
    items[1].events = ZMQ_POLLIN;
    items[2].socket = rSocketList[2];
    items[2].events = ZMQ_POLLIN;
    zmq_poll(items, 3, 0);

    if (items[0].revents & ZMQ_POLLIN)
    {
      char msg[81920] = {0};
      int size = zmq_recv(rSocketList[0], msg, 81920, 0);
      if (size != -1)
      {
        if (!imu.ParseFromArray(msg, size))
        {
          std::cout << "ImuData parse error :(" << std::endl;
        }
      }
      // std::cout<<"imu data: "<<imu.gaussx()<<";" <<imu.gaussy()<<std::endl;//测试假的imu数据
    }

    if (items[1].revents & ZMQ_POLLIN)
    {
      char msg[81920] = {0};
      int size = zmq_recv(rSocketList[1], msg, 81920, 0);
      std::cout << "prediction data size: " << size << std::endl;
      if (size != -1)
      {
        if (!prediction.ParseFromArray(msg, size))
        {
          std::cout << "PredictionData parse error :(" << std::endl;
        }
      }
      std::cout << "prediction data: " << prediction.object(0).predictpoint(0).x() << std::endl; //测试假的预测数据
    }

    if (items[2].revents & ZMQ_POLLIN)
    {
      char msg[81920] = {0};
      int size = zmq_recv(rSocketList[2], msg, 81920, 0);
      if (size != -1)
      {
        if (!chassisInfo.ParseFromArray(msg, size))
        {
          std::cout << "chassisInfoData parse error :(" << std::endl;
        }
      }
    }

    mutex.unlock();

    //=======end of your works======
    auto end = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    std::this_thread::sleep_for(std::chrono::milliseconds(this->recvRate - duration.count()));
  }
}

void Jobs::publisher()
{
  // 20220824

  while (1)
  {
    auto start = std::chrono::steady_clock::now();

    // =======do your works here======
    mutex.lock();
    Planning::TrajectoryPoint *trajectoryPoint;
    Planning::TrajectoryPointVec trajectoryPointVec;

    for (size_t i = 0; i < decisionData.optimalGlobalTrajectory.globalTrajectoryPoints.size(); i++)
    {
      trajectoryPoint = trajectoryPointVec.add_trajectorypoints();
      trajectoryPoint->set_x(decisionData.optimalGlobalTrajectory.globalTrajectoryPoints[i].GaussX);
      trajectoryPoint->set_y(decisionData.optimalGlobalTrajectory.globalTrajectoryPoints[i].GaussY);
      trajectoryPoint->set_theta(decisionData.optimalGlobalTrajectory.globalTrajectoryPoints[i].yaw);
      trajectoryPoint->set_speed(decisionData.optimalGlobalTrajectory.globalTrajectoryPoints[i].v);
      trajectoryPoint->set_curvature(decisionData.optimalGlobalTrajectory.globalTrajectoryPoints[i].curvature);
      trajectoryPoint->set_s(decisionData.optimalGlobalTrajectory.globalTrajectoryPoints[i].s);
    }

    //序列化
    size_t size = trajectoryPointVec.ByteSize();
    void *buffer = malloc(size);
    if (!trajectoryPointVec.SerializeToArray(buffer, size))
    {
      std::cerr << "Failed to write msg." << std::endl;
    }
    //发送
    zmq_msg_t msg;
    zmq_msg_init_size(&msg, size);
    memcpy(zmq_msg_data(&msg), buffer, size); // copy data from buffer to zmq msg
    zmq_msg_send(&msg, sSocketList[0], 0);

    mutex.unlock();
    //=======end of your works======

    auto end = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    std::this_thread::sleep_for(std::chrono::milliseconds(this->sendRate - duration.count()));
  }
}

void Jobs::processorLocalPlanning()
{
  while (1)
  {
    auto start = std::chrono::steady_clock::now();

    // =======do your works here======
    mutex.lock();
    // std::cout << std::endl;
    // std::cout << std::endl;
    // std::cout << std::endl;

    //收到全局规划结果，开始局部规划
    initLocalPlanning(decisionData);
    if (routingList.size())
    {
      // routingList.clear();
      // routingList.push_back(std::tuple<int32_t, int32_t>(3, 0));
      localPlanning(imu, chassisInfo.speed(), map, decisionData, prediction, routingList, stopPoints);
      std::ofstream fs0("bezierpath.txt", std::ofstream::trunc);
      for (uint32_t i = 0; i < decisionData.finalPathList.size(); i++)
      {
        for (uint32_t j = 0; j < 50; j++)
        {
          fs0 << std::setprecision(15) << decisionData.finalPathList[i].points[j].x << std::endl;
        }
        for (uint32_t j = 0; j < 50; j++)
        {
          fs0 << std::setprecision(15) << decisionData.finalPathList[i].points[j].y << std::endl;
        }
        fs0 << std::endl;
      }
      fs0.close();

      std::ofstream fs1("localoptimalpath.txt", std::ofstream::trunc);
      for (auto optimalPoint : decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex].trajectoryPoints)
      {
        fs1 << std::setprecision(15) << optimalPoint.x << std::endl;
      }
      fs1 << std::endl;
      for (auto optimalPoint : decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex].trajectoryPoints)
      {
        fs1 << std::setprecision(15) << optimalPoint.y << std::endl;
      }
      fs1 << std::endl;
      for (auto optimalPoint : decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex].trajectoryPoints)
      {
        fs1 << std::setprecision(15) << optimalPoint.v << std::endl;
      }
      fs1.close();

      std::ofstream fs("globaloptimalpath.txt", std::ofstream::trunc);
      for (auto optimalPoint : decisionData.optimalGlobalTrajectory.globalTrajectoryPoints)
      {
        fs << std::setprecision(15) << optimalPoint.GaussX << std::endl;
      }
      fs << std::endl;
      for (auto optimalPoint : decisionData.optimalGlobalTrajectory.globalTrajectoryPoints)
      {
        fs << std::setprecision(15) << optimalPoint.GaussY << std::endl;
      }
      fs << std::endl;
      for (auto optimalPoint : decisionData.optimalGlobalTrajectory.globalTrajectoryPoints)
      {
        fs << std::setprecision(15) << optimalPoint.v << std::endl;
      }
      fs << std::endl;
      for (auto optimalPoint : decisionData.optimalGlobalTrajectory.globalTrajectoryPoints)
      {
        fs << std::setprecision(15) << optimalPoint.yaw << std::endl;
      }
      fs << std::endl;
      for (auto optimalPoint : decisionData.optimalGlobalTrajectory.globalTrajectoryPoints)
      {
        fs << std::setprecision(15) << optimalPoint.curvature << std::endl;
      }
      fs << std::endl;
      for (auto optimalPoint : decisionData.optimalGlobalTrajectory.globalTrajectoryPoints)
      {
        fs << std::setprecision(15) << optimalPoint.s << std::endl;
      }
    }
    //std::cout << "optimalCurveIndex: " << decisionData.optimalTrajectoryIndex << std::endl; //测试输出optimal trajectory index

    clearPathList(decisionData.firstPathList);
    clearPathList(decisionData.secondPathList);
    clearPathList(decisionData.finalPathList);

    mutex.unlock();
    //=======end of your works======

    auto end = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    std::this_thread::sleep_for(std::chrono::milliseconds(this->localPlanningRate - duration.count()));
  }
}

void Jobs::processorGlobalPlanning()
{
  while (1)
  {
    auto start = std::chrono::steady_clock::now();

    // =======do your works here======
    mutex.lock();
    if (!getCurrentPosition(decisionData, imu, map)) //获取当前路点
    {
      //如果没有获取到路点
      std::cout << RED << "get current position failed :(" << RESET << std::endl;
    }
    else //在路点上
    {
      Astar as;
      as.mapToAstar(map, &as);

      int destinationRoad = std::get<0>(stopPointRoadLaneId);
      int destinationLane = std::get<1>(stopPointRoadLaneId);
      int originRoad = decisionData.currentId;
      int originLane = decisionData.currentLaneId;
      std::cout << ">>RoadId" << originRoad << ">>LaneId" << originLane << std::endl;

      as.path = as.getPath(originRoad, destinationRoad);         //这一步确定roads
      as.pathLanes.push_back(make_pair(originRoad, originLane)); //初始化

      as.findLane(map, as.path);
      if (as.pathLanes.back().second != destinationLane)
        as.pathLanes.push_back(make_pair(destinationRoad, destinationLane)); //保证终点的lane

      std::cout << std::endl
                << "* * * 用Astar找到的最短路为 : " << std::endl;
      as.moduleSelfCheckPrint(as.pathLanes);

      //格式转换
      routingList.clear();
      routingList.reserve(as.pathLanes.size());
      for (int i = 0; i < as.pathLanes.size(); i++)
      {
        routingList.push_back(std::tuple<int32_t, int32_t>(as.pathLanes[i].first, as.pathLanes[i].second));
      }
    }

    mutex.unlock();

    //=======end of your works======
    auto end = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    std::this_thread::sleep_for(std::chrono::milliseconds(this->sendRate - duration.count()));
  }
}

long int cvtThreadId2Long(thread::id id)
{
  stringstream ss;
  ss << id;

  return stol(ss.str());
}

void checkThreadStatus(int tStatus, thread::id id)
{
  if (tStatus == ESRCH)
  {
    cout << "thread  " << id << " not exist" << endl;
  }
  else if (tStatus == EINVAL)
  {
    cout << "signal " << id << " is invalid" << endl;
  }
  else
  {
    cout << "thread  " << id << " is alive" << endl;
  }
}

int main()
{
  void *context = zmq_ctx_new();

  // imu receiving socket
  int conflateFlag1=1;
  int conflateFlag2=1;
  int conflateFlag3=1;
  void *imuSocket = zmq_socket(context, ZMQ_SUB);
  zmq_setsockopt(imuSocket,ZMQ_CONFLATE,&conflateFlag1,sizeof(conflateFlag1));
  zmq_connect(imuSocket, "tcp://127.0.0.1:5003");
  zmq_setsockopt(imuSocket, ZMQ_SUBSCRIBE, "", 0);
  // prediction receiving socket
  void *predictionSocket = zmq_socket(context, ZMQ_SUB);
  zmq_setsockopt(imuSocket,ZMQ_CONFLATE,&conflateFlag2,sizeof(conflateFlag2));
  zmq_connect(predictionSocket, "tcp://127.0.0.1:5009");
  zmq_setsockopt(predictionSocket, ZMQ_SUBSCRIBE, "", 0);
  // chassis receiving socket
  void *actuatorSocket = zmq_socket(context, ZMQ_SUB);
  zmq_setsockopt(imuSocket,ZMQ_CONFLATE,&conflateFlag3,sizeof(conflateFlag3));
  zmq_connect(actuatorSocket, "tcp://127.0.0.1:5011");
  zmq_setsockopt(actuatorSocket, ZMQ_SUBSCRIBE, "", 0);

  std::vector<void *> receivingSocketList; // 0: imuSocket; 1:predictionSocket; 2:actuatorSocket
  receivingSocketList.push_back(imuSocket);
  receivingSocketList.push_back(predictionSocket);
  receivingSocketList.push_back(actuatorSocket);

  // zmq sending sockets

  void *publisherSocket = zmq_socket(context, ZMQ_PUB);
  zmq_bind(publisherSocket, "tcp://127.0.0.1:5010");

  std::vector<void *> sendingSocketList;
  sendingSocketList.push_back(publisherSocket);

  // load road net
  RoadMap map(MAP_PATH);
  std::cout << "test for map initial: "<< map.roads[2].lanes[0].id<< std::endl;
  std::cout << "test for map initial: "<< map.roads[2].lanes[1].id<< std::endl;
  // load stop point
  std::vector<GaussRoadPoint> rawStopPoints;
  std::tuple<int32_t, int32_t> stopPointRoadLaneId;
  loadStopPoints(STOP_POINT_FILE_NAME, rawStopPoints, stopPointRoadLaneId, OFFSET_X, OFFSET_Y);

  std::vector<std::tuple<int32_t, int32_t>> routingList;
  

  Jobs jobs(receivingSocketList, sendingSocketList, map, rawStopPoints, stopPointRoadLaneId, routingList);

  thread thread1(&Jobs::subscriber, &jobs);
  thread thread2(&Jobs::publisher, &jobs);
  thread thread3(&Jobs::processorGlobalPlanning, &jobs);
  thread thread4(&Jobs::processorLocalPlanning, &jobs);

  thread::id threadID1 = thread1.get_id();
  thread::id threadID2 = thread2.get_id();
  thread::id threadID3 = thread3.get_id();
  thread::id threadID4 = thread4.get_id();

  thread1.detach();
  thread2.detach();
  thread3.detach();
  thread4.detach();

  while (1)
  {
    // do what you want in main

    int thread1Status = pthread_kill(cvtThreadId2Long(threadID1), 0);
    checkThreadStatus(thread1Status, threadID1);

    int thread2Status = pthread_kill(cvtThreadId2Long(threadID2), 0);
    checkThreadStatus(thread2Status, threadID2);

    int thread3Status = pthread_kill(cvtThreadId2Long(threadID3), 0);
    checkThreadStatus(thread3Status, threadID3);

    int thread4Status = pthread_kill(cvtThreadId2Long(threadID4), 0);
    checkThreadStatus(thread4Status, threadID4);

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  return 0;
}
