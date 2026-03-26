#include "PlanningMsg.pb.h"
#include "defineColor.h"
#include "globalPlanning.h"
#include "localPlanning.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "planning.hpp"
#include "sys/time.h"
#include "unistd.h"
#include <chrono>
#include <thread>

namespace planning{

#define STOP_POINT_FILE_NAME "../mapMsg/stopPoint.txt" // TODO
#define STOP_POINT_FILE_NAME_YAML "../mapMsg/stopPoint.yaml"

#define RECV_PERIOD 40
#define SEND_PERIOD 100
#define GLOBAL_PLANNING_PERIOD 100 // 全局规划执行周期(ms)
#define LOCAL_PLANNING_PERIOD 100  // 局部规划执行周期(ms)

// by syp ??? 这个路径在调试环境和cmake中是不一致的
// #define MAP_PATH "../mapMsg/roadMap_GCinKCY.xodr"
#define MAP_PATH "../mapMsg/roadMap_KCYTemp.xodr" // 地图位置TODO
// /home/adlink/thicvPilot_0924/planningFigure/mapMsg/roadMap验证正确环岛切角.xodr
//  #define MAP_PATH "mapMsg/roadMap.xodr" //地图位置TODO

#define gridInOneMeter 5  // 10车上 // 10 //40
#define mapSizeXMeter 120 // 350车上  // 160 本机全景//60//20
#define mapSizeYMeter 120 // 160车上  // 120本机全景//60//20
#define mapYZero 200

// add by syp
#define HEARTBEAT_PERIOD 500 // in millisecond 心跳或数据必达周期
auto lastTheadTime = std::chrono::steady_clock::now();
auto lastTheadTimeofSub = std::chrono::steady_clock::now();
auto durationFroShowofSub = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - lastTheadTimeofSub);
#define myselfVehicleID 15   // 本车ID，这个后续应该挪到配置文件里面
#define myselfVehicleeType 0 // type
#define myselfVCType 1       // 6
// end  of by syp

using namespace std;
using namespace cv;

Jobs::Jobs(std::vector<void *> &receivingSocketList, std::vector<void *> &sendingSocketList, RoadMap &m, std::vector<GaussRoadPoint> &sPoints,
           std::tuple<int32_t, int32_t, int32_t> &sPointRoadLanePointId, std::vector<std::tuple<int32_t, int32_t>> &rList)
    : recvRate(RECV_PERIOD), sendRate(SEND_PERIOD), globalPlanningRate(GLOBAL_PLANNING_PERIOD), localPlanningRate(LOCAL_PLANNING_PERIOD), rSocketList(receivingSocketList),
      sSocketList(sendingSocketList), map(m), stopPoints(sPoints), stopPointRoadLanePointId(sPointRoadLanePointId), routingList(rList)
{
    // add by syp 20221031 for pad UI app 交互通讯

    lastPacketTime4uiPad2V = boost::posix_time::microsec_clock::local_time();
    // 初始化调度命令
    dispatchCmd.set_curstatus(Jobs::CAR_STATUS::FREE); // 车辆为空闲状态
    dispatchCmd.set_desx(-1.0);                        // 车辆目的地，无效位置
    dispatchCmd.set_desy(-1.0);
    // 测试-------------------------------------------4429751.029 442727.689
    //  dispatchCmd.set_curstatus( Jobs::CAR_STATUS::PASSENGER_ON); //车辆为空闲状态
    //  dispatchCmd.set_desx(4429751.029);//车辆目的地，无效位置
    //  dispatchCmd.set_desy(442727.689);
    //   dispatchCmd.clear_road();
    //  dispatchCmd.add_road(38);
    //  dispatchCmd.add_road(32);
    //  dispatchCmd.add_road(24);
    //  dispatchCmd.add_road(26);
    //  dispatchCmd.add_road(28);
    //  dispatchCmd.add_road(15);
    //  dispatchCmd.add_road(38);
    // 初始化停车点信息
    stopPointsFromLocal.assign(stopPoints.begin(), stopPoints.end());                       // 本地文件读取的停车点坐标
    std::get<0>(stopPointRoadLanePointIdFromLocal) = std::get<0>(stopPointRoadLanePointId); // 本地文件读取的停车点ID
    std::get<1>(stopPointRoadLanePointIdFromLocal) = std::get<1>(stopPointRoadLanePointId);
    std::get<2>(stopPointRoadLanePointIdFromLocal) = std::get<2>(stopPointRoadLanePointId);

    // end of add by syp
}

Jobs::~Jobs() { ; }

// add by syp
// 与 server的通讯request
void Jobs::request()
{
    //
    // string strAddr = "tcp://localhost:5501";
    void *context = zmq_ctx_new();
    if (context == nullptr)
    {
        cout << " serverReqSocket context error" << endl;
        return;
    }

    while (true) // 包含创建socket的大循环
    {
        //////////////////////////////////////////////////////
        // void * serverReqSocket = nullptr;
        void *serverReqSocket = zmq_socket(context, ZMQ_REQ);
        if (serverReqSocket == nullptr)
        {
            cout << " serverReqSocket  error" << endl;
            return;
        }

        // int rc = zmq_connect(serverReqSocket,  "tcp://localhost:5501");////166.111.50.39 5501
        // int rc = zmq_connect(serverReqSocket, "tcp://192.168.6.12:15501"); // 李老师本机
        // int rc = zmq_connect(serverReqSocket, "tcp://166.111.50.39:5501"); // 清华融合服务器地址
        // int rc = zmq_connect(serverReqSocket, "tcp://58.210.238.158:5504"); // 苏州服务器地址
        int rc = zmq_connect(serverReqSocket, "tcp://58.210.18.98:5525"); // 苏州服务器地址

        if (rc != 0)
        {
            cout << "connect serverReqSocket error = " << errno << "error string =" << zmq_strerror(errno) << endl;
            return;
        }

        //  Configure socket to not wait at close time,快速关闭socket
        int linger = 0;
        zmq_setsockopt(serverReqSocket, ZMQ_LINGER, &linger, sizeof(linger));

        while (true) // 数据发送和接收的循序
        {
            // sleep(1);

            // int request_nbr = 100;
            //     string strTemp = to_string(request_nbr);
            //     int nn = zmq_send (serverReqSocket,"hello", 5,0);

            // printf ("正在发送  %d   %d  %s...\n", nn,sizeof(strTemp),strTemp.c_str());
            // cout << "serverReqSocket ---------------------------------while()"<<endl;

            auto start = std::chrono::steady_clock::now();
            // =======do your works here======

            // 发送数据//////////////////////////////////////////////////////
            infopack::ObjectsVec msgvec;
            infopack::ObjectsProto *fmsg = msgvec.add_objmsg();
            fmsg->set_type(myselfVehicleeType);  // 交通参与者：0 车辆
            fmsg->set_objectid(myselfVehicleID); // 交通参与者编号 ???目前没有这个信息，写成定值
            fmsg->set_lat(imu.latitude());
            fmsg->set_lon(imu.longitude());
            fmsg->set_x(imu.gaussx());
            fmsg->set_y(imu.gaussy());
            fmsg->set_yaw(imu.yaw());
            fmsg->set_velocity(imu.velocity());
            fmsg->set_power(100);
            fmsg->set_finish(true);
            fmsg->set_height(200);
            fmsg->set_len(200);
            fmsg->set_width(200);
            // chrono::time_point<chrono::system_clock, chrono::milliseconds> tp =
            // chrono::time_point_cast<chrono::milliseconds>(chrono::system_clock::now()); int64_t ts =
            // tp.time_since_epoch().count(); fmsg->set_timestamp(ts);
            // fmsg->set_timestamp(imu.time()*1000);//INS时间是北京时间的当天秒计数，单位double，转为整数毫秒
            //  double msTemp = imu.time() * 1000;
            //  int hhTemp = int(msTemp / 3600 / 1000);
            //  int mmTemp = int((msTemp - hhTemp * 3600 * 1000) / 60 / 1000);
            //  int ssTemp = int((msTemp - hhTemp * 3600 * 1000 - mmTemp * 60 * 1000) / 1000);
            //  int sssTemp = (int(msTemp)) % 1000;
            //  cout << "set_timestamp:" << setprecision(10) << msTemp << " HHMMSS " << hhTemp << " " <<
            //  mmTemp << " " << ssTemp << " " << sssTemp << endl;
            // 将imu时间转换为longlong
            int64 timeForSend;                   // 单位是毫秒
            time_t tNow = time(nullptr);         // 当前时间
            struct tm *tmNow = localtime(&tNow); // 当前时间
            int secondsOfDay = tmNow->tm_hour * 3600 + tmNow->tm_min * 60 + tmNow->tm_sec;
            timeForSend = (tNow - secondsOfDay) * 1000 + int64(imu.time() * 1000);
            if (secondsOfDay - imu.time() > 43200) // 时间差超过半天，应该加一天
            {
                timeForSend = timeForSend + 86400 * 1000;
            }
            else if (secondsOfDay - imu.time() < -43200) // 时间差超过负半天，倒退一天
            {
                timeForSend = timeForSend - 86400 * 1000;
            }
            fmsg->set_timestamp(timeForSend);
            // std::cout << "-------------------------timeForSend" << timeForSend << std::endl;

            fmsg->set_vctype(myselfVCType); // 车辆类型，自动驾驶乘用车

            // fmsg->set_status(3);
            // 20231010  syp根据接收到的车辆状态和自车与停车点的距离，设置状态
            // fmsg->set_status(0);
            // std::cout << "stopPoints[1].yaw  =  " << stopPoints[1].yaw   <<std::endl;
            fmsg->set_status(dispatchCmd.curstatus()); // 来啥回啥
            double parkingDistance = 100;              // 车辆当前位置与停车点的距离
            double stopYawDiff;
            bool bStop = false;
            if (dispatchCmd.curstatus() == 1 || dispatchCmd.curstatus() == 3) // 停车时 1-->5  //3-->4
            {
                // 判断是否是在停车点附近停车，角度在一定范围内，距离也较近

                stopYawDiff = fabs(stopPoints[1].yaw - imu.yaw()); // 车辆行驶方向与停车点的方向
                if (stopYawDiff >= 180)
                {
                    stopYawDiff = 360 - stopYawDiff;
                }

                // if (stopYawDiff < 120)//加入角度判断
                if (imu.velocity() < 0.1) // 加入角度判断
                {
                    if (inArea(imu.gaussx(), imu.gaussy(), stopPoints[1].GaussX, stopPoints[1].GaussY))
                    {
                        bStop = true;
                    }

                    parkingDistance = getDistance(imu.gaussx(), imu.gaussy(), stopPoints[1].GaussX,
                                                  stopPoints[1].GaussY); // 终点坐标
                    std::cout << RED << setprecision(10) << " parkingDistance = " << parkingDistance << "," << imu.gaussx() << "," << imu.gaussy() << "," << stopPoints[1].GaussX << ","
                              << stopPoints[1].GaussY << RESET << std::endl;
                }

                // if(parkingDistance  <= (DISTANCETOPARK(imu.velocity() ) + 10)  )
                // //车辆停止距离有时候会比这个原
                if (bStop)
                {
                    if (dispatchCmd.curstatus() == 1)
                        fmsg->set_status(5);
                    else
                        fmsg->set_status(4);

                    std::cout << RED << " change  fmsg->set_status(5) to  " << fmsg->status() << RESET << std::endl;
                }

                // exit(0);
            }

            std::cout << "dispatchCmd.curstatus() =  " << dispatchCmd.curstatus() << " fmsg->status()=" << fmsg->status() << " stopYawDiff = " << stopYawDiff
                      << " parkingDistance = " << parkingDistance << std::endl;

            //////////////////////////////////////////////////

            size_t size = msgvec.ByteSize();
            void *buffer = malloc(size);
            if (!msgvec.SerializeToArray(buffer, size))
            {
                std::cerr << "Failed to write msg." << std::endl;
                break;
            }
            zmq_msg_t req;
            if (0 != zmq_msg_init_size(&req, size))
            {
                std::cerr << "zmq_msg_init failed..." << std::endl;
                break;
            }
            memcpy(zmq_msg_data(&req), buffer, size);
            /*  int timeout = 0;
             zmq_setsockopt(sc, ZMQ_SNDTIMEO, &timeout, sizeof(timeout)); */
            int nSend = zmq_msg_send(&req, serverReqSocket, 0);
            if (size != nSend)
            {
                // zmq_msg_close(&req);
                std::cerr << "send faliled... size =" << size << " length = " << nSend << std::endl;
                break;
            }
            else
            {
                // cout << "**************send successed size  =" << size << std::endl;
            }

            // std::cout << "send sucess!" << std::endl;
            zmq_msg_close(&req);
            // 清空緩存
            //  memset(buffer, 0, size * sizeof(char));
            free(buffer);

            // 接收数据//////////////////////////////////////////////////////
            //   Poll socket for a reply, with timeout
            zmq_pollitem_t items[1];
            items[0].socket = serverReqSocket;
            items[0].events = ZMQ_POLLIN;
            zmq_poll(items, 1, 2500); // 2500  msecs, (> 1000!)设置超时

            //  If we got a reply, process it
            if (items[0].revents & ZMQ_POLLIN)
            {
                //  We got a reply from the server, must match sequence
                zmq_msg_t reply;
                zmq_msg_init(&reply);
                int len = zmq_msg_recv(&reply, serverReqSocket, 0);
                // cout << "items[0].revents & ZMQ_POLLIN Receive data length=" << len << endl;

                if (len != -1) // 正常接收数据
                {
                    // 解析数据
                    void *str_recv = malloc(len);
                    memcpy(str_recv, zmq_msg_data(&reply), len);
                    // decisionListProto   、、DecisionsListProto dlist;
                    decisionListProto.ParseFromArray(str_recv, len);
                    // cout << "**********vehicle start out****************" << endl;
                    for (int i = 0; i < decisionListProto.dispatch_size(); i++)
                    {
                        const infopack::DispatchProto &dispatchProto = decisionListProto.dispatch(i);
                        if (dispatchProto.vehicleid() == myselfVehicleID)
                        {
                            dispatchCmd = dispatchProto; // 调度指令信息，只有 跟自车ID一致的才是自己的

                            // 将接口中的经纬度转换为gauss坐标
                            double gaussXTemp, gaussYTemp;
                            // cout << "desx:" << std::setprecision(10) << dispatchCmd.desx() << "\t"; // 经度
                            //  cout << "desy:" <<std::setprecision(10)<< dispatchCmd.desy() << "\n";//纬度

                            if (dispatchCmd.desx() < 10 || dispatchCmd.desy() < 10)
                            {
                                // 无效的经纬度，无需转换，
                                gaussXTemp = -1;
                                gaussYTemp = -1;

                                dispatchCmd.set_desx(gaussXTemp);
                                dispatchCmd.set_desy(gaussYTemp);
                            }
                            else
                            {
                                // gaussConvert(dispatchCmd.desx(), dispatchCmd.desy(), gaussXTemp,
                                // gaussYTemp);
                            }

                            // cout << "((((((()))))vehicleid:" << dispatchCmd.vehicleid() << "\t";
                            // cout << "curstatus:" << dispatchCmd.curstatus() << "\t";
                            // cout << "desx:" << std::setprecision(10) << dispatchCmd.desx() << "\t"; //
                            // gauss cout << "desy:" << std::setprecision(10) << dispatchCmd.desy() << "\n";
                            // // gauss cout << "RoadID:";

                            // for (int j = 0; j < dispatchCmd.road_size(); j++)
                            // {
                            //     cout << dispatchCmd.road(j) << ",";
                            // }
                            // cout << "---------------------------------" << endl;
                        }
                    }

                    if (decisionListProto.objects_size() > 0)
                    {
                        objectsCmd.clear();
                        for (int i = 0; i < decisionListProto.objects_size(); i++)
                        {
                            if (decisionListProto.objects(i).type() == myselfVehicleeType && decisionListProto.objects(i).objectid() == myselfVehicleID) // 不显示自车
                                continue;

                            objectsCmd.push_back(decisionListProto.objects(i));
                        }
                    }

                    // for (int i = 0; i < decisionListProto.objects_size(); i++)
                    // {
                    //     const infopack::ObjectsProto &objProto = decisionListProto.objects(i);
                    //     // cout << "type:" << objProto.type() << "\t";
                    //     cout << "objectid:" << objProto.objectid() << "\t";
                    //     //     cout << "lat:" << std::setprecision(10) <<objProto.lat() << "\t";
                    //     //     cout << "lon:" << std::setprecision(10) <<objProto.lon() << "\t";
                    //     cout << "x:" << std::setprecision(10) << objProto.x() << "\t";
                    //     cout << "y:" << std::setprecision(10) << objProto.y() << "\t";
                    //     //     cout << "yaw:" << std::setprecision(10) <<objProto.yaw() << "\t";
                    //     //     cout << "velocity:" << objProto.velocity() << "\t";
                    //     //     cout << "power:" << objProto.power() << "\t";
                    //     //     cout << "finish:" << objProto.finish() << "\t";
                    //     //     cout << "height:" << objProto.height() << "\t";
                    //     cout << "len:" << objProto.len() << "\t";
                    //     cout << "width:" << objProto.width() << endl;
                    //     // cout << "timestamp:" << objProto.timestamp() << "\n";
                    // }
                    // cout << "************decisionListProto**************" << endl;

                    free(str_recv);
                    zmq_msg_close(&reply);

                    //=======end of your works======
                    auto end = std::chrono::steady_clock::now();
                    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

                    // std::this_thread::sleep_for(std::chrono::milliseconds(this->recvRate -
                    // duration.count()));
                    std::this_thread::sleep_for(std::chrono::milliseconds(40 - duration.count()));
                }    // if (len != -1)
                else // 接收数据不正常
                {
                    std::cout << "E: malformed reply from server: " << std::endl;

                    zmq_msg_close(&reply);
                    zmq_close(serverReqSocket);

                    break;
                }
            }    // if (items[0].revents & ZMQ_POLLIN)
            else // 超时未接收到数据
            {
                // cout << "if (items[0].revents & ZMQ_POLLIN)   errno=" << errno << " errstr=" <<
                // zmq_strerror(errno) << endl;
                zmq_close(serverReqSocket);
                sleep(1);
                break;
            }

        } // while (bSocketOK)//数据发送和接收的循序

    } // while

    // 6.关闭套接字、销毁上下文

    zmq_ctx_destroy(context);
}

// end of add by ayp
void Jobs::subscriber()
{
    while (1)
    {
        // std::cout << "------------------  subscriber   -while ------------------ " << std::endl; //
        // 测试假的imu数据
        auto start = std::chrono::steady_clock::now();

        // =======do your works here======
        mutex.lock();

        // by syp 20221031 for pad UI app 交互通讯
        //  zmq_pollitem_t items[3];
        //  items[0].socket = rSocketList[0];
        //  items[0].events = ZMQ_POLLIN;
        //  items[1].socket = rSocketList[1];
        //  items[1].events = ZMQ_POLLIN;
        //  items[2].socket = rSocketList[2];
        //  items[2].events = ZMQ_POLLIN;

        // zmq_poll(items, 3, 0);

        zmq_pollitem_t items[4];
        items[0].socket = rSocketList[0];
        items[0].events = ZMQ_POLLIN;
        items[1].socket = rSocketList[1];
        items[1].events = ZMQ_POLLIN;
        items[2].socket = rSocketList[2];
        items[2].events = ZMQ_POLLIN;
        items[3].socket = rSocketList[3];
        items[3].events = ZMQ_POLLIN;

        zmq_poll(items, 4, 0);
        // end by syp

        if (items[0].revents & ZMQ_POLLIN)
        {
            // std::cout<<"items[0].revents &
            // ZMQ_POLLIN---------------------------------------------"<<std::endl;//测试假的imu数据
            char msg[81920] = {0};
            int size = zmq_recv(rSocketList[0], msg, 81920, 0);
            if (size != -1)
            {
                if (!imu.ParseFromArray(msg, size))
                {
                    std::cout << "ImuData parse error :(" << std::endl;
                }
                // std::cout<<"imu data: "<<imu.gaussx()<<";" <<imu.gaussy()<<std::endl;//测试假的imu数据
            }
        }

        if (items[1].revents & ZMQ_POLLIN)
        {
            // std::cout << "items[1].revents & ZMQ_POLLIN---------------------------------------------" <<
            // std::endl; // 测试假的imu数据
            durationFroShowofSub = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - lastTheadTimeofSub);
            lastTheadTimeofSub = std::chrono::steady_clock::now();

            std::cout << durationFroShowofSub.count()
                      << "while "
                         "-----------------------------------------------------------------------------------------"
                      << std::endl; // 测试假的imu数据
            char msg[819200] = {0};
            int size = zmq_recv(rSocketList[1], msg, 819200, 0);

            // std::cout << "prediction data size: " << size << std::endl;
            if (size != -1)
            {
                if (!prediction.ParseFromArray(msg, size))
                {
                    std::cout << "PredictionData parse error :(" << std::endl;
                }
            }
            std::cout << "prediction data size " << size << "prediction.object_size()   " << prediction.object_size() << std::endl; // 预测数据
        }

        /* ----- Fake prediction data ----- */
        //     prediction.clear_object();
        //    // double deltaX = 4429731 - imu.gaussx();
        //     //double deltaY = 442840 - imu.gaussy();
        //     double obstacleX  =  3482570;//3482590;
        //     double obstacleY = 561539.5;
        //     double deltaX = obstacleX - imu.gaussx(); //3482590
        //     double deltaY = obstacleY - imu.gaussy();
        //     double Y = deltaX * sin(imu.yaw() / 180 * M_PI) - deltaY * cos(imu.yaw() / 180 * M_PI);
        //     double X = deltaX * cos(imu.yaw() / 180 * M_PI) + deltaY * sin(imu.yaw() / 180 * M_PI);
        //     //if (fabs(442840 - imu.gaussy()) < 15 && fabs(4429751 - imu.gaussx()) < 3)
        //     if (fabs(obstacleY - imu.gaussy()) < 150 && fabs(obstacleX - imu.gaussx()) < 150)
        //     {
        //       prediction::Object *object1 = prediction.add_object();
        //       for (int i = 0; i < 21; i++)
        //       {
        //         prediction::PredictPoint *predictPoint = object1->add_predictpoint();

        //         predictPoint->set_x(X);
        //         predictPoint->set_y(Y);
        //         predictPoint->set_vx(0);
        //         predictPoint->set_vy(0);
        //       }
        //       object1->set_z(2);
        //       object1->set_h(2);
        //       object1->set_l(1);
        //       object1->set_w(1);
        //       object1->set_type(1);
        //       object1->set_trackid(1);
        //     }

        // 区域内无避障
        //  if(imu.gaussx() <   3482483 && imu.gaussx() > 3482462  &&
        //  imu.gaussy() <  561627 &&  imu.gaussy()  > 561611)
        //  {
        //      prediction.clear_object();
        //      cout<<"-----------------------prediction.clear_object();"<<endl;
        //  }

        /* Fake traffic light data */
        // constexpr double green_time = 20.0;   //绿灯时间
        // constexpr double yellow_time = 5.0;     //黄灯时间
        // constexpr double red_time = 20.0;   //红灯时间
        // constexpr double cycle_time = green_time + yellow_time + red_time;
        // constexpr double offset_time = 40.0;
        // constexpr double stop_line_x = 4429698.286;
        // constexpr double stop_line_y = 442774.922;
        // constexpr double stop_line_yaw = 270.0;
        // static auto launchTime = std::chrono::steady_clock::now();
        // double ego_x = imu.gaussx();
        // double ego_y = imu.gaussy();
        // double ego_yaw = imu.yaw(); // In angle
        // if (fabs(442781.922 - ego_y) < 7.5 && fabs(4429698.286 - ego_x) < 3 && fabs(ego_yaw -
        // stop_line_yaw) < 15)
        // {
        //   trafficLight.Clear();
        //   trafficLight.set_active(true);
        //   trafficLight.set_lane_length_before_intersection(std::fabs(stop_line_y - ego_y));
        //   auto currentTime = std::chrono::steady_clock::now();
        //   double secondsSinceLaunch = std::chrono::duration_cast<std::chrono::nanoseconds>(currentTime -
        //   launchTime).count() * 1e-9; double time_in_cycle = std::fmod(secondsSinceLaunch + offset_time,
        //   cycle_time); if (time_in_cycle < green_time)
        //   {
        //     trafficLight.set_state(infopack::TrafficLight::GREEN_LIGHT);
        //     trafficLight.set_remaining_time(green_time - time_in_cycle);
        //   }
        //   else if (time_in_cycle < green_time + yellow_time)
        //   {
        //     trafficLight.set_state(infopack::TrafficLight::YELLOW_LIGHT);
        //     trafficLight.set_remaining_time(green_time + yellow_time - time_in_cycle);
        //   }
        //   else
        //   {
        //     trafficLight.set_state(infopack::TrafficLight::RED_LIGHT);
        //     trafficLight.set_remaining_time(cycle_time - time_in_cycle);
        //   }
        // }
        // else
        // {
        //   trafficLight.set_active(false);
        //   trafficLight.set_lane_length_before_intersection(-1);
        //   trafficLight.set_state(infopack::TrafficLight::GREEN_LIGHT);
        //   trafficLight.set_remaining_time(-1);
        // }

        if (items[2].revents & ZMQ_POLLIN)
        {
            char msg[81920] = {0};
            int size = zmq_recv(rSocketList[2], msg, 81920, 0);
            // if (size != -1)
            // {
            //   if (!chassisInfo.ParseFromArray(msg, size))
            //   {
            //     std::cout << "chassisInfoData parse error :(" << std::endl;
            //   }
            // }
        }

        // by syp 20221031 for pad UI app 交互通讯
        // std::cout << RED<<"pad UI InfoData get size = "<<std::endl;
        if (items[3].revents & ZMQ_POLLIN)
        {
            char msg[81920] = {0};
            int size = zmq_recv(rSocketList[3], msg, 81920, 0);
            // std::cout << RED<<"pad UI InfoData get size = "<<size <<std::endl;
            if (size != -1)
            {
                if (!uiPad2Vdata.ParseFromArray(msg, size))
                {
                    std::cout << "uiPad2Vdata parse error :(" << std::endl;
                }
                else
                {
                    uiPad2Vdata.set_b_isalive(true);
                    lastPacketTime4uiPad2V = boost::posix_time::microsec_clock::local_time();
                }
            }

        } // if (items[3].revents & ZMQ_POLLIN)

        boost::posix_time::time_duration diff;
        boost::posix_time::ptime now = boost::posix_time::microsec_clock::local_time();
        diff = now - lastPacketTime4uiPad2V;
        if (diff.total_milliseconds() > HEARTBEAT_PERIOD)
        {
            uiPad2Vdata.set_b_isalive(false);
            // std::cout << RED << "UI Communication DIED!!" << RESET << std::endl;
        }

        //  cout << "----------------------uiPad2Vdata.b_stopFlag = " << uiPad2Vdata.b_stopflag() << endl;
        //  cout << "----------------------uiPad2Vdata.b_speedOff = " << uiPad2Vdata.b_speedoff() << endl;
        // // cout << "uiPad2Vdata.b_steeringOff = " << uiPad2Vdata.b_steeringoff() << endl;
        // cout << "----------------------uiPad2Vdata.b_isValid = " << uiPad2Vdata.b_isvalid() << endl;
        // // cout << "uiPad2Vdata.b_config1 = " << uiPad2Vdata.b_config1() << endl;
        //  cout << "---------------------uiPad2Vdata.b_isAlive = " << uiPad2Vdata.b_isalive() <<"diff = "
        //  <<diff.total_milliseconds() << endl;
        // // cout << "uiPad2Vdata.s_targetpath = " << uiPad2Vdata.s_targetpath() << endl;
        // end by syp

        mutex.unlock();

        //=======end of your works======
        auto end = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

        // std::this_thread::sleep_for(std::chrono::milliseconds(this->recvRate - duration.count()));
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

// 用于约束点的坐标，防止超过画布范围
cv::Point drawRegulate(cv::Point input)
{
    int mapSizeX = mapSizeXMeter * gridInOneMeter;
    int mapSizeY = mapSizeYMeter * gridInOneMeter;

    cv::Point result = input;
    result.y = result.y - mapYZero;
    if (result.x > mapSizeX)
    {
        result.x = mapSizeX;
    }
    if (result.x < 0)
    {
        result.x = 0;
    }
    if (result.y > mapSizeY)
    {
        result.y = mapSizeY;
    }
    if (result.y < 0)
    {
        result.y = 0;
    }
    return result;
}

void Jobs::publisher()
{

    // 画图
    int mapSizeX = mapSizeXMeter * gridInOneMeter;
    int mapSizeY = mapSizeYMeter * gridInOneMeter;

    // 画图结束

    while (1)
    {
        auto start = std::chrono::steady_clock::now();

        // =======do your works here======
        mutex.lock();
        Planning::TrajectoryPoint *trajectoryPoint;
        Planning::TrajectoryPointVec trajectoryPointVec;

        // 画图
        Mat img = Mat::zeros(Size(mapSizeX, mapSizeY), CV_8UC3);
        img.setTo(255); // 设置屏幕为白色
        double vehicleWidth = 1.36;
        double vehicleLength = 2.4; // 定位中心到车头位置

        cv::Point point1;
        cv::Point point2;
        point1.x = int(mapSizeX / 2 - vehicleWidth * gridInOneMeter / 2.0);
        point1.y = mapSizeY;
        point2.x = int(mapSizeX / 2 + vehicleWidth * gridInOneMeter / 2.0);
        point2.y = mapSizeY - vehicleLength * gridInOneMeter;
        rectangle(img, drawRegulate(point1), drawRegulate(point2), Scalar(0, 0, 255), -1); // 画自车位置

        ///////////////////////////////////////////////////////////////////////////
        // by syp 20220920
        string strTemp;
        strTemp = "IMU time = " + to_string(imu.time()) + ",X=" + to_string(imu.gaussx()) + ",Y=" + to_string(imu.gaussy()) + ",Yaw=" + to_string(imu.yaw()) + ",SP =" + to_string(imu.velocity());
        putText(img, strTemp, cv::Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1, 4);
        strTemp = "current RoadID =" + to_string(decisionData.currentId) + ",LaneID =" + to_string(decisionData.currentLaneId) + ",index = " + to_string(decisionData.currentIndex);
        putText(img, strTemp, cv::Point(10, 40), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1, 4);

        auto durationFroShow = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - lastTheadTime);
        lastTheadTime = std::chrono::steady_clock::now();
        strTemp = "publisher thread diff time  =" + to_string(durationFroShow.count() / 1000.) + "sub thread diff time  =" + to_string(durationFroShowofSub.count() / 1000.) +
                  "chassis speed = " + to_string(chassisInfo.speed()) + to_string(chassisInfo.steeringangle());
        putText(img, strTemp, cv::Point(10, 60), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1, 4);

        if (decisionData.optimalGlobalTrajectory.planningPoints.size() > 0)
        {
            // strTemp =
            // "pointNum="+to_string(decisionData.optimalGlobalTrajectory.globalTrajectoryPoints.size())+"Set
            // Speed 0="  + to_string(decisionData.optimalGlobalTrajectory.globalTrajectoryPoints[0].v)+
            //+"Set Speed N="  +
            // to_string(decisionData.optimalGlobalTrajectory.globalTrajectoryPoints[decisionData.optimalGlobalTrajectory.globalTrajectoryPoints.size()-1].v)
            //;
            strTemp = "pointNum=" + to_string(decisionData.optimalGlobalTrajectory.planningPoints.size()) + "Set Speed 0=" + to_string(decisionData.optimalGlobalTrajectory.planningPoints[0].v) +
                      "Set Speed N=" + to_string(decisionData.optimalGlobalTrajectory.planningPoints[decisionData.optimalGlobalTrajectory.planningPoints.size() - 1].v) +
                      "SetX 0=" + to_string(decisionData.optimalGlobalTrajectory.planningPoints[0].gaussX) + "SetY 0=" + to_string(decisionData.optimalGlobalTrajectory.planningPoints[0].gaussY);
            putText(img, strTemp, cv::Point(10, 80), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1, 4);
        }

        /* Put traffic light text */
        strTemp = "traffic light active = " + to_string(trafficLight.active()) + ",state = " + to_string(trafficLight.state()) + ",remaining time = " + to_string(trafficLight.remaining_time()) +
                  ",distance = " + to_string(trafficLight.lane_length_before_intersection());
        putText(img, strTemp, cv::Point(10, 150), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1, 4);

        double dXForShow, dYForShow, dYawForShow, dXVehicleForShow, dYVehicleForShow, dYawVehicleForShow;
        dXVehicleForShow = imu.gaussy();
        dYVehicleForShow = imu.gaussx();
        dYawVehicleForShow = M_PI / 2 - imu.yaw() * M_PI / 180.;
        // std::cout << "Vehicle X  = " << dXVehicleForShow << " Y = " << dYVehicleForShow << " Yaw =" <<
        // dYawVehicleForShow << std::endl;
        //  int ii =0,  jj=0, kk=0;
        for (int ii = 0; ii < map.roads.size(); ii++)
        {
            for (int jj = 0; jj < map.roads[ii].lanes.size(); jj++)
            {

                // std::cout<<"road  = "<<map.roads[ii].id<<"lane = "<<map.roads[ii].lanes[jj].id<<"point size
                // ="<<map.roads[ii].lanes[jj].gaussRoadPoints.size()<<std::endl;
                for (int kk = 0; kk < map.roads[ii].lanes[jj].gaussRoadPoints.size(); kk = kk + 3)
                {
                    dXForShow = map.roads[ii].lanes[jj].gaussRoadPoints[kk].GaussY;
                    dYForShow = map.roads[ii].lanes[jj].gaussRoadPoints[kk].GaussX;
                    dYawForShow = M_PI / 2 - map.roads[ii].lanes[jj].gaussRoadPoints[kk].yaw * M_PI / 180.;

                    // std::cout<<"Point X  = "<<dXForShow<<"Vehicle Y = "<<dYForShow<<"Vehicle Yaw
                    // ="<<dYawForShow<<std::endl;

                    CoordTran2DForNew0INOld(dXForShow, dYForShow, dYawForShow, dXVehicleForShow, dYVehicleForShow, dYawVehicleForShow);
                    // std::cout<<"new Point X  = "<<dXForShow<<"Vehicle Y = "<<dYForShow<<"Vehicle Yaw
                    // ="<<dYawForShow<<std::endl;
                    cv::Point point3;
                    point3.x = int(mapSizeX / 2.0 - dYForShow * gridInOneMeter);
                    point3.y = int(mapSizeY - dXForShow * gridInOneMeter);

                    circle(img, drawRegulate(point3), 1, Scalar(0, 0, 0)); // (B, G, R)(225, 105, 65)
                }
            }
        }
        // 测试用，画dest点
        // dispatchCmd.desx()
        // dXForShow = dispatchCmd.desy();                         // 561534.796;//
        // dYForShow = dispatchCmd.desx();                           // 3482533.422;//

        // 画stop点
        dXForShow = stopPoints[1].GaussY;                         // 561534.796;//
        dYForShow = stopPoints[1].GaussX;                         // 3482533.422;//
        dYawForShow = M_PI / 2 - stopPoints[1].yaw * M_PI / 180.; // 10.15;//
        strTemp = "endX=" + to_string(dXForShow) + "endY=" + to_string(dYForShow) + "Yaw=" + to_string(dYawForShow);
        putText(img, strTemp, cv::Point(10, 100), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1, 4);

        // std::cout<<"Point X  = "<<dXForShow<<"Vehicle Y = "<<dYForShow<<"Vehicle Yaw
        // ="<<dYawForShow<<std::endl;

        CoordTran2DForNew0INOld(dXForShow, dYForShow, dYawForShow, dXVehicleForShow, dYVehicleForShow, dYawVehicleForShow);
        // std::cout<<"new Point X  = "<<dXForShow<<"Vehicle Y = "<<dYForShow<<"Vehicle Yaw
        // ="<<dYawForShow<<std::endl;
        cv::Point point4;
        point4.x = int(mapSizeX / 2.0 - dYForShow * gridInOneMeter);
        point4.y = int(mapSizeY - dXForShow * gridInOneMeter);

        circle(img, drawRegulate(point4), 10, Scalar(0, 0, 255)); //

        // // 画信号灯trafficLight
        // //  constexpr double stop_line_x = 4429698.286;
        // //   constexpr double stop_line_y = 442774.922;
        // //   constexpr double stop_line_yaw = 270.0;
        // dXForShow = 442774.922;                       // 561534.796;//
        // dYForShow = 4429698.286;                      // 3482533.422;//
        // dYawForShow = M_PI / 2 - 270.0 * M_PI / 180.; // 10.15;//
        // CoordTran2DForNew0INOld(dXForShow, dYForShow, dYawForShow, dXVehicleForShow, dYVehicleForShow,
        // dYawVehicleForShow);
        // // std::cout<<"new Point X  = "<<dXForShow<<"Vehicle Y = "<<dYForShow<<"Vehicle Yaw
        // ="<<dYawForShow<<std::endl; cv::Point point6; point6.x = int(mapSizeX / 2.0 - dYForShow *
        // gridInOneMeter); point6.y = int(mapSizeY - dXForShow * gridInOneMeter);

        // // std::cout<<"----------------------------------trafficLight.state=
        // "<<trafficLight.state()<<std::endl; if (trafficLight.state() == 0)
        //   circle(img, drawRegulate(point6), 8, Scalar(0, 0, 255)); //
        // else if (trafficLight.state() == 1)
        //   circle(img, drawRegulate(point6), 8, Scalar(255, 255, 0)); //
        // else if (trafficLight.state() == 2)
        //   circle(img, drawRegulate(point6), 8, Scalar(0, 255, 0)); //
        //////////////////////////////////////////////////////////////////////////////

        cv::Point trajPoint1;
        cv::Point trajPoint2;

        if (stopPointJudge(imu, stopPoints))
        {
            std::cout << BOLDBLUE << "Vehicle Stopping!!!" << RESET << std::endl;
        }
        else
        {
            // std::cout << "controlTrajectoryList.size: " << decisionData.controlTrajectoryList.size() <<
            // std::endl; 画出所有备选轨迹
            for (int i = 0; i < decisionData.controlTrajectoryList.size(); i++)
            {
                // std::cout << "controlTrajectoryListiiiiiiiiiiiiiiii: " <<
                // decisionData.controlTrajectoryList[i].planningPoints[49].x << std::endl; std::cout <<
                // "controlTrajectoryListiiiiiiiiiiiiiiii: " <<
                // decisionData.controlTrajectoryList[i].planningPoints[49].y << std::endl;
                for (int j = 0; j < decisionData.controlTrajectoryList[i].planningPoints.size() - 1; j++)
                {
                    trajPoint1.x = int(decisionData.controlTrajectoryList[i].planningPoints[j].x * gridInOneMeter + mapSizeX / 2.0);
                    trajPoint1.y = int(mapSizeY - decisionData.controlTrajectoryList[i].planningPoints[j].y * gridInOneMeter);
                    trajPoint2.x = int(decisionData.controlTrajectoryList[i].planningPoints[j + 1].x * gridInOneMeter + mapSizeX / 2.0);
                    trajPoint2.y = int(mapSizeY - decisionData.controlTrajectoryList[i].planningPoints[j + 1].y * gridInOneMeter);
                    line(img, drawRegulate(trajPoint1), drawRegulate(trajPoint2), Scalar(0, 255, 0),
                         2); // 画轨迹点连线，lenghuise，绿色
                }
            }

            // 画出safe轨迹
            for (int iter = 0; iter < decisionData.feasibleTrajectoryIndexList.size(); iter++)
            {
                // std::cout << "controlTrajectoryListiiiiiiiiiiiiiiii: " <<
                // decisionData.controlTrajectoryList[i].planningPoints[49].x << std::endl; std::cout <<
                // "controlTrajectoryListiiiiiiiiiiiiiiii: " <<
                // decisionData.controlTrajectoryList[i].planningPoints[49].y << std::endl;
                int i = decisionData.feasibleTrajectoryIndexList[iter];
                for (int j = 0; j < decisionData.controlTrajectoryList[i].planningPoints.size() - 1; j++)
                {
                    trajPoint1.x = int(decisionData.controlTrajectoryList[i].planningPoints[j].x * gridInOneMeter + mapSizeX / 2.0);
                    trajPoint1.y = int(mapSizeY - decisionData.controlTrajectoryList[i].planningPoints[j].y * gridInOneMeter);
                    trajPoint2.x = int(decisionData.controlTrajectoryList[i].planningPoints[j + 1].x * gridInOneMeter + mapSizeX / 2.0);
                    trajPoint2.y = int(mapSizeY - decisionData.controlTrajectoryList[i].planningPoints[j + 1].y * gridInOneMeter);
                    line(img, drawRegulate(trajPoint1), drawRegulate(trajPoint2), Scalar(255, 255, 0),
                         2); // 画轨迹点连线，青色
                }
            }

            // 画出被选择轨迹
            // 从这里可以看出decision的坐标系XY是右前

            for (int i = 0; i < decisionData.controlTrajectoryList.size(); i++)
            {
                if (i == decisionData.optimalTrajectoryIndex)
                {
                    for (int j = 0; j < decisionData.controlTrajectoryList[i].planningPoints.size() - 1; j++)
                    {
                        // std::cout << "controlTrajectoryList.size: " <<
                        // decisionData.controlTrajectoryList.size() << std::endl;
                        trajPoint1.x = int(decisionData.controlTrajectoryList[i].planningPoints[j].x * gridInOneMeter + mapSizeX / 2.0);
                        trajPoint1.y = int(mapSizeY - decisionData.controlTrajectoryList[i].planningPoints[j].y * gridInOneMeter);
                        trajPoint2.x = int(decisionData.controlTrajectoryList[i].planningPoints[j + 1].x * gridInOneMeter + mapSizeX / 2.0);
                        trajPoint2.y = int(mapSizeY - decisionData.controlTrajectoryList[i].planningPoints[j + 1].y * gridInOneMeter);
                        line(img, drawRegulate(trajPoint1), drawRegulate(trajPoint2), Scalar(255, 0, 0),
                             4); // 画轨迹点连线，蓝色

                        // cout <<"---------x "<<decisionData.controlTrajectoryList[i].planningPoints[j].x<<"
                        // Y = " <<decisionData.controlTrajectoryList[i].planningPoints[j].y<<endl;
                    }
                }
            }
        }

        // 画障碍物
        // //<roadPoint gaussX="3482556.214" gaussY="561526.295" yaw="189.917" curvature="0.0" s="36.0"
        // speedMax="4"/>
        // //roadPoint gaussX="3482592.151" gaussY="561544.976" yaw="344.615" c
        // dXForShow = 561544.976;                         // 561534.796;//
        // dYForShow =3482592.151;                         // 3482533.422;//
        // dYawForShow = M_PI / 2 - 344.615 * M_PI / 180.; // 10.15;//

        // // std::cout<<"Point X  = "<<dXForShow<<"Vehicle Y = "<<dYForShow<<"Vehicle Yaw
        // ="<<dYawForShow<<std::endl;

        // CoordTran2DForNew0INOld(dXForShow, dYForShow, dYawForShow, dXVehicleForShow, dYVehicleForShow,
        // dYawVehicleForShow);
        //  strTemp = "fake endX=" + to_string(dXForShow) + "endY=" + to_string(dYForShow) + "Yaw=" +
        //  to_string(dYawForShow);
        // putText(img, strTemp, cv::Point(10, 250), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1, 4);
        // // std::cout<<"new Point X  = "<<dXForShow<<"Vehicle Y = "<<dYForShow<<"Vehicle Yaw
        // ="<<dYawForShow<<std::endl; cv::Point point7; point7.x = int(mapSizeX / 2.0 - dYForShow *
        // gridInOneMeter); point7.y = int(mapSizeY - dXForShow * gridInOneMeter);

        // circle(img, drawRegulate(point7), 15, Scalar(0, 0, 255)); //

        // 从这里可以看出，prediction的坐标系XY是前左
        for (int i = 0; i < prediction.object_size(); i++)
        // for (int i = 0; i < min(2, prediction.object_size()); i++)
        {
            double deltaX = prediction.object(i).predictpoint(0).x();
            double deltaY = prediction.object(i).predictpoint(0).y();
            double deltaW = abs(prediction.object(i).w());
            double deltaL = abs(prediction.object(i).l());
            // strTemp = "relX=" + to_string(deltaX) + "relY=" + to_string(deltaY);
            // putText(img, strTemp, cv::Point(10, 200), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1, 4);
            cv::Point point5;
            point5.x = int(mapSizeX / 2.0 - deltaY * gridInOneMeter);
            point5.y = int(mapSizeY - deltaX * gridInOneMeter);
            // point5.x = int(mapSizeX / 2.0 -  deltaX * gridInOneMeter );
            // point5.y = int(mapSizeY - deltaY * gridInOneMeter);
            // circle(img, drawRegulate(point5), 20, Scalar(0, 0, 255)); //
            cv::Point point11; // 障碍物左上角
            cv::Point point12; // 障碍物右下角

            point11.x = int(point5.x - deltaL / 2 * gridInOneMeter); // deltaX小车激光雷达到障碍物中心距离
            point11.y = int(point5.y - deltaW / 2 * gridInOneMeter);
            point12.x = int(point5.x + deltaL / 2 * gridInOneMeter);
            point12.y = int(point5.y + deltaW / 2 * gridInOneMeter);
            rectangle(img, drawRegulate(point11), drawRegulate(point12), Scalar(165, 0, 255), -1);
        }

        // 画感知失效区域
        //  double noPreX1 = 561627;//左上
        //  double noPreY1 = 3482462;
        //  double noPreX2 = 561611;//右下
        //  double noPreY2 = 3482483;
        //  double noPreYaw1 = 0; double noPreYaw2 = 0;

        // CoordTran2DForNew0INOld(noPreX1, noPreY1, noPreYaw1, dXVehicleForShow, dYVehicleForShow,
        // dYawVehicleForShow); CoordTran2DForNew0INOld(noPreX2, noPreY2, noPreYaw2, dXVehicleForShow,
        // dYVehicleForShow, dYawVehicleForShow); cv::Point pointNoPre1, pointNoPre2; pointNoPre1.x =
        // int(mapSizeX / 2.0 - noPreY1 * gridInOneMeter); pointNoPre1.y = int(mapSizeY - noPreX1 *
        // gridInOneMeter);

        // pointNoPre2.x = int(mapSizeX / 2.0 - noPreY2 * gridInOneMeter);
        //  pointNoPre2.y = int(mapSizeY - noPreX2 * gridInOneMeter);

        //  cout<< "no pre -- "<<  pointNoPre1.x<<","<<pointNoPre1.y <<","<<  pointNoPre2.x<<","<<
        //  pointNoPre2.y<<endl;

        //  rectangle(img, drawRegulate(pointNoPre1), drawRegulate(pointNoPre2), Scalar(0, 0, 255), -1);

        // 画路侧感知信息
        for (int i = 0; i < objectsCmd.size(); i++)
        {
            const infopack::ObjectsProto objProto = objectsCmd[i];
            if (objProto.type() == myselfVehicleeType && objProto.objectid() == myselfVehicleID) // 不显示自车
                continue;

            // 转换xy经纬度为平面坐标
            double latitudeTemp = objProto.lat();
            double longitudeTemp = objProto.lon();
            double gaussNorthTemp, gaussEastTemp;
            gaussConvert(longitudeTemp, latitudeTemp, gaussNorthTemp, gaussEastTemp);

            // 计算4个顶点平面坐标
            dXForShow = gaussEastTemp;
            dYForShow = gaussNorthTemp;
            dYawForShow = M_PI / 2 - objProto.yaw() * M_PI / 180.;
            double lengthTemp = objProto.len() / 100.;  // 原单位是cm，转成m
            double widthTemp = objProto.width() / 100.; // 原单位是cm，转成m

            // cout << "***objectid = " << objProto.objectid() << "," << longitudeTemp << "," << latitudeTemp
            // << ","
            //      << dXForShow << "," << dYForShow << "," << objProto.yaw() << "," << objProto.timestamp()
            //      << endl;

            double pointTemp[4][3]; // 障碍物矩形的4个角点，右上、右下、左下、左上；坐标 x，y,yaw
            pointTemp[0][0] = dXForShow + (lengthTemp / 2) * cos(dYawForShow) - (widthTemp / 2) * sin(dYawForShow);
            pointTemp[0][1] = dYForShow + (lengthTemp / 2) * sin(dYawForShow) + (widthTemp / 2) * cos(dYawForShow);
            pointTemp[1][0] = dXForShow + (lengthTemp / 2) * cos(dYawForShow) - (-widthTemp / 2) * sin(dYawForShow);
            pointTemp[1][1] = dYForShow + (lengthTemp / 2) * sin(dYawForShow) + (-widthTemp / 2) * cos(dYawForShow);
            pointTemp[2][0] = dXForShow + (-lengthTemp / 2) * cos(dYawForShow) - (-widthTemp / 2) * sin(dYawForShow);
            pointTemp[2][1] = dYForShow + (-lengthTemp / 2) * sin(dYawForShow) + (-widthTemp / 2) * cos(dYawForShow);
            pointTemp[3][0] = dXForShow + (-lengthTemp / 2) * cos(dYawForShow) - (widthTemp / 2) * sin(dYawForShow);
            pointTemp[3][1] = dYForShow + (-lengthTemp / 2) * sin(dYawForShow) + (widthTemp / 2) * cos(dYawForShow);
            pointTemp[0][2] = pointTemp[1][2] = pointTemp[2][2] = pointTemp[3][2] = dYawForShow;

            // cout<< "conner = " <<  pointTemp[0][0] <<","<<pointTemp[0][1] <<","<<
            //                                               pointTemp[1][0] <<","<<pointTemp[1][1] <<","<<
            //                                               pointTemp[2][0] <<","<<pointTemp[2][1] <<","<<
            //                                               pointTemp[3][0] <<","<<pointTemp[3][1] <<endl ;
            // 将平面直角坐标转化为车辆局部坐标，右上坐标系,--再转屏幕坐标系
            cv::Point cvPointTemp[4];
            for (int j = 0; j < 4; j++)
            {
                CoordTran2DForNew0INOld(pointTemp[j][0], pointTemp[j][1], pointTemp[j][2], dXVehicleForShow, dYVehicleForShow, dYawVehicleForShow);
                cvPointTemp[j].x = (int)(mapSizeX / 2.0 - pointTemp[j][1] * gridInOneMeter);
                ;
                cvPointTemp[j].y = (int)(mapSizeY - pointTemp[j][0] * gridInOneMeter);
            }

            //  cout<< "conner_at_vehicle = " <<  pointTemp[0][0] <<","<<pointTemp[0][1] <<","<<
            //                                             pointTemp[1][0] <<","<<pointTemp[1][1] <<","<<
            //                                             pointTemp[2][0] <<","<<pointTemp[2][1] <<","<<
            //                                             pointTemp[3][0] <<","<<pointTemp[3][1] <<endl ;
            //  cout<< "conner_at_screen = " <<  cvPointTemp[0].x <<","<<cvPointTemp[0].y <<","<<
            //                                             cvPointTemp[1].x <<","<<cvPointTemp[1].y <<","<<
            //                                             cvPointTemp[2].x <<","<<cvPointTemp[2].y <<","<<
            //                                             cvPointTemp[3].x <<","<<cvPointTemp[3].y <<endl ;

            // 画矩形框
            for (int k = 0; k < 4; k++)
            {
                line(img, drawRegulate(cvPointTemp[k]), drawRegulate(cvPointTemp[(k + 1) % 4]), Scalar(0, 255, 0), 4);
                // line(img, cvPointTemp[k],cv::Point (0,0), Scalar(0, 255, 0),4);
                //  line(img, drawRegulate(trajPoint1), drawRegulate(trajPoint2), Scalar(255, 0, 0), 4); //
                //  画轨迹点连线，蓝色
                // circle(img, drawRegulate(cvPointTemp[k]), 8, Scalar(0, 255, 0)); //
            }
        }

        imshow("globalTrajectoryPoints", img);
        waitKey(1);

        // 画图结束
        // cout <<  "????????????????????" << decisionData.optimalGlobalTrajectory.planningPoints.size()
        // <<"??????????????????????????????"<<endl;
        for (size_t i = 0; i < decisionData.optimalGlobalTrajectory.planningPoints.size(); i++)
        {
            trajectoryPoint = trajectoryPointVec.add_trajectorypoints();
            trajectoryPoint->set_x(decisionData.optimalGlobalTrajectory.planningPoints[i].gaussX);
            trajectoryPoint->set_y(decisionData.optimalGlobalTrajectory.planningPoints[i].gaussY);
            trajectoryPoint->set_theta(decisionData.optimalGlobalTrajectory.planningPoints[i].gaussAngle);
            trajectoryPoint->set_speed(decisionData.optimalGlobalTrajectory.planningPoints[i].v);

            // add by syp 平板控制车前进停止
            if (uiPad2Vdata.b_isalive() == true && uiPad2Vdata.b_isvalid() == true && uiPad2Vdata.b_speedoff() == false && uiPad2Vdata.b_stopflag() == true)
                trajectoryPoint->set_speed(0);

            // //end of add by syp
            trajectoryPoint->set_curvature(decisionData.optimalGlobalTrajectory.planningPoints[i].curvature);
            trajectoryPoint->set_s(decisionData.optimalGlobalTrajectory.planningPoints[i].s);

            // cout <<  "????????????????????" <<"x =
            // "<<decisionData.optimalGlobalTrajectory.planningPoints[i].gaussX
            // <<"??????????????????????????????"<<endl;
        }
        // cout <<  "????????????????????" <<"x =
        // "<<decisionData.optimalGlobalTrajectory.planningPoints[0].gaussX
        // <<"??????????????????????????????"<<endl; cout <<  "????????????????????" <<"y =
        // "<<decisionData.optimalGlobalTrajectory.planningPoints[0].gaussY
        // <<"??????????????????????????????"<<endl; 序列化
        size_t size = trajectoryPointVec.ByteSize();
        void *buffer = malloc(size);
        if (!trajectoryPointVec.SerializeToArray(buffer, size))
        {
            std::cerr << "Failed to write msg." << std::endl;
        }
        // 发送
        zmq_msg_t msg;
        zmq_msg_init_size(&msg, size);
        memcpy(zmq_msg_data(&msg), buffer, size); // copy data from buffer to zmq msg
        zmq_msg_send(&msg, sSocketList[0], 0);
        // cout <<  "????????????????????" <<"size = "<<size <<"??????????????????????????????"<<endl;

        // by syp 20221031 释放资源
        zmq_msg_close(&msg);
        free(buffer);

        // for pad UI app 交互通讯
        // 赋值
        ui::UiV2PadData uiV2PadDataTemp;

        ui::Posture *pPostureTemp = uiV2PadDataTemp.add_posture();

        ui::Imu *pImuTemp = pPostureTemp->add_imu();
        pImuTemp->set_b_isalive(true);
        pImuTemp->set_velocity(imu.velocity());
        pImuTemp->set_yaw(imu.yaw());

        ui::Gnss *pGnssTemp = pPostureTemp->add_gnss();
        pGnssTemp->set_b_isalive(true);
        pGnssTemp->set_gaussx(imu.gaussx());
        pGnssTemp->set_gaussy(imu.gaussy());
        pGnssTemp->set_gpsquality(imu.gpsvalid());

        pPostureTemp->set_b_isalive(true);

        ui::Iov *pIovTemp = uiV2PadDataTemp.add_iov();
        pIovTemp->set_b_isalive(true);
        pIovTemp->set_lightphase(1); ///////////////////////////////////
        pIovTemp->set_lighttime(2);  /////////////////////////////////////

        ui::Cv *pCVTemp = uiV2PadDataTemp.add_cv();
        pCVTemp->set_b_isalive(true);

        // 序列化
        size = uiV2PadDataTemp.ByteSize();
        buffer = malloc(size);
        if (!uiV2PadDataTemp.SerializeToArray(buffer, size))
        {
            std::cerr << "Failed to write uiV2PadDataTemp msg." << std::endl;
        }
        // 发送
        //  zmq_msg_t msg;
        zmq_msg_init_size(&msg, size);
        memcpy(zmq_msg_data(&msg), buffer, size); // copy data from buffer to zmq msg
        zmq_msg_send(&msg, sSocketList[1], 0);

        zmq_msg_close(&msg);
        free(buffer);
        // end by syp

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

        // 收到全局规划结果，开始局部规划
        initLocalPlanning(decisionData);
        if (routingList.size())
        {
            // routingList.clear();
            // routingList.push_back(std::tuple<int32_t, int32_t>(3, 0));
            // std::vector<infopack::ObjectsProto> objectsCmd;
            localPlanning(imu, imu.velocity(), map, decisionData, prediction, routingList, stopPoints, trafficLight, objectsCmd);

            // debug
            std::ofstream fs0("bezierpath.txt", std::ofstream::trunc);
            for (uint32_t i = 0; i < decisionData.finalPathList.size(); i++)
            {
                // for (uint32_t j = 0; j < 50; j++)
                for (auto planningPoint : decisionData.finalPathList[i].planningPoints)
                {
                    // fs0 << std::setprecision(15) << decisionData.finalPathList[i].planningPoints[j].s <<
                    // std::endl;
                    fs0 << std::setprecision(15) << planningPoint.s << std::endl;
                }

                // for (uint32_t j = 0; j < 50; j++)
                for (auto planningPoint : decisionData.finalPathList[i].planningPoints)
                {
                    // fs0 << std::setprecision(15) << decisionData.finalPathList[i].planningPoints[j].l <<
                    // std::endl;
                    fs0 << std::setprecision(15) << planningPoint.l << std::endl;
                }
                fs0 << std::endl;
            }
            fs0.close();

            std::ofstream fs1("localoptimalpath.txt", std::ofstream::trunc);
            for (auto optimalPoint : decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex].planningPoints)
            {
                fs1 << std::setprecision(15) << optimalPoint.x << std::endl;
            }
            fs1 << std::endl;
            for (auto optimalPoint : decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex].planningPoints)
            {
                fs1 << std::setprecision(15) << optimalPoint.y << std::endl;
            }
            fs1 << std::endl;
            for (auto optimalPoint : decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex].planningPoints)
            {
                fs1 << std::setprecision(15) << optimalPoint.v << std::endl;
            }
            fs1.close();

            std::ofstream fs("globaloptimalpath.txt", std::ofstream::trunc);
            for (auto optimalPoint : decisionData.optimalGlobalTrajectory.planningPoints)
            {
                fs << std::setprecision(9) << optimalPoint.gaussX << std::endl;
            }
            fs << std::endl;
            for (auto optimalPoint : decisionData.optimalGlobalTrajectory.planningPoints)
            {
                fs << std::setprecision(9) << optimalPoint.gaussY << std::endl;
            }
            fs << std::endl;
            for (auto optimalPoint : decisionData.optimalGlobalTrajectory.planningPoints)
            {
                fs << std::setprecision(15) << optimalPoint.v << std::endl;
            }
            fs << std::endl;
            for (auto optimalPoint : decisionData.optimalGlobalTrajectory.planningPoints)
            {
                fs << std::setprecision(15) << optimalPoint.gaussAngle << std::endl;
            }
            fs << std::endl;
            for (auto optimalPoint : decisionData.optimalGlobalTrajectory.planningPoints)
            {
                fs << std::setprecision(15) << optimalPoint.curvature << std::endl;
            }
            fs << std::endl;
            for (auto optimalPoint : decisionData.optimalGlobalTrajectory.planningPoints)
            {
                fs << std::setprecision(15) << optimalPoint.s << std::endl;
            }
        }
        // std::cout << "optimalCurveIndex: " << decisionData.optimalTrajectoryIndex << std::endl; //
        // 测试输出optimal trajectory index

        clearPathList(decisionData.finalPathList);

        mutex.unlock();
        //=======end of your works======

        auto end = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        // cout << "processorLocalPlanning   " << duration.count() << " sleep " <<
        // (std::chrono::milliseconds(this->localPlanningRate - duration.count())).count() << endl;
        int64 sleepspan = this->localPlanningRate - duration.count();
        sleepspan = max(sleepspan, int64(1));
        std::this_thread::sleep_for(std::chrono::milliseconds(sleepspan));
    }
}

// 根据坐标和路ID，找laneID和pointID
void Jobs::FindPointIDatRoad(GaussRoadPoint roadPoint, int32_t roadID, int32_t &laneID, int32_t &pointID)
{
    // cout << "FindPointIDatRoad----------------------------------" << endl;
    // 得先根据roadID找到road的索引值
    int roadIndexTemp = -1; // 查找出对应road在map中的索引
    for (int i = 0; i < map.roads.size(); i++)
    {
        if (roadID == map.roads[i].id)
        {
            // cout << roadID << "map.roads[i].id" << map.roads[i].id << endl;
            roadIndexTemp = i;
            break;
        }
    }

    if (roadIndexTemp == -1) // 没有找到road
    {
        cout << "cnanot find road index by road ID" << endl;
        return;
    }

    // 在路上找laneID 和 pointID
    for (int i = 0; i < map.roads[roadIndexTemp].lanes.size(); i++) // lane循环
    {
        cout << " find lane ID" << i << endl;
        for (int j = 0; j < map.roads[roadIndexTemp].lanes[i].gaussRoadPoints.size(); j++) // point 循环
        {
            // cout << " find point ID" << j << endl;
            if (map.roads[roadIndexTemp].lanes[i].gaussRoadPoints[j].GaussX - roadPoint.GaussX < 2 && map.roads[roadIndexTemp].lanes[i].gaussRoadPoints[j].GaussY - roadPoint.GaussY < 2) // 找到点
            {
                laneID = map.roads[roadIndexTemp].lanes[i].id;
                pointID = j;
                return;
            }
        }
    } // lane循环
}
void Jobs::processorGlobalPlanning()
{

    while (1)
    {
        auto start = std::chrono::steady_clock::now();
        // =======do your works here======

        // 暂时不考虑平板设置云控和本地控

        // 根据是否有服务器指令，确定全局规划的方法，
        // 有调度控制命令,就按照命令执行，除非调度发送无效坐标命令，不论后续是否服务器在线
        // 其实现在是所有的调度状态都按照调度命令控制了，这部分待接口明确
        if ((dispatchCmd.curstatus() == Jobs::GOTO_PASSENGER ||                                                  // 未定义的状态，但是接口发出来了，现在都用这个
             dispatchCmd.curstatus() == Jobs::WAIT_SCAN ||                                                       // 车辆被预约
             dispatchCmd.curstatus() == Jobs::PASSENGER_ONBOARD ||                                               // 车上有乘客
             dispatchCmd.curstatus() == Jobs::SECOND_FINISH || dispatchCmd.curstatus() == Jobs::FIRST_FINISH) && // 车空闲
            dispatchCmd.desx() > 10 &&
            dispatchCmd.desy() > 10 && dispatchCmd.road_size() > 0) // 调度坐标和路点都要有效
        {

            // 命令坐标与原坐标不一致，更新停车点信息,

            // cout<<dispatchCmd.desx()<<","<<dispatchCmd.desy()<<","<<stopPoints.size()<<endl;

            if ((abs(dispatchCmd.desx() + OFFSET_X - stopPoints[stopPoints.size() - 1].GaussX) >= 1) ||
                (abs(dispatchCmd.desy() + OFFSET_Y - stopPoints[stopPoints.size() - 1].GaussY) >= 1)) // 位置有变化,1米的偏差
            {

                // 查找停车点  roadID、laneID，pointID等信息并设置
                GaussRoadPoint roadPointTemp; // 命令中的停车点坐标
                roadPointTemp.GaussX = dispatchCmd.desx();
                roadPointTemp.GaussY = dispatchCmd.desy();

                // 停车点应该在命令中最后一条road上,

                int32_t roadIDTemp = -1;
                if (dispatchCmd.road_size() != 0) // 如果出现没有路点的异常情况，
                {
                    roadIDTemp = dispatchCmd.road(dispatchCmd.road_size() - 1); // 命令中的roadID
                }

                int32_t laneIDTemp = -1;  // 待检索的laneID
                int32_t pointIDTemp = -1; // 待检索的pointID

                FindPointIDatRoad(roadPointTemp, roadIDTemp, laneIDTemp, pointIDTemp);
                cout << "FindPointIDatRoad" << laneIDTemp << "," << pointIDTemp << endl;

                if (laneIDTemp != -1 && pointIDTemp != -1) // 找到ID
                {
                    // 设置停车点坐标
                    roadPointTemp.GaussX += OFFSET_X; // 其实不知道这里是在补偿什么，参照loadStopPoints，难道是安装位置
                    roadPointTemp.GaussY += OFFSET_Y;
                    stopPoints.clear();
                    stopPoints.push_back(roadPointTemp); // 注意第0点不被使用
                    stopPoints.push_back(roadPointTemp);
                    cout << "stopPoints GaussX" << stopPoints.size() << "," << stopPoints[1].GaussX << "stopPoints GaussX" << stopPoints[1].GaussY << endl;

                    // 设置停车点ID
                    std::get<0>(stopPointRoadLanePointId) = roadIDTemp;
                    std::get<1>(stopPointRoadLanePointId) = laneIDTemp;
                    std::get<2>(stopPointRoadLanePointId) = pointIDTemp;

                    // 基于服务器信息的全局规划

                    processorGlobalPlanningBaseServer();
                }
            }    //
            else // 无需更新停车点信息
            {
                // 基于服务器信息的全局规划
                // cout<<"processorGlobalPlanningBaseServer by old position"<<endl;
                processorGlobalPlanningBaseServer();
            }
        }    // if( dispatchCmd.curstatus() == Jobs::APPOINTMENT ||  //车辆被预约
        else // 车空闲，调度坐标无效
        {

            // 重新恢复文件中加载停车点
            stopPoints.assign(stopPointsFromLocal.begin(), stopPointsFromLocal.end());
            std::get<0>(stopPointRoadLanePointId) = std::get<0>(stopPointRoadLanePointIdFromLocal);
            std::get<1>(stopPointRoadLanePointId) = std::get<1>(stopPointRoadLanePointIdFromLocal);
            std::get<2>(stopPointRoadLanePointId) = std::get<2>(stopPointRoadLanePointIdFromLocal);

            // 本地算法的全局规划
            processorGlobalPlanningBaseLocal();
        }

        //=======end of your works======
        auto end = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

        std::this_thread::sleep_for(std::chrono::milliseconds(this->sendRate - duration.count()));
    }
}

// 基于服务器信息全局规划
void Jobs::processorGlobalPlanningBaseServer()
{
    cout << "processorGlobalPlanningBaseServer" << endl;
    mutex.lock();

    if (!getCurrentPosition(decisionData, imu, map)) // 获取当前路点
    {
        // 如果没有获取到路点，不做处理，局部规划自己会同样作不在路点判断和处理
        // 是不是这里应该把routingList清空？？？
        // std::cout << RED << "get current position failed :(" << RESET << std::endl;
    }
    else // 在路点上
    {
        Astar as;
        as.mapToAstar(map, &as);

        int destinationRoad = std::get<0>(stopPointRoadLanePointId);
        int destinationLane = std::get<1>(stopPointRoadLanePointId);
        int destinationPoint = std::get<2>(stopPointRoadLanePointId);
        int originRoad = decisionData.currentId;
        int originLane = decisionData.currentLaneId;
        int originPoint = decisionData.currentIndex;

        // 用调度指令中的道路ID赋值AStar道路信息
        // 有可能调度指令的道路ID包含已经走完的道路，从头开始找第一次与当前roidID相一致的路，把之前的删了
        //
        std::cout << "bFindfinishRoadIDIndexTemp = false:" << originRoad << std::endl;
        bool bFindfinishRoadIDIndexTemp = false;
        for (int i = 0; i < dispatchCmd.road_size(); i++)
        {
            std::cout << " as.path.push_back(dispatchCmd.road(i)):" << dispatchCmd.road(i) << std::endl;
            as.path.push_back(dispatchCmd.road(i));
            if (!bFindfinishRoadIDIndexTemp)
            {
                if (dispatchCmd.road(i) == originRoad)
                {
                    bFindfinishRoadIDIndexTemp = true;
                    as.path.clear();                        // 删掉之前的road
                    as.path.push_back(dispatchCmd.road(i)); // 当前road还是要补回来
                }
            }
        }
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // 由于苏州工业园区的地图很小，调度发送过来的全局规划存在缺失道路的情况，不用调度的全局规划，
        // 之前的调度处理还保留，但是在这里重新作本地的全局规划，将调度的规划覆盖掉了

        // std::cout << "origin >>RoadId" << originRoad << " >>LaneId" << originLane << " >>pointId" <<
        // decisionData.currentIndex << std::endl; std::cout << "destination >>RoadId" << destinationRoad << "
        // >>LaneId" << destinationLane << " >>pointId" << destinationPoint << std::endl;
        as.path = as.getPath(originRoad, destinationRoad, originPoint, destinationPoint); // 这一步确定roads

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        std::cout << "before------------ server as.path:" << as.path.size();
        for (list<int>::iterator iter = as.path.begin(); iter != as.path.end(); iter++)
        {
            std::cout << "," << (*iter);
        }
        std::cout << "------------" << std::endl;

        as.pathLanes.push_back(make_pair(originRoad, originLane)); // 初始化,将当前路点作为规划中的第一条路

        as.findLane(map, as.path);
        std::cout << "as.pathLanes.back().second:" << as.pathLanes.back().second << std::endl;
        if (as.pathLanes.back().second != destinationLane || as.pathLanes.back().first != destinationRoad)
            as.pathLanes.push_back(make_pair(destinationRoad, destinationLane)); // 保证终点的lane

        std::cout << std::endl << "* * * 用Astar找到的最短路为 : " << std::endl;
        as.moduleSelfCheckPrint(as.pathLanes);

        // 格式转换
        routingList.clear();
        routingList.reserve(as.pathLanes.size());
        for (int i = 0; i < as.pathLanes.size(); i++)
        {
            routingList.push_back(std::tuple<int32_t, int32_t>(as.pathLanes[i].first, as.pathLanes[i].second));
        }

    } // else // 在路点上

    mutex.unlock();
}

// 基于本地信息全局规划
void Jobs::processorGlobalPlanningBaseLocal()
{

    cout << "processorGlobalPlanningBaseLocal" << endl;
    mutex.lock();

    if (!getCurrentPosition(decisionData, imu, map)) // 获取当前路点
    {
        // 如果没有获取到路点，不做处理，局部规划自己会同样作不在路点判断和处理
        // 是不是这里应该把routingList清空？？？
        std::cout << RED << "get current position failed :(" << RESET << std::endl;
    }
    else // 在路点上
    {
        Astar as;
        as.mapToAstar(map, &as);

        int destinationRoad = std::get<0>(stopPointRoadLanePointId);
        int destinationLane = std::get<1>(stopPointRoadLanePointId);
        int destinationPoint = std::get<2>(stopPointRoadLanePointId);
        int originRoad = decisionData.currentId;
        int originLane = decisionData.currentLaneId;
        int originPoint = decisionData.currentIndex;

        std::cout << "origin >>RoadId" << originRoad << " >>LaneId" << originLane << " >>pointId" << decisionData.currentIndex << std::endl;
        std::cout << "destination >>RoadId" << destinationRoad << " >>LaneId" << destinationLane << " >>pointId" << destinationPoint << std::endl;
        as.path = as.getPath(originRoad, destinationRoad, originPoint, destinationPoint); // 这一步确定roads

        // std::cout << "------------as.path size = :" << as.path.size() << "Road ID:";
        // for (list<int>::iterator iter = as.path.begin(); iter != as.path.end(); iter++)
        // {
        //   std::cout << "," << (*iter);
        // }
        // std::cout << "------------" << std::endl;

        as.pathLanes.push_back(make_pair(originRoad, originLane)); // 初始化,将当前路点作为规划中的第一条路

        // as.findLane(map, as.path);
        as.findLane(map.roads, as.path, as.pathLanes, destinationLane);

        if (as.pathLanes.back().second != destinationLane)
            as.pathLanes.push_back(make_pair(destinationRoad, destinationLane)); // 保证终点的lane

        std::cout << std::endl << "* * * 用Astar找到的最短路为 : " << std::endl;
        as.moduleSelfCheckPrint(as.pathLanes);

        // 格式转换
        routingList.clear();
        routingList.reserve(as.pathLanes.size());
        for (int i = 0; i < as.pathLanes.size(); i++)
        {
            routingList.push_back(std::tuple<int32_t, int32_t>(as.pathLanes[i].first, as.pathLanes[i].second));
        }

    } // else // 在路点上

    mutex.unlock();
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

    cout << "planning main begin" << endl;
    void *context = zmq_ctx_new();
    int queueLength = 2;

    // imu receiving socket
    void *imuSocket = zmq_socket(context, ZMQ_SUB);
    zmq_setsockopt(imuSocket, ZMQ_RCVHWM, &queueLength, sizeof(queueLength));
    zmq_connect(imuSocket, "tcp://127.0.0.1:5003");
    zmq_setsockopt(imuSocket, ZMQ_SUBSCRIBE, "", 0);
    // prediction receiving socket
    void *predictionSocket = zmq_socket(context, ZMQ_SUB);
    zmq_setsockopt(predictionSocket, ZMQ_RCVHWM, &queueLength, sizeof(queueLength));
    zmq_connect(predictionSocket, "tcp://127.0.0.1:5009");
    zmq_setsockopt(predictionSocket, ZMQ_SUBSCRIBE, "", 0);
    // chassis receiving socket
    void *actuatorSocket = zmq_socket(context, ZMQ_SUB);
    zmq_connect(actuatorSocket, "tcp://127.0.0.1:3151");
    zmq_setsockopt(actuatorSocket, ZMQ_SUBSCRIBE, "", 0);

    // add  by syp 20221031 for pad UI app 交互通讯
    void *padUIReceiveSocket = zmq_socket(context, ZMQ_SUB);
    // zmq_connect(padUIReceiveSocket, "tcp://192.168.1.111:3161"); // 192.168.8.103
    zmq_connect(padUIReceiveSocket, "tcp://192.168.1.111:3161");
    zmq_setsockopt(padUIReceiveSocket, ZMQ_SUBSCRIBE, "", 0);
    // end of add  by syp

    std::vector<void *> receivingSocketList; // 0: imuSocket; 1:predictionSocket; 2:actuatorSocket
    receivingSocketList.push_back(imuSocket);
    receivingSocketList.push_back(predictionSocket);
    receivingSocketList.push_back(actuatorSocket);

    // add by syp 20221031 for pad UI app 交互通讯
    receivingSocketList.push_back(padUIReceiveSocket); // 4: UI 平板通讯
    // end of add  by syp

    // zmq sending sockets

    void *publisherSocket = zmq_socket(context, ZMQ_PUB);
    zmq_bind(publisherSocket, "tcp://127.0.0.1:5010");

    // add by syp 20221031
    // for pad UI app 交互通讯
    void *pubSocket2 = zmq_socket(context, ZMQ_PUB);
    int rc = zmq_bind(pubSocket2, "tcp://*:3101");
    if (rc < 0)
    {
        cout << "zmq_bind(pubSocket, tcp://*:3101)  rc = " << rc << "errno=" << errno << endl;
    }
    // end of add by syp

    std::vector<void *> sendingSocketList;
    sendingSocketList.push_back(publisherSocket);

    // add  by syp 20221031 for pad UI app 交互通讯
    sendingSocketList.push_back(pubSocket2);
    // end of add by syp

    // load road net
    std::cout << "before map initial: " << std::endl;
    RoadMap map(MAP_PATH);
    std::cout << "test for map initial: " << map.roads[0].successorId[0] << std::endl;

    // load stop point
    std::vector<GaussRoadPoint> rawStopPoints;
    std::tuple<int32_t, int32_t, int32_t> stopPointRoadLanePointId;
    // loadStopPoints(STOP_POINT_FILE_NAME, rawStopPoints, stopPointRoadLanePointId, OFFSET_X, OFFSET_Y);
    loadStopPointsFromYaml(map, STOP_POINT_FILE_NAME_YAML, rawStopPoints, stopPointRoadLanePointId, OFFSET_X,
                           OFFSET_Y); // 从yaml中读取文件

    std::vector<std::tuple<int32_t, int32_t>> routingList;
    std::cout << "raw stop points: " << rawStopPoints.size() << "," << rawStopPoints[0].GaussX << "," << rawStopPoints[1].GaussX << "," << rawStopPoints[1].GaussY << "," << rawStopPoints[1].yaw
              << std::endl;

    Jobs jobs(receivingSocketList, sendingSocketList, map, rawStopPoints, stopPointRoadLanePointId, routingList);

    thread thread1(&Jobs::subscriber, &jobs);
    thread thread2(&Jobs::publisher, &jobs);
    thread thread3(&Jobs::processorGlobalPlanning, &jobs);
    thread thread4(&Jobs::processorLocalPlanning, &jobs);
    //  add by syp
    //  与服务器通讯的线程
    thread thread5(&Jobs::request, &jobs);
    // end of add by syp

    thread::id threadID1 = thread1.get_id();
    thread::id threadID2 = thread2.get_id();
    thread::id threadID3 = thread3.get_id();
    thread::id threadID4 = thread4.get_id();
    // add by syp
    thread::id threadID5 = thread5.get_id();
    // end of add by syp

    thread1.detach();
    thread2.detach();
    thread3.detach();
    thread4.detach();
    // add by syp
    thread5.detach();
    // end of add by syp

    while (1)
    {
        // do what you want in main

        int thread1Status = pthread_kill(cvtThreadId2Long(threadID1), 0);
        // checkThreadStatus(thread1Status, threadID1);

        int thread2Status = pthread_kill(cvtThreadId2Long(threadID2), 0);
        // // checkThreadStatus(thread2Status, threadID2);

        int thread3Status = pthread_kill(cvtThreadId2Long(threadID3), 0);
        // // checkThreadStatus(thread3Status, threadID3);

        // int thread4Status = pthread_kill(cvtThreadId2Long(threadID4), 0);
        //  // checkThreadStatus(thread4Status, threadID4);

        // // add by syp
        int thread5Status = pthread_kill(cvtThreadId2Long(threadID5), 0);
        // // checkThreadStatus(thread4Status, threadID4);
        // //  end of add by syp

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    return 0;
}

}