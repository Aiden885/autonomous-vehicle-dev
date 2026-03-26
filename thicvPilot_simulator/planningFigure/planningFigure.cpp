#include "PlanningMsg.pb.h"
#include "defineColor.h"
#include "globalPlanning.h"
#include "localPlanning.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "planning.hpp"
#include "sys/time.h"
#include "unistd.h"
#include "ztgeographycoordinatetransform.h"
#include <chrono>
#include <thread>

// #define STOP_POINT_FILE_NAME "../mapMsg/stopPoint.txt" // TODO
// #define STOP_POINT_FILE_NAME_YAML "../mapMsg/stopPoint.yaml"
#define FILE_NAME_YAML "../mapMsg/stopPoint.yaml"
#define FILE_NAME_TRAFFICLIGHT "../mapMsg/trafficLight.txt"

#define RECV_PERIOD 40
#define SEND_PERIOD 100
#define GLOBAL_PLANNING_PERIOD 100 // 全局规划执行周期(ms)
#define LOCAL_PLANNING_PERIOD 100  // 局部规划执行周期(ms)

// by syp ??? 这个路径在调试环境和cmake中是不一致的
// #define MAP_PATH "../mapMsg/roadMap.xodr" // 地图位置TODO
// #define MAP_PATH "mapMsg/roadMap.xodr" //地图位置TODO

#define gridInOneMeter 4// 10车上 // 10 //40
#define mapSizeXMeter 240 // 350车上  // 160 本机全景//60//20
#define mapSizeYMeter 200 // 160车上  // 120本机全景//60//20
#define mapYZero 100
// #define gridInOneMeter 4  // 10车上 // 10 //40
// #define mapSizeXMeter 200 // 350车上  // 160 本机全景//60//20
// #define mapSizeYMeter 160 // 160车上  // 120本机全景//60//20
// #define mapYZero 300

// #define gridInOneMeter 4  // 10车上 // 10 //40
// #define mapSizeXMeter 200 // 350车上  // 160 本机全景//60//20
// #define mapSizeYMeter 160 // 160车上  // 120本机全景//60//20
// #define mapYZero 300
// add by syp
#define HEARTBEAT_PERIOD 500 // in millisecond 心跳或数据必达周期
auto lastTheadTime = std::chrono::steady_clock::now();
auto lastTheadTimeofSub = std::chrono::steady_clock::now();
auto durationFroShowofSub = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - lastTheadTimeofSub);
#define OBJECT_SOURCE_FROM_SERVER false // 障碍物的信息源是服务器 true， 车端设备，如常熟均联OBU、高铁新城  false

// end  of by syp

using namespace std;
using namespace cv;

Jobs::Jobs(std::vector<void *> &receivingSocketList, std::vector<void *> &sendingSocketList, RoadMap &m,
           std::vector<GaussRoadPoint> &sPoints, std::tuple<int32_t, int32_t, int32_t> &sPointRoadLanePointId) : recvRate(RECV_PERIOD), sendRate(SEND_PERIOD),
                                                                                                                 globalPlanningRate(GLOBAL_PLANNING_PERIOD), localPlanningRate(LOCAL_PLANNING_PERIOD),
                                                                                                                 rSocketList(receivingSocketList), sSocketList(sendingSocketList),
                                                                                                                 map(m), stopPoints(sPoints), stopPointRoadLanePointId(sPointRoadLanePointId) //,
                                                                                                                                                                                              // routingListVector(rList)
{

    lastPacketTime4Prediction = boost::posix_time::microsec_clock::local_time();
    lastPacketTrafficLightFromPerc = boost::posix_time::microsec_clock::local_time();

    lastPacketTimeCloudPrediction = boost::posix_time::microsec_clock::local_time();
    lastPacketTime4PredictionFromVision  = boost::posix_time::microsec_clock::local_time();

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
    stopTargetPathSource = 1;

    // end of add by syp
}

Jobs::~Jobs()
{
    ;
}

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
        // cout << "Jobs::request() ---------------------------------while()"<<endl;

        //////////////////////////////////////////////////////
        // void * serverReqSocket = nullptr;
        void *serverReqSocket = zmq_socket(context, ZMQ_REQ);
        if (serverReqSocket == nullptr)
        {
            cout << " serverReqSocket  error" << endl;
            return;
        }

        int rc = zmq_connect(serverReqSocket, "tcp://localhost:5501"); ////166.111.50.39 5501
        // int rc = zmq_connect(serverReqSocket, "tcp://192.168.6.12:15501"); // 李老师本机
        // int rc = zmq_connect(serverReqSocket, "tcp://166.111.50.39:5501"); // 清华融合服务器地址
        // int rc = zmq_connect(serverReqSocket, "tcp://58.210.18.98:5501"); // 苏州服务器地址
        // int rc = zmq_connect(serverReqSocket, "tcp://192.168.6.82:5501"); // 苏州李老师本机

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
            // claculate vehicle center
            double centergaussXTemp;
            double centergaussYTemp;
            double centerYawTemp;

            centerYawTemp = fmod((90. - imu.yaw() + 360.), 360.) / 180 * M_PI;
            centergaussYTemp = imu.gaussy() + cos(centerYawTemp);
            centergaussXTemp = imu.gaussx() + sin(centerYawTemp) * 1.3;
            // cout << std::setprecision(10) << "gaussx " << imu.gaussx() << " centergaussXTemp " << centergaussXTemp << "gaussy " << imu.gaussy() << " centergaussYTemp " << centergaussYTemp << "yaw" << imu.yaw() << "centerYawTemp" << centerYawTemp << endl;
            double centerLongitude, centerLatitude;
            ZtGeographyCoordinateTransform ztTranTemp;
            double xTemp, yTemp;
            ztTranTemp.BL2XY(imu.latitude(), imu.longitude(), xTemp, yTemp);
            ztTranTemp.XY2BL(centergaussYTemp, centergaussXTemp, centerLatitude, centerLongitude);

            infopack::ObjectsVec msgvec;
            infopack::ObjectsProto *fmsg = msgvec.add_objmsg();
            fmsg->set_type(myselfVehicleeType);  // 交通参与者：0 车辆
            fmsg->set_objectid(myselfVehicleID); // 交通参与者编号 ???目前没有这个信息，写成定值
            // fmsg->set_lat(imu.latitude());
            // fmsg->set_lon(imu.longitude());
            fmsg->set_lat(centerLatitude);
            fmsg->set_lon(centerLongitude);
            // fmsg->set_x(imu.gaussx());
            // fmsg->set_y(imu.gaussy());
            fmsg->set_x(centergaussXTemp);
            fmsg->set_y(centergaussYTemp);
            fmsg->set_yaw(imu.yaw());
            fmsg->set_velocity(imu.velocity());
            fmsg->set_power(100);
            fmsg->set_finish(true);
            fmsg->set_height(200);
            fmsg->set_len(450);
            fmsg->set_width(186);
            // chrono::time_point<chrono::system_clock, chrono::milliseconds> tp = chrono::time_point_cast<chrono::milliseconds>(chrono::system_clock::now());
            // int64_t ts = tp.time_since_epoch().count();
            // fmsg->set_timestamp(ts);
            // fmsg->set_timestamp(imu.time()*1000);//INS时间是北京时间的当天秒计数，单位double，转为整数毫秒
            //  double msTemp = imu.time() * 1000;
            //  int hhTemp = int(msTemp / 3600 / 1000);
            //  int mmTemp = int((msTemp - hhTemp * 3600 * 1000) / 60 / 1000);
            //  int ssTemp = int((msTemp - hhTemp * 3600 * 1000 - mmTemp * 60 * 1000) / 1000);
            //  int sssTemp = (int(msTemp)) % 1000;
            //  cout << "set_timestamp:" << setprecision(10) << msTemp << " HHMMSS " << hhTemp << " " << mmTemp << " " << ssTemp << " " << sssTemp << endl;
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

            // 20231010  syp根据接收到的车辆状态和自车与停车点的距离，设置状态
            // fmsg->set_status(0);
            std::cout << "stopPoints[1].yaw  =  " << stopPoints[1].yaw << std::endl;
            fmsg->set_status(dispatchCmd.curstatus()); // 来啥回啥
            double parkingDistance = 100;              // 车辆当前位置与停车点的距离
            double stopYawDiff;
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
                    parkingDistance = getDistance(imu.gaussx(), imu.gaussy(), stopPoints[1].GaussX, stopPoints[1].GaussY); // 终点坐标
                }

                if (parkingDistance <= (DISTANCETOPARK(imu.velocity()) + 10)) // 车辆停止距离有时候会比这个原
                {
                    if (dispatchCmd.curstatus() == 1)
                        fmsg->set_status(5);
                    else
                        fmsg->set_status(4);
                }
            }

            // std::cout << "dispatchCmd.curstatus() =  " << dispatchCmd.curstatus()  <<" fmsg->status()=" <<  fmsg->status()<<
            // "DISTANCETOPARK(imu.velocity()) = "  << DISTANCETOPARK(imu.velocity()) <<" stopYawDiff = "<< stopYawDiff <<" parkingDistance = "<<parkingDistance <<std::endl;

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
            if ((int)size != nSend)
            {
                // zmq_msg_close(&req);
                std::cerr << "send faliled... size =" << size << " length = " << nSend << std::endl;
                break;
            }
            else
            {
                // cout << "send successed size  =" << size << std::endl;
            }

            // std::cout << "send sucess!" << std::endl;
            zmq_msg_close(&req);
            // 清空緩存
            //  memset(buffer, 0, size * sizeof(char));
            if (NULL != buffer)
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
                    mutex.lock();

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
                            }
                            else
                            {
                                // gaussConvert(dispatchCmd.desx(), dispatchCmd.desy(), gaussXTemp, gaussYTemp);
                                gaussXTemp = dispatchCmd.desx();
                                gaussYTemp = dispatchCmd.desy();
                            }
                            dispatchCmd.set_desx(gaussXTemp);
                            dispatchCmd.set_desy(gaussYTemp);

                            // cout << "vehicleid:" << dispatchCmd.vehicleid() << "\t";
                            // cout << "curstatus:" << dispatchCmd.curstatus() << "\t";
                            // cout << "desx:" << std::setprecision(10) << dispatchCmd.desx() << "\t"; // gauss
                            // cout << "desy:" << std::setprecision(10) << dispatchCmd.desy() << "\n"; // gauss
                            // cout << "RoadID:";

                            // for (int j = 0; j < dispatchCmd.road_size(); j++)
                            // {
                            //     cout << dispatchCmd.road(j) << ",";
                            // }
                            // cout << "---------------------------------" << endl;
                        }
                    }

                    if (decisionListProto.objects_size() > 0 && OBJECT_SOURCE_FROM_SERVER)
                    {
                        objectsCmd.clear();
                        for (int i = 0; i < (int)decisionListProto.objects_size(); i++)
                        {
                            // if (decisionListProto.objects(i).type() == myselfVehicleeType && decisionListProto.objects(i).objectid() == myselfVehicleID) // 不显示自车
                            //  continue;

                            objectsCmd.push_back(decisionListProto.objects(i));
                        }

                        // predict
                        // cout << "OBJECT_SOURCE_FROM_SERVER  objectsCmd.size()@@@@@@@@@@@ = " << objectsCmd.size() << endl;
                        simplePredictionCmd.clear();
                        // simplePredictionCmd = SPD.simplePrediction(objectsCmd, imu);
                    }
                    // cout << "-------objectsCmd.size() = " << objectsCmd.size() << endl;

                    mutex.unlock();
                    //
                    double obiectXTemp = 0;
                    double obiectYTemp = 0;
                    for (int i = 0; i < (int)decisionListProto.objects_size(); i++)
                    {
                        if (decisionListProto.objects(i).type() == myselfVehicleeType && decisionListProto.objects(i).objectid() == myselfVehicleID)
                        {
                            obiectXTemp = decisionListProto.objects(i).x();
                            obiectYTemp = decisionListProto.objects(i).y();
                        }
                    }

                    // for (int i = 0; i < decisionListProto.objects_size(); i++)
                    // {
                    //     const infopack::ObjectsProto &objProto = decisionListProto.objects(i);
                    //     // cout << "type:" << objProto.type() << "\t";
                    //     cout << "objectid:" << objProto.objectid() << "\t";
                    //     cout << "lat:" << std::setprecision(10) << objProto.lat() << "\t";
                    //     cout << "lon:" << std::setprecision(10) << objProto.lon() << "\t";
                    //     cout << "x:" << std::setprecision(10) << objProto.x() << "\t";
                    //     cout << "y:" << std::setprecision(10) << objProto.y() << "\t";
                    //     cout << "yaw:" << std::setprecision(10) << objProto.yaw() << "\t";
                    //     cout << "velocity:" << std::setprecision(10) << objProto.velocity() << "\t";
                    //     //     cout << "power:" << objProto.power() << "\t";
                    //     //     cout << "finish:" << objProto.finish() << "\t";
                    //     //     cout << "height:" << objProto.height() << "\t";
                    //     cout << "len:" << objProto.len() << "\t";
                    //     cout << "width:" << objProto.width();
                    //     // cout << "timestamp:" << objProto.timestamp() << "\n";

                    //     cout << " diffX:" << std::setprecision(5) << (objProto.x() - obiectXTemp) << " diffY:" << std::setprecision(5) << (objProto.y() - obiectYTemp) << endl;
                    //     // cout << endl;
                    // }
                    // cout << "************decisionListProto**************" << endl;

                    if (NULL != str_recv)
                        free(str_recv);
                    zmq_msg_close(&reply);

                    //=======end of your works======
                    auto end = std::chrono::steady_clock::now();
                    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

                    int64 sleepspan = this->recvRate - duration.count();
                    sleepspan = max(sleepspan, int64(1));

                    // std::cout << "Thread request duration time = " << duration.count() <<" sleep time = " << (this->recvRate - duration.count()) <<" real sleep time = " <<sleepspan<< std::endl;

                    std::this_thread::sleep_for(std::chrono::milliseconds(sleepspan));

                    // std::this_thread::sleep_for(std::chrono::milliseconds(this->recvRate - duration.count()));
                    // std::this_thread::sleep_for(std::chrono::milliseconds(40 - duration.count()));

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
                // cout << "if (items[0].revents & ZMQ_POLLIN)   errno=" << errno << " errstr=" << zmq_strerror(errno) << endl;
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
        // std::cout << "------------------  subscriber   -while ------------------ " << std::endl; // 测试假的imu数据
        auto start = std::chrono::steady_clock::now();

        // =======do your works here======

        // by syp 20221031 for pad UI app 交互通讯
        //  zmq_pollitem_t items[3];
        //  items[0].socket = rSocketList[0];
        //  items[0].events = ZMQ_POLLIN;
        //  items[1].socket = rSocketList[1];
        //  items[1].events = ZMQ_POLLIN;
        //  items[2].socket = rSocketList[2];
        //  items[2].events = ZMQ_POLLIN;

        // zmq_poll(items, 3, 0);

        int sizeofReceiveSocket = 9; //现在有9个输入的socket了，越来越多了
        zmq_pollitem_t items[sizeofReceiveSocket];
        
        for (int i = 0; i < sizeofReceiveSocket; i++)
        {
            items[i].socket = rSocketList[i];
            items[i].events = ZMQ_POLLIN;
        }

        // items[0].socket = rSocketList[0];
        // items[0].events = ZMQ_POLLIN;
        // items[1].socket = rSocketList[1];
        // items[1].events = ZMQ_POLLIN;
        // items[2].socket = rSocketList[2];
        // items[2].events = ZMQ_POLLIN;
        // items[3].socket = rSocketList[3];
        // items[3].events = ZMQ_POLLIN;
        // items[4].socket = rSocketList[4];
        // items[4].events = ZMQ_POLLIN;
        // items[5].socket = rSocketList[5];
        // items[5].events = ZMQ_POLLIN;

        // 几个接收数据的临时变量及状态更新标识
        pc::Imu imuTemp; // 0、
        bool bImuTemp = false;
        prediction::ObjectList predictionTemp; // 1
        bool bPredictionTemp = false;

        ui::UiPad2VData uiPad2VdataTemp; // 3
        bool bUiPad2Vdata = false;

        // for 常熟 均联 OBU
        infopack::ObjectsVec objectsCmdCsjl;                // 常熟均联发送的路测信息
        std::vector<infopack::ObjectsProto> objectsCmdTemp; // 服务器消息中解析到的路测信息 4
        bool bObjectsCmdTemp = false;
        // 聚类julei
        infopack::ObjectsVec clusterObjectsCmdTemp; // 聚类消息中解析到的障碍物信息 8
        bool bClusterObjectsCmdTemp = false;
        std::vector<infopack::ObjectsProto> cluObjectsCmdTemp;
        prediction::ObjectList predictionTemplry; // 7
        bool bPredictionTemplry = false;
        // 栅格点云
        prediction::ObjectList predictioncloudTemp; // 1
        bool bPredictionCloudTemplry = false;

        std::map<std::string, infopack::IntersectionState> spatTrafficLightMapTemp; // 交通信号灯信息5
        bool bSpatTrafficLightMapTemp = false;

        infopack::TrafficLightFromPerc trafficLightFromPercTemp; // perception 发送的红绿灯信息
        bool bTrafficLightFromPercTemp = false;

        prediction::ObjectList predictionFromVisionTemp; // 9
        bool bPredictionFromVisionTemp = false;

        zmq_poll(items, sizeofReceiveSocket, 0);
        // end by syp

        if (items[0].revents & ZMQ_POLLIN)
        {
            // std::cout<<"items[0].revents & ZMQ_POLLIN---------------------------------------------"<<std::endl;//测试假的imu数据
            char msg[81920] = {0};
            int size = zmq_recv(rSocketList[0], msg, 81920, 0);
            if (size != -1)
            {
                if (!imuTemp.ParseFromArray(msg, size))
                {
                    std::cout << "ImuData parse error :(" << std::endl;
                }
                else
                {
                    bImuTemp = true;
                }
                // std::cout<<"imu data: "<<imu.gaussx()<<";" <<imu.gaussy()<<imu.gpsvalid()<<std::endl;//测试假的imu数据
            }
        }

        if (items[1].revents & ZMQ_POLLIN)
        {
            // std::cout << "items[1].revents & ZMQ_POLLIN---------------------------------------------" << std::endl; // 测试假的imu数据
            durationFroShowofSub = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - lastTheadTimeofSub);
            lastTheadTimeofSub = std::chrono::steady_clock::now();

            // std::cout << durationFroShowofSub.count() << "while -----------------------------------------------------------------------------------------" << std::endl; // 测试假的imu数据
            char msg[819200] = {0};
            int size = zmq_recv(rSocketList[1], msg, 819200, 0);

            // std::cout << "prediction data size: " << size << std::endl;
            if (size != -1)
            {
                if (!predictionTemp.ParseFromArray(msg, size))
                {
                    std::cout << "PredictionData parse error :(" << std::endl;
                }
                else
                {
                    bPredictionTemp = true;
                    lastPacketTime4Prediction = boost::posix_time::microsec_clock::local_time();
                }
            }
            // std::cout << "聚类"<<std::endl;
            // predictionClu=SPD2.simplePredictionClu(predictionTemp,imu);

            // std::cout << "prediction data size " << size << "prediction.object_size()   " << prediction.object_size() << std::endl; // 预测数据
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

        // // 障碍物

        // mutex.lock();
        //                 prediction.Clear();

        //             for(int k = 1; k<2; k++)
        //             {
        //                 prediction::Object *object1 = prediction.add_object();
        //                 for (int i =0; i <= 2; i++)
        //                 {
        //                     prediction::PredictPoint *predictPoint = object1->add_predictpoint();

        //                     predictPoint->set_x(15);
        //                     predictPoint->set_y(k);
        //                     predictPoint->set_vx(0);
        //                     predictPoint->set_vy(0);
        //                 }
        //                 object1->set_z(1);
        //                 object1->set_h(1);
        //                 object1->set_l(1);
        //                 object1->set_w(1);
        //                 object1->set_type(1);
        //                 object1->set_trackid(1);
        //             }

        //             //      for(int k = 0; k<10; k++)
        //             // {
        //             //     prediction::Object *object1 = prediction.add_object();
        //             //     for (int i =0; i <= 2; i++)
        //             //     {
        //             //         prediction::PredictPoint *predictPoint = object1->add_predictpoint();

        //             //         predictPoint->set_x(13);
        //             //         predictPoint->set_y(k*0.1);
        //             //         predictPoint->set_vx(0);
        //             //         predictPoint->set_vy(0);
        //             //     }
        //             //     object1->set_z(1);
        //             //     object1->set_h(1);
        //             //     object1->set_l(1);
        //             //     object1->set_w(1);
        //             //     object1->set_type(1);
        //             //     object1->set_trackid(1);
        //             // }
        //   mutex.unlock();
        //  for(int k = 2; k<4; k++)
        // {
        //     prediction::Object *object1 = prediction.add_object();
        //     for (int i =0; i <= 2; i++)
        //     {
        //         prediction::PredictPoint *predictPoint = object1->add_predictpoint();

        //         predictPoint->set_x(-5);
        //         predictPoint->set_y(k);
        //         predictPoint->set_vx(0);
        //         predictPoint->set_vy(0);
        //     }
        //     object1->set_z(1);
        //     object1->set_h(1);
        //     object1->set_l(1);
        //     object1->set_w(1);
        //     object1->set_type(1);
        //     object1->set_trackid(1);
        // }

        // fake 障碍物s
        //      predictionTemp.Clear();

        // for(int k = 0; k<1000; k++)
        // {
        //     prediction::Object *object1 = predictionTemp.add_object();
        //     for (int i =0; i <= 2; i++)
        //     {
        //         prediction::PredictPoint *predictPoint = object1->add_predictpoint();

        //         predictPoint->set_x(15);
        //         predictPoint->set_y(k*0.001);
        //         predictPoint->set_vx(0);
        //         predictPoint->set_vy(0);
        //     }
        //     object1->set_z(1);
        //     object1->set_h(1);
        //     object1->set_l(1);
        //     object1->set_w(1);
        //     object1->set_type(1);
        //     object1->set_trackid(1);
        // }

        // bPredictionTemp = true;
        ////////////////////////////////////////////////////////
        //  for(int k = 2; k<4; k++)
        // {
        //     prediction::Object *object1 = prediction.add_object();
        //     for (int i =0; i <= 2; i++)
        //     {
        //         prediction::PredictPoint *predictPoint = object1->add_predictpoint();

        //         predictPoint->set_x(-5);
        //         predictPoint->set_y(k);
        //         predictPoint->set_vx(0);
        //         predictPoint->set_vy(0);
        //     }
        //     object1->set_z(1);
        //     object1->set_h(1);
        //     object1->set_l(1);
        //     object1->set_w(1);
        //     object1->set_type(1);
        //     object1->set_trackid(1);
        // }

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
        // if (fabs(442781.922 - ego_y) < 7.5 && fabs(4429698.286 - ego_x) < 3 && fabs(ego_yaw - stop_line_yaw) < 15)
        // {
        //   trafficLight.Clear();
        //   trafficLight.set_active(true);
        //   trafficLight.set_lane_length_before_intersection(std::fabs(stop_line_y - ego_y));
        //   auto currentTime = std::chrono::steady_clock::now();
        //   double secondsSinceLaunch = std::chrono::duration_cast<std::chrono::nanoseconds>(currentTime - launchTime).count() * 1e-9;
        //   double time_in_cycle = std::fmod(secondsSinceLaunch + offset_time, cycle_time);
        //   if (time_in_cycle < green_time)
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
            zmq_recv(rSocketList[2], msg, 81920, 0);
            // int size = zmq_recv(rSocketList[2], msg, 81920, 0);
            //  if (size != -1)
            //  {
            //    if (!chassisInfo.ParseFromArray(msg, size))
            //    {
            //      std::cout << "chassisInfoData parse error :(" << std::endl;
            //    }
            //  }
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
                if (!uiPad2VdataTemp.ParseFromArray(msg, size))
                {
                    std::cout << "uiPad2Vdata parse error :(" << std::endl;
                }
                else
                {
                    uiPad2VdataTemp.set_b_isalive(true);
                    lastPacketTime4uiPad2V = boost::posix_time::microsec_clock::local_time();
                    bUiPad2Vdata = true;
                }
            }

        } // if (items[3].revents & ZMQ_POLLIN)

        boost::posix_time::time_duration diff;
        boost::posix_time::ptime now = boost::posix_time::microsec_clock::local_time();
        diff = now - lastPacketTime4uiPad2V;
        if (diff.total_milliseconds() > HEARTBEAT_PERIOD)
        {
            uiPad2VdataTemp.set_b_isalive(false);
            // std::cout << RED << "UI Communication DIED!!" << RESET << std::endl;
        }

        //      std::cout << "----------------------uiPad2Vdata.b_stopFlag = " << uiPad2Vdata.b_stopflag() << endl;
        //      std::cout << "----------------------uiPad2Vdata.b_speedOff = " << uiPad2Vdata.b_speedoff() << endl;
        //     // cout << "uiPad2Vdata.b_steeringOff = " << uiPad2Vdata.b_steeringoff() << endl;
        //     std::cout << "----------------------uiPad2Vdata.b_isValid = " << uiPad2Vdata.b_isvalid() << endl;
        //     // cout << "uiPad2Vdata.b_config1 = " << uiPad2Vdata.b_config1() << endl;
        //     std::cout << "---------------------uiPad2Vdata.b_isAlive = " << uiPad2Vdata.b_isalive() <<"diff = " <<diff.total_milliseconds() << endl;

        //  std::cout << " uiPad2Vdata.s_targetpath = " << uiPad2Vdata.s_targetpath() << endl;
        //  std::cout << " .uiPad2Vdata.destx() = " << uiPad2Vdata.destx() ;
        //  std::cout << " .desty() = " << uiPad2Vdata.desty() ;
        // std::cout << " .dest_isvalid() = " << uiPad2Vdata.dest_isvalid() ;
        //  std::cout << " . uiPad2Vdata.destrid() = " << uiPad2Vdata.destrid() << endl;
        // end by syp

        // by syp 20230423常熟均联OBU
        // infopack::ObjectsVec msgObjectVecCsjl;
        if (items[4].revents & ZMQ_POLLIN)
        {
            char msg[819200] = {0};
            int size = zmq_recv(rSocketList[4], msg, 819200, 0);
            // std::cout << RED<<"pad msgObjectVecCsjl  get size = "<<rSocketList[4]<< RESET<<std::endl;
            // std::cout << RED<<"pad msgObjectVecCsjl  get size = "<<size <<","<<msg<< RESET<<std::endl;
            if (size != -1)
            {
                if (!objectsCmdCsjl.ParseFromArray(msg, size))
                {
                    std::cout << "objectsCmdCsjl parse error :(" << std::endl;
                }

                for (int i = 0; i < objectsCmdCsjl.objmsg_size(); i++)
                {
                    const infopack::ObjectsProto objCsjlProto = objectsCmdCsjl.objmsg(i);
                    // std::cout  <<"objectsCmdCsjl.objmsg(i)"<< i << " " <<std::setprecision(10) <<   objCsjlProto.lat() << " " << objCsjlProto.lon()   << " "<< objCsjlProto.yaw()
                    //  << " "<< objCsjlProto.velocity()<<std::endl;
                }
                if (objectsCmdCsjl.objmsg_size() > 0 && imu.longitude() > 10 && imu.latitude() > 10 && imu.gpsvalid() > 0 && (!OBJECT_SOURCE_FROM_SERVER))
                {
                    objectsCmdTemp.clear();
                    for (int i = 0; i < (int)objectsCmdCsjl.objmsg_size(); i++)
                    {
                        // 这里得自己剔除自车了
                        //  if (decisionListProto.objects(i).type() == myselfVehicleeType && decisionListProto.objects(i).objectid() == myselfVehicleID) // 不显示自车
                        //   continue;
                        // 先不转换到车辆坐标系，简单粗暴比较经纬度
                        if (abs(objectsCmdCsjl.objmsg(i).lon() - imu.longitude()) < 2 / 1.0e5 && abs(objectsCmdCsjl.objmsg(i).lat() - imu.latitude()) < 2 / 1.0e5 &&
                            abs(objectsCmdCsjl.objmsg(i).yaw() - imu.yaw()) < 10)
                        {
                            // objectsCmdCsjl.objmsg(i).set_type (6);//myselfVehicleeType) ;这样硬赋值还不对，
                            // objectsCmdCsjl.objmsg(i).set_objectid( 1);//myselfVehicleID);
                            continue;
                        }

                        objectsCmdTemp.push_back(objectsCmdCsjl.objmsg(i));
                        bObjectsCmdTemp = true;
                    }
                    // predict
                    // cout << "路测信息预测开始了objectsCmdTemp.size()= " << objectsCmdTemp.size() << endl;
                    simplePredictionCmdClu.clear();
                    // simplePredictionCmdClu = SPD1.simplePrediction(objectsCmdTemp, imu);
                }
            }

            // std::cout << "objectsCmdCsjl data size " << size << "objectsCmdCsjl. object_size()   " << objectsCmdCsjl.objmsg_size() << std::endl; // 常熟均联OBU
        }

        // 创建fake 障碍物 for unity
        // cout << "---------------------------------create   fake objectsCmd"  << endl;
        // objectsCmd.clear();
        // infopack::ObjectsProto fakeObjectTemp;
        // fakeObjectTemp.set_type(1);

        // fakeObjectTemp.set_len(100);
        // fakeObjectTemp.set_width(100);

        // //常熟
        // // double xx = 3477841.593;
        // // double yy =  560885.281 +4;
        // // double yaw = 122.385;
        // //高铁新城 <roadPoint gaussX="3478247.078" gaussY="560227.381" yaw="121.397" curvature="0.0" s="68.1" speedMax="4"/>
        // //<roadPoint gaussX="3478252.287" gaussY="560218.845" yaw="121.397" curvature="0.0" s="58.1" speedMax="4"/>前方卡死
        // //	<roadPoint gaussX="3478269.323" gaussY="560190.933" yaw="121.397" curvature="0.0" s="25.4" speedMax="4"/> 后方安全
        // //<roadPoint gaussX="3478265.572" gaussY="560197.079" yaw="121.397" curvature="0.0" s="32.6" speedMax="4"/>后方卡死
        // // <roadPoint gaussX="3478262.029" gaussY="560202.883" yaw="121.397" curvature="0.0" s="39.4"

        //  double xx = 3478252;
        //  double yy =  560218;
        //  double yaw = 121.397;
        // fakeObjectTemp.set_x(xx );
        // fakeObjectTemp.set_y( yy );
        // fakeObjectTemp.set_yaw(yaw);
        // ZtGeographyCoordinateTransform ztTranTempFake;
        // ztTranTempFake.meridianLine = 120;
        // double  latTempFake, lonTempFake;
        // //ztTranTempFake.BL2XY(imu.latitude(), imu.longitude(), xTemp, yTemp);
        //  ztTranTempFake.XY2BL(yy, xx, latTempFake, lonTempFake);
        //  fakeObjectTemp.set_lat(latTempFake);
        //  fakeObjectTemp.set_lon(lonTempFake);
        // objectsCmd.push_back(fakeObjectTemp);

        // xx = 3477830.733;//3477830.733" gaussY="560902.073" yaw="123.99
        // yy =  560902.073  - 4;
        //  yaw = 122.554;
        // fakeObjectTemp.set_x(xx );
        // fakeObjectTemp.set_y( yy );
        // fakeObjectTemp.set_yaw(yaw);
        // //ZtGeographyCoordinateTransform ztTranTempFake;
        // //ztTranTempFake.meridianLine = 120;
        // //double  latTempFake, lonTempFake;
        // //ztTranTempFake.BL2XY(imu.latitude(), imu.longitude(), xTemp, yTemp);
        //  ztTranTempFake.XY2BL(yy, xx, latTempFake, lonTempFake);
        //  fakeObjectTemp.set_lat(latTempFake);
        //  fakeObjectTemp.set_lon(lonTempFake);
        // objectsCmd.push_back(fakeObjectTemp);

        // by syp 20230711 高铁新城红绿灯信息
        //  infopack::SPAT spatTrafficLightl; //交通信号灯信息
        if (items[5].revents & ZMQ_POLLIN)
        {
            char msg[81920] = {0};
            int size = zmq_recv(rSocketList[5], msg, 81920, 0);
            // std::cout << RED<<"spatTrafficLightl Data get size = "<<rSocketList[5]<<RESET<<std::endl;
            // std::cout << "spatTrafficLightl Data get size = " << size << std::endl;
            if (size != -1)
            {
                infopack::SPAT spatTrafficLightTemp;
                if (!spatTrafficLightTemp.ParseFromArray(msg, size))
                {
                    std::cout << "spatTrafficLightTemp parse error :(" << std::endl;
                }
                else
                {

                    // std::cout  <<"spatTrafficLight.msgcnt() "<<spatTrafficLight.msgcnt()<<std::endl;
                    // std::cout  <<"RRRR   patTrafficLight.moy() "<<spatTrafficLightTemp.moy()<<std::endl;
                    // std::cout  <<"RRRR   patTrafficLight.timestamp() "<<spatTrafficLightTemp.timestamp()<<std::endl;
                    // std::cout  <<"RRRR   timestamp() "<<spatTrafficLightTemp.moy()%60 << ":"<<spatTrafficLightTemp.timestamp()<<std::endl;

                    // time_t now_time;
                    // now_time = time(NULL);
                    // cout<<now_time;

                    // std::cout  <<"spatTrafficLightTemp.name() "<<spatTrafficLightTemp.name()<<std::endl;
                    // std::cout  <<"spatTrafficLight.intersections_size() "<<spatTrafficLight.intersections_size() <<std::endl;
                    for (int i = 0; i < (int)spatTrafficLightTemp.intersections_size(); i++) // 每个路口
                    {
                        const infopack::IntersectionState intersectionState = spatTrafficLightTemp.intersections(i);

                        //   if(intersectionState.intersectionid().compare("G32050700040") == 0 )
                        //         std::cout << "RRRR   spatTrafficLight.intersections(i).intersectionid " << intersectionState.intersectionid() << std::endl;

                        // spatTrafficLightMap.insert(pair<string, infopack::IntersectionState>(intersectionState.intersectionid(),intersectionState));//insert添加元素
                        spatTrafficLightMapTemp[intersectionState.intersectionid()] = intersectionState; // insert添加元素
                        // std::map<std::string,infopack::SPAT > spatTrafficLightMap; //交通信号灯信息
                        bSpatTrafficLightMapTemp = true;

                        // std::cout << "spatTrafficLight.intersections(i).phases_size(); " << spatTrafficLightTemp.intersections(i).phases_size() << std::endl;
                        // for (int j = 0; j < (int)spatTrafficLightTemp.intersections(i).phases_size(); j++)
                        // {
                        //     // if(intersectionState.intersectionid().compare("G32050700054") == 0 )
                        //     //     continue;
                        //     const infopack::Phase phase = spatTrafficLightTemp.intersections(i).phases(j);
                        //     std::cout << "phaseid " << phase.phaseid();
                        //     std::cout << ", " << phase.lightstate();
                        //     std::cout << ", " << phase.starttime();
                        //     std::cout << ", " << phase.likelyendtime() << std::endl;
                        // }
                    }
                }
            }

            // std::cout << "spatTrafficLightl data size " << size << "objectsCmdCsjl. object_size()   " << objectsCmdCsjl.objmsg_size() << std::endl; // 常熟均联OBU

        } // if (items[5].revents & ZMQ_POLLIN)

        // by syp 20230902 自车红绿灯信息

        if (items[6].revents & ZMQ_POLLIN)
        {
            char msg[81920] = {0};
            int size = zmq_recv(rSocketList[6], msg, 81920, 0);
            // std::cout << RED<<"spatTrafficLightl Data get size = "<<rSocketList[5]<<RESET<<std::endl;
            std::cout << RED << "trafficLightFromPercTemp Data get size = " << size << RESET << std::endl;
            if (size != -1)
            {
                if (!trafficLightFromPercTemp.ParseFromArray(msg, size))
                {
                    std::cout << "trafficLightFromPercTemp parse error :(" << std::endl;
                }
                else
                {
                    if (trafficLightFromPercTemp.trafficlightgaussx() > 10 && trafficLightFromPercTemp.trafficlightgaussx() > 10 && trafficLightFromPercTemp.active() == true)
                    {
                        lastPacketTrafficLightFromPerc = boost::posix_time::microsec_clock::local_time();
                        bTrafficLightFromPercTemp = true;
                    }

                    std::cout << RED << "trafficLightFromPercTemp.trafficlightgaussx() = " << trafficLightFromPercTemp.trafficlightgaussx() << ","
                              << trafficLightFromPercTemp.state() << RESET << std::endl;
                }
            }
        } // if (items[6].revents & ZMQ_POLLIN)

        // 栅格点云 lry
        if (items[7].revents & ZMQ_POLLIN)
        {
            char msg[819200] = {0};
            int size = zmq_recv(rSocketList[7], msg, 819200, 0);
            // std::cout << "prediction data size: " << size << std::endl;
            if (size != -1)
            {
                if (!predictioncloudTemp.ParseFromArray(msg, size))
                {
                    std::cout << " PredictioncloudData parse error :(" << std::endl;
                }
                else
                {
            std::cout << "正确收到栅格点云信息" << std::endl;

                    bPredictionCloudTemplry = true;
                    lastPacketTime4Prediction = boost::posix_time::microsec_clock::local_time();
                }
            }
            std::cout << "收到栅格点云信息" << std::endl;
        }

         if (items[8].revents & ZMQ_POLLIN)//视觉感知障碍物信息
        {
            // std::cout << "items[8].revents & ZMQ_POLLIN---------------------------------------------" << std::endl; // 测试假的imu数据
            char msg[819200] = {0};
            int size = zmq_recv(rSocketList[8], msg, 819200, 0);

            // std::cout << "prediction data size: " << size << std::endl;
            if (size != -1)
            {
                if (!predictionFromVisionTemp.ParseFromArray(msg, size))
                {
                    std::cout << "PredictionFromVisionData parse error :(" << std::endl;
                }
                else
                {
                    bPredictionFromVisionTemp = true;
                    lastPacketTime4PredictionFromVision = boost::posix_time::microsec_clock::local_time();
                }
            }
            // std::cout << "聚类"<<std::endl;
            // predictionClu=SPD2.simplePredictionClu(predictionTemp,imu);

            // std::cout << "prediction data size " << size << "prediction.object_size()   " << prediction.object_size() << std::endl; // 预测数据
        }
        // // 聚类预测 lry
        // if (items[7].revents & ZMQ_POLLIN)
        // {
        //     char msg[819200] = {0};
        //     int size = zmq_recv(rSocketList[7], msg, 819200, 0);
        //     // std::cout << "prediction data size: " << size << std::endl;
        //     if (size != -1)
        //     {
        //         if (!predictionTemplry.ParseFromArray(msg, size))
        //         {
        //             std::cout << "cluster PredictionData parse error :(" << std::endl;
        //         }
        //         else
        //         {
        //             bPredictionTemplry = true;
        //             lastPacketTime4Prediction = boost::posix_time::microsec_clock::local_time();
        //         }
        //     }
        //     std::cout << "收到激光聚类信息" << std::endl;
        //     // 激光聚类输入高斯坐标转经纬度，并加入drawforInt，单独画图
        //     drawForInt.clear();
        //     ZtGeographyCoordinateTransform ztTranTemp1;
        //     ztTranTemp1.meridianLine = 120;
        //     for (int i = 0; i < predictionTemplry.object_size(); i++)
        //     {
        //         double latTemp, lonTemp, gaussx, gaussy;
        //         gaussx = predictionTemplry.object(i).predictpoint(0).x();
        //         gaussy = predictionTemplry.object(i).predictpoint(0).y();
        //         ztTranTemp1.XY2BL(gaussy, gaussx, latTemp, lonTemp);
        //         // std::cout << "gaussx:" << latTemp << "gaussy: " << lonTemp << std::endl;
        //         // prediction::Object tempPrid = predictionTemplry.object(i);
        //         (*predictionTemplry.mutable_object(i)).set_z(lonTemp);
        //         // predictionTemplry.mutable_object(i)->set_h(latTemp);
        //         (*predictionTemplry.mutable_object(i)).set_h(latTemp);
        //         // std::cout << "gaussx:" << gaussx << "gaussy: " << gaussy << std::endl;
        //         // std::cout << "predictionTemplry.object(i).z():" << std::setprecision(10) << predictionTemplry.object(i).z()
        //         //           << " predictionTemplry.object(i).h():" << predictionTemplry.object(i).h() << std::endl;
        //         drawForInt.push_back(predictionTemplry.object(i));
        //     }
        //     predictionTempGlobal.Clear();
        //     predictionTempGlobal = predictionTemplry; // 转存一下
        //     // 激光聚类预测函数
        //     // std::cout << "激光聚类预测开始了predictionTemplry.object_size():" << predictionTempGlobal.object_size() << std::endl;
        //     predictionClu.clear();
        //     // predictionClu = SPD2.simplePredictionClu(predictionTempGlobal, imu);
        // }
        // 障碍物
        // {
        //     mutex.lock();
        //     predictionTemplry.Clear();

        //     // for(int k = 1; k<2; k++)
        //     // {
        //     prediction::Object *object = predictionTemplry.add_object();
        //     // for (int i =0; i <=0; i++)
        //     // {
        //     prediction::PredictPoint *predictPoint = object->add_predictpoint();

        //     // predictPoint->set_x(3478542.032+((imu.time()-(int) imu.time()))*10);
        //     predictPoint->set_x(3478542.032+((imu.time()-(int) imu.time()))*10); // 高斯x
        //     predictPoint->set_y(559627.015);
        //     predictPoint->set_vx(1);
        //     // predictPoint->set_vy(105.16);
        //     predictPoint->set_vy(imu.yaw());
        //     // }
        //     object->set_z(0); // lat
        //     // object->set_h(120.62714822 +((imu.time()-(int) imu.time()))*0.0001);
        //     object->set_h(0); // lon
        //     object->set_l(10);
        //     object->set_w(5);
        //     object->set_type(1);
        //     object->set_trackid(1);
        //     // }

        //     std::cout << "聚类2" << std::endl;
        //     ZtGeographyCoordinateTransform ztTranTemp2;
        //     ztTranTemp2.meridianLine = 120;
        //     for (int i = 0; i < predictionTemplry.object_size(); i++)
        //     {
        //         double latTemp, lonTemp, gaussx, gaussy;
        //         gaussx = predictionTemplry.object(i).predictpoint(0).x();
        //         gaussy = predictionTemplry.object(i).predictpoint(0).y();

        //         //  ztTranTemp.XY2BL( dEast_Y, dNorth_X,latitude1, longitude1);
        //         ztTranTemp2.XY2BL(gaussy, gaussx, latTemp, lonTemp);
        //         // std::cout << "gaussx:" << latTemp << "gaussy: " << lonTemp << std::endl;
        //         // prediction::Object tempPrid = predictionTemplry.object(i);

        //         (*predictionTemplry.mutable_object(i)).set_z(lonTemp);
        //         (*predictionTemplry.mutable_object(i)).set_h(latTemp);
        //         // std::cout << "gaussx:" << gaussx << "gaussy: " << gaussy << std::endl;
        //         // std::cout << "predictionTemplry.object(i).z():" << std::setprecision(10) << predictionTemplry.object(i).z()
        //         //           << " predictionTemplry.object(i).h():" << predictionTemplry.object(i).h() << std::endl;
        //     }
        //     predictionClu = SPD2.simplePredictionClu(predictionTemplry, imu);
        //     //      for(int k = 0; k<10; k++)
        //     // {
        //     //     prediction::Object *object1 = prediction.add_object();
        //     //     for (int i =0; i <= 2; i++)
        //     //     {
        //     //         prediction::PredictPoint *predictPoint = object1->add_predictpoint();

        //     //         predictPoint->set_x(13);
        //     //         predictPoint->set_y(k*0.1);
        //     //         predictPoint->set_vx(0);
        //     //         predictPoint->set_vy(0);
        //     //     }
        //     //     object1->set_z(1);
        //     //     object1->set_h(1);
        //     //     object1->set_l(1);
        //     //     object1->set_w(1);
        //     //     object1->set_type(1);
        //     //     object1->set_trackid(1);
        //     // }
        //       mutex.unlock();
        //     // std::cout << "聚类2" << std::endl;
        //     // predictionClu = SPD2.simplePredictionClu(predictionTemp, imu);
        //     // mutex.unlock();
        //     // for(int i=0;i<predictionTemp.object_size();i++){
        //     //     std::vector<prediction::Object> predictionClu1;
        //     //     predictionClu1.push_back(predictionTemp.object(i));
        //     // }
        //     // std::cout << "objectsCmdCsjl data size " << size << "objectsCmdCsjl. object_size()   " << objectsCmdCsjl.objmsg_size() << std::endl; // 常熟均联OBU
        // }
        // boost::posix_time::time_duration diff;


        now = boost::posix_time::microsec_clock::local_time();

        // 在这里更新数据给处理线程，
        //  if(mutex.try_lock())//如果可以进入，更新数据，否则继续循环
        {
            mutex.lock();

            // 数据更新或超时清空

            if (bImuTemp)
                imu = imuTemp; // 0、

            if (bPredictionTemp)
                prediction = predictionTemp; // 1

            diff = now - lastPacketTime4Prediction; // 长时间未收到数据，清空
            if (diff.total_milliseconds() > HEARTBEAT_PERIOD)
            {
                prediction.Clear();
                std::cout << " prediction diff.total_milliseconds() > HEARTBEAT_PERIOD " << diff.total_milliseconds() << std::endl;
                // exit(0);
            }

            if (bUiPad2Vdata)
                uiPad2Vdata = uiPad2VdataTemp; // 3

            if (bObjectsCmdTemp)
                objectsCmd = objectsCmdTemp; // 服务器消息中解析到的路测信息 4

            if (bSpatTrafficLightMapTemp)
            {
                bSpatTrafficLightMapTemp = false;

                // spatTrafficLightMap =  spatTrafficLightMapTemp; //交通信号灯信息5
                std::map<std::string, infopack::IntersectionState>::iterator it;
                for (it = spatTrafficLightMapTemp.begin(); it != spatTrafficLightMapTemp.end(); it++)
                {
                    spatTrafficLightMap[it->first] = it->second;
                }
            }

            if (bTrafficLightFromPercTemp) // 自车感知识别的红绿灯信息
            {
                bTrafficLightFromPercTemp = false;
                trafficLightFromPerc = trafficLightFromPercTemp;
            }

            diff = now - lastPacketTrafficLightFromPerc; // 长时间未收到数据，清空
            if (diff.total_milliseconds() > HEARTBEAT_PERIOD)
            {
                trafficLightFromPerc.Clear();
            }

            // std::cout << "renew Thread subscriber duration time =+++++ "<< std::endl;
            predictioncloud = predictioncloudTemp;      // 栅格点云加入全局变量
            diff = now - lastPacketTimeCloudPrediction; // 长时间未收到数据，清空
            if (diff.total_milliseconds() > HEARTBEAT_PERIOD)
            {
                predictioncloud.Clear();
            }

            if (bPredictionFromVisionTemp)
                predictionFromVision = predictionFromVisionTemp; // 8视觉感知障碍物信息
              
            diff = now - lastPacketTime4PredictionFromVision; // 长时间未收到数据，清空
            if (diff.total_milliseconds() > HEARTBEAT_PERIOD)
            {
                predictionFromVision.Clear();
                std::cout << " predictionFromVision diff.total_milliseconds() > HEARTBEAT_PERIOD " << diff.total_milliseconds() << std::endl;
               
            }


            mutex.unlock();
        }
        // else
        // {
        //      std::cout << "Thread subscriber cannot get thread lock  =------------- "<< std::endl;
        // }

        //=======end of your works======
        auto end = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

        int64 sleepspan = this->recvRate - duration.count();
        sleepspan = max(sleepspan, int64(1));

        // std::cout << "Thread subscriber duration time = " << duration.count() <<" sleep time = " << (this->recvRate - duration.count()) <<" real sleep time = " <<sleepspan<< std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(sleepspan));
        // std::this_thread::sleep_for(std::chrono::milliseconds(20));
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

// 20230725将画图从publish函数中提取出来，先放在这里，后续建立一个独立线程专门用于画图
void Jobs::Draw()
{
    while (1)
    {
        auto start = std::chrono::steady_clock::now();

        // =======do your works here======
        mutex.lock();
        // 画图
        int mapSizeX = mapSizeXMeter * gridInOneMeter;
        int mapSizeY = mapSizeYMeter * gridInOneMeter;

        // 画图
        Mat img = Mat::zeros(Size(mapSizeX, mapSizeY), CV_8UC3);
        img.setTo(255); // 设置屏幕为白色
        double vehicleWidth = 2.0;
        double vehicleLengthfront = 3.8; // 定位中心到车头位置
        double vehicleLengthback = 0.7;  // 定位中心到车back位置

        cv::Point point1;
        cv::Point point2;
        point1.x = int(mapSizeX / 2 - vehicleWidth * gridInOneMeter / 2.0);
        point1.y = mapSizeY + vehicleLengthback * gridInOneMeter;
        point2.x = int(mapSizeX / 2 + vehicleWidth * gridInOneMeter / 2.0);
        point2.y = mapSizeY - vehicleLengthfront * gridInOneMeter;
        rectangle(img, drawRegulate(point1), drawRegulate(point2), Scalar(0, 0, 255), -1); // 画自车位置

        // add by ztz20230629
        //  draw speed curve

        //        Point origin(mapSizeX / 2 + 500, 150 + offset_y);
        Point origin(500, mapSizeY - 5);
        int offset_y = origin.y - 150;
        line(img, Point(origin.x - 500, origin.y), Point(img.cols, origin.y), Scalar(0, 0, 0), 1);
        line(img, Point(origin.x, 0 + offset_y), Point(origin.x, 150 + offset_y), Scalar(0, 0, 0), 1);
        line(img, Point(origin.x - 500, 100 + offset_y), Point(img.cols, 100 + offset_y), Scalar(96, 96, 96), 1);
        line(img, Point(origin.x - 500, 50 + offset_y), Point(img.cols, 50 + offset_y), Scalar(96, 96, 96), 1);
        line(img, Point(origin.x - 500, 0 + offset_y), Point(img.cols, 0 + offset_y), Scalar(96, 96, 96), 1);
        for (int i = 0; i < 151; i += 5)
        {
            if (i % 50 == 0)
                continue;
            if (i % 25 == 0)
            {
                line(img, Point(origin.x - 500, i + offset_y), Point(img.cols, i + offset_y), Scalar(168, 168, 168), 1);
            }
            else
            {
                line(img, Point(origin.x - 500, i + offset_y), Point(img.cols, i + offset_y), Scalar(224, 224, 224), 1);
            }
        }
        for (int i = 50; i < 501; i += 50)
        {
            line(img, Point(origin.x - i, 0 + offset_y), Point(origin.x - i, 150 + offset_y), Scalar(224, 224, 224), 1);
        }

        if (history_velocity.size() == 100)
            history_velocity.pop_front();
        // history_velocity.push_back(double(random())/RAND_MAX * 30.0);
        history_velocity.push_back(imu.velocity());
        if (!pause_draw_history_flag)
        {
            showing_velocity = history_velocity;
        }

        putText(img, "current velocity::" + to_string(showing_velocity.back()), Point(origin.x, origin.y - 10),
                FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1, 4);
        Point prev_point;
        int iter = 0;
        for (auto &hv : showing_velocity)
        {
            Point pt(origin.x + (iter - showing_velocity.size()) * 5, 150 + offset_y - (hv * 25));
            if (iter > 0)
            {
                line(img, prev_point, pt, Scalar(0, 0, 255), 1);
            }
            prev_point = pt;
            iter++;
        }

        if (history_planning_speed.size() == 100)
            history_planning_speed.pop_front();
        // history_planning_speed.push_back(double(random())/RAND_MAX * 30.0);
        if (!decisionData.optimalGlobalTrajectory.planningPoints.empty())
        {
            history_planning_speed.push_back(decisionData.optimalGlobalTrajectory.planningPoints.at(1).v);
        }
        else
        {
            history_planning_speed.push_back(0);
        }

        if (!pause_draw_history_flag)
        {
            showing_planning_speed = history_planning_speed;
        }
        putText(img, "planning speed::" + to_string(showing_planning_speed.back()), Point(origin.x, origin.y - 60),
                FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0), 1, 4);
        Point prev_point_planning;
        iter = 0;
        bool equal_speed_and_velocity = std::fabs(showing_planning_speed.back() - showing_velocity.back()) > 0.01;
        for (auto &hv : showing_planning_speed)
        {
            Point pt(origin.x + (iter - showing_planning_speed.size()) * 5, 150 + offset_y - (hv * 25));
            if (iter > 0)
            {
                if (equal_speed_and_velocity)
                {
                    line(img, prev_point_planning, pt, Scalar(255, 0, 0), 1);
                }
                else
                {
                    line(img, prev_point_planning, pt, Scalar(0, 255, 0), 1);
                }
            }
            prev_point_planning = pt;
            iter++;
        }

        setMouseCallback(
            "globalTrajectoryPoints", [](int event, int x, int y, int flags, void *param)
            {
            auto self = reinterpret_cast<Jobs*>(param);
            self->onMouse(event, x, y, flags); },
            this);

        // end of ztz

        ///////////////////////////////////////////////////////////////////////////
        // by syp 20220920
        string strTemp;

        // 20231012

        Scalar textColor(0, 51, 0); // 提示文字的颜色
        string strTemp1;
        int textYPosition = 20; // 提示文字在y方向上的位置
        int textYSpace = 20;    // 提示文字在y方向上的间距

        strTemp = "IMU Time=";
        strTemp1 = to_string(imu.time());
        strTemp1 = strTemp1.substr(0, strTemp1.find(".") + 4);
        strTemp += strTemp1;
        strTemp += ", X=";
        strTemp1 = to_string(imu.gaussx());
        strTemp1 = strTemp1.substr(0, strTemp1.find(".") + 3);
        strTemp += strTemp1;
        strTemp += ", Y=";
        strTemp1 = to_string(imu.gaussy());
        strTemp1 = strTemp1.substr(0, strTemp1.find(".") + 3);
        strTemp += strTemp1;
        strTemp += ", Yaw=";
        strTemp1 = to_string(imu.yaw());
        strTemp1 = strTemp1.substr(0, strTemp1.find(".") + 3);
        strTemp += strTemp1;
        strTemp += ", Speed=";
        strTemp1 = to_string(imu.velocity());
        strTemp1 = strTemp1.substr(0, strTemp1.find(".") + 3);
        strTemp += strTemp1;

        putText(img, strTemp, cv::Point(10, textYPosition), FONT_HERSHEY_SIMPLEX, 0.5, textColor, 1, 4);
        textYPosition += textYSpace;

        strTemp = "RoadID=" + to_string(decisionData.currentId) + ", LaneID=" + to_string(decisionData.currentLaneId) + ", PointID=" + to_string(decisionData.currentIndex);
        putText(img, strTemp, cv::Point(10, textYPosition), FONT_HERSHEY_SIMPLEX, 0.5, textColor, 1, 4);
        textYPosition += textYSpace;

        auto durationFroShow = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - lastTheadTime);
        lastTheadTime = std::chrono::steady_clock::now();
        strTemp = "publisher thread diff time  =" + to_string(durationFroShow.count() / 1000.) +
                  "sub thread diff time  =" + to_string(durationFroShowofSub.count() / 1000.) +
                  "chassis speed = " + to_string(chassisInfo.speed()) + to_string(chassisInfo.steeringangle());
        // putText(img, strTemp, cv::Point(10, 60), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1, 4);

        if (decisionData.optimalGlobalTrajectory.planningPoints.size() > 1)
        {
            // strTemp = "pointNum="+to_string(decisionData.optimalGlobalTrajectory.globalTrajectoryPoints.size())+"Set Speed 0="  + to_string(decisionData.optimalGlobalTrajectory.globalTrajectoryPoints[0].v)+
            //+"Set Speed N="  + to_string(decisionData.optimalGlobalTrajectory.globalTrajectoryPoints[decisionData.optimalGlobalTrajectory.globalTrajectoryPoints.size()-1].v) ;
            // strTemp = "pointNum=" + to_string(decisionData.optimalGlobalTrajectory.planningPoints.size()) + "Set Speed 0=" + to_string(decisionData.optimalGlobalTrajectory.planningPoints[0].v) + "Set Speed N=" + to_string(decisionData.optimalGlobalTrajectory.planningPoints[decisionData.optimalGlobalTrajectory.planningPoints.size() - 1].v) + "SetX 0=" + to_string(decisionData.optimalGlobalTrajectory.planningPoints[0].gaussX) + "SetY 0=" + to_string(decisionData.optimalGlobalTrajectory.planningPoints[0].gaussY);

            strTemp = "Set First Point Speed=";
            strTemp1 = to_string(decisionData.optimalGlobalTrajectory.planningPoints[1].v);
            strTemp1 = strTemp1.substr(0, strTemp1.find(".") + 3);
            strTemp += strTemp1;
            strTemp += ", X=";
            strTemp1 = to_string(decisionData.optimalGlobalTrajectory.planningPoints[1].gaussX);
            strTemp1 = strTemp1.substr(0, strTemp1.find(".") + 3);
            strTemp += strTemp1;
            strTemp += ", Y=";
            strTemp1 = to_string(decisionData.optimalGlobalTrajectory.planningPoints[1].gaussY);
            strTemp1 = strTemp1.substr(0, strTemp1.find(".") + 3);
            strTemp += strTemp1;

            putText(img, strTemp, cv::Point(10, textYPosition), FONT_HERSHEY_SIMPLEX, 0.5, textColor, 1, 4);
            textYPosition += textYSpace;
        }

        // predict 预测数量打印
        // strTemp = "prediction size" + to_string(simplePredictionCmd.size()); // OBU侧预测数量
        // putText(img, strTemp, cv::Point(10, 190), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1, 4);

        // strTemp = "cluprediction size" + to_string(simplePredictionCmdClu.size()); // 苏州路侧预测数量
        // putText(img, strTemp, cv::Point(10, 210), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1, 4);

        // strTemp = "clupredictions size" + to_string(predictionClu.size()); // 激光聚类预测数量
        // putText(img, strTemp, cv::Point(10, 230), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1, 4);

        // end of predict
        double dXForShow, dYForShow, dYawForShow, dXVehicleForShow, dYVehicleForShow, dYawVehicleForShow;
        dXVehicleForShow = imu.gaussy();
        dYVehicleForShow = imu.gaussx();
        dYawVehicleForShow = M_PI / 2 - imu.yaw() * M_PI / 180.;
        // std::cout << "Vehicle X  = " << dXVehicleForShow << " Y = " << dYVehicleForShow << " Yaw =" << dYawVehicleForShow << std::endl;
        //  int ii =0,  jj=0, kk=0;
        for (int ii = 0; ii < (int)map.roads.size(); ii++)
        {
            for (int jj = 0; jj < (int)map.roads[ii].lanes.size(); jj++)
            {

                // std::cout<<"road  = "<<map.roads[ii].id<<"lane = "<<map.roads[ii].lanes[jj].id<<"point size ="<<map.roads[ii].lanes[jj].gaussRoadPoints.size()<<std::endl;
                for (int kk = 0; kk < (int)map.roads[ii].lanes[jj].gaussRoadPoints.size(); kk = kk + 3)
                {
                    dXForShow = map.roads[ii].lanes[jj].gaussRoadPoints[kk].GaussY;
                    dYForShow = map.roads[ii].lanes[jj].gaussRoadPoints[kk].GaussX;
                    dYawForShow = M_PI / 2 - map.roads[ii].lanes[jj].gaussRoadPoints[kk].yaw * M_PI / 180.;

                    // std::cout<<"Point X  = "<<dXForShow<<"Vehicle Y = "<<dYForShow<<"Vehicle Yaw ="<<dYawForShow<<std::endl;

                    CoordTran2DForNew0INOld(dXForShow, dYForShow, dYawForShow, dXVehicleForShow, dYVehicleForShow, dYawVehicleForShow);
                    // std::cout<<"new Point X  = "<<dXForShow<<"Vehicle Y = "<<dYForShow<<"Vehicle Yaw ="<<dYawForShow<<std::endl;
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
        strTemp = "End X=";
        strTemp1 = to_string(dYForShow);
        strTemp1 = strTemp1.substr(0, strTemp1.find(".") + 3);
        strTemp += strTemp1;
        strTemp += ", Y=";
        strTemp1 = to_string(dXForShow);
        strTemp1 = strTemp1.substr(0, strTemp1.find(".") + 3);
        strTemp += strTemp1;
        strTemp += ", Yaw=";
        strTemp1 = to_string(dYawForShow);
        strTemp1 = strTemp1.substr(0, strTemp1.find(".") + 3);
        strTemp += strTemp1;

        putText(img, strTemp, cv::Point(10, textYPosition), FONT_HERSHEY_SIMPLEX, 0.5, textColor, 1, 4);
        textYPosition += textYSpace;

        // std::cout<<"Point X  = "<<dXForShow<<"Vehicle Y = "<<dYForShow<<"Vehicle Yaw ="<<dYawForShow<<std::endl;

        CoordTran2DForNew0INOld(dXForShow, dYForShow, dYawForShow, dXVehicleForShow, dYVehicleForShow, dYawVehicleForShow);
        // std::cout<<"new Point X  = "<<dXForShow<<"Vehicle Y = "<<dYForShow<<"Vehicle Yaw ="<<dYawForShow<<std::endl;
        cv::Point point4;
        point4.x = int(mapSizeX / 2.0 - dYForShow * gridInOneMeter);
        point4.y = int(mapSizeY - dXForShow * gridInOneMeter);

        circle(img, drawRegulate(point4), 10, Scalar(0, 0, 255)); //

        // Put traffic light text //信号灯文字信息
        strTemp = "traffic light " + to_string(trafficLight.stoproadid()) + "," + to_string(trafficLight.stoplaneid()) + "," + to_string(trafficLight.stoppointid()) +
                  "active = " + to_string(trafficLight.active()) +
                  ",state = " + to_string(trafficLight.state()) +
                  ",remaining time = " + to_string(trafficLight.remaining_time()) +
                  ",distance = " + to_string(trafficLight.lane_length_before_intersection());
        // putText(img, strTemp, cv::Point(10, 150), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1, 4);

        // 根据红绿灯的ID获取位置信息，
        GaussRoadPoint gaussPointTrafficLight;
        // std::cout << "GetLaneByRoadIDLaneID X  = " << trafficLight.stoproadid() << " = " << trafficLight.stoplaneid() << " = " << trafficLight.stoppointid() << std::endl;
        if (map.GetLaneByRoadIDLaneID(trafficLight.stoproadid(), trafficLight.stoplaneid(), trafficLight.stoppointid(), gaussPointTrafficLight))
        {
            // 画信号灯trafficLight
            //  constexpr double stop_line_x = 4429698.286;
            //   constexpr double stop_line_y = 442774.922;
            //   constexpr double stop_line_yaw = 270.0;
            dXForShow = gaussPointTrafficLight.GaussY;                         // 561534.796;//SS
            dYForShow = gaussPointTrafficLight.GaussX;                         // 3482533.422;//
            dYawForShow = M_PI / 2 - gaussPointTrafficLight.yaw * M_PI / 180.; // 10.15;//
            // std::cout << RED << "trafficLight X  = " << dXForShow << "trafficLight Y = " << dYForShow << "trafficLight Yaw =" << dYawForShow << RESET << std::endl;
            CoordTran2DForNew0INOld(dXForShow, dYForShow, dYawForShow, dXVehicleForShow, dYVehicleForShow, dYawVehicleForShow);
            // std::cout<<"new trafficLight X  = "<<dXForShow<<"trafficLight Y = "<<dYForShow<<"trafficLight Yaw ="<<dYawForShow<<std::endl;
            cv::Point point6;
            point6.x = int(mapSizeX / 2.0 - dYForShow * gridInOneMeter) + 10;
            point6.y = int(mapSizeY - dXForShow * gridInOneMeter);

            // std::cout<<"----------------------------------trafficLight.state= "<<trafficLight.state()<<std::endl;
            if (trafficLight.state() == infopack::TrafficLight_State_RED_LIGHT)
            {
                // std::cout << RED  <<" trafficLight.state() == infopack::TrafficLight_State_RED_LIGHT"  <<RESET<<std::endl;
                circle(img, drawRegulate(point6), 8, Scalar(0, 0, 255), -1); //
            }
            else if (trafficLight.state() == infopack::TrafficLight_State_YELLOW_LIGHT)
                circle(img, drawRegulate(point6), 8, Scalar(255, 255, 0), -1); //
            else if (trafficLight.state() == infopack::TrafficLight_State_GREEN_LIGHT)
                circle(img, drawRegulate(point6), 8, Scalar(0, 255, 0), -1); //
            else
                ;

            // circle(img, drawRegulate(point6), 8, Scalar(255, 255, 0)); //
        }
        else
        {
            // std::cout << RED << " not find  gaussPointTrafficLight" << RESET << std::endl;
        }

        //////////////////////////////////////////////////////////////////////////////  circle(img, drawRegulate(point4), 10, Scalar(0, 0, 255)); //

        cv::Point trajPoint1;
        cv::Point trajPoint2;
        // std::cout << BOLDBLUE << "stopbool!!!" <<getStopBool()<< RESET << std::endl;
        if (stopPointJudge(imu, stopPoints))
        {
            std::cout << BOLDBLUE << "Vehicle Stopping!!!" << RESET << std::endl;
        }
        else // 不在停车区
        {
            // std::cout << "controlTrajectoryList.size: " << decisionData.controlTrajectoryList.size() << std::endl;
            // 画出所有备选轨迹
            for (int i = 0; i < (int)decisionData.controlTrajectoryList.size(); i++)
            {
                // std::cout << "controlTrajectoryListiiiiiiiiiiiiiiii: " << decisionData.controlTrajectoryList[i].planningPoints[49].x << std::endl;
                // std::cout << "controlTrajectoryListiiiiiiiiiiiiiiii: " << decisionData.controlTrajectoryList[i].planningPoints[49].y << std::endl;
                // std::cout << "controlTrajectoryList.size=" << decisionData.controlTrajectoryList.size() <<
                // " planningPoints.size() ="<<decisionData.controlTrajectoryList[i].planningPoints.size()<<std::endl;
                for (int j = 0; j < (int)decisionData.controlTrajectoryList[i].planningPoints.size() - 1; j++)
                {
                    trajPoint1.x = int(decisionData.controlTrajectoryList[i].planningPoints[j].x * gridInOneMeter + mapSizeX / 2.0);
                    trajPoint1.y = int(mapSizeY - decisionData.controlTrajectoryList[i].planningPoints[j].y * gridInOneMeter);
                    trajPoint2.x = int(decisionData.controlTrajectoryList[i].planningPoints[j + 1].x * gridInOneMeter + mapSizeX / 2.0);
                    trajPoint2.y = int(mapSizeY - decisionData.controlTrajectoryList[i].planningPoints[j + 1].y * gridInOneMeter);
                    line(img, drawRegulate(trajPoint1), drawRegulate(trajPoint2), Scalar(0, 255, 0), 1); // 画轨迹点连线，lenghuise，绿色
                    // std::cout << "x=" <<  decisionData.controlTrajectoryList[i].planningPoints[j].x <<
                    // " y="<<decisionData.controlTrajectoryList[i].planningPoints[j].y <<std::endl;
                }
            }

            // 画出safe轨迹
            // std::cout << "decisionData.feasibleTrajectoryIndexList.size()" << decisionData.feasibleTrajectoryIndexList.size() << std::endl;
            // std::cout << "decisionData.controlTrajectoryList.size()" << decisionData.controlTrajectoryList.size() << std::endl;

            // for (int iter = 0; iter < min(decisionData.feasibleTrajectoryIndexList.size(), decisionData.controlTrajectoryList.size()); iter++)
            for (int iter = 0; iter < (int)decisionData.feasibleTrajectoryIndexList.size(); iter++)
            {
                // std::cout << "controlTrajectoryListiiiiiiiiiiiiiiii: " << decisionData.controlTrajectoryList[i].planningPoints[49].x << std::endl;
                // std::cout << "controlTrajectoryListiiiiiiiiiiiiiiii: " << decisionData.controlTrajectoryList[i].planningPoints[49].y << std::endl;
                int i = decisionData.feasibleTrajectoryIndexList[iter];

                // std::cout << " decisionData.controlTrajectoryList[i].planningPoints.size()" << decisionData.controlTrajectoryList[i].planningPoints.size() << endl;
                for (int j = 0; j < (int)decisionData.controlTrajectoryList[i].planningPoints.size() - 1; j++)
                {
                    // std::cout << "decisionData.controlTrajectoryList[i].planningPoints[j].x" << decisionData.controlTrajectoryList[i].planningPoints[j].x << endl;
                    trajPoint1.x = int(decisionData.controlTrajectoryList[i].planningPoints[j].x * gridInOneMeter + mapSizeX / 2.0);
                    trajPoint1.y = int(mapSizeY - decisionData.controlTrajectoryList[i].planningPoints[j].y * gridInOneMeter);
                    trajPoint2.x = int(decisionData.controlTrajectoryList[i].planningPoints[j + 1].x * gridInOneMeter + mapSizeX / 2.0);
                    trajPoint2.y = int(mapSizeY - decisionData.controlTrajectoryList[i].planningPoints[j + 1].y * gridInOneMeter);
                    line(img, drawRegulate(trajPoint1), drawRegulate(trajPoint2), Scalar(255, 255, 0), 1); // 画轨迹点连线，青色
                }
            }

            // 画出被选择轨迹
            // 从这里可以看出decision的坐标系XY是右前

            for (int i = 0; i < (int)decisionData.controlTrajectoryList.size(); i++)
            {
                if (i == decisionData.optimalTrajectoryIndex)
                {
                    for (int j = 0; j < (int)decisionData.controlTrajectoryList[i].planningPoints.size() - 1; j++)
                    {
                        // std::cout << "controlTrajectoryList.size: " << decisionData.controlTrajectoryList.size() << std::endl;
                        trajPoint1.x = int(decisionData.controlTrajectoryList[i].planningPoints[j].x * gridInOneMeter + mapSizeX / 2.0);
                        trajPoint1.y = int(mapSizeY - decisionData.controlTrajectoryList[i].planningPoints[j].y * gridInOneMeter);
                        trajPoint2.x = int(decisionData.controlTrajectoryList[i].planningPoints[j + 1].x * gridInOneMeter + mapSizeX / 2.0);
                        trajPoint2.y = int(mapSizeY - decisionData.controlTrajectoryList[i].planningPoints[j + 1].y * gridInOneMeter);
                        line(img, drawRegulate(trajPoint1), drawRegulate(trajPoint2), Scalar(255, 0, 0), 4); // 画轨迹点连线，蓝色

                        // cout <<"---------x "<<decisionData.controlTrajectoryList[i].planningPoints[j].x<<"   Y = " <<decisionData.controlTrajectoryList[i].planningPoints[j].y<<endl;
                    }
                }
            }

            // 参考线
            for (int i = 0; i < (int)decisionData.referenceLine.referenceLinePoints.size() - 1; i++)
            {
                //     //全局坐标系转换
                ////cout <<"decisionData.referenceLine.referenceLinePoints.size "<<decisionData.referenceLine.referenceLinePoints.size()<<endl;
                double dXForShow1 = decisionData.referenceLine.referenceLinePoints[i].gaussY;
                double dYForShow1 = decisionData.referenceLine.referenceLinePoints[i].gaussX;
                double dYawForShow1 = decisionData.referenceLine.referenceLinePoints[i].gaussAngle; // 这个角度应该没赋值，但是在这里也没用
                CoordTran2DForNew0INOld(dXForShow1, dYForShow1, dYawForShow1, dXVehicleForShow, dYVehicleForShow, dYawVehicleForShow);

                double dXForShow2 = decisionData.referenceLine.referenceLinePoints[i + 1].gaussY;
                double dYForShow2 = decisionData.referenceLine.referenceLinePoints[i + 1].gaussX;
                double dYawForShow2 = decisionData.referenceLine.referenceLinePoints[i + 1].gaussAngle; // 这个角度应该没赋值，但是在这里也没用
                CoordTran2DForNew0INOld(dXForShow2, dYForShow2, dYawForShow2, dXVehicleForShow, dYVehicleForShow, dYawVehicleForShow);

                trajPoint1.x = int(mapSizeX / 2.0 - dYForShow1 * gridInOneMeter);
                trajPoint1.y = int(mapSizeY - dXForShow1 * gridInOneMeter);
                trajPoint2.x = int(mapSizeX / 2.0 - dYForShow2 * gridInOneMeter);
                trajPoint2.y = int(mapSizeY - dXForShow2 * gridInOneMeter);
                line(img, drawRegulate(trajPoint1), drawRegulate(trajPoint2), Scalar(0, 125, 255), 2); // 画参考线连线，赭石色

                //     // cout <<"---------x "<<decisionData.controlTrajectoryList[i].planningPoints[j].x<<"   Y = " <<decisionData.controlTrajectoryList[i].planningPoints[j].y<<endl;
            }

            // 画换道的check线
            for (int i = 0; i < (int)decisionData.checkLineVector.size(); i++)
            {
                // std::cout << "controlTrajectoryListiiiiiiiiiiiiiiii: " << decisionData.controlTrajectoryList[i].planningPoints[49].x << std::endl;
                // std::cout << "controlTrajectoryListiiiiiiiiiiiiiiii: " << decisionData.controlTrajectoryList[i].planningPoints[49].y << std::endl;
                // int i = decisionData.checkLineVector[iter];

                // std::cout << " decisionData.controlTrajectoryList[i].planningPoints.size()" << decisionData.controlTrajectoryList[i].planningPoints.size() << endl;
                for (int j = 0; j < (int)decisionData.checkLineVector[i].referenceLinePoints.size() - 1; j++)
                {
                    double dXForShow1 = decisionData.checkLineVector[i].referenceLinePoints[j].gaussY;
                    double dYForShow1 = decisionData.checkLineVector[i].referenceLinePoints[j].gaussX;
                    double dYawForShow1 = decisionData.checkLineVector[i].referenceLinePoints[j].gaussAngle; // 这个角度应该没赋值，但是在这里也没用
                    CoordTran2DForNew0INOld(dXForShow1, dYForShow1, dYawForShow1, dXVehicleForShow, dYVehicleForShow, dYawVehicleForShow);

                    double dXForShow2 = decisionData.checkLineVector[i].referenceLinePoints[j + 1].gaussY;
                    double dYForShow2 = decisionData.checkLineVector[i].referenceLinePoints[j + 1].gaussX;
                    double dYawForShow2 = decisionData.checkLineVector[i].referenceLinePoints[j + 1].gaussAngle; // 这个角度应该没赋值，但是在这里也没用
                    CoordTran2DForNew0INOld(dXForShow2, dYForShow2, dYawForShow2, dXVehicleForShow, dYVehicleForShow, dYawVehicleForShow);

                    trajPoint1.x = int(mapSizeX / 2.0 - dYForShow1 * gridInOneMeter);
                    trajPoint1.y = int(mapSizeY - dXForShow1 * gridInOneMeter);
                    trajPoint2.x = int(mapSizeX / 2.0 - dYForShow2 * gridInOneMeter);
                    trajPoint2.y = int(mapSizeY - dXForShow2 * gridInOneMeter);
                    line(img, drawRegulate(trajPoint1), drawRegulate(trajPoint2), Scalar(0, 225, 255), 2); // 画参考线连线，黄色
                }
            }
        } // else//不在停车区

        // 从这里可以看出，prediction的坐标系XY是前左
        for (int i = 0; i < prediction.object_size(); i++)
        // for (int i = 0; i < min(2, prediction.object_size()); i++)
        {
            double deltaX = prediction.object(i).predictpoint(0).x() + LASER_OFFSET_FRONT;
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

        // 画栅格ACC
        grid_planning = getGridPlanning();
        for (int i = 0; i < grid_planning.size(); i++)
        // for (int i = 0; i < min(2, prediction.object_size()); i++)
        {
            // grid_planning[i].x
            double deltaX = grid_planning[i].x + LASER_OFFSET_FRONT;
            double deltaY = grid_planning[i].y;
            double deltaW = 3;
            double deltaL = 3;
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
            rectangle(img, drawRegulate(point11), drawRegulate(point12), Scalar(165, 165, 0), -1);
        }

        // clu画激光聚类预测后 有角度的
        for (int i = 0; i < predictionClu.size(); i++)
        {
            // cout<<GREEN<<"start drawing prediction"<<endl;
            const prediction::Object objProto = predictionClu[i];

            double dXForShow, dYForShow, dYawForShow;
            dXForShow = objProto.predictpoint(0).x();
            dYForShow = objProto.predictpoint(0).y();
            // dYawForShow = M_PI / 2 - objProto.predictpoint(0).vy() * M_PI / 180.;
            dYawForShow = M_PI / 2 - objProto.predictpoint(0).vy();
            // std::cout << dXForShow << "  " << dYForShow << " /////////////////" << endl;
            double lengthTemp = objProto.l(); //
            double widthTemp = objProto.w();  //

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

            // std::cout << "conner = " << pointTemp[0][0] << "," << pointTemp[0][1] << "," << pointTemp[1][0] << "," << pointTemp[1][1] << "," << pointTemp[2][0] << "," << pointTemp[2][1] << "," << pointTemp[3][0] << "," << pointTemp[3][1] << std::endl;
            //  将平面直角坐标转化为车辆局部坐标，右上坐标系,--再转屏幕坐标系
            cv::Point cvPointTemp[4];
            for (int j = 0; j < 4; j++)
            {
                CoordTran2DForNew0INOld(pointTemp[j][0], pointTemp[j][1], pointTemp[j][2], dXVehicleForShow, dYVehicleForShow, dYawVehicleForShow);
                cvPointTemp[j].x = (int)(mapSizeX / 2.0 - pointTemp[j][1] * gridInOneMeter);
                cvPointTemp[j].y = (int)(mapSizeY - pointTemp[j][0] * gridInOneMeter);
                // std::cout<<pointTemp[0][0]<<"  "<< pointTemp[0][1]<<" ##########"<<endl;
                // std::cout << "conner2 = " << pointTemp[0][0] << "," << pointTemp[0][1] << "," << pointTemp[1][0] << "," << pointTemp[1][1] << "," << pointTemp[2][0] << "," << pointTemp[2][1] << "," << pointTemp[3][0] << "," << pointTemp[3][1] << std::endl;
            }

            // 画矩形框
            for (int k = 0; k < 4; k++)
            {
                line(img, drawRegulate(cvPointTemp[k]), drawRegulate(cvPointTemp[(k + 1) % 4]), Scalar(11, 23, 70), 2);
            }
        }
        // 画激光聚类原始图
        for (int i = 0; i < drawForInt.size(); i++)
        {
            // cout<<GREEN<<"start drawing prediction"<<endl;
            const prediction::Object objProto = drawForInt[i];

            double dXForShow, dYForShow, dYawForShow;
            dXForShow = objProto.predictpoint(0).y();
            dYForShow = objProto.predictpoint(0).x();
            // dYawForShow = M_PI / 2 - objProto.predictpoint(0).vy() * M_PI / 180.;
            // std::cout << dXForShow << "  " << dYForShow << "*************" << endl;
            dYawForShow = M_PI / 2 - objProto.predictpoint(0).vy();
            // std::cout<<dXForShow<<"  "<<dXForShow<<" /////////////////"<<endl;
            double lengthTemp = objProto.l(); //
            double widthTemp = objProto.w();  //

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

            // std::cout << "conner = " << pointTemp[0][0] << "," << pointTemp[0][1] << "," << pointTemp[1][0] << "," << pointTemp[1][1] << "," << pointTemp[2][0] << "," << pointTemp[2][1] << "," << pointTemp[3][0] << "," << pointTemp[3][1] << std::endl;
            //  将平面直角坐标转化为车辆局部坐标，右上坐标系,--再转屏幕坐标系
            cv::Point cvPointTemp[4];
            for (int j = 0; j < 4; j++)
            {
                CoordTran2DForNew0INOld(pointTemp[j][0], pointTemp[j][1], pointTemp[j][2], dXVehicleForShow, dYVehicleForShow, dYawVehicleForShow);
                cvPointTemp[j].x = (int)(mapSizeX / 2.0 - pointTemp[j][1] * gridInOneMeter);
                ;
                cvPointTemp[j].y = (int)(mapSizeY - pointTemp[j][0] * gridInOneMeter);
                // std::cout<<pointTemp[0][0]<<"  "<< pointTemp[0][1]<<" ##########"<<endl;
                // std::cout << "conner2 = " << pointTemp[0][0] << "," << pointTemp[0][1] << "," << pointTemp[1][0] << "," << pointTemp[1][1] << "," << pointTemp[2][0] << "," << pointTemp[2][1] << "," << pointTemp[3][0] << "," << pointTemp[3][1] << std::endl;
            }
            // 画矩形框
            for (int k = 0; k < 4; k++)
            {
                line(img, drawRegulate(cvPointTemp[k]), drawRegulate(cvPointTemp[(k + 1) % 4]), Scalar(0, 0, 0), 2);
            }
        }
        // 画感知失效区域
        //  double noPreX1 = 561627;//左上
        //  double noPreY1 = 3482462;
        //  double noPreX2 = 561611;//右下
        //  double noPreY2 = 3482483;
        //  double noPreYaw1 = 0; double noPreYaw2 = 0;

        // CoordTran2DForNew0INOld(noPreX1, noPreY1, noPreYaw1, dXVehicleForShow, dYVehicleForShow, dYawVehicleForShow);
        // CoordTran2DForNew0INOld(noPreX2, noPreY2, noPreYaw2, dXVehicleForShow, dYVehicleForShow, dYawVehicleForShow);
        // cv::Point pointNoPre1, pointNoPre2;
        // pointNoPre1.x = int(mapSizeX / 2.0 - noPreY1 * gridInOneMeter);
        // pointNoPre1.y = int(mapSizeY - noPreX1 * gridInOneMeter);

        // pointNoPre2.x = int(mapSizeX / 2.0 - noPreY2 * gridInOneMeter);
        //  pointNoPre2.y = int(mapSizeY - noPreX2 * gridInOneMeter);

        //  cout<< "no pre -- "<<  pointNoPre1.x<<","<<pointNoPre1.y <<","<<  pointNoPre2.x<<","<<  pointNoPre2.y<<endl;

        //  rectangle(img, drawRegulate(pointNoPre1), drawRegulate(pointNoPre2), Scalar(0, 0, 255), -1);

        // 画路侧感知信息
        // std::cout << " 画路侧感知信息 objectsCmd.size()" << objectsCmd.size() << std::endl;
        for (int i = 0; i < (int)objectsCmd.size(); i++)
        {
            const infopack::ObjectsProto objProto = objectsCmd[i];
            // if (objProto.type() == myselfVehicleeType && objProto.objectid() == myselfVehicleID) // 不显示自车
            // {
            //     continue;
            // }

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

            // cout << "***objectid = " << objProto.objectid() << "," << longitudeTemp << "," << latitudeTemp << ","
            //      << dXForShow << "," << dYForShow << "," << objProto.yaw() << "," << objProto.timestamp() << endl;

            // cout << "imu position "<< dXVehicleForShow <<","<< dYVehicleForShow<<","<<dYawVehicleForShow<< endl;

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

            // std::cout << "conner = " << pointTemp[0][0] << "," << pointTemp[0][1] << "," << pointTemp[1][0] << "," << pointTemp[1][1] << "," << pointTemp[2][0] << "," << pointTemp[2][1] << "," << pointTemp[3][0] << "," << pointTemp[3][1] << std::endl;
            //  将平面直角坐标转化为车辆局部坐标，右上坐标系,--再转屏幕坐标系
            cv::Point cvPointTemp[4];
            for (int j = 0; j < 4; j++)
            {
                CoordTran2DForNew0INOld(pointTemp[j][0], pointTemp[j][1], pointTemp[j][2], dXVehicleForShow, dYVehicleForShow, dYawVehicleForShow);
                cvPointTemp[j].x = (int)(mapSizeX / 2.0 - pointTemp[j][1] * gridInOneMeter);
                ;
                cvPointTemp[j].y = (int)(mapSizeY - pointTemp[j][0] * gridInOneMeter);
            }

            // std::cout << "conner_at_vehicle = " << pointTemp[0][0] << "," << pointTemp[0][1] << "," << pointTemp[1][0] << "," << pointTemp[1][1] << "," << pointTemp[2][0] << "," << pointTemp[2][1] << "," << pointTemp[3][0] << "," << pointTemp[3][1] << std::endl;
            //  cout<< "conner_at_screen = " <<  cvPointTemp[0].x <<","<<cvPointTemp[0].y <<","<<
            //                                             cvPointTemp[1].x <<","<<cvPointTemp[1].y <<","<<
            //                                             cvPointTemp[2].x <<","<<cvPointTemp[2].y <<","<<
            //                                             cvPointTemp[3].x <<","<<cvPointTemp[3].y <<endl ;

            // 画矩形框
            for (int k = 0; k < 4; k++)
            {
                if (objProto.type() == myselfVehicleeType && objProto.objectid() == myselfVehicleID) // 不显示自车
                    line(img, drawRegulate(cvPointTemp[k]), drawRegulate(cvPointTemp[(k + 1) % 4]), Scalar(0, 125, 255), 2);
                else
                    line(img, drawRegulate(cvPointTemp[k]), drawRegulate(cvPointTemp[(k + 1) % 4]), Scalar(255, 0, 0), 2);

                // line(img, cvPointTemp[k],cv::Point (0,0), Scalar(0, 255, 0),4);
                //  line(img, drawRegulate(trajPoint1), drawRegulate(trajPoint2), Scalar(255, 0, 0), 4); // 画轨迹点连线，蓝色
                // circle(img, drawRegulate(cvPointTemp[k]), 8, Scalar(0, 255, 0)); //
            }
        }

        // predict  路测感知预测后画图
        for (int i = 0; i < simplePredictionCmd.size(); i++)
        {
            // cout<<GREEN<<"start drawing prediction"<<endl;
            const infopack::ObjectsProto objProto = simplePredictionCmd[i];
            // if (objProto.type() == myselfVehicleeType && objProto.objectid() == myselfVehicleID) // 不显示自车
            // {
            //     continue;
            // }

            // 转换xy经纬度为平面坐标
            double latitudeTemp = objProto.lat();
            // cout << GREEN << "object prediction lat::" << latitudeTemp << endl;
            double longitudeTemp = objProto.lon();
            double gaussNorthTemp, gaussEastTemp;
            gaussConvert(longitudeTemp, latitudeTemp, gaussNorthTemp, gaussEastTemp);

            // 计算4个顶点平面坐标
            dXForShow = gaussEastTemp;
            dYForShow = gaussNorthTemp;
            dYawForShow = M_PI / 2 - objProto.yaw() * M_PI / 180.;
            double lengthTemp = objProto.len() / 100.;  // 原单位是cm，转成m
            double widthTemp = objProto.width() / 100.; // 原单位是cm，转成m

            // cout << "***objectid = " << objProto.objectid() << "," << longitudeTemp << "," << latitudeTemp << ","
            //      << dXForShow << "," << dYForShow << "," << objProto.yaw() << "," << objProto.timestamp() << endl;

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

            // std::cout << "conner = " << pointTemp[0][0] << "," << pointTemp[0][1] << "," << pointTemp[1][0] << "," << pointTemp[1][1] << "," << pointTemp[2][0] << "," << pointTemp[2][1] << "," << pointTemp[3][0] << "," << pointTemp[3][1] << std::endl;
            //  将平面直角坐标转化为车辆局部坐标，右上坐标系,--再转屏幕坐标系
            cv::Point cvPointTemp[4];
            for (int j = 0; j < 4; j++)
            {
                CoordTran2DForNew0INOld(pointTemp[j][0], pointTemp[j][1], pointTemp[j][2], dXVehicleForShow, dYVehicleForShow, dYawVehicleForShow);
                cvPointTemp[j].x = (int)(mapSizeX / 2.0 - pointTemp[j][1] * gridInOneMeter);
                ;
                cvPointTemp[j].y = (int)(mapSizeY - pointTemp[j][0] * gridInOneMeter);
            }

            // 画矩形框
            for (int k = 0; k < 4; k++)
            {
                line(img, drawRegulate(cvPointTemp[k]), drawRegulate(cvPointTemp[(k + 1) % 4]), Scalar(0, 0, 155), 2);
            }
        }
        // end of predict

        // predict (路测模式输入)
        for (int i = 0; i < simplePredictionCmdClu.size(); i++)
        {
            // cout<<GREEN<<"start drawing prediction"<<endl;
            const infopack::ObjectsProto objProto = simplePredictionCmdClu[i];
            // if (objProto.type() == myselfVehicleeType && objProto.objectid() == myselfVehicleID) // 不显示自车
            // {
            //     continue;
            // }

            // 转换xy经纬度为平面坐标
            double latitudeTemp = objProto.lat();
            // cout << GREEN << "object prediction lat::" << latitudeTemp << endl;
            double longitudeTemp = objProto.lon();
            double gaussNorthTemp, gaussEastTemp;
            gaussConvert(longitudeTemp, latitudeTemp, gaussNorthTemp, gaussEastTemp);

            // 计算4个顶点平面坐标
            dXForShow = gaussEastTemp;
            dYForShow = gaussNorthTemp;
            dYawForShow = M_PI / 2 - objProto.yaw() * M_PI / 180.;
            double lengthTemp = objProto.len() / 100.;  // 原单位是cm，转成m
            double widthTemp = objProto.width() / 100.; // 原单位是cm，转成m

            // cout << "***objectid = " << objProto.objectid() << "," << longitudeTemp << "," << latitudeTemp << ","
            //      << dXForShow << "," << dYForShow << "," << objProto.yaw() << "," << objProto.timestamp() << endl;

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

            // std::cout << "conner = " << pointTemp[0][0] << "," << pointTemp[0][1] << "," << pointTemp[1][0] << "," << pointTemp[1][1] << "," << pointTemp[2][0] << "," << pointTemp[2][1] << "," << pointTemp[3][0] << "," << pointTemp[3][1] << std::endl;
            //  将平面直角坐标转化为车辆局部坐标，右上坐标系,--再转屏幕坐标系
            cv::Point cvPointTemp[4];
            for (int j = 0; j < 4; j++)
            {
                CoordTran2DForNew0INOld(pointTemp[j][0], pointTemp[j][1], pointTemp[j][2], dXVehicleForShow, dYVehicleForShow, dYawVehicleForShow);
                cvPointTemp[j].x = (int)(mapSizeX / 2.0 - pointTemp[j][1] * gridInOneMeter);
                ;
                cvPointTemp[j].y = (int)(mapSizeY - pointTemp[j][0] * gridInOneMeter);
            }

            // std::cout << "conner_at_vehicle = " << pointTemp[0][0] << "," << pointTemp[0][1] << "," << pointTemp[1][0] << "," << pointTemp[1][1] << "," << pointTemp[2][0] << "," << pointTemp[2][1] << "," << pointTemp[3][0] << "," << pointTemp[3][1] << std::endl;
            //  cout<< "conner_at_screen = " <<  cvPointTemp[0].x <<","<<cvPointTemp[0].y <<","<<
            //                                             cvPointTemp[1].x <<","<<cvPointTemp[1].y <<","<<
            //                                             cvPointTemp[2].x <<","<<cvPointTemp[2].y <<","<<
            //                                             cvPointTemp[3].x <<","<<cvPointTemp[3].y <<endl ;

            // 画矩形框
            for (int k = 0; k < 4; k++)
            {
                line(img, drawRegulate(cvPointTemp[k]), drawRegulate(cvPointTemp[(k + 1) % 4]), Scalar(255, 192, 0), 2);

                // line(img, cvPointTemp[k],cv::Point (0,0), Scalar(0, 255, 0),4);
                //  line(img, drawRegulate(trajPoint1), drawRegulate(trajPoint2), Scalar(255, 0, 0), 4); // 画轨迹点连线，蓝色
                // circle(img, drawRegulate(cvPointTemp[k]), 8, Scalar(0, 255, 0)); //
            }
        }
        // end of predict

        // 画常熟均联路测OBU
        //  std::cout << " 画常熟均联路测OBU objectsCmd.size()" << objectsCmd.size() << std::endl;
        //  for (int i = 0; i < (int)objectsCmdCsjl.objmsg_size(); i++)
        //  {
        //      const infopack::ObjectsProto objCsjlProto = objectsCmdCsjl.objmsg(i);
        //      // if (objProto.type() == myselfVehicleeType && objProto.objectid() == myselfVehicleID) // 不显示自车
        //      // {
        //      //     continue;
        //      // }

        //     double gaussNorthTemp = objCsjlProto.x();
        //     double gaussEastTemp = objCsjlProto.y();

        //     // 计算4个顶点平面坐标
        //     dXForShow = gaussEastTemp;
        //     dYForShow = gaussNorthTemp;
        //     dYawForShow = M_PI / 2 - objCsjlProto.yaw() * M_PI / 180.;
        //     double lengthTemp = objCsjlProto.len() / 100.;  // 原单位是cm，转成m
        //     double widthTemp = objCsjlProto.width() / 100.; // 原单位是cm，转成m

        //     // cout << "***objectsCmdCsjl = " << objCsjlProto.objectid() << "," << gaussNorthTemp << "," << gaussEastTemp << ","
        //     //      << dXForShow << "," << dYForShow << "," <<dYawForShow << "," << objCsjlProto.timestamp() << endl;

        //     // cout << "imu position "<< dXVehicleForShow <<","<< dYVehicleForShow<<","<<dYawVehicleForShow<< endl;

        //     double pointTemp[4][3]; // 障碍物矩形的4个角点，右上、右下、左下、左上；坐标 x，y,yaw
        //     pointTemp[0][0] = dXForShow + (lengthTemp / 2) * cos(dYawForShow) - (widthTemp / 2) * sin(dYawForShow);
        //     pointTemp[0][1] = dYForShow + (lengthTemp / 2) * sin(dYawForShow) + (widthTemp / 2) * cos(dYawForShow);
        //     pointTemp[1][0] = dXForShow + (lengthTemp / 2) * cos(dYawForShow) - (-widthTemp / 2) * sin(dYawForShow);
        //     pointTemp[1][1] = dYForShow + (lengthTemp / 2) * sin(dYawForShow) + (-widthTemp / 2) * cos(dYawForShow);
        //     pointTemp[2][0] = dXForShow + (-lengthTemp / 2) * cos(dYawForShow) - (-widthTemp / 2) * sin(dYawForShow);
        //     pointTemp[2][1] = dYForShow + (-lengthTemp / 2) * sin(dYawForShow) + (-widthTemp / 2) * cos(dYawForShow);
        //     pointTemp[3][0] = dXForShow + (-lengthTemp / 2) * cos(dYawForShow) - (widthTemp / 2) * sin(dYawForShow);
        //     pointTemp[3][1] = dYForShow + (-lengthTemp / 2) * sin(dYawForShow) + (widthTemp / 2) * cos(dYawForShow);
        //     pointTemp[0][2] = pointTemp[1][2] = pointTemp[2][2] = pointTemp[3][2] = dYawForShow;

        //     // std::cout << "conner = " << pointTemp[0][0] << "," << pointTemp[0][1] << "," << pointTemp[1][0] << "," << pointTemp[1][1] << "," << pointTemp[2][0] << "," << pointTemp[2][1] << "," << pointTemp[3][0] << "," << pointTemp[3][1] << std::endl;
        //     //  将平面直角坐标转化为车辆局部坐标，右上坐标系,--再转屏幕坐标系
        //     cv::Point cvPointTemp[4];
        //     for (int j = 0; j < 4; j++)
        //     {
        //         CoordTran2DForNew0INOld(pointTemp[j][0], pointTemp[j][1], pointTemp[j][2], dXVehicleForShow, dYVehicleForShow, dYawVehicleForShow);
        //         cvPointTemp[j].x = (int)(mapSizeX / 2.0 - pointTemp[j][1] * gridInOneMeter);
        //         ;
        //         cvPointTemp[j].y = (int)(mapSizeY - pointTemp[j][0] * gridInOneMeter);
        //     }

        //     // std::cout << "conner_at_vehicle = " << pointTemp[0][0] << "," << pointTemp[0][1] << "," << pointTemp[1][0] << "," << pointTemp[1][1] << "," << pointTemp[2][0] << "," << pointTemp[2][1] << "," << pointTemp[3][0] << "," << pointTemp[3][1] << std::endl;
        //     //  cout<< "conner_at_screen = " <<  cvPointTemp[0].x <<","<<cvPointTemp[0].y <<","<<
        //     //                                             cvPointTemp[1].x <<","<<cvPointTemp[1].y <<","<<
        //     //                                             cvPointTemp[2].x <<","<<cvPointTemp[2].y <<","<<
        //     //                                             cvPointTemp[3].x <<","<<cvPointTemp[3].y <<endl ;

        //     // 画矩形框
        //     for (int k = 0; k < 4; k++)
        //     {
        //         // if (objProto.type() == myselfVehicleeType && objProto.objectid() == myselfVehicleID) // 不显示自车
        //         //     line(img, drawRegulate(cvPointTemp[k]), drawRegulate(cvPointTemp[(k + 1) % 4]), Scalar(0, 125, 255), 2);
        //         // else
        //             line(img, drawRegulate(cvPointTemp[k]), drawRegulate(cvPointTemp[(k + 1) % 4]), Scalar(0, 255, 0), 2);

        //         // line(img, cvPointTemp[k],cv::Point (0,0), Scalar(0, 255, 0),4);
        //         //  line(img, drawRegulate(trajPoint1), drawRegulate(trajPoint2), Scalar(255, 0, 0), 4); // 画轨迹点连线，蓝色
        //         // circle(img, drawRegulate(cvPointTemp[k]), 8, Scalar(0, 255, 0)); //
        //     }

        //     //写障碍物信息
        //     //strTemp = to_string(objCsjlProto.objectid()) + "," + to_string(objCsjlProto.type() )+ "," + to_string(objCsjlProto.timestamp()) ;
        //    // putText(img, strTemp,drawRegulate(cvPointTemp[0]), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 1, 2);
        // }

        // 画视觉感知障碍物信息，XY是前左
        for (int i = 0; i < predictionFromVision.object_size(); i++)
        // for (int i = 0; i < min(2, prediction.object_size()); i++)
        {
            double deltaX = predictionFromVision.object(i).predictpoint(0).x() + VISION_OFFSET_FRONT;
            double deltaY = predictionFromVision.object(i).predictpoint(0).y();
            double deltaW = abs(predictionFromVision.object(i).w());
            double deltaL = abs(predictionFromVision.object(i).l());
            // strTemp = "relX=" + to_string(deltaX) + "relY=" + to_string(deltaY);
            // putText(img, strTemp, cv::Point(10, 200), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1, 4);
            cv::Point pointForVision;
            pointForVision.x = int(mapSizeX / 2.0 - deltaY * gridInOneMeter);
            pointForVision.y = int(mapSizeY - deltaX * gridInOneMeter);
            // point5.x = int(mapSizeX / 2.0 -  deltaX * gridInOneMeter );
            // point5.y = int(mapSizeY - deltaY * gridInOneMeter);
            // circle(img, drawRegulate(point5), 20, Scalar(0, 0, 255)); //
            cv::Point pointForVisionLU; // 障碍物左上角
            cv::Point pointForVisionRD; // 障碍物右下角

            pointForVisionLU.x = int(pointForVision.x - deltaL / 2 * gridInOneMeter); // deltaX小车激光雷达到障碍物中心距离
            pointForVisionLU.y = int(pointForVision.y - deltaW / 2 * gridInOneMeter);
            pointForVisionRD.x = int(pointForVision.x + deltaL / 2 * gridInOneMeter);
            pointForVisionRD.y = int(pointForVision.y + deltaW / 2 * gridInOneMeter);
            rectangle(img, drawRegulate(pointForVisionLU), drawRegulate(pointForVisionRD), Scalar(255,255, 255), -1);
        }

        ///////////////////////////////////
        mutex.unlock();
        imshow("globalTrajectoryPoints", img);
        waitKey(1);

        // 画图结束
        auto end = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

        // std::cout << "Thread Draw duration time = " << duration.count() << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }
}
// add byztz
void Jobs::onMouse(int event, int x, int y, int flags)
{
    // if(event == CV_EVENT_RBUTTONDOWN){
    //     pause_draw_history_flag = true;
    // }
    // if(event == CV_EVENT_RBUTTONUP){
    //     pause_draw_history_flag = false;
    // }
}
// end ofztz
void Jobs::publisher()
{

    while (1)
    {
        auto start = std::chrono::steady_clock::now();

        // =======do your works here======
        mutex.lock();
        Planning::TrajectoryPoint *trajectoryPoint;
        Planning::TrajectoryPointVec trajectoryPointVec;

        // 向控制发送规划轨迹信息
        // cout <<  "????????????????????" << decisionData.optimalGlobalTrajectory.planningPoints.size() <<"??????????????????????????????"<<endl;
        for (size_t i = 0; i < decisionData.optimalGlobalTrajectory.planningPoints.size(); i++)
        {
            trajectoryPoint = trajectoryPointVec.add_trajectorypoints();
            trajectoryPoint->set_x(decisionData.optimalGlobalTrajectory.planningPoints[i].gaussX);
            trajectoryPoint->set_y(decisionData.optimalGlobalTrajectory.planningPoints[i].gaussY);
            trajectoryPoint->set_theta(decisionData.optimalGlobalTrajectory.planningPoints[i].gaussAngle);
            trajectoryPoint->set_speed(decisionData.optimalGlobalTrajectory.planningPoints[i].v);

            // add by syp 平板控制车前进停止
            if (uiPad2Vdata.b_isalive() == true && uiPad2Vdata.b_isvalid() == true && uiPad2Vdata.b_speedoff() == false &&
                uiPad2Vdata.b_stopflag() == true)
                trajectoryPoint->set_speed(0);

            // //end of add by syp
            trajectoryPoint->set_curvature(decisionData.optimalGlobalTrajectory.planningPoints[i].curvature);
            trajectoryPoint->set_s(decisionData.optimalGlobalTrajectory.planningPoints[i].s);
            trajectoryPoint->set_obstacledistance(decisionData.obstacleDistance[decisionData.optimalTrajectoryIndex]);

            // cout <<  "????????????????????" <<"x = "<<decisionData.optimalGlobalTrajectory.planningPoints[i].gaussX <<"??????????????????????????????"<<endl;
        }

        // if(decisionData.optimalGlobalTrajectory.planningPoints.size() > 0)
        // {
        //     cout <<  "????????????????????" <<"x = "<<decisionData.optimalGlobalTrajectory.planningPoints[0].gaussX <<"??????????????????????????????"<<endl;
        //     cout <<  "????????????????????" <<"y = "<<decisionData.optimalGlobalTrajectory.planningPoints[0].gaussY <<"??????????????????????????????"<<endl;
        //     cout <<  "????????????????????" <<"yaw = "<<decisionData.optimalGlobalTrajectory.planningPoints[0].gaussAngle <<"??????????????????????????????"<<endl;
        // }

        // 在这里补充为unity输出被选择轨迹
        // std::ofstream fsOptimalPath("optimalpath.txt", std::ofstream::out|std::ofstream::app);
        //  for (size_t i = 0; i < decisionData.optimalGlobalTrajectory.planningPoints.size(); i++)
        // {
        //     fsOptimalPath << std::setprecision(15) << decisionData.optimalGlobalTrajectory.planningPoints[i].gaussX <<","
        //     <<decisionData.optimalGlobalTrajectory.planningPoints[i].gaussY<<","
        //     <<decisionData.optimalGlobalTrajectory.planningPoints[i].gaussAngle<<",";

        //     // add by syp 平板控制车前进停止
        //     if (uiPad2Vdata.b_isalive() == true && uiPad2Vdata.b_isvalid() == true && uiPad2Vdata.b_speedoff() == false &&
        //         uiPad2Vdata.b_stopflag() == true)
        //     {
        //          fsOptimalPath << std::setprecision(15) <<0<<",";
        //     }
        //     else
        //     {
        //         fsOptimalPath << std::setprecision(15) <<decisionData.optimalGlobalTrajectory.planningPoints[i].v<<",";
        //     }

        //     // //end of add by syp
        //     fsOptimalPath << std::setprecision(15) <<decisionData.optimalGlobalTrajectory.planningPoints[i].curvature<<",";
        //     fsOptimalPath << std::setprecision(15) <<decisionData.optimalGlobalTrajectory.planningPoints[i].s<<","<<endl;

        //     //cout <<  "????????????????????" <<"x = "<<decisionData.optimalGlobalTrajectory.planningPoints[i].gaussX <<"??????????????????????????????"<<endl;
        // }

        // fsOptimalPath << "-------------------------------------------------"<<endl;
        // fsOptimalPath.close();

        // 临时行增加unity需要的车辆等障碍物信息
        Planning::ObjectsProtoForUnity *ptrObjectsForUnity;
        for (size_t i = 0; i < objectsCmd.size(); i++)
        {
            ptrObjectsForUnity = trajectoryPointVec.add_objectsforunity();

            ptrObjectsForUnity->set_type(objectsCmd[i].type());
            ptrObjectsForUnity->set_objectid(objectsCmd[i].objectid());
            ptrObjectsForUnity->set_lat(objectsCmd[i].lat());
            ptrObjectsForUnity->set_lon(objectsCmd[i].lon());
            ptrObjectsForUnity->set_yaw(objectsCmd[i].yaw());
            ptrObjectsForUnity->set_velocity(objectsCmd[i].velocity());
            ptrObjectsForUnity->set_len(objectsCmd[i].len());
            ptrObjectsForUnity->set_width(objectsCmd[i].width());
            ptrObjectsForUnity->set_height(objectsCmd[i].height());
            ptrObjectsForUnity->set_timestamp(objectsCmd[i].timestamp());
            ptrObjectsForUnity->set_status(objectsCmd[i].status());
            ptrObjectsForUnity->set_vctype(objectsCmd[i].vctype());

            // std::cout << "??????????objectsCmd[i].len()=" << objectsCmd[i].len()<<", objectsCmd[i].width()"<<objectsCmd[i].width()<<std::endl;
        }

        // 再次为unity增加road list
        //  20230620 添加roadlist信息,要对roadlist进行剔重处理
        if (lastRoadList.bNewRoadList)
        {
            // 赋值
            list<int>::iterator IterLastPath;

            for (IterLastPath = lastRoadList.roadlist.begin(); IterLastPath != lastRoadList.roadlist.end(); IterLastPath++)
            {
                trajectoryPointVec.add_roadforunity((*IterLastPath));
                // std::cout << " pImuTemp->add_road( (*IterLastPath) )  ="<< (*IterLastPath) <<std::endl;
            }
        }

        Planning::TrafficLightForUnity *trafficLightForUnity = trajectoryPointVec.mutable_trafficlightforunity();
        trafficLightForUnity->set_state(trafficLight.state());
        trafficLightForUnity->set_lightdirerction(trafficLight.directionoftravel());

        //  std::cout << "trafficLight result=" << trafficLight.stoproadid()<<","<<trafficLight.stoplaneid()<<","<<trafficLight.stoppointid()
        //                    <<",light state "<<trafficLight.state()<<", light state V2X"<<intersectionState.phases(i).lightstate()<<std::endl;
        // 20231010 为unity增加停止点相关信息
        trajectoryPointVec.set_stopgaussx(stopPoints[1].GaussX);
        trajectoryPointVec.set_stopgaussy(stopPoints[1].GaussY);

        // 计算每段的距离
        double distanceToStop = 0;
        // 如果只有一段，就是当前位置到停车点的距离

        if (lastRoadList.bNewRoadList) // 似乎这里一直是true了，这部分回来得去掉
        {
            if (lastRoadList.roadlist.size() <= 0)
            {
                // 没有路点，奇葩了
            }
            if (lastRoadList.roadlist.size() == 1) // 就这一段了，不管对不对，就用这个road
            {
                Road roadTemp;
                if (map.GetRoadByRoadID(lastRoadList.roadlist.front(), roadTemp))
                {
                    int startIndex = std::min((int)roadTemp.lanes[0].gaussRoadPoints.size() - 1, decisionData.currentIndex);
                    double startS = roadTemp.lanes[0].gaussRoadPoints[startIndex].s;
                    int endIndex = std::min((int)roadTemp.lanes[0].gaussRoadPoints.size() - 1, std::get<2>(stopPointRoadLanePointId));
                    double endS = roadTemp.lanes[0].gaussRoadPoints[endIndex].s;
                    distanceToStop = std::max(0.0, endS - startS);
                }
                // double start = map.roads[decisionData.rrentId ].lanes[0].gaussRoadPoints[std::get<2>(stopPointRoadLanePointId)].s;
                // double end  = map.roads[std::get<0>(stopPointRoadLanePointId) ].lanes[0].gaussRoadPoints[std::get<2>(stopPointRoadLanePointId)].s;
            }
            else
            {
                list<int>::iterator IterLastPath;

                for (IterLastPath = lastRoadList.roadlist.begin(); IterLastPath != lastRoadList.roadlist.end(); IterLastPath++)
                {
                    Road roadTemp;
                    if (map.GetRoadByRoadID((*IterLastPath), roadTemp))
                    {
                        if (IterLastPath == lastRoadList.roadlist.begin()) // 第一个
                        {
                            int startIndex = std::min((int)roadTemp.lanes[0].gaussRoadPoints.size() - 1, decisionData.currentIndex);
                            double startS = roadTemp.lanes[0].gaussRoadPoints[(int)roadTemp.lanes[0].gaussRoadPoints.size() - 1].s -
                                            roadTemp.lanes[0].gaussRoadPoints[startIndex].s;

                            distanceToStop += startS;
                            // std::cout << "  trajectoryPointVec.set_distancetostop IterLastPath ="<< (*IterLastPath)  <<" (start)  ="<<  startS<< " distanceToStop = "  << distanceToStop <<std::endl;
                        }
                        else if (IterLastPath == (--lastRoadList.roadlist.end())) // 最后一个
                        {
                            int endIndex = std::min((int)roadTemp.lanes[0].gaussRoadPoints.size() - 1, std::get<2>(stopPointRoadLanePointId));
                            double endS = roadTemp.lanes[0].gaussRoadPoints[endIndex].s;
                            distanceToStop += endS;
                            // std::cout << "  trajectoryPointVec.set_distancetostop IterLastPath ="<< (*IterLastPath)  <<" (endS)  ="<<  endS<< " distanceToStop = "  << distanceToStop <<std::endl;
                        }
                        else
                        {
                            double middle = roadTemp.lanes[0].gaussRoadPoints[(int)roadTemp.lanes[0].gaussRoadPoints.size() - 1].s;
                            distanceToStop += middle;
                            // std::cout << "  trajectoryPointVec.set_distancetostop IterLastPath ="<< (*IterLastPath)  <<" (middle)  ="<<  middle<< " distanceToStop = "  << distanceToStop <<std::endl;
                        }
                    }

                    // std::cout << " pImuTemp->add_road( (*IterLastPath) )  ="<< (*IterLastPath) <<std::endl;
                }
            }
            // 赋值
        }

        // std::cout << "  trajectoryPointVec.set_distancetostop (distanceToStop)  ="<<  distanceToStop <<std::endl;

        trajectoryPointVec.set_distancetostop(distanceToStop);

        // double distanceToStop=7;//当前位置到停车点的距离，单位为m

        // 序列化
        size_t size;
        void *buffer;
        zmq_msg_t msg;
        if (decisionData.optimalGlobalTrajectory.planningPoints.size() > 0)
        {
            size = trajectoryPointVec.ByteSize();
            buffer = malloc(size);
            if (!trajectoryPointVec.SerializeToArray(buffer, size))
            {
                std::cerr << "Failed to write msg." << std::endl;
            }
            // 发送

            zmq_msg_init_size(&msg, size);
            memcpy(zmq_msg_data(&msg), buffer, size); // copy data from buffer to zmq msg
            zmq_msg_send(&msg, sSocketList[0], 0);
            // cout <<  "????????????????????" <<"size = "<<size <<"??????????????????????????????"<<endl;

            // by syp 20221031 释放资源
            zmq_msg_close(&msg);

            if (NULL != buffer)
                free(buffer);
        }

        // for pad UI app 交互通讯
        // 赋值
        ui::UiV2PadData uiV2PadDataTemp;

        ui::Posture *pPostureTemp = uiV2PadDataTemp.add_posture();

        ui::Imu *pImuTemp = pPostureTemp->add_imu();
        pImuTemp->set_b_isalive(true);
        pImuTemp->set_velocity(imu.velocity());
        pImuTemp->set_yaw(imu.yaw());

        // 20230620 添加roadlist信息,要对roadlist进行剔重处理
        if (lastRoadList.bNewRoadList)
        {
            // 赋值
            list<int>::iterator IterLastPath;

            for (IterLastPath = lastRoadList.roadlist.begin(); IterLastPath != lastRoadList.roadlist.end(); IterLastPath++)
            {
                pImuTemp->add_road((*IterLastPath));
                // std::cout << " pImuTemp->add_road( (*IterLastPath) )  ="<< (*IterLastPath) <<std::endl;
            }
        }
        // 停车点的经纬度
        ZtGeographyCoordinateTransform ztTemp;
        double gaussXTemp, gussYTemp;
        ztTemp.BL2XY(imu.latitude(), imu.longitude(), gussYTemp, gaussXTemp); // 用于计算中央子午线
        double dLatTemp, dLonTemp;
        ztTemp.XY2BL(stopPoints[1].GaussY, stopPoints[1].GaussX, dLatTemp, dLonTemp);
        pImuTemp->set_destlat(dLatTemp); // double destLat=15;
        pImuTemp->set_destlon(dLonTemp); // double destLon=16;
        pImuTemp->set_if_to_dest(stopPointJudge(imu, stopPoints));

        ui::Gnss *pGnssTemp = pPostureTemp->add_gnss();
        pGnssTemp->set_b_isalive(true);
        pGnssTemp->set_longitude(imu.longitude());
        pGnssTemp->set_latitude(imu.latitude());
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

        uiV2PadDataTemp.set_s_targetpath(stopTargetPathSource);

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
        if (NULL != buffer)
            free(buffer);
        // end by syp

        // for 高铁新城发送全局规划道路信息,之前想只在全局规划更新的时候发送数据，发现不能及时相应新连接，修改为实施发送当前全局规划结果
        // 赋值;
        Planning::RoutingPointVec routingPointVectTemp;

        // std::cout<< RED << "imu.gpsvalid()   ="<<imu.gpsvalid() << "&& lastRoadList.bNewRoadList "<< lastRoadList.bNewRoadList
        // <<  " lastRoadList.roadlist.size(),"<< lastRoadList.roadlist.size()<< RESET<<std::endl;

        // if(imu.gpsvalid() > 0 && lastRoadList.bNewRoadList)
        if (imu.gpsvalid() > 0 && lastRoadList.bNewRoadList)
        {
            // 赋值
            list<int>::iterator IterLastPath;

            // std::cout << "lastRoadList.roadlist.size()  ="<<lastRoadList.roadlist.size() <<std::endl;
            for (IterLastPath = lastRoadList.roadlist.begin(); IterLastPath != lastRoadList.roadlist.end(); IterLastPath++)
            {
                Planning::RoutingPoint *rPointTemp;

                //    if(! map.GetStartPointByRoadID((*IterLastPath),gaussXTemp,gussYTemp))//获取道路起点的坐标
                //    {
                //         std::cout << "GetStartPointByRoadID(*IterLastPath)  ="<<(*IterLastPath) <<std::endl;
                //         break;
                //    }

                //     //转经纬度
                //     double dLatTemp, dLonTemp;
                //     ztTemp.XY2BL(gussYTemp,gaussXTemp,dLatTemp, dLonTemp);
                //     Planning::RoutingPoint * rPointTemp  = routingPointVectTemp.add_routingpoints();
                //     rPointTemp ->set_latitude(dLatTemp);
                //     rPointTemp ->set_longtitude(dLonTemp);
                //     rPointTemp ->set_height(0);
                //     std::cout << setprecision(10) << "start dLatTemp  ="<<dLatTemp << " dLonTemp "<<dLonTemp<<std::endl;

                //     if(! map.GetEndPointByRoadID((*IterLastPath),gaussXTemp,gussYTemp))//获取道路终点的坐标
                //     {
                //        // std::cout << "GetEndPointByRoadID(*IterLastPath)  ="<<(*IterLastPath) <<std::endl;
                //         break;
                //    }

                //     //转经纬度
                //     //double dLatTemp, dLonTemp;
                //     ztTemp.XY2BL(gussYTemp,gaussXTemp,dLatTemp, dLonTemp);
                //     rPointTemp  = routingPointVectTemp.add_routingpoints();
                //     rPointTemp ->set_latitude(dLatTemp);
                //     rPointTemp ->set_longtitude(dLonTemp);
                //     rPointTemp ->set_height(0);
                //     std::cout << setprecision(10) <<"end dLatTemp  ="<<dLatTemp << " dLonTemp "<<dLonTemp<<std::endl;
                Lane laneTemp;
                if (!map.GetLaneByRoadIDLaneID((*IterLastPath), 0, laneTemp)) // 获取道路终点的坐标
                {
                    // std::cout << "GetEndPointByRoadID(*IterLastPath)  ="<<(*IterLastPath) <<std::endl;
                    break;
                }

                // std::cout << "GetEndPointByRoadID(*IterLastPath)  ="<<(*IterLastPath) <<std::endl;

                // 转经纬度
                // double dLatTemp, dLonTemp;
                for (int i = 0; i < (int)laneTemp.gaussRoadPoints.size(); i = i + 25) // 没隔5个点，约为1米，产生一个点
                {
                    ztTemp.XY2BL(laneTemp.gaussRoadPoints[i].GaussY, laneTemp.gaussRoadPoints[i].GaussX, dLatTemp, dLonTemp);
                    rPointTemp = routingPointVectTemp.add_routingpoints();
                    rPointTemp->set_latitude(dLatTemp);
                    rPointTemp->set_longtitude(dLonTemp);
                    rPointTemp->set_height(0);
                    // std::cout << setprecision(10) <<"end dLatTemp  ="<<dLatTemp << " dLonTemp "<<dLonTemp<<std::endl;
                }

            } // for(IterLastPath=lastRoadList.roadlist.begin();IterLastPath!=lastRoadList.roadlist.end();IterLastPath++)

            // 添加后续路口信息
            // 提取当前路和下一段路的road、lane，
            // 注意，如果是换道的话就用当前lane的红绿灯,如果是最后一段路，也用当前lane的红绿灯
            TrafficLight trafficLightTemp;
            bool bTrafficLightTemp;
            if (decisionData.nextIdList.size() == 0) // 后续没有路了
            {
                bTrafficLightTemp = map.trafficeLightMap_.GetTrafficLightByRoadLane(decisionData.currentId, decisionData.currentLaneId, decisionData.currentIndex,
                                                                                    decisionData.currentId, decisionData.currentLaneId, trafficLightTemp); // 根据roadlane信息查找对象

                std::cout << "check road lane for intersection" << decisionData.currentId << "," << decisionData.currentLaneId << "," << decisionData.currentIndex
                          << "," << decisionData.currentId << "," << decisionData.currentLaneId << std::endl;
            }
            else
            {
                bTrafficLightTemp = map.trafficeLightMap_.GetTrafficLightByRoadLane(decisionData.currentId, decisionData.currentLaneId, decisionData.currentIndex,
                                                                                    std::get<0>(decisionData.nextIdList[0]), std::get<1>(decisionData.nextIdList[0]), trafficLightTemp);

                //   std::cout << "check road lane intersection" << decisionData.currentId << ","  << decisionData.currentLaneId<< "," << decisionData.currentIndex
                // << ","  <<  std::get<0>(decisionData.nextIdList[0]) << ","  <<std::get<1>(decisionData.nextIdList[0]) <<std::endl;
            }

            Planning::NextIntersection *nextInersectionTemp = routingPointVectTemp.mutable_nextintersection();
            if (bTrafficLightTemp) // 找到下一个路口
            {
                nextInersectionTemp->set_currentroadid(trafficLightTemp.currentRoadID_);
                nextInersectionTemp->set_currentlaneid(trafficLightTemp.currentLaneID_);
                nextInersectionTemp->set_currentpointid(trafficLightTemp.stopPoint_);
                nextInersectionTemp->set_nextroadid(trafficLightTemp.nextRoadID_);
                nextInersectionTemp->set_nextlaneid(trafficLightTemp.nextLaneID_);

                nextInersectionTemp->set_trafficlightgaussx(trafficLightTemp.gaussX_);
                nextInersectionTemp->set_trafficlightgaussy(trafficLightTemp.gaussY_);
                nextInersectionTemp->set_trafficlightheight(trafficLightTemp.height_);
                // 20231012
                nextInersectionTemp->set_directionoftravel(trafficLightTemp.directionofTravel_);

                nextInersectionTemp->set_intersectionid(trafficLightTemp.intersetionID_);
                nextInersectionTemp->set_phaseid(trafficLightTemp.phaseID_);

                nextInersectionTemp->set_shape(trafficLightTemp.lightShape_);
                nextInersectionTemp->set_lightsum(trafficLightTemp.groupSize_);
                nextInersectionTemp->set_lightidx(trafficLightTemp.groupIndex_);
            }
            else
            {
                nextInersectionTemp->set_currentroadid(-1);
                nextInersectionTemp->set_currentlaneid(-1);
                nextInersectionTemp->set_currentpointid(-1);
                nextInersectionTemp->set_nextroadid(-1);
                nextInersectionTemp->set_nextlaneid(-1);

                nextInersectionTemp->set_trafficlightgaussx(0.0);
                nextInersectionTemp->set_trafficlightgaussy(0.0);
                nextInersectionTemp->set_trafficlightheight(0.0);

                nextInersectionTemp->set_intersectionid("");
                nextInersectionTemp->set_phaseid(-1);

                nextInersectionTemp->set_shape("");
                nextInersectionTemp->set_lightsum(0);
                nextInersectionTemp->set_lightidx(-1);
            }

            // nextInersectionTemp->set

            // if(!bTrafficLightTemp)//没找到有用的路口信息
            // {
            //     std::cout << "no find trafficLightTemp" <<std::endl;
            // }
            // routingPointVectTemp.set_n

            // std::cout << RED << "send nextInersectionTemp" << nextInersectionTemp->currentroadid()
            //           << "," << nextInersectionTemp->nextlaneid()
            //           << "," << nextInersectionTemp->trafficlightgaussx()
            //           << "," << nextInersectionTemp->trafficlightgaussy() << RESET << std::endl;

            // 序列化
            size = routingPointVectTemp.ByteSize();
            // std::cout << "routingPointVectTemp.ByteSize() =" << size << std::endl;
            buffer = malloc(size);
            if (!routingPointVectTemp.SerializeToArray(buffer, size))
            {
                std::cerr << "Failed to write routingPointVectTemp msg." << std::endl;
            }

            // 发送
            //  zmq_msg_t msg;
            zmq_msg_init_size(&msg, size);
            memcpy(zmq_msg_data(&msg), buffer, size); // copy data from buffer to zmq msg
            zmq_msg_send(&msg, sSocketList[2], 0);

            zmq_msg_close(&msg);
            if (NULL != buffer)
                free(buffer);

            // std::cout << "send routingPointVectTemp OK" <<std::endl;

        } // if(imu.gpsvalid() > 0 && lastRoadList.bNewRoadList)

        mutex.unlock();
        //=======end of your works======

        auto end = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

        // std::cout << "Thread publisher duration time = " << duration.count() <<" sleep time = " << (this->sendRate - duration.count()) << std::endl;

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
        if ((int)routingListVector.size() > 0 && (int)routingListVector[0].roadlanelist.size() > 0)
        {
            // routingList.clear();
            // routingList.push_back(std::tuple<int32_t, int32_t>(3, 0));
            // std::vector<infopack::ObjectsProto> objectsCmd;
            // predict
            //  objectsCmd.insert(objectsCmd.end(), simplePredictionCmd.begin(), simplePredictionCmd.end());

            // 存放路测收到的障碍物信息+预测出来的障碍物信息
            objectsCmdAndPredict.clear();
            objectsCmdAndPredict.insert(objectsCmdAndPredict.begin(), objectsCmd.begin(), objectsCmd.end());                       // 路测收到的障碍物信息
            objectsCmdAndPredict.insert(objectsCmdAndPredict.end(), simplePredictionCmd.begin(), simplePredictionCmd.end());       // 预测出来的障碍物信息
            objectsCmdAndPredict.insert(objectsCmdAndPredict.end(), simplePredictionCmdClu.begin(), simplePredictionCmdClu.end()); // 激光按照路侧格式预测出来的障碍物信息
            //  for (int i = 0; i < objectsCmdAndPredict.size(); i++){
            //             std::cout<<i<<" 's"<<"objectsCmdAndPredict[i]objectid()"<<objectsCmdAndPredict[i].objectid()<<std::endl;
            //  }
            // prediction::ObjectList  predictionTempClus;
            predictionAndClu.Clear();
            predictionAndClu = prediction;
            // 聚类的原始数据加入到prediction
            for (int i = 0; i < drawForInt.size(); i++)
            {
                double dXForShow, dYForShow, dYawForShow;
                dXForShow = drawForInt[i].predictpoint(0).y();
                dYForShow = drawForInt[i].predictpoint(0).x();
                // dYawForShow = M_PI / 2 - objProto.predictpoint(0).vy() * M_PI / 180.;
                dYawForShow = M_PI / 2 - drawForInt[i].predictpoint(0).vy();
                prediction::Object *object = predictionAndClu.add_object();
                // for (int i =0; i <=0; i++)
                // {
                prediction::PredictPoint *predictPoint = object->add_predictpoint();
                double v = drawForInt[i].predictpoint(0).vx();
                double veX, veY, veYaw;
                veX = imu.gaussy(); // 自车的位置
                veY = imu.gaussx();
                veYaw = M_PI / 2 - imu.yaw() * M_PI / 180.;
                // cout << "veYaw.yaw:" << veYaw << endl;
                CoordTran2DForNew0INOld(dXForShow, dYForShow, dYawForShow, veX, veY, veYaw); //  高斯坐标转换车当前坐标
                predictPoint->set_x(dXForShow);
                predictPoint->set_y(dYForShow);
                predictPoint->set_vx(v);
                predictPoint->set_vy(dYawForShow);
                // }
                object->set_z(drawForInt[i].z());
                object->set_h(drawForInt[i].h());
                object->set_l(drawForInt[i].l());
                object->set_w(drawForInt[i].w());
                object->set_type(drawForInt[i].type());
                object->set_trackid(drawForInt[i].trackid());
            }
            // 聚类的预测加入到prediction
            for (int i = 0; i < predictionClu.size(); i++)
            {
                double dXForShow, dYForShow, dYawForShow;
                dXForShow = predictionClu[i].predictpoint(0).x();
                dYForShow = predictionClu[i].predictpoint(0).y();
                // dYawForShow = M_PI / 2 - objProto.predictpoint(0).vy() * M_PI / 180.;
                dYawForShow = M_PI / 2 - predictionClu[i].predictpoint(0).vy();
                prediction::Object *object = predictionAndClu.add_object();
                // for (int i =0; i <=0; i++)
                // {
                prediction::PredictPoint *predictPoint = object->add_predictpoint();
                double v = predictionClu[i].predictpoint(0).vx();
                double veX, veY, veYaw;
                veX = imu.gaussy(); // 自车的位置
                veY = imu.gaussx();
                veYaw = M_PI / 2 - imu.yaw() * M_PI / 180.;
                // cout << "veYaw.yaw:" << veYaw << endl;
                CoordTran2DForNew0INOld(dXForShow, dYForShow, dYawForShow, veX, veY, veYaw); //  高斯坐标转换车当前坐标
                predictPoint->set_x(dXForShow);
                predictPoint->set_y(dYForShow);
                predictPoint->set_vx(v);
                predictPoint->set_vy(dYawForShow);
                // }
                object->set_z(predictionClu[i].z());
                object->set_h(predictionClu[i].h());
                object->set_l(predictionClu[i].l());
                object->set_w(predictionClu[i].w());
                object->set_type(predictionClu[i].type());
                object->set_trackid(predictionClu[i].trackid());
            }

            // std::cout << "聚类+prediction.size()::" << predictionAndClu.object_size() << std::endl;
            // localPlanning(imu, imu.velocity(), map, decisionData, prediction, routingListVector[0].routingList, stopPoints, trafficLight, objectsCmd);
            // localPlanning(imu, imu.velocity(), map, decisionData, prediction, routingListVector, stopPoints, trafficLight, objectsCmd, spatTrafficLightMap, trafficLightFromPerc);
            localPlanning(imu, imu.velocity(), map, decisionData, predictionAndClu, routingListVector, stopPoints, trafficLight, objectsCmdAndPredict, spatTrafficLightMap, trafficLightFromPerc, predictioncloud);
            // prediction.Clear();
            //// debug//////////////////////////////////////////////////////////////
            // std::ofstream fs0("bezierpath.txt", std::ofstream::trunc);
            // for (uint32_t i = 0; i < decisionData.finalPathList.size(); i++)
            // {
            //     // for (uint32_t j = 0; j < 50; j++)
            //     for (auto planningPoint : decisionData.finalPathList[i].planningPoints)
            //     {
            //         // fs0 << std::setprecision(15) << decisionData.finalPathList[i].planningPoints[j].s << std::endl;
            //         fs0 << std::setprecision(15) << planningPoint.s << std::endl;
            //     }

            //     // for (uint32_t j = 0; j < 50; j++)
            //     for (auto planningPoint : decisionData.finalPathList[i].planningPoints)
            //     {
            //         // fs0 << std::setprecision(15) << decisionData.finalPathList[i].planningPoints[j].l << std::endl;
            //         fs0 << std::setprecision(15) << planningPoint.l << std::endl;
            //     }
            //     fs0 << std::endl;
            // }
            // fs0.close();

            // std::ofstream fs1("localoptimalpath.txt", std::ofstream::trunc);
            // for (auto optimalPoint : decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex].planningPoints)
            // {
            //     fs1 << std::setprecision(15) << optimalPoint.x << std::endl;
            // }
            // fs1 << std::endl;
            // for (auto optimalPoint : decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex].planningPoints)
            // {
            //     fs1 << std::setprecision(15) << optimalPoint.y << std::endl;
            // }
            // fs1 << std::endl;
            // for (auto optimalPoint : decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex].planningPoints)
            // {
            //     fs1 << std::setprecision(15) << optimalPoint.v << std::endl;
            // }
            // fs1.close();

            // std::ofstream fs("globaloptimalpath.txt", std::ofstream::trunc);
            // for (auto optimalPoint : decisionData.optimalGlobalTrajectory.planningPoints)
            // {
            //     fs << std::setprecision(9) << optimalPoint.gaussX << std::endl;
            // }
            // fs << std::endl;
            // for (auto optimalPoint : decisionData.optimalGlobalTrajectory.planningPoints)
            // {
            //     fs << std::setprecision(9) << optimalPoint.gaussY << std::endl;
            // }
            // fs << std::endl;
            // for (auto optimalPoint : decisionData.optimalGlobalTrajectory.planningPoints)
            // {
            //     fs << std::setprecision(15) << optimalPoint.v << std::endl;
            // }
            // fs << std::endl;
            // for (auto optimalPoint : decisionData.optimalGlobalTrajectory.planningPoints)
            // {
            //     fs << std::setprecision(15) << optimalPoint.gaussAngle << std::endl;
            // }
            // fs << std::endl;
            // for (auto optimalPoint : decisionData.optimalGlobalTrajectory.planningPoints)
            // {
            //     fs << std::setprecision(15) << optimalPoint.curvature << std::endl;
            // }
            // fs << std::endl;
            // for (auto optimalPoint : decisionData.optimalGlobalTrajectory.planningPoints)
            // {
            //     fs << std::setprecision(15) << optimalPoint.s << std::endl;
            // }

        } // if (routingList.size())
        // std::cout << "optimalCurveIndex: " << decisionData.optimalTrajectoryIndex << std::endl; // 测试输出optimal trajectory index

        ////////////////////////////

        clearPathList(decisionData.finalPathList);

        mutex.unlock();
        //=======end of your works======

        auto end = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        // cout << "processorLocalPlanning   " << duration.count() << " sleep " << (std::chrono::milliseconds(this->localPlanningRate - duration.count())).count() << endl;
        int64 sleepspan = this->localPlanningRate - duration.count();
        sleepspan = max(sleepspan, int64(1));

        // std::cout << "Thread localpanning  duration time = " << duration.count() <<" sleep time = " << ( this->localPlanningRate - duration.count()) << "real sleep time" << sleepspan <<std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(sleepspan));
    }
}

// 根据坐标和路ID，找laneID和pointID
void Jobs::FindPointIDatRoad(GaussRoadPoint roadPoint, int32_t roadID, int32_t &laneID, int32_t &pointID)
{
    cout << "FindPointIDatRoad----------------------------------" << endl;
    // 得先根据roadID找到road的索引值
    int roadIndexTemp = -1; // 查找出对应road在map中的索引
    for (int i = 0; i < (int)map.roads.size(); i++)
    {
        if (roadID == map.roads[i].id)
        {
            cout << roadID << "map.roads[i].id" << map.roads[i].id << endl;
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
    for (int i = 0; i < (int)map.roads[roadIndexTemp].lanes.size(); i++) // lane循环
    {
        cout << " find lane ID" << i << endl;
        for (int j = 0; j < (int)map.roads[roadIndexTemp].lanes[i].gaussRoadPoints.size(); j++) // point 循环
        {
            // cout << " find point ID" << j << endl;
            if (abs(map.roads[roadIndexTemp].lanes[i].gaussRoadPoints[j].GaussX - roadPoint.GaussX) < 2.5 &&
                abs(map.roads[roadIndexTemp].lanes[i].gaussRoadPoints[j].GaussY - roadPoint.GaussY) < 2.5) // 找到点
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
        mutex.lock();
        // =======do your works here======
        // 先分析一下平板的指令，然后看看平板和调度是否提供了合适的坐标，用于确定怎么走,这实在是混乱，能想明白就不容易了，能看懂真是牛
        // 平板的等级最高，先看平板的要求
        // std::cout << "switch( uiPad2Vdata.s_targetpath() )" << dispatchCmd.desx() << "&&" << dispatchCmd.desy() << " " << dispatchCmd.road_size() << std::endl;

        switch (uiPad2Vdata.s_targetpath())
        {

        case -1: // 平板链接上了，但是不知道车辆当前状态，车辆按照当前状态继续走，这是一个瞬时的状态，
        case 0:  // 平板没有链接上，但是变量有个默认值，车辆按照当前状态继续走
            if (dispatchCmd.desx() > 10 && dispatchCmd.desy() > 10 && dispatchCmd.road_size() > 0)
                stopTargetPathSource = 2; // 调度的坐标是否有效
            else
                stopTargetPathSource = 1;
            break;
        case 1: // 强制要求按照本地终点走，那就不客气了，这个是兜底的处理
            stopTargetPathSource = 1;
            break;
        case 2: // 强制要求按照调度终点走，但是可能调度没有链接上，平板一厢情愿了，后续再进行判断
            if (dispatchCmd.desx() > 10 && dispatchCmd.desy() > 10 && dispatchCmd.road_size() > 0)
                stopTargetPathSource = 2;
            else
                stopTargetPathSource = 1;
            break;
        case 3: // 强制要求按照平板终点走，但是平板也可能没有位置，后续再进行判断，不过
            if (uiPad2Vdata.destx() > 10 && uiPad2Vdata.desty() > 10 && uiPad2Vdata.dest_isvalid() == true)
                stopTargetPathSource = 3;
            // else  if(dispatchCmd.desx() > 10 &&  dispatchCmd.desy() > 10 &&   dispatchCmd.road_size() > 0)  stopTargetPathSource = 2;
            else
                stopTargetPathSource = 1;
            break;
        }

        // std::cout << "stopTargetPathSource?? = " << stopTargetPathSource << std::endl;
        // 根据是否有服务器指令，确定全局规划的方法，
        // 有调度控制命令,就按照命令执行，除非调度发送无效坐标命令，不论后续是否服务器在线
        // 其实现在是所有的调度状态都按照调度命令控制了，这部分待接口明确
        // if ((dispatchCmd.curstatus() == 0 ||                  // 未定义的状态，但是接口发出来了，现在都用这个
        //      dispatchCmd.curstatus() == Jobs::APPOINTMENT ||  // 车辆被预约
        //      dispatchCmd.curstatus() == Jobs::PASSENGER_ON || // 车上有乘客
        //      dispatchCmd.curstatus() == Jobs::FREE) &&        // 车空闲
        //     dispatchCmd.desx() > 10 &&
        //     dispatchCmd.desy() > 10 &&
        //     dispatchCmd.road_size() > 0) // 调度坐标和路点都要有效
        // 这部分的判断简化了，采用上面处理的结果
        if (stopTargetPathSource == 2)
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

                    // 20231011 补充停止点的yaw角度
                    roadPointTemp.yaw = map.roads[roadIDTemp].lanes[laneIDTemp].gaussRoadPoints[pointIDTemp].yaw;

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
                else // 没找到终点,只能按照本地坐标进行
                {
                    stopTargetPathSource = 1;
                }
            }    //
            else // 无需更新停车点信息
            {
                // 基于服务器信息的全局规划
                // cout<<"processorGlobalPlanningBaseServer by old position"<<endl;
                processorGlobalPlanningBaseServer();
            }
        } // if( dispatchCmd.curstatus() == Jobs::APPOINTMENT ||  //车辆被预约

        if (stopTargetPathSource == 3) // 按照平板的终端进行控制
        {

            // 命令坐标与原坐标不一致，更新停车点信息,
            if ((abs(uiPad2Vdata.destx() + OFFSET_X - stopPoints[stopPoints.size() - 1].GaussX) >= 1) ||
                (abs(uiPad2Vdata.desty() + OFFSET_Y - stopPoints[stopPoints.size() - 1].GaussY) >= 1)) // 位置有变化,1米的偏差
            {

                // 查找停车点  roadID、laneID，pointID等信息并设置
                GaussRoadPoint roadPointTemp; // 命令中的停车点坐标
                roadPointTemp.GaussX = uiPad2Vdata.destx();
                roadPointTemp.GaussY = uiPad2Vdata.desty();

                // 停车点应该在命令中最后一条road上,

                int32_t roadIDTemp = uiPad2Vdata.destrid();

                int32_t laneIDTemp = -1;  // 待检索的laneID
                int32_t pointIDTemp = -1; // 待检索的pointID

                FindPointIDatRoad(roadPointTemp, roadIDTemp, laneIDTemp, pointIDTemp);
                cout << "FindPointIDatRoad" << laneIDTemp << "," << pointIDTemp << endl;

                if (laneIDTemp != -1 && pointIDTemp != -1) // 找到ID
                {
                    // 设置停车点坐标
                    GaussRoadPoint roadPointTemp;                          // 停车点坐标
                    roadPointTemp.GaussX = uiPad2Vdata.destx() + OFFSET_X; // 其实不知道这里是在补偿什么，参照loadStopPoints，难道是安装位置
                    roadPointTemp.GaussY = uiPad2Vdata.desty() + OFFSET_Y;
                    stopPoints.clear();
                    stopPoints.push_back(roadPointTemp); // 注意第0点不被使用
                    stopPoints.push_back(roadPointTemp);
                    cout << "stopPoints GaussX" << stopPoints.size() << "," << stopPoints[1].GaussX << "stopPoints GaussX" << stopPoints[1].GaussY << endl;

                    // 设置停车点ID
                    std::get<0>(stopPointRoadLanePointId) = uiPad2Vdata.destrid();
                    std::get<1>(stopPointRoadLanePointId) = laneIDTemp;
                    std::get<2>(stopPointRoadLanePointId) = pointIDTemp;

                    // 基于本地信息的全局规划

                    processorGlobalPlanningBaseLocal();
                }
                else // 没找到终点,只能按照本地坐标进行
                {
                    stopTargetPathSource = 1;
                }
            }    //
            else // 无需更新停车点信息
            {
                // 本地算法的全局规划
                processorGlobalPlanningBaseLocal();
            }
        }

        if (stopTargetPathSource == 1) // 车空闲，调度坐标无效
        {

            // 重新恢复文件中加载停车点
            stopPoints.assign(stopPointsFromLocal.begin(), stopPointsFromLocal.end());
            std::get<0>(stopPointRoadLanePointId) = std::get<0>(stopPointRoadLanePointIdFromLocal);
            std::get<1>(stopPointRoadLanePointId) = std::get<1>(stopPointRoadLanePointIdFromLocal);
            std::get<2>(stopPointRoadLanePointId) = std::get<2>(stopPointRoadLanePointIdFromLocal);

            // 本地算法的全局规划
            processorGlobalPlanningBaseLocal();
        }

        mutex.unlock();
        //=======end of your works======
        auto end = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

        // cout << "processorGlobalPlanning   " << duration.count() << " sleep " << (std::chrono::milliseconds(this->sendRate - duration.count())).count() << endl;
        // int64 sleepspan = this->localPlanningRate - duration.count();
        // sleepspan = max(sleepspan, int64(1));
        // std::cout << "Thread globalpanning  duration time = " << duration.count() <<" sleep time = " << ( this->globalPlanningRate - duration.count()) << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(this->globalPlanningRate - duration.count()));
    }
}

// 基于服务器信息全局规划
void Jobs::processorGlobalPlanningBaseServer()
{
    cout << "processorGlobalPlanningBaseServer" << endl;
    // mutex.lock();

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

        // int destinationRoad = std::get<0>(stopPointRoadLanePointId);
        // int destinationLane = std::get<1>(stopPointRoadLanePointId);
        // int destinationPoint = std::get<2>(stopPointRoadLanePointId);
        int originRoad = decisionData.currentId;
        int originLane = decisionData.currentLaneId;
        // int originPoint = decisionData.currentIndex;

        //  for (int i = 0; i < dispatchCmd.road_size(); i++)
        // {
        //    bool bFindRoad = false;
        //     list<int>::iterator IterLastPath;
        //      for(IterLastPath=lastRoadList.roadlist.begin();IterLastPath!=lastRoadList.roadlist.end();IterLastPath++)
        //     {
        //         if( dispatchCmd.road(i) == (* IterLastPath) )//在历史轨迹中包含这个road
        //         {
        //             bFindRoad  = true;
        //             break;
        //         }
        //     }

        //     if( bFindRoad == false)//如果不包含在原有路点中间
        //     {
        //         lastRoadList.roadlist.clear();
        //         lastRoadList.roadlist = as.path;
        //         lastRoadList.bNewRoadList = true;
        //         break;
        //     }
        // }

        // 用调度指令中的道路ID赋值AStar道路信息
        // 有可能调度指令的道路ID包含已经走完的道路，从头开始找第一次与当前roidID相一致的路，把之前的删了
        //
        bool bFindfinishRoadIDIndexTemp = false;
        for (int i = 0; i < dispatchCmd.road_size(); i++)
        {
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

        // 检查roadlist是否更新,只要在原来的道路中包含这条路，就可以不更新，这样会减少发送的频率，应该也不会出现道路缺失的问题
        std::cout << "dispatchCmd.road_size() " << dispatchCmd.road_size() << " as.path =" << as.path.size() << std::endl;
        if (dispatchCmd.road_size() > 0)
        {
            lastRoadList.roadlist.clear();
            lastRoadList.roadlist = as.path;
            lastRoadList.bNewRoadList = true;
        }

        // as.path = as.getPath(originRoad, destinationRoad, originPoint, destinationPoint); // 这一步确定roads

        // std::cout<<"------------as.path:"<<as.path.size();
        // for(list<int>::iterator iter= as.path.begin();iter!= as.path.end();iter++)
        // {
        //   std::cout<<","<< (*iter);
        // }
        //  std::cout<<"------------"<<std::endl;

        as.pathLanes.push_back(make_pair(originRoad, originLane)); // 初始化,将当前路点作为规划中的第一条路

        FindRoadLaneListByRoads(as);

    } // else // 在路点上

    // mutex.unlock();
}

// 基于本地信息全局规划
void Jobs::processorGlobalPlanningBaseLocal()
{

    // cout << "processorGlobalPlanningBaseLocal" << endl;
    // mutex.lock();

    if (!getCurrentPosition(decisionData, imu, map)) // 获取当前路点
    {
        // 如果没有获取到路点，不做处理，局部规划自己会同样作不在路点判断和处理
        // 是不是这里应该把routingList清空？？？
        std::cout << RED << "??? processorGlobalPlanningBaseLocal get current position failed :(" << RESET << std::endl;
    }
    else // 在路点上
    {
        // std::cout << RED << "??? znemmejilaidene" << RESET << std::endl;
        Astar as;
        as.mapToAstar(map, &as);

        int destinationRoad = std::get<0>(stopPointRoadLanePointId);
        int destinationLane = std::get<1>(stopPointRoadLanePointId);
        int destinationPoint = std::get<2>(stopPointRoadLanePointId);
        int originRoad = decisionData.currentId;
        int originLane = decisionData.currentLaneId;
        int originPoint = decisionData.currentIndex;

        // std::cout << "origin >>RoadId" << originRoad << " >>LaneId" << originLane << " >>pointId" << decisionData.currentIndex << std::endl;
        // std::cout << "destination >>RoadId" << destinationRoad << " >>LaneId" << destinationLane << " >>pointId" << destinationPoint << std::endl;
        as.path = as.getPath(originRoad, destinationRoad, originPoint, destinationPoint); // 这一步确定roads

        // 输出road list
        //   std::cout << "------------as.path size = :" << as.path.size() << "Road ID:";
        //   for (list<int>::iterator iter = as.path.begin(); iter != as.path.end(); iter++)
        //   {
        //       std::cout << "," << (*iter);
        //   }
        //   std::cout << "------------" << std::endl;

        // 检查roadlist是否更新,只要在原来的道路中包含这条路，就可以不更新，这样会减少发送的频率，应该也不会出现道路缺失的问题
        // 不用比较了，直接更新
        if (!as.path.empty())
        {
            lastRoadList.roadlist.clear();
            lastRoadList.roadlist = as.path;
            lastRoadList.bNewRoadList = true;
        }
        // list<int>::iterator IterPath;
        // for(IterPath=as.path.begin();IterPath!=as.path.end();IterPath++)
        // {
        //    bool bFindRoad = false;
        //     list<int>::iterator IterLastPath;
        //      for(IterLastPath=lastRoadList.roadlist.begin();IterLastPath!=lastRoadList.roadlist.end();IterLastPath++)
        //     {
        //         if( (*IterPath) == (* IterLastPath) )//在历史轨迹中包含这个road
        //         {
        //             bFindRoad  = true;
        //             break;
        //         }
        //     }

        //     if( bFindRoad == false)//如果不包含在原有路点中间
        //     {
        //         lastRoadList.roadlist.clear();
        //         lastRoadList.roadlist = as.path;
        //         lastRoadList.bNewRoadList = true;
        //         break;
        //     }
        // }

        // std::cout << " lastRoadList.bNewRoadList" <<  lastRoadList.bNewRoadList << std::endl;

        FindRoadLaneListByRoads(as);

    } // else // 在路点上

    // 查找当前road

    // mutex.unlock();
}

void Jobs::FindRoadLaneListByRoads(Astar &as)
{
    if (as.path.empty())
        return;

    int destinationRoad = std::get<0>(stopPointRoadLanePointId);
    int destinationLane = std::get<1>(stopPointRoadLanePointId);
    // int destinationPoint = std::get<2>(stopPointRoadLanePointId);
    int originRoad = decisionData.currentId;
    int originLane = decisionData.currentLaneId;
    int originPoint = decisionData.currentIndex;

    // 如果road不更换的情况，没有必要在计算新的轨迹，用之前的就可以吧
    // 这里可能会出现的问题是，在行走的过程中接收到了调度发送的全局规划
    //  if(routingListVector.size() > 0 &&routingListVector[0].routingList.size() > 0 &&
    //  std::get<0> (routingListVector[0].routingList[0]) ==originRoad )
    //      return;

    routingListVector.clear();
    routingListVector.reserve(1);

    // std::cout << RED << "routingListVector.size()  = " << routingListVector.size() <<RESET <<std::endl;

    // 从当前点沿着roads 找 lane，因为之前找road的时候是成功的，所以这次查找一定能找到
    std::vector<std::tuple<int32_t, int32_t>> routingListPass; // 从中心lane到当前lane之间的lane，包含中心lane

    FindRoadLaneByRoads(as, routingListPass, originRoad, originLane, originPoint, destinationRoad, destinationLane, RoutingList::MIDDLE);

    // std::cout << RED << "after middle routingListVector.size()  = " << routingListVector.size() <<RESET <<std::endl;

    // 查找road其他lane是否可以到达终点
    Road currentRoad;
    Lane currentLane;
    // 20230913 如果当前路没有可行通路，不进行左右lane的查找
    if ((routingListVector.size() > 0) && map.GetRoadByRoadID(originRoad, currentRoad))
    {
        if (map.GetLaneByLaneID(originLane, currentRoad, currentLane))
        {
            routingListPass.clear();
            routingListPass.reserve(1);
            routingListPass.push_back(std::tuple<int32_t, int32_t>(originRoad, originLane));

            for (int i = 0; i < (int)currentLane.leftLaneId.size(); i++) // 左边的道路
            {
                // std::cout << "road id " << currentRoad.id << "lane id " << currentLane.id<< "has lane left ID  = "<<  currentLane.leftLaneId[i]<< std::endl;
                // originPoint 这里没有对应到本lane的点，因为函数里面这个参数只是计算一个比率关系，就不找了
                FindRoadLaneByRoads(as, routingListPass, originRoad, currentLane.leftLaneId[i], originPoint, destinationRoad, destinationLane, RoutingList::LEFT);
                routingListPass.push_back(std::tuple<int32_t, int32_t>(originRoad, currentLane.leftLaneId[i]));
            }

            routingListPass.clear();
            routingListPass.reserve(1);
            routingListPass.push_back(std::tuple<int32_t, int32_t>(originRoad, originLane));

            for (int i = 0; i < (int)currentLane.rightLaneId.size(); i++) // 右边的道路
            {
                // std::cout  << "road id " << currentRoad.id  << "lane id " << currentLane.id<<  " has lane right  ID  = "<<  currentLane.rightLaneId[i]  << std::endl;
                // originPoint 这里没有对应到本lane的点，因为函数里面这个参数只是计算一个比率关系，就不找了
                FindRoadLaneByRoads(as, routingListPass, originRoad, currentLane.rightLaneId[i], originPoint, destinationRoad, destinationLane, RoutingList::RIGHT);
                routingListPass.push_back(std::tuple<int32_t, int32_t>(originRoad, currentLane.rightLaneId[i]));
            }
        }
    }

    // 输出全局规划road lane list
    // std::cout << "toatl road line list  =  " << routingListVector.size() << std::endl;
    // for (int i = 0; i < (int)routingListVector.size(); i++)
    // {
    //     if (i > 0)
    //         break;
    //     std::cout << "global  road line list  =  " << i << "/// ";
    //     for (int j = 0; j < (int)routingListVector[i].roadlanelist.size(); j++)
    //     {
    //         std::cout << get<0>(routingListVector[i].roadlanelist[j]) << "," << get<1>(routingListVector[i].roadlanelist[j]) << ";";
    //     }
    //     std::cout << "---------global  road " << std::endl;
    // }
}

void Jobs::FindRoadLaneByRoads(Astar &as, std::vector<std::tuple<int32_t, int32_t>> routingListPass, int originRoad, int originLane, int originPoint, int destinationRoad, int destinationLane, RoutingList::LANE_DIRECTION leftrightLane)
{
    // std::cout << "fine lane from : " <<originRoad<<":"<<originLane<<":"<< originPoint<<" to " << destinationRoad<<","<<destinationLane<<std::endl;
    as.pathLanes.clear();
    as.pathLanes.reserve(1);

    as.pathLanes.push_back(make_pair(originRoad, originLane)); // 初始化,将当前路点作为规划中的第一条路

    if (!as.seekLane(map.roads, as.path, as.pathLanes, destinationLane, originPoint)) // 没找到
    {
        // std::cout << "* * * 当前起点没有可通行路径 : " << std::endl;
        return;
    }

    if (as.pathLanes.back().second != destinationLane)
        as.pathLanes.push_back(make_pair(destinationRoad, destinationLane)); // 保证终点的lane

    //    std::cout << std::endl
    //               << "* * * 用Astar找到的最短路为 : " << std::endl;
    //     as.moduleSelfCheckPrint(as.pathLanes);

    // 如果这条道又与之前规划的lane 有重复的，剔除   , routingListPass,是从中间lane到本lane前
    //??跟中心道路的路径进行比较,在当前道路road上进行对比
    if (routingListVector.size() > 0) // 不是中心道路本尊
    {
        for (int i = 0; i < (int)routingListVector[0].roadlanelist.size(); i++) // 每一条已经生成的road lane
        {
            if (std::get<0>(routingListVector[0].roadlanelist[0]) != std::get<0>(routingListVector[0].roadlanelist[i])) // 只比较本road
                continue;
            // std::cout << "routingListVector[0].roadlanelist.size()"<<routingListVector[0].roadlanelist.size()<<std::endl;
            for (int j = 0; j < (int)as.pathLanes.size(); j++)
            {
                if (as.pathLanes[0].first != as.pathLanes[j].first) // 只比较本road
                    continue;
                // std::cout << "check point:" << as.pathLanes.size() << ":" <<std::get<0>(routingListVector[0].roadlanelist[i]) << ","<< std::get<1>(routingListVector[0].roadlanelist[i]) << "??" << as.pathLanes[j].first <<"," <<  as.pathLanes[j].second <<std::endl;
                if ((std::get<0>(routingListVector[0].roadlanelist[i]) == as.pathLanes[j].first) && (std::get<1>(routingListVector[0].roadlanelist[i]) == as.pathLanes[j].second))
                {
                    // std::cout << "回环道路"<<std::endl;
                    return;
                }
            }
        }

    } // if(routingListVector.size() > 0)//不是中心道路本尊

    RoutingList routingListTemp;
    routingListTemp.roadlanelist.clear();
    routingListTemp.roadlanelist.reserve(as.pathLanes.size());

    // 补充中央车道到本车道中间的车道
    if (routingListPass.size() > 0)
    {
        routingListTemp.roadlanelist = routingListPass;
    }

    // 格式转换
    for (int i = 0; i < (int)as.pathLanes.size(); i++)
    {
        routingListTemp.roadlanelist.push_back(std::tuple<int32_t, int32_t>(as.pathLanes[i].first, as.pathLanes[i].second));
    }

    // 这里增加一个限制，就是只能换一次道，换多个道的情况，目前referencelane 的模式还不能很好支持，不知道在一个lane上可以行走多长，且防碰撞也很复杂
    // 这里还不对，如果是本来就要换2个lane，还有问题
    // for(int i=0,i<routingListTemp.routingList.size();i++ )
    // {
    //     if(std::get<0>(routingListTemp.routingList[0]) == std::get<0> (routingListTemp.routingList[2])  )
    //     return;
    // }

    routingListTemp.leftRightLane = leftrightLane;
    routingListTemp.nChangeLane = (int)routingListPass.size();

    // 添加可行道路
    routingListVector.push_back(routingListTemp);

    // for(int  i=0;i < routingListTemp.routingList.size();i++)
    // {
    //     std::cout << "routingListTemp.routingList =" <<std::get<0>(routingListTemp.routingList[i] )  <<"," << std::get<1>(routingListTemp.routingList[i] ) <<";";
    // }
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
        // cout << "thread  " << id << " is alive" << endl;
    }
}

// 获取线程的调度策略和优先级
void printThreadPolicy_priority(pthread_t tid)
{
    // pthread_t tid = pthread_self();  // 获取当前线程的pthread id
    // thread1.native_handle()
    // pthread_t  tid = this_thread().native_handle();
    int policy;
    struct sched_param param;
    pthread_getschedparam(tid, &policy, &param);
    printf("Thread %ld scheduling policy is %s, priority is %d\n", (long)tid,
           (policy == SCHED_FIFO ? "SCHED_FIFO" : (policy == SCHED_RR ? "SCHED_RR" : (policy == SCHED_OTHER ? "SCHED_OTHER" : "unknown"))),
           param.sched_priority);
}

int main()
{

    //     auto start = std::chrono::steady_clock::now();
    //    for(int i=0; i < 5000; i++)
    //    {
    //         getDistance(i,i,i,i);
    //    }

    //     auto end = std::chrono::steady_clock::now();
    //     auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);

    //     cout << "start = " << start.time_since_epoch().count() << " end= " << end.time_since_epoch().count()<<
    //     " duration(ms) = "   <<duration.count() /1000.0 << endl;
    //     return  0;

    cout << "planning main begin" << endl;
    void *context = zmq_ctx_new();
    int queueLength = 1;

    // imu receiving socket
    void *imuSocket = zmq_socket(context, ZMQ_SUB);
    int resultTemp = zmq_setsockopt(imuSocket, ZMQ_RCVHWM, &queueLength, sizeof(queueLength));
    if (resultTemp == -1)
    {
        std::cout << "zmq_setsockopt(imuSocket, ZMQ_RCVHWM, &queueLength, sizeof(queueLength)); failed " << std::endl;
    }
    else
    {
        std::cout << "zmq_setsockopt(imuSocket, ZMQ_RCVHWM, &queueLength, sizeof(queueLength)); success " << std::endl;
    }

    // size_t sizeQueueLength;
    // zmq_getsockopt(imuSocket, ZMQ_RCVHWM, &queueLength, &sizeQueueLength);
    // std::cout <<  "zmq_getsockopt(imuSocket, ZMQ_RCVHWM, &queueLength, sizeof(queueLength));" <<queueLength << " "  <<sizeQueueLength<<std::endl;

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
    zmq_connect(padUIReceiveSocket, "tcp://192.168.6.167:3161"); // 192.168.8.103
                                                                 // zmq_connect(padUIReceiveSocket, "tcp://192.168.1.111:3161");
    zmq_setsockopt(padUIReceiveSocket, ZMQ_SUBSCRIBE, "", 0);
    // end of add  by syp

    // 常熟均联路测设备  receiving socket 障碍物信息
    void *OBUCsjlSocket = zmq_socket(context, ZMQ_SUB);
    zmq_setsockopt(OBUCsjlSocket, ZMQ_RCVHWM, &queueLength, sizeof(queueLength));
    // zmq_connect(OBUCsjlSocket, "tcp://127.0.0.1:5502");
    // zmq_connect(OBUCsjlSocket, "tcp://192.168.6.82:5581");  //李老师高铁新城路测设备也用这个接口，大家一起来嗨
    zmq_connect(OBUCsjlSocket, "tcp://127.0.0.1:5581");
    zmq_setsockopt(OBUCsjlSocket, ZMQ_SUBSCRIBE, "", 0);

    // 高铁新城路测红绿灯信息  receiving socket
    void *spatTrafficLightSocket = zmq_socket(context, ZMQ_SUB);
    zmq_setsockopt(spatTrafficLightSocket, ZMQ_RCVHWM, &queueLength, sizeof(queueLength));
    // zmq_connect(spatTrafficLightSocket, "tcp://192.168.6.82:5583");
    zmq_connect(spatTrafficLightSocket, "tcp://127.0.0.1:5583");
    zmq_setsockopt(spatTrafficLightSocket, ZMQ_SUBSCRIBE, "", 0);

    // 自车感知红绿灯信息  receiving socket
    void *percTrafficLightSocket = zmq_socket(context, ZMQ_SUB);
    zmq_setsockopt(percTrafficLightSocket, ZMQ_RCVHWM, &queueLength, sizeof(queueLength));
    zmq_connect(percTrafficLightSocket, "tcp://127.0.0.1:5021");
    zmq_setsockopt(percTrafficLightSocket, ZMQ_SUBSCRIBE, "", 0);

    // 自车感知障碍物聚类信息   //配置订阅方
    void *clusterSocket = zmq_socket(context, ZMQ_SUB);                           // 绑定上下文和模式
    zmq_setsockopt(clusterSocket, ZMQ_RCVHWM, &queueLength, sizeof(queueLength)); // 配置
    zmq_connect(clusterSocket, "tcp://127.0.0.1:5666");                           // 配置 tcp绑定ip地址和端口（端口唯一）
    zmq_setsockopt(clusterSocket, ZMQ_SUBSCRIBE, "", 0);                          // 配置

    // prediction receiving socket From Vision20231106
    void *predictionFromVisionSocket = zmq_socket(context, ZMQ_SUB);
    zmq_setsockopt(predictionFromVisionSocket, ZMQ_RCVHWM, &queueLength, sizeof(queueLength));
    zmq_connect(predictionFromVisionSocket, "tcp://127.0.0.1:5022");
    zmq_setsockopt(predictionFromVisionSocket, ZMQ_SUBSCRIBE, "", 0);

    std::vector<void *> receivingSocketList; // 0: imuSocket; 1:predictionSocket; 2:actuatorSocket
    receivingSocketList.push_back(imuSocket);// 1: 定位
    receivingSocketList.push_back(predictionSocket);// 2: 预测信息，用于传输激光感知信息
    receivingSocketList.push_back(actuatorSocket);// 3: 自车传感器信息

    // add by syp 20221031 for pad UI app 交互通讯
    receivingSocketList.push_back(padUIReceiveSocket); // 4: UI 平板通讯
    // end of add  by syp
    receivingSocketList.push_back(OBUCsjlSocket);          // 5:常熟均联OBU障碍物信息，高铁新城也用该接口
    receivingSocketList.push_back(spatTrafficLightSocket); // 6:高铁新城红绿灯信息
    receivingSocketList.push_back(percTrafficLightSocket); // 7:自车感知红绿灯信息
    receivingSocketList.push_back(clusterSocket);          // 8:自车感知障碍物聚类信息
    // prediction receiving socket From Vision20231106
    receivingSocketList.push_back(predictionFromVisionSocket);// 9:视觉感知获得的障碍物信息
    // zmq sending sockets/////////////////////////////////////////////////////

    void *publisherSocket = zmq_socket(context, ZMQ_PUB);
    int rc = zmq_bind(publisherSocket, "tcp://*:5010");
    if (rc < 0)
    {
        cout << "zmq_bind(pubSocket, tcp://*:5010)  rc = " << rc << "errno=" << errno << endl;
    }

    // add by syp 20221031
    // for pad UI app 交互通讯
    void *pubSocket2 = zmq_socket(context, ZMQ_PUB);
    rc = zmq_bind(pubSocket2, "tcp://*:3101");
    if (rc < 0)
    {
        cout << "zmq_bind(pubSocket, tcp://*:3101)  rc = " << rc << "errno=" << errno << endl;
    }
    // end of add by syp

    // for 高铁新城发送全局规划道路信息
    // for 感知发送下一个路口信息，用于获取红绿灯状态
    void *pubSocket3 = zmq_socket(context, ZMQ_PUB);
    rc = zmq_bind(pubSocket3, "tcp://*:5582");
    if (rc < 0)
    {
        cout << "zmq_bind(pubSocket3, tcp://*:5582)  rc = " << rc << "errno=" << errno << endl;
    }
    // end of add by syp

    std::vector<void *> sendingSocketList;
    sendingSocketList.push_back(publisherSocket);
    sendingSocketList.push_back(pubSocket2); // add  by syp 20221031 for pad UI app 交互通讯
    sendingSocketList.push_back(pubSocket3); // add  by syp 20220530 for 高铁新城发送全局规划道路信息

    // load road net
    std::string mapFilePathName;
    if (!loadMapFilePathName(FILE_NAME_YAML, mapFilePathName))
    {
        std::cout << "loadMapFilePathName failed: " << FILE_NAME_YAML << std::endl;
        return 0;
    }
    // std::cout << "before map initial: " << std::endl;
    RoadMap map(mapFilePathName);
    // std::cout << "test for map initial: " << map.roads[0].successorId[0] << std::endl;

    // load stop point
    std::vector<GaussRoadPoint> rawStopPoints;
    std::tuple<int32_t, int32_t, int32_t> stopPointRoadLanePointId;
    // loadStopPoints(STOP_POINT_FILE_NAME, rawStopPoints, stopPointRoadLanePointId, OFFSET_X, OFFSET_Y);
    loadStopPointsFromYaml(map, FILE_NAME_YAML, rawStopPoints, stopPointRoadLanePointId, OFFSET_X, OFFSET_Y); // 从yaml中读取文件

    // std::vector<std::tuple<int32_t, int32_t>> routingList;
    std::cout << "raw stop points: " << rawStopPoints.size() << "," << rawStopPoints[0].GaussX << "," << rawStopPoints[1].GaussX << "," << rawStopPoints[1].GaussY << "," << rawStopPoints[1].yaw << std::endl;

    // load traffic Light 20230712

    if (!map.trafficeLightMap_.InitMapFromFile(FILE_NAME_TRAFFICLIGHT))
    {
        std::cout << "load TrafficLightMap  failed: " << FILE_NAME_TRAFFICLIGHT << std::endl;
        return 0;
    }

    // TrafficLight  trafficeLight;
    // if(!map.trafficeLightMap_ .GetTrafficLightByRoadLane(53, 4 ,  0, 54 , 0 ,  trafficeLight))
    // {
    //     std::cout << "GetTrafficLightByRoadLane   failed: " <<  std::endl;
    // }

    //   std::cout << "trafficeLight id " <<  trafficeLight.intersetionID_ << "phase id  "  <<  trafficeLight.phaseID_<< ::endl;

    // return 0;

    Jobs jobs(receivingSocketList, sendingSocketList, map, rawStopPoints, stopPointRoadLanePointId);

    thread thread1(&Jobs::subscriber, &jobs);
    thread thread2(&Jobs::publisher, &jobs);
    thread thread3(&Jobs::processorGlobalPlanning, &jobs);
    thread thread4(&Jobs::processorLocalPlanning, &jobs);

    //  add by syp
    //  与服务器通讯的线程
    thread thread5(&Jobs::request, &jobs);
    // end of add by syp
    thread thread6(&Jobs::Draw, &jobs);

    thread::id threadID1 = thread1.get_id();
    thread::id threadID2 = thread2.get_id();
    thread::id threadID3 = thread3.get_id();
    thread::id threadID4 = thread4.get_id();
    // add by syp
    thread::id threadID5 = thread5.get_id();
    // end of add by syp
    thread::id threadID6 = thread6.get_id();

    // 线程优先级
    // printThreadPolicy_priority(thread1.native_handle());
    // printThreadPolicy_priority(thread2.native_handle());
    // printThreadPolicy_priority(thread3.native_handle());
    // printThreadPolicy_priority(thread4.native_handle());
    // printThreadPolicy_priority(thread5.native_handle());
    // printThreadPolicy_priority(thread6.native_handle());

    int max_priority = sched_get_priority_max(SCHED_FIFO);
    printf("SCHED_FIFO max priority: %d\n", max_priority);
    int min_priority = sched_get_priority_min(SCHED_FIFO);
    printf("SCHED_FIFO min priority: %d\n", min_priority);

    int policy = SCHED_FIFO; // 设置为先进先出
    int priority = 50;       // 设置优先级为 10
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setschedpolicy(&attr, policy);
    sched_param param;
    param.sched_priority = priority;
    pthread_attr_setschedparam(&attr, &param);
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    pthread_t thread_handle = thread1.native_handle(); // 接收线程
    pthread_setschedparam(thread_handle, policy, &param);

    pthread_attr_destroy(&attr);
    printThreadPolicy_priority(thread1.native_handle());

    // return 0;

    thread1.detach();
    thread2.detach();
    thread3.detach();
    thread4.detach();
    // add by syp
    thread5.detach();
    // end of add by syp
    thread6.detach();

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

        // // add by syp
        int thread5Status = pthread_kill(cvtThreadId2Long(threadID5), 0);
        checkThreadStatus(thread5Status, threadID5);
        // //  end of add by syp

        int thread6Status = pthread_kill(cvtThreadId2Long(threadID6), 0);
        checkThreadStatus(thread6Status, threadID6);

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    return 0;
}
