#include "include/V2x.h"
#include <iostream>
#include <thread>
#include <zmq.h>
#include <iomanip>
#include <unistd.h>
#include "include/Config.h"
#include "proto/replymsg.pb.h"
#include "proto/rsmsg.pb.h"

using namespace std;
// 定义时间
typedef std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds> miliClock_type;

const double eps = 0.000001;
SelfVehicle myVehicle;
// comore the last RoutingPoint and current RoutingPoint
bool compareCoordinates(double lat1, double lon1, double lat2, double lon2)
{
    if ((fabs(lat1 - lat2) > eps) || (fabs(lon1 - lon2) > eps))
    {
        return true;
    }
    return false;
}

// custome compare RoutingPointVec
bool compareRoutingPointVec(const Planning::RoutingPointVec lastVec, const Planning::RoutingPointVec currentVec)
{
    if (lastVec.routingpoints_size() != currentVec.routingpoints_size())
    {
        return true;
    }
    for (size_t i = 0; i < lastVec.routingpoints_size(); i++)
    {
        const ::Planning::RoutingPoint &lastRoutingPoint = lastVec.routingpoints(i);
        const ::Planning::RoutingPoint &currentRoutingPoint = currentVec.routingpoints(i);
        if (compareCoordinates(lastRoutingPoint.latitude(), lastRoutingPoint.longtitude(),
                               currentRoutingPoint.latitude(), currentRoutingPoint.longtitude()))
        {
            return true;
        }
    }
    return false;
}
// load route info from txt file, using to test
void loadRouteFromTxt(const std::string &file_path, std::vector<Ivics::Vector3D> *points)
{
    std::ifstream ifs;
    std::string line;
    ifs.open(file_path);
    if (ifs.fail())
    {
        std::cerr << "--------Route data file read failed!" << std::endl;
        return;
    }

    while (getline(ifs, line))
    {
        std::vector<double> nums;
        int j = 0;
        for (int i = 0; i < line.length(); i++)
        {
            if (isblank(line[i]))
            {
                auto word = line.substr(j, i - j);
                auto num = stod(word);
                nums.push_back(num);
                j = i + 1;
            }
        }
        Ivics::Vector3D point;

        coorconv::WGS84Corr wgs;
        coorconv::UTMCoor utm;
        wgs.lon = nums[1]; // 经度
        wgs.lat = nums[2]; // 纬度
        coorconv::LatLonToUTMXY(wgs, utm);
        // 此处天翼交通带偏移
        point.x = utm.x - 275000.02;
        point.y = utm.y - 3479281.5;
        point.z = 0;
        points->push_back(point);
    }
}

/**
 * 通过read file方式获取车辆位置信息
 */
void getFilePosFromVehicle(std::string address, int vehiclePosWriteFlag, std::string vposFilePath)
{
    std::vector<Ivics::Vector3D> route1_points;
    route1_points = {{-311.00, -142.95, 0}, {-261.94, -1.55, 0}, {-243.72, 4.35, 0}, {-201.45, -19.96, 0}, {-134.43, -56.79, 0}, {-106.68, -72.52, 0}, {-80.81, -87.19, 0}, {-57.28, -108.00, 0}, {-52.50, -135.16, 0}, {-77.59, -203.69, 0}, {-108.41, -274.87, 0}, {-133.74, -333.35, 0}, {-148.35, -362.15, 0}, {-198.39, -348.88, 0}, {-263.82, -322.08, 0}, {-318.12, -299.91, 0}, {-360.47, -281.40, 0}, {-316.56, -157.85, 0}};
    loadRouteFromTxt("/home/develop/project/V2xServer/testData/track.txt", &route1_points);

    while (true)
    {
        auto pBSM = std::make_shared<Ivics::BSM>();
        for (int i = 0; i < route1_points.size(); i++)
        {
            pBSM->pos.x = route1_points[i].x;
            pBSM->pos.y = route1_points[i].y;
            // std::cout << "---------->send position , x:" << route1_points[i].x << "y: " << route1_points[i].y << std::endl;
            Ivics::sendBSM(pBSM);
            usleep(30000);
        }
    }
}

/**
 * 通过zmq pub-sub方式获取车辆位置信息
 */
void getPosFromVehicle(std::string address, int vehiclePosWriteFlag, std::string vposFilePath)
{
    // sub vehile pos data
    void *context = zmq_ctx_new();
    void *subscriber = zmq_socket(context, ZMQ_SUB);
    int rc = zmq_connect(subscriber, address.c_str());
    rc = zmq_setsockopt(subscriber, ZMQ_SUBSCRIBE, "", 0);
    int timeout = 5 * 1000; // 5秒钟超时限制
    pc::Imu imu;
    try
    {
        // 设置非阻塞模式
        int64_t rec_timeout = timeout;
        zmq_setsockopt(subscriber, ZMQ_RCVTIMEO, &rec_timeout, sizeof(rec_timeout));

        while (true)
        {
            // 判断是否可以sub到数据，超时时间设置为一分钟
            zmq::pollitem_t items[] = {{static_cast<void *>(subscriber), 0, ZMQ_POLLIN, 0}};
            zmq::poll(items, 1, timeout);
            // 有可读消息
            if (items[0].revents & ZMQ_POLLIN)
            {
                auto pBSM = std::make_shared<Ivics::BSM>();
                char msg[81920] = {0};
                int size = zmq_recv(subscriber, msg, 81920, 0);

                if (size != -1)
                {
                    if (!imu.ParseFromArray(msg, size))
                    {
                        std::cerr << "\033[1;31m ????????????【Error】: vehicle pos data  parse failed!\033[0m" << std::endl;
                    }
                    else
                    {
                        // vehicle up lat and lon
                        double lat = imu.latitude();
                        double lon = imu.longitude();
                        double gaussX = imu.gaussx();
                        double gaussY = imu.gaussy();
                        double yaw = imu.yaw();
                        double time = imu.time();
                        // myVehicle set data
                        myVehicle.gaussX = gaussX;
                        myVehicle.gaussY = gaussY;
                        myVehicle.yaw = yaw;
                        myVehicle.lat = lat;
                        myVehicle.lon = lon;
                        // std::cout<<"********getPosFromVehicle************"<<std::endl;
                        // std::cout<<"gaussX::"<<gaussX<<"\t";
                        // std::cout<<"gaussY::"<<gaussY<<"\t";
                        // std::cout<<"yaw::"<<yaw<<"\n";

                        // transfor utm
                        coorconv::WGS84Corr wgs;
                        coorconv::UTMCoor utm;
                        wgs.lon = lon; // 经度
                        wgs.lat = lat; // 纬度

                        coorconv::LatLonToUTMXY(wgs, utm);
                        // 此处天翼交通带偏移
                        pBSM->pos.x = utm.x - 275000.02;
                        pBSM->pos.y = utm.y - 3479281.5;
                        // 发送车辆位置数据
                        Ivics::sendBSM(pBSM);

                        // write pos data to file
                        if (vehiclePosWriteFlag == 1)
                        {
                            std::ofstream locFile;
                            locFile.open(vposFilePath, std::ios::out | std::ios::app);
                            locFile << "**********************************************" << std::endl;
                            locFile << std::setprecision(10) << "lat:" << lat << "\t"
                                    << "lon:" << lon << "\t"
                                    << "utm x:" << pBSM->pos.x << "\t"
                                    << "utm y:" << pBSM->pos.y << "\n";
                            locFile.close();
                        }
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(30));
                }
            }
            else // 没有可读消息
            {
                if (zmq_errno() == EAGAIN)
                {

                    std::cerr << "\033[1;31m????????????【Error】  get pos from vehicle no message received!\033[0m" << std::endl;
                    // 取消订阅
                    Ivics::unsubscribeSSM();
                    Ivics::unsubscribeSPAT();
                }
            }
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "\033[1;31m????????????【Error】  sub vehicle pos data happen exception::" << e.what() << "\033[0m" << std::endl;
    }
    zmq_close(subscriber);
    zmq_ctx_destroy(context);
}

/**
 * 通过zmq pub-sub方式获取车辆全局规划信息
 */
void getGlobalPlanningFromVehicle(std::string address, int writeFlag, std::string filePath)
{
    void *context = zmq_ctx_new();
    void *subscriber = zmq_socket(context, ZMQ_SUB);
    int rc = zmq_connect(subscriber, address.c_str());
    rc = zmq_setsockopt(subscriber, ZMQ_SUBSCRIBE, "", 0);

    Planning::RoutingPointVec lastRoutingPointVec;
    Planning::RoutingPointVec routingPointVec;
    V2x mV2x;
    int timeout = 5 * 1000;
    try
    {
        // 设置非阻塞模式
        int64_t rec_timeout = timeout;
        zmq_setsockopt(subscriber, ZMQ_RCVTIMEO, &rec_timeout, sizeof(rec_timeout));
        while (true)
        {
            // 判断是否可以sub到数据，超时时间设置为5秒钟
            zmq::pollitem_t items[] = {{static_cast<void *>(subscriber), 0, ZMQ_POLLIN, 0}};
            zmq::poll(items, 1, timeout);
            // 有可读消息
            if (items[0].revents & ZMQ_POLLIN)
            {
                std::vector<Ivics::Vector3D> route1_points;
                // 动态分配缓冲区
                char *msg = nullptr;
                zmq_msg_t msgObj;
                int size = zmq_msg_init(&msgObj);

                // char msg[819200] = {0};
                //  int size = zmq_recv(subscriber, msg, 819200, 0);
                bool sendFlag = false;
                if (size != -1)
                {
                    size = zmq_msg_recv(&msgObj, subscriber, 0);
                    if (size != -1)
                    {
                        msg = static_cast<char *>(zmq_msg_data(&msgObj));
                        routingPointVec.Clear();
                        if (!routingPointVec.ParseFromArray(msg, size))
                        {
                            std::cerr << "\033[1;31m????????????【Error】:   getGlobalPlanningFromVehicle Failed to parse the received message!\033[0m" << std::endl;
                        }
                        else
                        {

                            // last has element ,then compare the element
                            if (lastRoutingPointVec.routingpoints_size() > 0)
                            {
                                bool result = compareRoutingPointVec(lastRoutingPointVec, routingPointVec);
                                // compare planning road changed
                                if (result)
                                {
                                    std::cout << "\033[1;34m "
                                              << "*****【info】 <<<<<<<<<<<<<<全局规划路径发生变化>>>>>>>>>>>>>>>>>\033[0m " << std::endl;

                                    for (size_t i = 0; i < routingPointVec.routingpoints_size(); i++)
                                    {
                                        const ::Planning::RoutingPoint &routingPoint = routingPointVec.routingpoints(i);
                                        double longtitude = routingPoint.longtitude();
                                        double latitude = routingPoint.latitude();
                                        //  transfor utm
                                        coorconv::WGS84Corr wgs;
                                        coorconv::UTMCoor utm;
                                        wgs.lon = longtitude; // 经度
                                        wgs.lat = latitude;   // 纬度
                                        coorconv::LatLonToUTMXY(wgs, utm);
                                        // 此处天翼交通带偏移
                                        double px = utm.x - 275000.02;
                                        double py = utm.y - 3479281.5;
                                        Ivics::Vector3D vector3D;
                                        vector3D.x = px;
                                        vector3D.y = py;
                                        vector3D.z = 0;
                                        route1_points.emplace_back(vector3D);
                                    }
                                    // update lastRoutingPointVec
                                    lastRoutingPointVec = routingPointVec;
                                    // update sendFlag status
                                    sendFlag = true;
                                }
                            }
                            // 首次进入
                            else
                            {
                                for (size_t i = 0; i < routingPointVec.routingpoints_size(); i++)
                                {
                                    const ::Planning::RoutingPoint &routingPoint = routingPointVec.routingpoints(i);
                                    double longtitude = routingPoint.longtitude();
                                    double latitude = routingPoint.latitude();
                                    // double height =  routingPoint.height();
                                    //  transfor utm
                                    coorconv::WGS84Corr wgs;
                                    coorconv::UTMCoor utm;
                                    wgs.lon = longtitude; // 经度
                                    wgs.lat = latitude;   // 纬度
                                    coorconv::LatLonToUTMXY(wgs, utm);
                                    // 此处天翼交通带偏移
                                    double px = utm.x - 275000.02;
                                    double py = utm.y - 3479281.5;
                                    Ivics::Vector3D vector3D;
                                    vector3D.x = px;
                                    vector3D.y = py;
                                    vector3D.z = 0;
                                    route1_points.emplace_back(vector3D);
                                }
                                // update lastRoutingPointVec
                                lastRoutingPointVec = routingPointVec;
                                sendFlag = true;
                            }
                        }
                    }
                    else
                    {
                        std::cerr << "\033[1;31m????????????【Error】:   Get global planning from vehicle  zmq_msg_recv return size: -1!\033[0m" << std::endl;
                    }

                    if (sendFlag)
                    {
                        // std::cout << "***********the global planning road changed,start send data to tyjt*************" << std::endl;
                        Ivics::sendRoutes(route1_points);
                        // write data to file
                        if (writeFlag == 1)
                        {
                            bool result = mV2x.writeGlobalPlanning2File(&routingPointVec, filePath);
                            if (result)
                            {
                                // std::cout << "\033[1;32m "
                                //           << "【Success】:  Write gloabl planning  data to  file!!\033[0m " << std::endl;
                            }
                            else
                            {
                                std::cerr << "\033[1;31m????????????【Error】:   Failed Write gloabl planning  data to  file !\033[0m" << std::endl;
                            }
                        }
                    }
                }
                else
                {
                    std::cerr << "\033[1;31m????????????【Error】:   Get global planning from vehicle zmq_msg_init return size :-1 !\033[0m" << std::endl;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(40));
            }
            else // 未读取到消息
            {
                if (zmq_errno() == EAGAIN)
                {
                    std::cerr << "\033[1;31m????????????【Error】:   Get global planning from vehicle  no message received !\033[0m" << std::endl;
                    // 取消订阅
                    Ivics::unsubscribeSSM();
                    Ivics::unsubscribeSPAT();
                }
            }
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "\033[1;31m????????????【Error】:   Get global planning from vehicle   happen exception::" << e.what() << "\033[0m" << std::endl;
    }
    zmq_close(subscriber);
    zmq_ctx_destroy(context);
}

/**
 * 通过zmq req-rep方式发送障碍物信息
 */
void requestForServerZMQ(ParameSct myStruct)
{

    void *context = zmq_ctx_new();
    if (context == nullptr)
    {
        std::cerr << "\033[1;31m????????????【Error】:   equestForServerZMQ context erro,context is null !\033[0m" << std::endl;
        return;
    }

    while (true) // 包含创建socket的大循环
    {
        // cout << "\033[31m "<<"Jobs::request() ---------------------------------while()" <<"\033[0m "<<endl;
        //////////////////////////////////////////////////////
        // void * serverReqSocket = nullptr;
        void *serverReqSocket = zmq_socket(context, ZMQ_REQ);
        if (serverReqSocket == nullptr)
        {
            std::cerr << " ?????????????【Error】 serverReqSocket  error " << endl;
            return;
        }

        int rc = zmq_connect(serverReqSocket, myStruct.serverReqAddress.c_str()); ////166.111.50.39 5501

        if (rc != 0)
        {
            std::cerr << "\033[1;31m????????????【Error】:   connect serverReqSocket error = !" << errno << "error string =" << zmq_strerror(errno) << "\033[0m" << std::endl;
            return;
        }

        //  Configure socket to not wait at close time,快速关闭socket
        int linger = 0;
        zmq_setsockopt(serverReqSocket, ZMQ_LINGER, &linger, sizeof(linger));

        while (true) // 数据发送和接收的循序
        {
            auto start = std::chrono::steady_clock::now();
            // =======do your works here======

            // 有数据发送数据//////////////////////////////////////////////////////
            ObjectsVec msgvecTemp; // 发送给服务器的消息
            bool bRenewMsgvecTemp;

            // 从交换空间中获取数据
            myStruct.ptrMutexMsgvec->lock();
            std::cout << "\033[34m "
                      << "*****【info】:(*myStruct.ptrBRenewMsgvec)" << (*myStruct.ptrBRenewMsgvec) << "myStruct.ptrMsgvec->objmsg_size()" << myStruct.ptrMsgvec->objmsg_size() << "\033[0m " << std::endl;

            bRenewMsgvecTemp = (*myStruct.ptrBRenewMsgvec);
            if (bRenewMsgvecTemp)
            {
                msgvecTemp = *myStruct.ptrMsgvec;
            }

            myStruct.ptrMutexMsgvec->unlock();

            if (!bRenewMsgvecTemp) // 没有消息的更新
            {
                usleep(50000);
                continue;
            }

            //         fmsg->set_status(3);

            size_t size = msgvecTemp.ByteSize();
            void *buffer = malloc(size);
            if (!msgvecTemp.SerializeToArray(buffer, size))
            {
                std::cerr << "\033[1;31m????????????【Error】:  requestForServerZMQ   msgvecTemp failed to write msg!\033[0m" << std::endl;
                break;
            }
            zmq_msg_t req;
            if (0 != zmq_msg_init_size(&req, size))
            {
                std::cerr << "\033[1;31m????????????【Error】:  requestForServerZMQ zmq_msg_init failed!\033[0m" << std::endl;
                break;
            }
            memcpy(zmq_msg_data(&req), buffer, size);
            /*  int timeout = 0;
             zmq_setsockopt(sc, ZMQ_SNDTIMEO, &timeout, sizeof(timeout)); */
            int nSend = zmq_msg_send(&req, serverReqSocket, 0);

            if ((int)size != nSend)
            {
                // zmq_msg_close(&req);
                std::cerr << "\033[1;31m????????????【Error】:  send  msgvecTemp faliled... size =" << size << ", length =" << nSend << "\033[0m" << std::endl;
                break;
            }
            else
            {
                std::cout << "\033[1;34m "
                          << "*****【info】:  serverReqSocket send successed size  =" << size << "\033[0m " << std::endl;
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
                std::cout << "\033[1;34m "
                          << "*****【info】 items[0].revents & ZMQ_POLLIN Receive data length=" << len << "\033[0m " << std::endl;

                if (len != -1) // 正常接收数据
                {
                    // 接收数据，但是这里不解析数据了
                    void *str_recv = malloc(len);
                    memcpy(str_recv, zmq_msg_data(&reply), len);

                    if (NULL != str_recv)
                        free(str_recv);
                    zmq_msg_close(&reply);

                    //=======end of your works======
                    auto end = std::chrono::steady_clock::now();
                    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

                    // std::this_thread::sleep_for(std::chrono::milliseconds(this->recvRate - duration.count()));
                    std::this_thread::sleep_for(std::chrono::milliseconds(40 - duration.count()));
                }    // if (len != -1)
                else // 接收数据不正常
                {
                    std::cerr << "\033[1;31m????????????【Error】:   malformed reply from server!!!"
                              << "\033[0m" << std::endl;
                    zmq_msg_close(&reply);
                    zmq_close(serverReqSocket);

                    break;
                }
            }    // if (items[0].revents & ZMQ_POLLIN)
            else // 超时未接收到数据
            {
                std::cerr << "?????????????【Error】 if (items[0].revents & ZMQ_POLLIN)   errno=" << errno << " errstr=" << zmq_strerror(errno)<< "\033[0m" << std::endl;
                zmq_close(serverReqSocket);
                sleep(1);
                break;
            }

        } // while (bSocketOK)//数据发送和接收的循序

    } // while

    // 6.关闭套接字、销毁上下文

    zmq_ctx_destroy(context);

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
}
int main()
{
    // set utf
    setlocale(LC_CTYPE, "zh_CN.utf8");
    void *context = zmq_ctx_new();
    // 1 read config info from setting.conf
    std::string posSubAddress;                          // obu get vehicle up pos by zmp address
    std::string planningSubAddress;                     // obu get vehicle up planning  by zmp address
    std::string ssmPubAddress;                          // obu pub ssm data to vehicle address
    std::string spartPubAddress;                        // obu pub spart data to vehicle address
    const char ConfigFile[] = "../config/setting.conf"; // file path
    Config configSettings(ConfigFile);
    // 1-1 get all log write file flag
    int ssmWriteFlag = configSettings.Read("ssmWriteFlag", ssmWriteFlag);
    int spartWriteFlag = configSettings.Read("spartWriteFlag", spartWriteFlag);
    int vehiclePosWriteFlag = configSettings.Read("vehiclePosWriteFlag", vehiclePosWriteFlag);
    int planningWriteFlag = configSettings.Read("planningWriteFlag", planningWriteFlag);
    // 1-2 get all log write file path
    std::string ssmFilePath = configSettings.Read("ssmFilePath", ssmFilePath);                   // ssm write file path
    std::string spartFilePath = configSettings.Read("spartFilePath", spartFilePath);             // spart write file path
    std::string vposFilePath = configSettings.Read("vposFilePath", vposFilePath);                // vehicle pos write file path
    std::string vplanningFilePath = configSettings.Read("vplanningFilePath", vplanningFilePath); // vehicle planning write file path

    // 1-3 zmq pub and sub address
    posSubAddress = configSettings.Read("posSubAddress", posSubAddress);
    planningSubAddress = configSettings.Read("planningSubAddress", planningSubAddress);
    ssmPubAddress = configSettings.Read("ssmPubAddress", ssmPubAddress);
    spartPubAddress = configSettings.Read("spartPubAddress", spartPubAddress);

    // 1-4 bind zmq pub to ssm
    void *pubssm = zmq_socket(context, ZMQ_PUB);
    // std::cout << "ssmPubAddress:" << ssmPubAddress << std::endl;
    int rc = zmq_bind(pubssm, ssmPubAddress.c_str());
    if (rc < 0)
    {
        std::cerr << "\033[1;31m????????????【Error】: ssm pub bind failed!\033[0m" << std::endl;
        exit(-1); // bind fail exit
    }
    // 1-5 bind zmq pub to spart
    void *pubspart = zmq_socket(context, ZMQ_PUB);
    int rc2 = zmq_bind(pubspart, spartPubAddress.c_str());
    if (rc2 < 0)
    {
        std::cerr << "\033[1;31m????????????【Error】: spart pub bind failed!\033[0m" << std::endl;
        exit(-1); // bind fail exit
    }

    // 1-6 init struct as parame
    // 1-6 init struct as parame
    // 20230723 by shyp增加与服务器进行数据交换的变量
    std::string serverReqAddress; // server ip & port
    serverReqAddress = configSettings.Read("serverReqAddress", serverReqAddress);
    // std::cout <<  "serverReqAddress"  <<serverReqAddress <<std::endl;
    ObjectsVec msgvec;         // 发送给服务器的消息
    bool bRenewMsgvec = false; // 发送给服务器消息的更新 状态
    std::mutex mutexMsgvec;    // 发送给服务器消息的锁

    ParameSct myStruct = {ssmWriteFlag, spartWriteFlag, ssmFilePath, spartFilePath, pubssm, pubspart, serverReqAddress, &msgvec, &bRenewMsgvec, &mutexMsgvec, &myVehicle};

    // 2  define callback function
    V2x cc;
    // define SSMCallback callback fuction and send struct parame
    std::function<void(const Ivics::SSM &)> pSSMcb = std::bind(&V2x::SSMCallback, cc, std::placeholders::_1, myStruct);
    // define SPATCallback callback fuction and send struct parame
    std::function<void(const Ivics::SPAT &)> pSPATcb = std::bind(&V2x::SPATCallback, cc, std::placeholders::_1, myStruct);
    // 3 init check login
    bool res = cc.init();
    if (!res)
    {
        std::cerr << "\033[1;31m????????????【Error】: Login check fail!\033[0m" << std::endl;
        exit(-1);
    }

    // 4 Obtain vehicle global planning information
    std::thread thread1(getGlobalPlanningFromVehicle, planningSubAddress, planningWriteFlag, vplanningFilePath);
    // std::cout<<"****************************"<<std::endl;
    // std::cout<<"===gaussX"<<posSct.gaussX<<"\t";
    // std::cout<<"===gaussX"<<posSct.gaussY<<"\t";
    // std::cout<<"===yaw"<<posSct.yaw<<"\n";
    // Subscription perception information
    Ivics::subscribeSSM(pSSMcb);
    // Subscribe to traffic light information
    Ivics::subscribeSPAT(pSPATcb);
    // Obtain real-time vehicle reporting perception information
    std::thread thread2(getPosFromVehicle, posSubAddress, vehiclePosWriteFlag, vposFilePath);

    // 20230723 by shyp增加一个与服务器交互的zmq
    // thread thread3(requestForServerZMQ, myStruct);

    // 5 thread join

    thread1.join();
    thread2.join();

    // thread3.join();

    // 6 closing and destroying resources
    Ivics::close();
    zmq_close(pubssm);
    zmq_close(pubspart);
    zmq_ctx_destroy(context);
    return 0;
}
