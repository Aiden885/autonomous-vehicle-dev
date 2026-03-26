/*==========================================


g++ -o stdThreadDemo stdThreadDemo.cpp -lpthread -lzmq

============================================*/

#include <chrono>
#include <iostream>
#include <thread>
#include <future>
#include <signal.h>
#include <string>
#include <sstream>
#include <map>
#include <zmq.h>
#include <math.h>
#include "time.h"

#include "steeringController.h"
#include "imu.pb.h"
#include "control.pb.h"
#include <yaml-cpp/yaml.h>
#include "defineColor.h"
#include "localization_MapAnalysis.h"
#include <iomanip>

using namespace std;
using namespace controlData;

// add by shyp 20220830 formular from Fusion program
void gaussConvert(double longitude1, double latitude1, double &dNorth_X, double &dEast_Y)
{

    double a = 6378137.0;

    double e2 = 0.0066943799013;

    double latitude2Rad = (M_PI / 180.0) * latitude1;

    int beltNo = int((longitude1 + 1.5) / 3.0);
    int L = beltNo * 3;
    double l0 = longitude1 - L;
    double tsin = sin(latitude2Rad);
    double tcos = cos(latitude2Rad);
    double t = tan(latitude2Rad);
    double m = (M_PI / 180.0) * l0 * tcos;
    double et2 = e2 * pow(tcos, 2);
    double et3 = e2 * pow(tsin, 2);
    double X = 111132.9558 * latitude1 - 16038.6496 * sin(2 * latitude2Rad) + 16.8607 * sin(4 * latitude2Rad) - 0.0220 * sin(6 * latitude2Rad);
    double N = a / sqrt(1 - et3);

    dNorth_X = X + N * t * (0.5 * pow(m, 2) + (5.0 - pow(t, 2) + 9.0 * et2 + 4 * pow(et2, 2)) * pow(m, 4) / 24.0 + (61.0 - 58.0 * pow(t, 2) + pow(t, 4)) * pow(m, 6) / 720.0);
    dEast_Y = 500000 + N * (m + (1.0 - pow(t, 2) + et2) * pow(m, 3) / 6.0 + (5.0 - 18.0 * pow(t, 2) + pow(t, 4) + 14.0 * et2 - 58.0 * et2 * pow(t, 2)) * pow(m, 5) / 120.0);
    //   std::cout << BOLDRED << "x1: " << x1 << std::endl;
    //   std::cout << BOLDRED << "y1: " << y1 << std::endl;
}

struct DGPS
{
    // double utc_second;
    // double latitude;
    // double longitude;
    // double heading;
    // double X;
    // double Y;
    // int satNum;
    // int status_main; //涓荤�??
    // int status_vice; //浠庣�??
    // double speed;// add by syp
    double longitude;
    double latitude;
    double gaussX;
    double gaussY;
    int psValid;
    double time;
    double velocity;
    double yaw;
};

class Vehicle
{

public:
    DGPS gps;
    double dBaseLength;
    double dTargetSteeringAngle;
    double dTargetSpeed;

    Vehicle(){};
    ~Vehicle(){};

    //���ݵ�ǰλ�˺�����,������һλ�õ�λ��
    void VehicleDynamic(double &x, double &y, double &theta, double D, double delta)
    {
        // x_dot = v_x * cos(theta); x_dot * t = v_x * t * cos(theta),�ڲ���ʱ��t��,����x = x + v_x * t * cos(theta)������v_x * t=D
        x = x + D * cos(theta);                       //�˶�ѧ��ʽ
        y = y + D * sin(theta);                       // �˶�ѧ��ʽ
        theta = theta + D / dBaseLength * tan(delta); // L�����?,�����?,theta_dot=v/R,R=L/tan(delta)
        theta = mod2pi(theta);
    }

    inline double mod2pi(double x)
    {
        double v = fmod(x, 2 * M_PI);
        if (v < -M_PI)
            v += 2 * M_PI;
        else if (v > M_PI)
            v -= 2 * M_PI;
        return v;
    }
};

class ThreadJobs
{
private:
    int sendRate, recvRate;

    void *socketSub;
    map<string, void *> pubMsgList;

    SteeringController steeringController;
    Vehicle vehicleCurrent;

    int count;//接收数据的计�?
    bool loadStopYamlPathName(const std::string yamlFileName, std::string & mapFilePathName)//从configl文件加载stop.yaml文件
    {
        try
        {
            YAML::LoadFile(yamlFileName); // this is rvalue
        }
        catch (YAML::BadFile &bfe)
        {
            std::cout << BOLDRED << "yaml file is not exist!" << std::endl;
            std::cout << "    " << bfe.what() << RESET << std::endl;
            return false;
        }

        YAML::Node yaml_node = YAML::LoadFile(yamlFileName);

        try
        {
            yaml_node["stratpointFile"].as<std::string>();                 // this is rvalue
        }
        catch (YAML::TypedBadConversion<std::string> &tbce_double)
        {
            std::cout << BOLDRED << "stratpointFile is not exist!" << std::endl;
            std::cout << "    " << tbce_double.what() << RESET << std::endl;
            return false;
        }

        mapFilePathName = yaml_node["stratpointFile"].as<std::string>();
        std::cout<<"stratpointFile"  <<mapFilePathName<<std::endl;
        if(mapFilePathName.empty())
            return false;

        return true;
    }    

bool loadMapFilePathName(const std::string yamlFileName, std::string & mapFilePathName)//从yaml文件加载地图文件全路径文件名
{
     try
    {
        YAML::LoadFile(yamlFileName); // this is rvalue
    }
    catch (YAML::BadFile &bfe)
    {
        std::cout << BOLDRED << "yaml file is not exist!" << std::endl;
        std::cout << "    " << bfe.what() << RESET << std::endl;
        return false;
    }

    YAML::Node yaml_node = YAML::LoadFile(yamlFileName);

    try
    {
        yaml_node["roadMapPathName"].as<std::string>();                 // this is rvalue
    }
    catch (YAML::TypedBadConversion<std::string> &tbce_double)
    {
        std::cout << BOLDRED << "roadMapPathName is not exist!" << std::endl;
        std::cout << "    " << tbce_double.what() << RESET << std::endl;
        return false;
    }

    mapFilePathName = yaml_node["roadMapPathName"].as<std::string>();
    std::cout<<"mapFilePathName"  <<mapFilePathName<<std::endl;
    if(mapFilePathName.empty())
        return false;

    return true;
}    

// 新增从yaml文件读取起点位置
//从planning中拷贝过来的，原来是读取stop点。现在是读取起点。相关参数名称不做修改了
void loadStopPointsFromYaml(const RoadMap &map_, const std::string &fileName, std::vector<GaussRoadPoint> &rawStartPoints,
                            std::tuple<int32_t, int32_t, int32_t> &startPointRoadLanePointId, double offsetX, double offsetY, double & meridianLine)
{

    try
    {
        YAML::LoadFile(fileName); // this is rvalue
    }
    catch (YAML::BadFile &bfe)
    {
        std::cout << BOLDRED << "yaml file is not exist!" << std::endl;
        std::cout << "    " << bfe.what() << RESET << std::endl;
        exit(0);
    }

    YAML::Node yaml_node = YAML::LoadFile(fileName);

    try
    {
        yaml_node["roadIdFromStart"].as<int>();                 // this is rvalue
        yaml_node["laneIdFromStart"].as<int>();                 // this is rvalue
        yaml_node["approximateLocationFromStart"].as<double>(); // this is rvalue
        yaml_node["meridianLine"].as<double>(); // this is rvalue
    }
    catch (YAML::TypedBadConversion<int> &tbce_int)
    {
        std::cout << BOLDRED << "roadIdFromStart or laneIdFromStart is not exist!" << std::endl;
        std::cout << "    " << tbce_int.what() << RESET << std::endl;
        exit(0);
    }
    catch (YAML::TypedBadConversion<double> &tbce_double)
    {
        std::cout << BOLDRED << "approximateLocationFromStart is not exist!" << std::endl;
        std::cout << "    " << tbce_double.what() << RESET << std::endl;
        exit(0);
    }

    int road_id = yaml_node["roadIdFromStart"].as<int>();
    int lane_id = yaml_node["laneIdFromStart"].as<int>();
    auto approximate_location = yaml_node["approximateLocationFromStart"].as<double>();
    meridianLine = yaml_node["meridianLine"].as<double>();

    if (approximate_location > 100.0)
        approximate_location = 100.0;
    if (approximate_location < 0.0)
        approximate_location = 0.0;

    bool find_flag = false;
    for (const auto &road_iter : map_.roads)
    {
        if (road_iter.id == road_id)
        {
            for (const auto &lane_iter : road_iter.lanes)
            {
                if (lane_iter.id == lane_id)
                {
                    auto target_index = static_cast<unsigned int>(
                        static_cast<unsigned int>(lane_iter.gaussRoadPoints.size()) * approximate_location);
                    std::cout << "this is the \'real\' index: " << target_index << std::endl;

                    if (target_index > 0)
                        target_index -= 1; // make 'zeroth' equals to first.
                    std::get<2>(startPointRoadLanePointId) =
                        static_cast<int>(target_index) + 1; // start from one instead of zero
                    std::get<0>(startPointRoadLanePointId) = road_id;
                    std::get<1>(startPointRoadLanePointId) = lane_id;

                    for (int i = 0; i <(int) lane_iter.gaussRoadPoints.size(); i++)
                    {
                        if (i == target_index)
                        {
                            std::cout << "stopPoint's gaussX: " << std::fixed << std::setprecision(3)
                                      << lane_iter.gaussRoadPoints.at(i).GaussX
                                      << " stopPoint's gaussY: "
                                      << lane_iter.gaussRoadPoints.at(i).GaussY
                                      << " stopPoint's yaw: "
                                      << lane_iter.gaussRoadPoints.at(i).yaw << std::endl;
                            std::cout.unsetf(std::ios::fixed);
                            rawStartPoints.clear();
                            GaussRoadPoint roadPoint{};
                            rawStartPoints.push_back(roadPoint);

                            roadPoint.GaussX = lane_iter.gaussRoadPoints.at(i).GaussX + offsetX;
                            roadPoint.GaussY = lane_iter.gaussRoadPoints.at(i).GaussY + offsetY;
                            roadPoint.yaw = lane_iter.gaussRoadPoints.at(i).yaw;
                            rawStartPoints.emplace_back(roadPoint);
                            find_flag = true;
                            break;
                        }
                    }
                    break;
                }
            }
            break;
        }
    }

    if (!find_flag)
    {
        std::cout << BOLDRED << "sorry, the value of roadIdFromStart or laneIdFromStart is illegal!"
                  << RESET << std::endl;
        exit(0);
    }
}

    
public:
    ThreadJobs() : steeringController(SteeringController()), sendRate(100), recvRate(40)
    {
        string pubName = "imgPub";
        void *contextPub = zmq_ctx_new();
        void *socketPublish = zmq_socket(contextPub, ZMQ_PUB);
       // int ret = zmq_bind(socketPublish, "tcp://127.0.0.1:5003");
        int ret = zmq_bind(socketPublish, "tcp://*:5003");
        this->pubMsgList[pubName] = socketPublish;

        string pubNameFromCOMOS = "comsPub";
        void *contextPubFromCOMOS = zmq_ctx_new();
        void *socketPublishFromCOMOS = zmq_socket(contextPubFromCOMOS, ZMQ_PUB);
        ret = zmq_bind(socketPublishFromCOMOS, "tcp://127.0.0.1:3151");
        this->pubMsgList[pubNameFromCOMOS] = socketPublishFromCOMOS;

        void *contextSub = zmq_ctx_new();
        this->socketSub = zmq_socket(contextSub, ZMQ_SUB);
        ret = zmq_connect(this->socketSub, "tcp://127.0.0.1:3171");
        ret = zmq_setsockopt(this->socketSub, ZMQ_SUBSCRIBE, "", 0);

        //从配置文件真是的配置文件路径，为了与planning公用同一个配置文件
        std::string StopFilePathName;
        if(! loadStopYamlPathName("../config.yaml",  StopFilePathName))
        {
                std::cout << "loadMapFilePathName failed: " << "../config.yaml"<< std::endl;
               
        }

        //获取真实的地图文件名
        std::string mapFilePathName;
        if(! loadMapFilePathName(StopFilePathName,  mapFilePathName))
        {
            std::cout << "loadMapFilePathName failed: " << StopFilePathName<< std::endl;
              
        }
   
    RoadMap map(mapFilePathName);
    //std::cout << "test for map initial: " << map.roads[0].successorId[0] << std::endl;
    //获取起点位置
     // load stop point
    std::vector<GaussRoadPoint> rawStartPoints;
    std::tuple<int32_t, int32_t, int32_t> stopPointRoadLanePointId;
    // loadStopPoints(STOP_POINT_FILE_NAME, rawStopPoints, stopPointRoadLanePointId, OFFSET_X, OFFSET_Y);
    double meridianLine;
    loadStopPointsFromYaml(map,StopFilePathName, rawStartPoints, stopPointRoadLanePointId, 0, 0,meridianLine); // 从yaml中读取文件

    //std::vector<std::tuple<int32_t, int32_t>> routingList;
    std::cout << "raw stop points: " << setprecision(10)<<rawStartPoints.size() << "," << rawStartPoints[0].GaussX << "," << rawStartPoints[1].GaussX << "," << rawStartPoints[1].GaussY << "," << rawStartPoints[1].yaw
    <<"meridianLine" <<meridianLine << std::endl;

    ZtGeographyCoordinateTransform tf;
    vehicleCurrent.gps.gaussX = rawStartPoints[1].GaussX;
    vehicleCurrent.gps.gaussY = rawStartPoints[1].GaussY;
    vehicleCurrent.gps.yaw = rawStartPoints[1].yaw * M_PI / 180.; // 10.15*M_PI/180.;
  
        tf.meridianLine = meridianLine;
        tf.XY2BL( this->vehicleCurrent.gps.gaussY, this->vehicleCurrent.gps.gaussX,this->vehicleCurrent.gps.latitude, this->vehicleCurrent.gps.longitude);
        vehicleCurrent.gps.psValid = 4;
        // //计算时间，当天秒计数，double
        // auto now = std::chrono::system_clock::now();
        // //通过不同精度获取相差的毫秒数
        // uint64_t dis_millseconds = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count()
        // - std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count() * 1000;
        // time_t tt = std::chrono::system_clock::to_time_t(now);
        // auto time_tm = localtime(&tt);
        //  char strTime[25] = { 0 };
        //  sprintf(strTime, "%d-%02d-%02d %02d:%02d:%02d %03d", time_tm->tm_year + 1900,
        //      time_tm->tm_mon + 1, time_tm->tm_mday, time_tm->tm_hour,
        //      time_tm->tm_min, time_tm->tm_sec, (int)dis_millseconds);
        //  std::cout << " strTime" << strTime << std::endl;
         //vehicleCurrent.gps.time =  time_tm->tm_hour*3600. + time_tm->tm_min*60. + time_tm->tm_sec + dis_millseconds/1000.;

         vehicleCurrent.gps.time = chrono::steady_clock::now().time_since_epoch().count() / 1000000000.;
       
        vehicleCurrent.gps.velocity = 0;
        

        vehicleCurrent.dBaseLength = 2.0; // 2.650;
        vehicleCurrent.dTargetSteeringAngle = 0.;
        vehicleCurrent.dTargetSpeed = 0.;

        count = 0;

        
    }

    ~ThreadJobs()
    {
        // nothing to do
        // cout << "ThreadJobs object destroyed" << endl;
    }
    void recvCallback()
    {
        while (1)
        {
            auto start = std::chrono::steady_clock::now();

            // =======do your works here======
            // cout << "I am recv thread " << endl;

            // char szBuf[1024] = {0};
            // int ret = zmq_recv(this->socketSub, szBuf, sizeof(szBuf) - 1, 0);
            // cout << "recv: " << szBuf << endl;

            // steeringController.setPath();
            // steeringController.calculateSteering();

            controlData::ControlCMD ctrlCmd;

            zmq_msg_t msg;
            zmq_msg_init(&msg);
            int size = zmq_msg_recv(&msg, this->socketSub, 0);
            // cout << "recv: " << size << endl;
            // count++;
            if (size == -1)
                continue;

            count++;
            //cout << "zmq recv: " << count  <<",size ="<<size << endl;

            void *str_recv = malloc(size);
            memcpy(str_recv, zmq_msg_data(&msg), size);
            ctrlCmd.ParseFromArray(str_recv, size);

            free(str_recv);

            vehicleCurrent.dTargetSteeringAngle = 0 - ctrlCmd.targetsteeringangle();
            vehicleCurrent.dTargetSpeed = ctrlCmd.targetspeed();
            printf("receive: targetspeed=%.3lf, targetsteeringangle=%.3lf  \n", ctrlCmd.targetspeed(), ctrlCmd.targetsteeringangle());

            //=======end of your works======

            auto end = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

            std::this_thread::sleep_for(std::chrono::milliseconds(this->recvRate - duration.count()));
        }
    }
    void sendCallback()
    {
        while (1)
        {
            auto start = std::chrono::steady_clock::now();

            // =======do your works here======

            // cout << "I am send thread " << endl;

            // double steeringToSend = this->steeringController.getSteering();

            // int ret = zmq_send(this->pubMsgList["imgPub"], "hello", 5, 0);

            // calculate new vehicle position
            // be careful the coor and angle
            double xForDynamic = vehicleCurrent.gps.gaussY;
            double yForDynamic = vehicleCurrent.gps.gaussX;
            double yawForDynamic = M_PI / 2 - vehicleCurrent.gps.yaw;
            double dDiffTime = chrono::steady_clock::now().time_since_epoch().count() / 1000000000. - vehicleCurrent.gps.time;
            yawForDynamic = vehicleCurrent.mod2pi(yawForDynamic);
            vehicleCurrent.gps.velocity = vehicleCurrent.dTargetSpeed;

            vehicleCurrent.VehicleDynamic(xForDynamic, yForDynamic, yawForDynamic, dDiffTime * vehicleCurrent.gps.velocity, vehicleCurrent.dTargetSteeringAngle);
            vehicleCurrent.gps.gaussX = yForDynamic;
            vehicleCurrent.gps.gaussY = xForDynamic;
            vehicleCurrent.gps.yaw = M_PI / 2 - yawForDynamic;
            vehicleCurrent.gps.yaw = vehicleCurrent.mod2pi(vehicleCurrent.gps.yaw);
            if (vehicleCurrent.gps.yaw < 0)
                vehicleCurrent.gps.yaw += (2 * M_PI);
            this->vehicleCurrent.gps.time = chrono::steady_clock::now().time_since_epoch().count() / 1000000000.;

            // send data by protobuf from IMU
            //计算经纬度 116.335587,40.001787
           ZtGeographyCoordinateTransform tf;
           double x,y;
           //this->vehicleCurrent.gps.longitude = 116.335587;
           //this->vehicleCurrent.gps.latitude = 40.001787;
           tf.BL2XY(this->vehicleCurrent.gps.latitude,  this->vehicleCurrent.gps.longitude, x, y);//为了计算中央子午线
           cout << "meline"<<tf.meridianLine<<endl;
           tf.XY2BL( this->vehicleCurrent.gps.gaussY, this->vehicleCurrent.gps.gaussX,this->vehicleCurrent.gps.latitude, this->vehicleCurrent.gps.longitude);
           //double  ht;
           //tf.XYZ2BLH(this->vehicleCurrent.gps.gaussX, this->vehicleCurrent.gps.gaussY, 0, this->vehicleCurrent.gps.latitude, this->vehicleCurrent.gps.longitude, ht);
           //cout << "send data" <<  this->vehicleCurrent.gps.gaussX << "," <<this->vehicleCurrent.gps.gaussY<< ","<<this->vehicleCurrent.gps.longitude  << "-----------------," << this->vehicleCurrent.gps.latitude << endl ;
          
            IMU::Imu imu;
            imu.set_longitude(this->vehicleCurrent.gps.longitude);
            imu.set_latitude(this->vehicleCurrent.gps.latitude);
            imu.set_gaussx(this->vehicleCurrent.gps.gaussX);
            imu.set_gaussy(this->vehicleCurrent.gps.gaussY);
            imu.set_gpsvalid(this->vehicleCurrent.gps.psValid);
            //计算时间，当天秒计数，double
            auto now = std::chrono::system_clock::now();
            //通过不同精度获取相差的毫秒数
            uint64_t dis_millseconds = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count()
            - std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count() * 1000;
            time_t tt = std::chrono::system_clock::to_time_t(now);
            auto time_tm = localtime(&tt);
            //  char strTime[25] = { 0 };
            //  sprintf(strTime, "%d-%02d-%02d %02d:%02d:%02d %03d", time_tm->tm_year + 1900,
            //      time_tm->tm_mon + 1, time_tm->tm_mday, time_tm->tm_hour,
            //      time_tm->tm_min, time_tm->tm_sec, (int)dis_millseconds);
            //  std::cout << " strTime" << strTime << std::endl;
           double timeSend =  time_tm->tm_hour*3600. + time_tm->tm_min*60. + time_tm->tm_sec + dis_millseconds/1000.;
            imu.set_time(timeSend);
            imu.set_velocity(vehicleCurrent.gps.velocity);
            imu.set_yaw(this->vehicleCurrent.gps.yaw * 180 / M_PI);

            size_t size = imu.ByteSize();
            void *buffer = malloc(size);

            // serialize your data, from pointVec to buffer
            if (!imu.SerializeToArray(buffer, size))
            {
                std::cerr << "Failed to write msg." << std::endl;
                return;
            }

            zmq_msg_t msg;
            zmq_msg_init_size(&msg, size);
            memcpy(zmq_msg_data(&msg), buffer, size); // copy data from buffer to zmq msg

            zmq_send(this->pubMsgList["imgPub"], buffer, size, 0);
            free(buffer);

            printf("send: lon=%.8lf, lat=%.8lf ,X=%.3lf,Y=%.3lf, status=%d,time=%.3lf,vel = %.2lf,hdg=%.2lf \n",
             this->vehicleCurrent.gps.longitude,this->vehicleCurrent.gps.latitude,
             this->vehicleCurrent.gps.gaussX,this->vehicleCurrent.gps.gaussY,
             this->vehicleCurrent.gps.psValid,
             timeSend,//this->vehicleCurrent.gps.time,
             this->vehicleCurrent.gps.velocity,
             this->vehicleCurrent.gps.yaw*180/M_PI);

            ///////////////////////////////////////////////////////////////////////////////////////////
            // send data form COMOS
            controlData::ChassisInfo chassisInfo;
            chassisInfo.set_speed(this->vehicleCurrent.gps.velocity);
            chassisInfo.set_steeringangle(this->vehicleCurrent.dTargetSteeringAngle);
            chassisInfo.set_mode(0);

            size = imu.ByteSize();
            buffer = malloc(size);

            // serialize your data, from pointVec to buffer
            if (!chassisInfo.SerializeToArray(buffer, size))
            {
                std::cerr << "Failed to write msg from COMOS." << std::endl;
                return;
            }

            zmq_msg_t msgFromCOMOS;
            zmq_msg_init_size(&msgFromCOMOS, size);
            memcpy(zmq_msg_data(&msgFromCOMOS), buffer, size); // copy data from buffer to zmq msg

            zmq_send(this->pubMsgList["comsPub"], buffer, size, 0);
            free(buffer);

            // printf("send: set_speed=%.3lf, set_steeringangle=%.3lf ,mode=%d\n",
            // this->vehicleCurrent.gps.velocity,  this->vehicleCurrent.dTargetSteeringAngle,   0);

            //=======end of your works======

            auto end = std::chrono::steady_clock::now();
            // std::chrono::duration<double, std::milli> duration = end - start;
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

            std::this_thread::sleep_for(std::chrono::milliseconds(this->sendRate - duration.count()));
        }
    }
};

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

    ThreadJobs threadJob;

    thread thread1(&ThreadJobs::recvCallback, &threadJob);
    thread thread2(&ThreadJobs::sendCallback, &threadJob);

    thread::id threadID1 = thread1.get_id();
    thread::id threadID2 = thread2.get_id();

    thread1.detach();
    thread2.detach();

    // thread1.join();
    // thread2.join();

    while (1)
    {
        // do what you want in main

        int thread1Status = pthread_kill(cvtThreadId2Long(threadID1), 0);
        // checkThreadStatus(thread1Status, threadID1);

        int thread2Status = pthread_kill(cvtThreadId2Long(threadID2), 0);
        // checkThreadStatus(thread2Status, threadID2);

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    return 0;
}