#include "zmqcommunication.h"
#include <zmq.h>
#include <iostream>
#include <fstream>
//#include <jsoncpp/json/json.h>
#include <unistd.h>
#include "mainwindow.h"
#include <QDebug>
#include <iomanip>

#include <chrono>
#include <thread>
#include <imu.pb.h>
#include "ztgeographycoordinatetransform.h"

#define SAVE_ROAD_FLAG true

typedef std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds> miliClock_type;

ZmqCommunication::ZmqCommunication()
{
    stopped = false;
}
void ZmqCommunication::stop()
{
    stopped = true;
}

//void ZmqCommunication::gaussConvert(double longitude1, double latitude1, double &dNorth_X, double &dEast_Y)
//{

//    double a = 6378137.0;

//    double e2 = 0.0066943799013;

//    double latitude2Rad = (M_PI / 180.0) * latitude1;

//    int beltNo = int((longitude1 + 1.5) / 3.0);
//    int L = beltNo * 3;
//    double l0 = longitude1 - L;
//    double tsin = sin(latitude2Rad);
//    double tcos = cos(latitude2Rad);
//    double t = tan(latitude2Rad);
//    double m = (M_PI / 180.0) * l0 * tcos;
//    double et2 = e2 * pow(tcos, 2);
//    double et3 = e2 * pow(tsin, 2);
//    double X = 111132.9558 * latitude1 - 16038.6496 * sin(2 * latitude2Rad) + 16.8607 * sin(4 * latitude2Rad) - 0.0220 * sin(6 * latitude2Rad);
//    double N = a / sqrt(1 - et3);

//    dNorth_X = X + N * t * (0.5 * pow(m, 2) + (5.0 - pow(t, 2) + 9.0 * et2 + 4 * pow(et2, 2)) * pow(m, 4) / 24.0 + (61.0 - 58.0 * pow(t, 2) + pow(t, 4)) * pow(m, 6) / 720.0);
//    dEast_Y = 500000 + N * (m + (1.0 - pow(t, 2) + et2) * pow(m, 3) / 6.0 + (5.0 - 18.0 * pow(t, 2) + pow(t, 4) + 14.0 * et2 - 58.0 * et2 * pow(t, 2)) * pow(m, 5) / 120.0);
//    //   std::cout << BOLDRED << "x1: " << x1 << std::endl;
//    //   std::cout << BOLDRED << "y1: " << y1 << std::endl;
//}

// 校正高斯坐标中IMU相对于车辆质心的位置偏移，offset向右为X正，向前为Y正
void ZmqCommunication::offsetRevise(double yaw, double &x1, double &y1)
{
    double yaw2Rad = (M_PI / 180.0) * yaw;
    x1 -= offsetY * cos(yaw2Rad) - offsetX * sin(yaw2Rad);
    y1 -= offsetY * sin(yaw2Rad) + offsetX * cos(yaw2Rad);
}

// bys syp 20220720暂时取消zmq的通讯
// 没有安装zmq的库，这部分先不发送出去，只是简单进行数据的调试信息显示
void ZmqCommunication::run()
{

    void *context = zmq_ctx_new();
    void *broadcastingSocket = zmq_socket(context, ZMQ_PUB);

    int queueLength = 2;
    zmq_setsockopt(broadcastingSocket, ZMQ_SNDHWM, &queueLength, sizeof(queueLength));
    //    zmq_bind(broadcastingSocket, "tcp://127.0.0.1:3124");
    zmq_bind(broadcastingSocket, "tcp://*:5003");

    IMU::Imu imu;

    GPS stuGINsDGPS;

    while (true) // 这是一个标准的死循环
    {

        ZmqSendSemphore.acquire();

        mutexListGInsDGPS.lock();
        if (!listGInsDGPS.isEmpty())
        {
            stuGINsDGPS = listGInsDGPS.takeFirst();
        }
        else
        {
            stuGINsDGPS.utime = 0;
        }
        mutexListGInsDGPS.unlock();

        if (stuGINsDGPS.utime != 0) // 这里用时间表示数据是否更新了
        {
        }
        else
        {
            // usleep(2000);
            continue;
        }

        double gaussX, gaussY;
        //gaussConvert(stuGINsDGPS.longitude, stuGINsDGPS.latitude, gaussX, gaussY);
        //gaussConvert(stuGINsDGPS.latitude, stuGINsDGPS.longitude, gaussX, gaussY);

        ZtGeographyCoordinateTransform ztTemp;
        ztTemp.BL2XY(stuGINsDGPS.latitude, stuGINsDGPS.longitude,  gaussY ,gaussX);
        
        qDebug() << "gaussConvert"
                //<< QString::number(stuGINsDGPS.ltime, 'f', 10) << ","
                 << QString::number(stuGINsDGPS.longitude, 'f', 8) << ","
                 << QString::number(stuGINsDGPS.latitude, 'f', 8) << ","
                 << QString::number(gaussX, 'f', 8) << ","
                 << QString::number(gaussY, 'f', 8) << ","
                 << QString::number(stuGINsDGPS.yaw, 'f', 8) << ","
                 << QString::number(stuGINsDGPS.gpsValid, 'f', 8);

        offsetRevise(stuGINsDGPS.yaw, gaussX, gaussY);

        imu.set_longitude(stuGINsDGPS.longitude);
        imu.set_latitude(stuGINsDGPS.latitude);
        imu.set_gaussx(gaussX);
        imu.set_gaussy(gaussY);

        imu.set_gpsvalid(stuGINsDGPS.gpsValid);

        double dtimeforSend ;
        dtimeforSend = stuGINsDGPS.ltime % (24*3600*1000);
        dtimeforSend = dtimeforSend/1000;
        imu.set_time(dtimeforSend);
        //imu.set_time(stuGINsDGPS.ltime);
        imu.set_velocity(stuGINsDGPS.velocity);
        imu.set_yaw(stuGINsDGPS.yaw);
        imu.set_pitch(stuGINsDGPS.pitch);
        imu.set_roll(stuGINsDGPS.roll);


        imu.set_ltime(stuGINsDGPS.ltime);
        imu.set_velocityeast(stuGINsDGPS.velocityEast);
        imu.set_velocitynorth(stuGINsDGPS.velocityNorth);
        imu.set_velocityup(stuGINsDGPS.velocityUp);
        imu.set_gyrox(stuGINsDGPS.gyroX);
        imu.set_gyroy(stuGINsDGPS.gyroY);
        imu.set_gyroz(stuGINsDGPS.gyroZ);
        imu.set_accx(stuGINsDGPS.accX);
        imu.set_accy(stuGINsDGPS.accY);
        imu.set_accz(stuGINsDGPS.accZ);

        auto now = std::chrono::system_clock::now();
        auto pjatime = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
        imu.set_pjatime(0.001 * pjatime );


        size_t size = imu.ByteSize();
        void *buffer = malloc(size);

        miliClock_type tp = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
        int64_t start = tp.time_since_epoch().count();
        // std::cout << start << std::endl;

        // serialize your data, from pointVec to buffer
        if (!imu.SerializeToArray(buffer, size))
        {
            std::cerr << "Failed to write protbuf msg." << std::endl;
            return;
        }

        zmq_msg_t msg;
        zmq_msg_init_size(&msg, size);
        memcpy(zmq_msg_data(&msg), buffer, size); // copy data from buffer to zmq msg
        zmq_msg_send(&msg, broadcastingSocket, 0);
        zmq_msg_close(&msg);

         qDebug() << "ZmqCommunication"
                  //<< QString::number(stuGINsDGPS.utime, 10) << ","
                  << QString::number(stuGINsDGPS.ltime, 'f', 10) << ","
                  << QString::number(stuGINsDGPS.longitude, 'f', 8) << ","
                  << QString::number(stuGINsDGPS.latitude, 'f', 8) << ","
                  << QString::number(gaussX, 'f', 8) << ","
                  << QString::number(gaussY, 'f', 8) << ","
                  << QString::number(stuGINsDGPS.yaw, 'f', 8) << ","
                  << QString::number(stuGINsDGPS.gpsValid, 'f', 8);

        tp = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
        int64_t end = tp.time_since_epoch().count();
        // std::cout << end << std::endl;

        // std::cout << "end-start: " << end - start << std::endl;
        if (saveflag)
        {
            txtOutput << QTime::currentTime().toString("hh:mm::ss.zzz") << ","
                          << QString::number(stuGINsDGPS.utime, 10) << ","
                          << QString::number(stuGINsDGPS.longitude, 'f', 8) << ","
                          << QString::number(stuGINsDGPS.latitude, 'f', 8) << ","
                          << QString::number(gaussX, 'f', 8) << ","
                          << QString::number(gaussY, 'f', 8) << ","
                          << QString::number(stuGINsDGPS.yaw, 'f', 8) << ","
                          << QString::number(stuGINsDGPS.gpsValid, 'f', 8) << endl;
            txtOutput.flush();
        }

        free(buffer);

        // std::this_thread::sleep_for(std::chrono::milliseconds(20 - (end - start)));

        // save file at here not suitable
        //        if (true == SAVE_ROAD_FLAG)
        //        {
        //            static std::ofstream fsRoad("rawMap.txt", std::ofstream::trunc);
        //            fsRoad << std::setprecision(15) << 1 << " " << stuGINsDGPS.longitude << " " << stuGINsDGPS.latitude << " " << stuGINsDGPS.yaw << " " << 4 << " " << 0 << std::endl;
        //        }

    } // while (true) //这是一个标准的死循环
}
