#include <chrono>
#include <future>
#include <iostream>
#include <map>
#include <signal.h>
#include <sstream>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <zmq.h>
#include <iomanip>
#include <fstream>
#define _USE_MATH_DEFINES
#include <cmath>

#include "printColor.h"
#include "controlData.hpp"
#include "controllerBase.hpp"
#include <imu.pb.h>
#include <control.pb.h>
#include <PlanningMsg.pb.h>

class ThreadJobs
{
private:
    void* context;
    std::map<std::string, void*> socketSubMap;
    void* socketPub;

    Control::Controller controller;
    Control::Traj traj;
    Control::State state;
    Control::ControlCMD cmd;

    std::mutex trajMtx;
    std::atomic<bool> isTrajInit;

public:
    ThreadJobs(const std::string& configFile) : controller(configFile), isTrajInit(false)
    {
        context = zmq_ctx_new();
        socketPub = zmq_socket(context, ZMQ_PUB);
        zmq_bind(socketPub, "tcp://127.0.0.1:3171");

        void* socketSubTraj = zmq_socket(context, ZMQ_SUB);
        zmq_connect(socketSubTraj, "tcp://127.0.0.1:5010");
        zmq_setsockopt(socketSubTraj, ZMQ_SUBSCRIBE, "", 0);
        socketSubMap["subTraj"] = socketSubTraj;

        void* socketSubState = zmq_socket(context, ZMQ_SUB);
        zmq_connect(socketSubState, "tcp://127.0.0.1:5003");
        zmq_setsockopt(socketSubState, ZMQ_SUBSCRIBE, "", 0);
        socketSubMap["subState"] = socketSubState;

        cmd.speed = 0;
        cmd.steer = 0;
        std::cout << "!!thread job init" << std::endl;
    }

    ~ThreadJobs()
    {
        for (auto& kv : socketSubMap)
        {
            zmq_close(kv.second);
        }
        zmq_close(socketPub);
        zmq_ctx_term(context);
    }

    void subTraj()
    {
        while (true)
        {
            zmq_msg_t trajBufMsg;
            zmq_msg_init(&trajBufMsg);
            int trajSize = zmq_msg_recv(&trajBufMsg, socketSubMap["subTraj"], 0);
            if (trajSize == -1)
            {
                std::cout << "!!Error traj" << std::endl;
                continue;
            }
            std::string trajData(static_cast<char*>(zmq_msg_data(&trajBufMsg)), trajSize);
            Planning::TrajectoryPointVec trajProto;
            trajProto.ParseFromString(trajData);


            {
                std::lock_guard<std::mutex> lock(trajMtx);
                traj.clear();
                for (const auto& tp : trajProto.trajectorypoints())
                {

                    traj.emplace_back(Control::TrajPoint{
                        tp.x(), tp.y(), tp.theta() / 180 * M_PI, tp.speed(), tp.curvature(), tp.s(), tp.acceleration()
                    });
                }
                isTrajInit = true;
            }

            std::cout << "traj : ";
            printf("\n");
            std::cout << "vel: ";
            for (size_t i = 0; i < std::min<size_t>(15, traj.size()); ++i)
            {
                printf("%.3f\t", traj[i].v);
            }
            printf("\n");
            std::cout << "acc: ";
            for (size_t i = 0; i < std::min<size_t>(15, traj.size()); ++i)
            {
                printf("%.3f\t", traj[i].acc);
            }
            printf("\n");

        }
    }

    void recvStateAndPub()
    {
        while (true)
        {
            
            auto start = std::chrono::steady_clock::now();

            zmq_msg_t stateBufMsg;
            zmq_msg_init(&stateBufMsg);
            int stateSize = zmq_msg_recv(&stateBufMsg, socketSubMap["subState"], 0);
            if (stateSize == -1)
            {
                std::cout << "!!!Data Error" << std::endl;
                continue;
            }
            std::string stateData(static_cast<char*>(zmq_msg_data(&stateBufMsg)), stateSize);
            IMU::Imu vehStateProto;
            vehStateProto.ParseFromString(stateData);

            state.x = vehStateProto.gaussx();
            state.y = vehStateProto.gaussy();
            state.yaw = vehStateProto.yaw() / 180.0 * M_PI; //to rad
            state.rtkMode = vehStateProto.gpsvalid();
            state.v = vehStateProto.velocity();
            state.accX = vehStateProto.accx();
            std::cout << GREEN<< ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" <<RESET<< std::endl;
            printf("State(IMU) received: x: %.2f, y: %.2f, yaw(deg): %.2f, cur_v(m/s): %.2f\n",
                   state.x, state.y, state.yaw * 180.0 /M_PI, state.v, state.rtkMode);

            if (!isTrajInit.load())
            {
                std::cout << RED << "===========> Planning Traj not received yet!!!!!!!" << RESET << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(100)); //dT0.01s\100Hz
                continue;
            }

            {
                std::lock_guard<std::mutex> lock(trajMtx);
                cmd = controller.calcControlCMD(state, traj);
            }

            controlData::ControlCMD cmdProto;
            cmdProto.set_targetspeed(cmd.speed);
            cmdProto.set_targetsteeringangle(cmd.steer);
            std::cout<<"FinalSteer: "<<cmd.steer / M_PI * 180 * 15.5<<std::endl;
            // cmdProto.set_targetacceleration(cmd.acceleration);
            // cmdProto.set_targetdrivemode(cmd.driveMode);
            cmdProto.set_obstacledis(cmd.acceleration);
            

            size_t cmdSize = cmdProto.ByteSizeLong();
            zmq_msg_t msg;
            zmq_msg_init_size(&msg, cmdSize);
            cmdProto.SerializeToArray(zmq_msg_data(&msg), cmdSize);
            zmq_msg_send(&msg, socketPub, 0);

            auto end = std::chrono::steady_clock::now();
            double used = std::chrono::duration<double>(end - start).count();

            // 控制周期s   20Hz
            // double left = 0.05 - used;
            // if (left > 0)
            //     std::this_thread::sleep_for(std::chrono::duration<double>(left));
            // else
            //     std::cout << "[WARN] Control loop overrun: " << -left << " s\n"; // English log

        }
    }
};
