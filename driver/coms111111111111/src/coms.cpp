/*
 *
 * Created on: Oct 5, 2021 05:03
 * Description:
 *
 * Copyright (c) 2021 Yang Yu
 */

#include "scout_base.hpp"

// for zmq:
#include <zmq.h>

#include "control.pb.h"

#include <chrono>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

using namespace westonrobot;
int g_count = 0;
class Jobs
{
  public:
    explicit Jobs(void *sSocket, void *rSocket, std::string &device_name_, int32_t &baud_rate_) : sendSocket_(sSocket), recvSocket_(rSocket) { scout.Connect(device_name_, baud_rate_); }

    ~Jobs() { ; }
    // return 0;

    bool SerialIsOpened() { return scout.IsOpened(); }

    void cmdSubscriber()
    {
        while (true)
        {
            std::cout << "cmdSubscriber------------------" << std::endl;
            controlData::ControlCMD ctrlCmd;

            zmq_msg_t msg;
            zmq_msg_init(&msg);
            int size = zmq_msg_recv(&msg, recvSocket_, 0);

            if (size == -1)
                return;

            void *str_recv = malloc(size);
            memcpy(str_recv, zmq_msg_data(&msg), size);
            ctrlCmd.ParseFromArray(str_recv, size);

            ScoutMotionCmd<double> cmd;

            // if (g_count < 200)
            // {
            //     cmd.linear_velocity = 4;
            //     cmd.steering_angle = 0;
            //     cmd.speed_gear = 5;
            // }
            // else
            // {
            //     cmd.linear_velocity = 1;
            //     cmd.steering_angle = 0;
            //     cmd.speed_gear = 5;
            // }
            // g_count++;
            // std::cout << g_count << std::endl;
            cmd.linear_velocity = ctrlCmd.targetspeed();
            cmd.steering_angle = ctrlCmd.targetsteeringangle();
            if (ctrlCmd.targetspeed() < 2)
            {
                cmd.speed_gear = 4;
                std::cout << cmd.speed_gear << std::endl;
            }
            // std::cout << "cmd_speed_gear----------------------" << cmd.speed_gear << std::endl;
            std::cout << "hhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhh" << std::endl;
            std::cout << "cmdSubscriber recv: " << ctrlCmd.targetspeed() << " " << ctrlCmd.targetsteeringangle() << std::endl;
            // cmd.linear_velocity = 0;
            // cmd.steering_angle = 10;

            scout.SetMotionCommand(cmd);
            // usleep(100000);
        }
    }

    void statePublisher()
    {
        while (true)
        {
            auto start = std::chrono::steady_clock::now();

            auto state = scout.GetScoutState();
            controlData::ChassisInfo info;

            info.set_speed(state.velocity);
            info.set_steeringangle(state.steering_angle);
            info.set_mode(state.mode);

            std::cout << "statePublisher velocity steering_angle mode: " << state.velocity << ", " << state.steering_angle << ", " << state.mode << std::endl;
            std::cout << "@@@@@mode::" << state.mode << std::endl;

            size_t size = info.ByteSize();
            void *buffer = malloc(size);

            if (!info.SerializeToArray(buffer, size))
            {
                std::cerr << "Failed to write msg." << std::endl;
                return;
            }

            zmq_msg_t msg;
            zmq_msg_init_size(&msg, size);
            memcpy(zmq_msg_data(&msg), buffer, size);
            zmq_msg_send(&msg, sendSocket_, 0);
            free(buffer);

            auto end = std::chrono::steady_clock::now();

            std::this_thread::sleep_for(std::chrono::milliseconds(100) - (end - start));
        }
    }

  private:
    void *sendSocket_;
    void *recvSocket_;

    ScoutBase scout;
};

long int cvtThreadId2Long(std::thread::id id)
{
    std::stringstream ss;
    ss << id;

    return std::stol(ss.str());
}

void checkThreadStatus(int tStatus, std::thread::id id)
{
    if (tStatus == ESRCH)
    {
        std::cout << "thread  " << id << " not exist" << std::endl;
    }
    else if (tStatus == EINVAL)
    {
        std::cout << "signal " << id << " is invalid" << std::endl;
    }
    else
    {
        std::cout << "thread  " << id << " is alive" << std::endl;
    }
}

int main(int argc, char **argv)
{
    // zmq communication
    void *context = zmq_ctx_new();

    // zmq sending sockets
    void *sendSocket = zmq_socket(context, ZMQ_PUB);
    zmq_bind(sendSocket, "tcp://*:3151");

    // zmq receiving sockets
    void *recvSocket = zmq_socket(context, ZMQ_SUB);
    zmq_connect(recvSocket, "tcp://127.0.0.1:3171");
    zmq_setsockopt(recvSocket, ZMQ_SUBSCRIBE, "", 0);

    // thread initial
    std::string device_name = "/dev/ttyS3";
    int32_t baud_rate = 9600;

    Jobs jobs(sendSocket, recvSocket, device_name, baud_rate);

    if (jobs.SerialIsOpened())
    {
        std::cout << "Specified serial: " << device_name << "@" << baud_rate << std::endl;
    }
    else
    {
        return -1;
    }

    std::thread subscriber_thread(&Jobs::cmdSubscriber, &jobs);
    std::thread publisher_thread(&Jobs::statePublisher, &jobs);

    std::thread::id subscriber_threadID = subscriber_thread.get_id();
    std::thread::id publisher_threadID = publisher_thread.get_id();

    subscriber_thread.detach();
    publisher_thread.detach();

    while (true)
    {
        int sub_threadStatus = pthread_kill(cvtThreadId2Long(subscriber_threadID), 0);
        int pub_threadStatus = pthread_kill(cvtThreadId2Long(publisher_threadID), 0);
        checkThreadStatus(sub_threadStatus, subscriber_threadID);
        checkThreadStatus(pub_threadStatus, publisher_threadID);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    return 0;
}
