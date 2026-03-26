// Lidar目标检测基类
#include "baseLidarObjectDetector.h"
#include <chrono>
#include <iostream>
#include <thread>
#include <future>
#include <signal.h>
#include <string>
#include <sstream>
#include <map>
#include <zmq.h>

typedef std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds> miliClock_type;

// base class is not responsible for zmq communication

// bool LidarBaseObjectDetector::sendResultInit()
// {
//     void *context = zmq_ctx_new();
//     publisher = zmq_socket(context, ZMQ_PUB);
//     zmq_bind(publisher, "tcp://127.0.0.1:5555");
//     return true;
// }

// void LidarBaseObjectDetector::sendResult()
// {
//     size_t size = objResultList.ByteSize();
//     void *buffer = malloc(size);

//     miliClock_type tp = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
//     int64_t start = tp.time_since_epoch().count();
//     std::cout << start << std::endl;

//     // serialize your data, from pointVec to buffer
//     if (!objResultList.SerializeToArray(buffer, size))
//     {
//         std::cerr << "Failed to write msg." << std::endl;
//     }

//     zmq_msg_t msg;
//     zmq_msg_init_size(&msg, size);
//     memcpy(zmq_msg_data(&msg), buffer, size); // copy data from buffer to zmq msg
//     zmq_msg_send(&msg, publisher, 0);

//     tp = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
//     int64_t end = tp.time_since_epoch().count();
//     std::cout << end << std::endl;

//     std::cout << "end-start: " << end - start << std::endl;
//     free(buffer);

//     std::this_thread::sleep_for(std::chrono::milliseconds(100 - (end - start)));
// }
