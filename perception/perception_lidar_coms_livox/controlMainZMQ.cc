#include <chrono>
#include <future>
#include <iostream>
#include <map>
#include <signal.h>
#include <sstream>
#include <string>
#include <thread>
#include <zmq.h>
#include "qingColor.h"

#include <iomanip>

#include "controlController.hpp"
#include "controlData.hpp"

#include <Imu/imu.pb.h>
#include <controlData/control.pb.h>
#include <planning/PlanningMsg.pb.h>

std::mutex trajMtx;

//parseTrajData()和parseStateData(): 这两个函数用于解析接收到的路径和状态数据，并将其转换为对应的数据类型（Control::Traj和Control::State）的实例。
/**
 * @brief 反序列化字符串，提取【轨迹】信息
 *
 * @param trajProto 接受两个参数：trajProto是一个Planning命名空间中的TrajectoryPointVec类型，代表规划器生成的轨迹数据；
 * @param trajRaw 待跟踪轨迹,trajRaw是一个Control命名空间中的Traj类型，代表转换后的轨迹数据。
 */
void parseTrajData(Planning::TrajectoryPointVec trajProto, Control::Traj &trajRaw)
{
    //将规划器（Planning）生成的轨迹数据（trajectoryPoints）转换成控制器（Control）需要的轨迹数据格式（trajRaw）。采用了Google的protobuf库中的数据类型（trajProto）
    //将其转换为一个vector类型的轨迹（trajRaw）。
    //使用trajectoryPoints()函数从trajProto中获取轨迹点集合，然后使用一个for循环逐个遍历这些轨迹点，并将其转换为Control::TrajPoint类型，最后添加到trajRaw中
    auto trajectoryPoints = trajProto.trajectorypoints();
    trajRaw.clear();
    for (const auto &tp : trajectoryPoints)
    {
        trajRaw.push_back(Control::TrajPoint{tp.x(), tp.y(), tp.theta(), tp.speed(), tp.curvature(), tp.s()});
    }
}

/**
 * @brief 反序列化字符串，提取【状态】信息
 *
 * @param stateBuf socket收到的buffer
 * 将 IMU 数据转换为车辆状态信息的函数。它接受一个 IMU::Imu 类型的参数 vehStateProto，以及一个 Control::State 类型的引用 state。函数将 IMU 数据中的一些字段赋值给车辆状态信息中对应的字段
 * @param state 自车状态
 */
void parseStateData(IMU::Imu vehStateProto, Control::State &state)
{

    state.x = vehStateProto.gaussx();
    state.y = vehStateProto.gaussy();
    state.yaw = vehStateProto.yaw() / 180.0 * M_PI; //将其转换为弧度制
    state.rtkMode = vehStateProto.gpsvalid();
    state.v = vehStateProto.velocity();
}

/**
 * @brief 序列化控制指令
 *
 * @param cmdString
 * @param cmd
 */
void serializeControlData(std::string &cmdString, Control::ControlCMD cmd)
{
    controlData::ControlCMD cmdProto;
    cmdProto.set_targetspeed(cmd.speed);
    cmdProto.set_targetsteeringangle(cmd.steer);
    cmdProto.SerializeToString(&cmdString);
    std::cout << cmdString << std::endl;
}

/**
 * 整个类的作用是接收路径和状态信息，并计算控制命令，然后将该命令发送出去。
 * 由于路径信息必须在状态信息之前接收和处理，因此该类使用isTrajInit变量来确保路径信息已经被接收和处理。类中还使用了一个互斥锁（trajMtx），以确保在计算控制命令时不会访问未准备好的轨迹信息。
*/
class ThreadJobs
{
  private:
    int sendRate, recvRate;

    std::map<std::string, void *> socketSubMap;
    void *socketPub;
    // SteeringController steeringController;

    //controller成员变量，它是另一个称为Control::Controller的类的实例，它从给定的配置文件中加载控制器参数，并用于计算控制命令。
    //traj成员变量，它是Control::Traj类型的实例，存储了路径跟踪的轨迹信息。
    //state成员变量，它是Control::State类型的实例，存储了当前车辆状态的信息。
    //cmd成员变量，它是Control::ControlCMD类型的实例，存储了计算出的控制命令的信息。
    Control::Controller controller;
    Control::Traj traj;
    Control::State state;
    Control::ControlCMD cmd;

    //路径是否已经被初始化。
    bool isTrajInit;

  public:
    ThreadJobs(std::string configFile) : controller(configFile), isTrajInit(false)
    {
        //初始化了一个发布器（socketPub）和两个订阅器（socketSubTraj和socketSubState），它们都使用tcp协议连接到本地主机（IP地址为127.0.0.1）的不同端口
        /*********** ZMQ initialize *************/
        int ret;
        void *context = zmq_ctx_new();
        socketPub = zmq_socket(context, ZMQ_PUB);
        ret = zmq_bind(socketPub, "tcp://127.0.0.1:3171");

        void *socketSubTraj = zmq_socket(context, ZMQ_SUB);
        ret = zmq_connect(socketSubTraj, "tcp://127.0.0.1:5010");
        ret = zmq_setsockopt(socketSubTraj, ZMQ_SUBSCRIBE, "", 0);
        this->socketSubMap["subTraj"] = socketSubTraj;

        void *socketSubState = zmq_socket(context, ZMQ_SUB);
        ret = zmq_connect(socketSubState, "tcp://127.0.0.1:5003");
        ret = zmq_setsockopt(socketSubState, ZMQ_SUBSCRIBE, "", 0);
        this->socketSubMap["subState"] = socketSubState;

        cmd.speed = 0;
        cmd.steer = 0;
        std::cout << "!!thread job init" << std::endl;
    }

    ~ThreadJobs()
    {
        // nothing to do
    }

    /**
     * @brief 接收并更新轨迹
     *
     */
    /**
     * @brief 该函数在一个无限循环中运行，它从socketSubTraj接收路径信息，并使用parseTrajData()函数将接收到的数据解析为轨迹信息，然后将isTrajInit设置为true。
     * 该函数在路径信息接收之前等待，因此它需要先收到路径信息，然后才能接收和处理状态信息。
    */
    void subTraj()
    {
        while (1)
        {
            zmq_msg_t trajBufMsg;
            zmq_msg_init(&trajBufMsg);

            int trajSize = zmq_msg_recv(&trajBufMsg, socketSubMap["subTraj"], 0);
            //std::cout << "traj recv already" << std::endl;

            if (trajSize == -1)
            {
                std::cout << "!!Error traj" << std::endl;
                continue;
            }
            void *str_recv = malloc(trajSize);
            memcpy(str_recv, zmq_msg_data(&trajBufMsg), trajSize); // copy recived data from zmq msg to str_recv.
            Planning::TrajectoryPointVec trajProto;

            trajProto.ParseFromArray(str_recv, trajSize);

            trajMtx.lock();
            //std::cout << "!!locking traj" << std::endl;

            parseTrajData(trajProto, traj);
            isTrajInit = true;
            trajMtx.unlock();
            //std::cout << "recv traj size: " << traj.size() << std::endl;
            free(str_recv);
        }
    }

    /**
     * 该函数在一个无限循环中运行，它从socketSubState接收状态信息，并使用parseStateData()函数将接收到的数据解析为车辆状态信息，
     * 然后使用calcControlCMD()函数计算控制命令，并将该命令通过socketPub发送出去。
    */
    void recvStateAndPub()
    {
        while (1)
        {
            //记录开始时间
            auto start = std::chrono::steady_clock::now();

            // =======do your works here======
            //使用ZMQ（ZeroMQ）消息队列库中的zmq_msg_recv函数从名为"subState"的消息队列中接收数据。
            //接收到的数据被存储在zmq_msg_t类型的stateBufMsg中，并通过zmq_msg_data函数获取数据的大小和内容
            zmq_msg_t stateBufMsg;
            zmq_msg_init(&stateBufMsg);
            int stateSize = zmq_msg_recv(&stateBufMsg, socketSubMap["subState"], 0);

            //如果接收到的数据大小为-1，说明接收到了错误数据，打印错误信息并继续下一次循环
            if (stateSize == -1)
            {
                std::cout << "!!!Data Error" << std::endl;
                continue;
            }

            //创建一个与接收到的数据大小相同的缓冲区str_recv，并通过memcpy函数将接收到的数据复制到str_recv中
            void *str_recv = malloc(stateSize);
            memcpy(str_recv, zmq_msg_data(&stateBufMsg), stateSize); // copy recived data from zmq msg to str_recv.

            //创建一个IMU::Imu类型的proto对象vehStateProto，调用其ParseFromArray函数将str_recv中的数据解析为proto对象
            IMU::Imu vehStateProto;
            vehStateProto.ParseFromArray(str_recv, stateSize);
            
            //调用parseStateData函数将proto对象中的数据转换为状态数据，并打印状态数据
            parseStateData(vehStateProto, state);

            std::cout << RED << "-------------------@@@-------------" << RESET <<std::endl;
            printf("state received (Original): x %.2f, y %.2f, yaw(rad) %.2f, v(m/s) %.1f, rtkMode %.0f \n",
                state.x, state.y, state.yaw, state.v, state.rtkMode);
            free(str_recv);
            //记录进入锁定时间，并使用锁定机制锁定trajMtx
            auto start2 = std::chrono::steady_clock::now();

            trajMtx.lock();
            //std::cout << "!!locking state" << std::endl;

            //如果轨迹数据还未接收到，打印消息并等待1秒钟，然后解锁trajMtx并继续下一次循环
            if (!isTrajInit)
            {
                std::cout << "traj not received yet! " << std::endl;
                trajMtx.unlock();
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                continue;
            }
            //std::cout << "entering calcControlCMD lock " << std::endl;
            auto start3 = std::chrono::steady_clock::now();

            // 使用控制器对象controller计算出控制命令cmd，传入当前的状态数据state和轨迹数据traj；
            cmd = controller.calcControlCMD(state, traj);
            trajMtx.unlock();


            // ============= send the cmd============
            //创建一个controlData::ControlCMD类型的proto对象cmdProto，并将计算出的控制命令cmd中的速度和转向角度存入该proto对象中
            controlData::ControlCMD cmdProto;
            cmdProto.set_targetspeed(cmd.speed);
            cmdProto.set_targetsteeringangle(cmd.steer);

            // 计算cmdProto对象的大小，并创建一个与其大小相同的缓冲区cmdBuffer
            size_t cmdSize = cmdProto.ByteSize();
            void *cmdBuffer = malloc(cmdSize);

            // 调用cmdProto对象的SerializeToArray函数将其序列化为二进制数据，并将其复制到cmdBuffer中
            // serialize your data, from pointVec to buffer
            if (!cmdProto.SerializeToArray(cmdBuffer, cmdSize))
            {
                std::cerr << "Failed to write msg." << std::endl;
            }

            // 使用ZMQ库中的zmq_msg_send函数将消息msg发送到名为socketPub的消息队列中
            zmq_msg_t msg;
            zmq_msg_init_size(&msg, cmdSize);
            memcpy(zmq_msg_data(&msg), cmdBuffer, cmdSize); // copy data from buffer to zmq msg
            zmq_msg_send(&msg, socketPub, 0);
            //=======end of your works======

            auto end = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start2);
            auto duration3 = std::chrono::duration_cast<std::chrono::milliseconds>(end - start3);
            //记录结束时间，并打印控制算法延迟时间和在锁定中的控制算法延迟时间
            std::cout << "control algorithm delay: " << duration.count() << std::endl;
            std::cout << "control algorithm delay inlock: " << duration3.count() << std::endl;
        }
    }
};

/**
 * 用于调试信息中记录线程 ID
*/
long int cvtThreadId2Long(std::thread::id id)
{
    std::stringstream ss;
    ss << id;

    return std::stol(ss.str());
}

/**
 * @brief 用于检查一个线程的状态，接受两个参数：tStatus和id。tStatus是指向线程状态的指针，由pthread_kill函数返回；id是要检查的线程ID。
*/
void checkThreadStatus(int tStatus, std::thread::id id)
{
    //首先检查tStatus，如果tStatus为ESRCH，则说明线程不存在，向控制台输出线程不存在的信息。
    //如果tStatus为EINVAL，则说明线程ID无效，向控制台输出线程ID无效的信息。如果tStatus既不是ESRCH也不是EINVAL，则说明线程仍然存在，向控制台输出线程存在的信息。
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



int main()
{
    //存储控制参数或配置文件的路径的字符串
    std::string configFile("../config/control.yaml");
    ThreadJobs threadJob(configFile);

    //创建 thread1 和 thread2 两个线程分别调用 ThreadJobs 类中的 subTraj 和 recvStateAndPub 方法
    std::thread thread1(&ThreadJobs::subTraj, &threadJob);
    std::thread thread2(&ThreadJobs::recvStateAndPub, &threadJob);

    //分别获取这两个线程的 ID
    std::thread::id threadID1 = thread1.get_id();
    std::thread::id threadID2 = thread2.get_id();

    thread1.detach();
    thread2.detach();
    // thread1.join();

    //在一个无限循环中调用 pthread_kill 函数监测子线程状态
    while (1)
    {
        // do what you want in main

        //在使用 pthread_kill 函数时，需要将 std::thread::id 类型的线程 ID 转换为 long int 类型，然后再传入 pthread_kill 函数中。在这里，使用了 cvtThreadId2Long 函数完成转换
        int thread1Status = pthread_kill(cvtThreadId2Long(threadID1), 0);
        int thread2Status = pthread_kill(cvtThreadId2Long(threadID2), 0);

        // checkThreadStatus(thread1Status, threadID1);
        // checkThreadStatus(thread2Status, threadID2);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    return 0;
}