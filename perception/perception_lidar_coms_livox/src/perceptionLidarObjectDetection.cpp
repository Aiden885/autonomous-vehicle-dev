#include "imu.pb.h"
#include "prediction.pb.h"
#include <baseLidarObjectDetector.h>
#include <chrono>
#include <classicLidarDetector.h>
#include <eigen3/Eigen/Core>
#include <functionLidarDetector.h>
#include <future>
#include <iostream>
#include <map>
#include <mutex>
#include <signal.h>
#include <sstream>
#include <string>
#include <thread>
#include <zmq.h>

std::mutex g_mtxViewer;

using namespace std;
auto lastTheadTime = std::chrono::steady_clock::now();
auto lastTheadTimeofSub = std::chrono::steady_clock::now();
auto durationFroShowofSub = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - lastTheadTimeofSub);

bool getMethodByName(string methodName = "Classic", LidarBaseObjectDetector *lidarDetector_ = NULL)
{
    if (methodName == "Classic")
    {
        // PointCloudsDetector *classicLidarDetector = new PointCloudsDetector();
        // //使用父类的指针，指向子类
        // lidarDetector_ = classicLidarDetector;
        lidarDetector_ = new PointCloudsDetector;

        return true;
    }

    else
    {
        return false;
    }
}

class ThreadJobs
{
  private:
    int pcd_file_index = 0;
    void *subPointCloud16_0;
    void *subPointCloud16_1;
    void *subPointCloud128;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud16_0;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud16_1;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud128;

    std::map<std::string, void *> socketSubMap;

    // LidarBaseObjectDetector *lidarObjectDetector;
    PointCloudsDetector *lidarObjectDetector;
    bool FLAG_lidarObjectDetection = false;

    void *pubObjectList;
    void *pubObjectList_unity;
    void *pubObjectList_cluster;
    std::mutex mtxCloud16_0;
    std::mutex mtxCloud16_1;
    std::mutex mtxCloud128;

    int index;

  public:
    ThreadJobs() : cloud16_0(new pcl::PointCloud<pcl::PointXYZI>), cloud16_1(new pcl::PointCloud<pcl::PointXYZI>), cloud128(new pcl::PointCloud<pcl::PointXYZI>), index(0)
    {
        this->lidarObjectDetector = new PointCloudsDetector;

        cout << "addr: " << this->lidarObjectDetector << endl;

        void *context = zmq_ctx_new();
        int queueLength = 2;

        this->subPointCloud16_0 = zmq_socket(context, ZMQ_SUB);
        zmq_connect(this->subPointCloud16_0, "tcp://127.0.0.1:5005"); // right side
        zmq_setsockopt(this->subPointCloud16_0, ZMQ_SUBSCRIBE, "", 0);

        this->subPointCloud16_1 = zmq_socket(context, ZMQ_SUB);
        zmq_connect(this->subPointCloud16_1, "tcp://127.0.0.1:5006"); // left side
        zmq_setsockopt(this->subPointCloud16_1, ZMQ_SUBSCRIBE, "", 0);

        this->subPointCloud128 = zmq_socket(context, ZMQ_SUB);
        zmq_connect(this->subPointCloud128, "tcp://127.0.0.1:5012");
        zmq_setsockopt(this->subPointCloud128, ZMQ_SUBSCRIBE, "", 0);

        void *context2 = zmq_ctx_new();
        this->pubObjectList = zmq_socket(context2, ZMQ_PUB);
        zmq_setsockopt(this->pubObjectList, ZMQ_SNDHWM, &queueLength, sizeof(queueLength));
        zmq_bind(this->pubObjectList, "tcp://127.0.0.1:5009");

        int ret;
        void *socketSubState = zmq_socket(context, ZMQ_SUB);
        ret = zmq_connect(socketSubState, "tcp://127.0.0.1:5003");
        ret = zmq_setsockopt(socketSubState, ZMQ_SUBSCRIBE, "", 0);
        this->socketSubMap["subState"] = socketSubState;

        void *context3 = zmq_ctx_new();
        this->pubObjectList_unity = zmq_socket(context3, ZMQ_PUB);
        zmq_setsockopt(this->pubObjectList_unity, ZMQ_SNDHWM, &queueLength, sizeof(queueLength));
        zmq_bind(this->pubObjectList_unity, "tcp://*:5030");

        // 聚类信息
        void *context4 = zmq_ctx_new();
        this->pubObjectList_cluster = zmq_socket(context4, ZMQ_PUB);
        zmq_setsockopt(this->pubObjectList_cluster, ZMQ_SNDHWM, &queueLength, sizeof(queueLength));
        zmq_bind(this->pubObjectList_cluster, "tcp://*:5031");
    }

    ~ThreadJobs() { delete this->lidarObjectDetector; }

    pcl::visualization::PCLVisualizer::Ptr getViewer() { return this->lidarObjectDetector->getViewer(); }

    // 通过ZMQ获取右侧VLP雷达的driver数据
    void recvPointCloud16_0()
    {
        YAML::Node config;
        config = YAML::LoadFile("../config/perception.yaml");
        // 从yaml文件中读取标定参数
        std::vector<std::vector<float>> transform_vector = config["Tranform16_0"].as<std::vector<std::vector<float>>>();
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        for (int i = 0; i < 4; ++i)
        {
            for (int j = 0; j < 4; ++j)
            {
                transform(i, j) = transform_vector[i][j];
            }
        }
        while (true)
        {
            auto start = std::chrono::steady_clock::now();
            CloudProto::CloudData cloudData;

            zmq_msg_t msg;
            zmq_msg_init(&msg);
            int size = zmq_msg_recv(&msg, this->subPointCloud16_0, 0);

            if (size == -1)
                continue;

            void *recvStr = malloc(size);
            memcpy(recvStr, zmq_msg_data(&msg), size);
            zmq_msg_close(&msg);

            cloudData.ParseFromArray(recvStr, size);

            free(recvStr);

            this->mtxCloud16_0.lock();
            this->cloud16_0->width = cloudData.width();
            this->cloud16_0->height = cloudData.height();
            this->cloud16_0->is_dense = cloudData.isdense();
            this->cloud16_0->resize(cloudData.width() * cloudData.height());
            memcpy(this->cloud16_0->points.data(), cloudData.pointsdata().data(), cloudData.pointsdata().size());

            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_trans(new pcl::PointCloud<pcl::PointXYZI>);

            // transform_xy(1, 3) = -0.39558;
            // transform_xy(2, 3) = 0.18834;
            // transform_xy(1, 3) = -0.39558;
            // transform_xy(2, 3) = -0.18834;

            pcl::transformPointCloud(*(this->cloud16_0), *cloud_trans, transform);
            this->cloud16_0->swap(*cloud_trans);

            this->mtxCloud16_0.unlock();

            auto end = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        }
    }

    // 通过ZMQ获取左侧VLP雷达的driver数据
    void recvPointCloud16_1()
    {
        YAML::Node config;
        config = YAML::LoadFile("../config/perception.yaml");
        // 从yaml文件中读取标定参数
        std::vector<std::vector<float>> transform_vector = config["Tranform16_1"].as<std::vector<std::vector<float>>>();
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        for (int i = 0; i < 4; ++i)
        {
            for (int j = 0; j < 4; ++j)
            {
                transform(i, j) = transform_vector[i][j];
            }
        }
        while (true)
        {

            auto start = std::chrono::steady_clock::now();
            CloudProto::CloudData cloudData;

            zmq_msg_t msg;
            zmq_msg_init(&msg);
            int size = zmq_msg_recv(&msg, this->subPointCloud16_1, 0);

            if (size == -1)
                continue;

            void *recvStr = malloc(size);
            memcpy(recvStr, zmq_msg_data(&msg), size);
            zmq_msg_close(&msg);

            cloudData.ParseFromArray(recvStr, size);

            free(recvStr);

            this->mtxCloud16_1.lock();
            this->cloud16_1->width = cloudData.width();
            this->cloud16_1->height = cloudData.height();
            this->cloud16_1->is_dense = cloudData.isdense();
            this->cloud16_1->resize(cloudData.width() * cloudData.height());

            memcpy(this->cloud16_1->points.data(), cloudData.pointsdata().data(), cloudData.pointsdata().size());

            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_trans(new pcl::PointCloud<pcl::PointXYZI>);

            // transform_xy(1, 3) = -0.39558;
            // transform_xy(2, 3) = 0.18834;
            // transform_xy(1, 3) = -0.39558;
            // transform_xy(2, 3) = -0.18834;

            pcl::transformPointCloud(*(this->cloud16_1), *cloud_trans, transform);

            this->cloud16_1->swap(*cloud_trans);

            this->mtxCloud16_1.unlock();

            auto end = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        }
    }

    // 通过ZMQ获取中间速腾雷达的driver数据
    void recvPointCloud128()
    {
        YAML::Node config;
        config = YAML::LoadFile("../config/perception.yaml");
        // 从yaml文件中读取标定参数
        std::vector<std::vector<float>> transform_vector = config["Tranform128"].as<std::vector<std::vector<float>>>();
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        for (int i = 0; i < 4; ++i)
        {
            for (int j = 0; j < 4; ++j)
            {
                transform(i, j) = transform_vector[i][j];
            }
        }
        while (true)
        {
            CloudProto::CloudData cloudData;

            zmq_msg_t msg;
            zmq_msg_init(&msg);
            int size = zmq_msg_recv(&msg, this->subPointCloud128, 0);

            if (size == -1)
                continue;

            void *recvStr = malloc(size);
            memcpy(recvStr, zmq_msg_data(&msg), size);
            zmq_msg_close(&msg);

            cloudData.ParseFromArray(recvStr, size);

            free(recvStr);

            this->mtxCloud128.lock();
            this->cloud128->width = cloudData.width();
            this->cloud128->height = cloudData.height();
            this->cloud128->is_dense = cloudData.isdense();
            this->cloud128->resize(cloudData.width() * cloudData.height());

            memcpy(this->cloud128->points.data(), cloudData.pointsdata().data(), cloudData.pointsdata().size());

            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_trans(new pcl::PointCloud<pcl::PointXYZI>);

            auto start = std::chrono::steady_clock::now();
            float temp_x = 0.0;
            float temp_y = 0.0;
            float temp_z = 0.0;
            for (int i = 0; i < this->cloud128->points.size(); i++)
            {
                temp_x = 0.934566020966 * this->cloud128->points.at(i).x + -0.005142990965 * this->cloud128->points.at(i).y + 0.355752527714 * this->cloud128->points.at(i).z + 0.75;
                temp_y = -0.005142990965 * this->cloud128->points.at(i).x + 0.999595761299 * this->cloud128->points.at(i).y + 0.027961509302 * this->cloud128->points.at(i).z;
                temp_z = -0.355752527714 * this->cloud128->points.at(i).x + -0.027961509302 * this->cloud128->points.at(i).y + 0.934161782265 * this->cloud128->points.at(i).z + 1.2;
                this->cloud128->points.at(i).x = temp_x;
                this->cloud128->points.at(i).y = temp_y;
                this->cloud128->points.at(i).z = temp_z;
            }
            // pcl::transformPointCloud(*(this->cloud128), *cloud_trans, transform);

            auto end = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

            // this->cloud128->swap(*cloud_trans);

            this->mtxCloud128.unlock();
        }
    }
    void recvImu()
    {
        while (1)
        {
            // 记录开始时间
            auto start = std::chrono::steady_clock::now();
            std::cout << "**********************************" << std::endl;
            // =======do your works here======
            // 使用ZMQ（ZeroMQ）消息队列库中的zmq_msg_recv函数从名为"subState"的消息队列中接收数据。
            // 接收到的数据被存储在zmq_msg_t类型的stateBufMsg中，并通过zmq_msg_data函数获取数据的大小和内容
            zmq_msg_t stateBufMsg;
            zmq_msg_init(&stateBufMsg);
            int stateSize = zmq_msg_recv(&stateBufMsg, socketSubMap["subState"], 0);

            // 如果接收到的数据大小为-1，说明接收到了错误数据，打印错误信息并继续下一次循环
            if (stateSize == -1)
            {
                std::cout << "!!!Data Error" << std::endl;
                continue;
            }

            // 创建一个与接收到的数据大小相同的缓冲区str_recv，并通过memcpy函数将接收到的数据复制到str_recv中
            void *str_recv = malloc(stateSize);
            memcpy(str_recv, zmq_msg_data(&stateBufMsg), stateSize); // copy recived data from zmq msg to str_recv.

            // //创建一个IMU::Imu类型的proto对象vehStateProto，调用其ParseFromArray函数将str_recv中的数据解析为proto对象
            IMU::Imu vehStateProto;
            vehStateProto.ParseFromArray(str_recv, stateSize);
            // State state;
            //   state.x = vehStateProto.gaussx();
            //     state.y = vehStateProto.gaussy();
            //     state.yaw = vehStateProto.yaw() / 180.0 * M_PI; //将其转换为弧度制
            //     state.rtkMode = vehStateProto.gpsvalid();
            //     state.v = vehStateProto.velocity();
            // state12.push_back(state);

            this->lidarObjectDetector->imu_x = vehStateProto.gaussx();
            this->lidarObjectDetector->imu_y = vehStateProto.gaussy();
            this->lidarObjectDetector->imu_yaw = vehStateProto.yaw() / 180.0 * M_PI; // 将其转换为弧度制
            this->lidarObjectDetector->imu_rtkMode = vehStateProto.gpsvalid();
            this->lidarObjectDetector->imu_v = vehStateProto.velocity();
            // std::cout <<  "-------------------@@@-------------" << std::endl;
            // printf("state received (Original): x %.2f, y %.2f, yaw(rad) %.2f, v(m/s) %.1f, rtkMode %.0f \n",
            //     state.x, state.y, state.yaw, state.v, state.rtkMode);
            free(str_recv);
        }
    }

    void processCloudPoints()
    {
        std::pair<std::vector<int>, std::vector<Frame>> DualArray;
        std::vector<int> Pos_array;
        std::vector<Frame> Obstacle_array;
        // 读取yaml文件参数
        YAML::Node config;
        config = YAML::LoadFile("../config/perception.yaml");
        int SavePCD = config["SavePCD"].as<int>();
        std::string SavePCDFile = config["SavePCDFile"].as<std::string>();
        int ReadPCD = config["ReadPCD"].as<int>();
        std::string ReadPCDFile = config["ReadPCDFile"].as<std::string>();
        double map_resolution = config["map_resolution"].as<double>();
        double GridSize = config["GridSize"].as<double>();
        double PassThroughFilter_x_min = config["PassThroughFilter_x_min"].as<double>();
        double PassThroughFilter_y_min = config["PassThroughFilter_y_min"].as<double>();

        int origin_in_grid_x = -PassThroughFilter_x_min / GridSize;
        int origin_in_grid_y = -PassThroughFilter_y_min / GridSize;

        // 在yaml文件中修改ReadPCD和PCDFile
        std::string folder_path = ReadPCDFile; // pcds
        std::vector<std::string> file_paths;
        if (ReadPCD == 0)
        {
            std::cout << "不读取PCD文件" << std::endl;
        }
        else
        {
            boost::filesystem::directory_iterator end_itr;
            for (boost::filesystem::directory_iterator itr(folder_path); itr != end_itr; ++itr)
            {
                if (boost::filesystem::is_regular_file(itr->path()) && itr->path().extension() == ".pcd")
                {
                    file_paths.push_back(itr->path().string());
                }
            }
            std::sort(file_paths.begin(), file_paths.end(),
                      [](const std::string &a, const std::string &b)
                      {
                          std::string numberA, numberB;
                          size_t startPosA = a.find_last_of('/');
                          size_t startPosB = b.find_last_of('/');

                          if (startPosA != std::string::npos)
                          {
                              numberA = a.substr(startPosA + 1);
                          }

                          if (startPosB != std::string::npos)
                          {
                              numberB = b.substr(startPosB + 1);
                          }

                          return std::stoi(numberA) < std::stoi(numberB);
                      });
        }

        int pcd_file_index = 0;
        Eigen::Matrix4f transform_xz = Eigen::Matrix4f::Identity();
        // transform_xz(0, 3) = 0.75;
        // transform_xz(2, 3) = 1.2;
        while (1)
        {
            durationFroShowofSub = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - lastTheadTimeofSub);
            lastTheadTimeofSub = std::chrono::steady_clock::now();

            auto start = std::chrono::steady_clock::now();
            auto end = std::chrono::steady_clock::now();

            if (FLAG_lidarObjectDetection == false)
            {
                pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

                // this->mtxCloud16_0.lock();
                // *(cloud) = *(cloud) + *(this->cloud16_0);
                // this->mtxCloud16_0.unlock();

                // this->mtxCloud16_1.lock();
                // *(cloud) = *(cloud) + *(this->cloud16_1);
                // this->mtxCloud16_1.unlock();

                this->mtxCloud128.lock();
                *(cloud) = *(cloud) + *(this->cloud128);
                this->mtxCloud128.unlock();

                // float temp_x = 0.0;
                // float temp_z = 0.0;

                // for (int i = 0; i < cloud->points.size(); i++)
                // {
                //     temp_x = cloud->points.at(i).x + 0.96;

                //     temp_z = cloud->points.at(i).z + 1.76;
                //     cloud->points.at(i).x = temp_x;

                //     cloud->points.at(i).z = temp_z;
                // }
                pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_trans_xz(new pcl::PointCloud<pcl::PointXYZI>);
                // pcl::transformPointCloud(*cloud, *cloud_trans_xz, transform_xz);
                // cloud->swap(*cloud_trans_xz);
                auto end = std::chrono::steady_clock::now();
                auto duration_trans = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
                std::cout << "duration_trans:" << duration_trans.count() << " ms" << std::endl;
                // 在yaml文件中修改SavePCD和SavePCDFile
                if (cloud->points.size() > 0 && SavePCD == 1)
                {
                    auto start_save = std::chrono::steady_clock::now();
                    pcl::io::savePCDFile<pcl::PointXYZI>(SavePCDFile + std::to_string(index) + ".pcd", *cloud);
                    auto end_save = std::chrono::steady_clock::now();
                    auto duration_save = std::chrono::duration_cast<std::chrono::milliseconds>(end_save - start_save);
                    std::cout << "保存点云文件至" << SavePCDFile + std::to_string(index) + ".pcd"
                              << " 耗时" << duration_save.count() << "ms" << std::endl;
                    index++;
                }

                std::cout << "*****开始点云处理*****" << std::endl;

                // 在yaml文件中修改ReadPCD
                if (ReadPCD)
                {
                    start = std::chrono::steady_clock::now();
                    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudData(new pcl::PointCloud<pcl::PointXYZI>());
                    std::string file_name = file_paths[pcd_file_index];
                    std::string file_path = file_name;
                    char strfilepath[file_paths.size()];
                    strcpy(strfilepath, file_path.c_str());
                    if (-1 == io::loadPCDFile(strfilepath, *cloudData))
                    { // 读取.pcd文件
                        PCL_ERROR("无法读取文件/n");
                    }
                    perception::ParamStruct08 param08 = {this->lidarObjectDetector};
                    perception::InputStruct08 input08 = {cloud};
                    perception::OutputStruct08 output08;
                    classicLidarDetectorReceiveCloud(param08, input08, output08);
                    this->lidarObjectDetector = param08.c;

                    // this->lidarObjectDetector->receiveCloud(cloudData);
                    auto end = std::chrono::steady_clock::now();
                    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
                    ++pcd_file_index;
                    // 总是读第一个点云
                    // pcd_file_index;
                    std::cout << "读取" << file_paths[pcd_file_index] << " 耗时" << duration.count() << "ms" << std::endl;
                }
                else
                {
                    // PointCloudsDetector c = this->lidarObjectDetector;
                    perception::ParamStruct08 param08 = {this->lidarObjectDetector};
                    perception::InputStruct08 input08 = {cloud};
                    perception::OutputStruct08 output08;
                    classicLidarDetectorReceiveCloud(param08, input08, output08);
                    // this->lidarObjectDetector->receiveCloud(cloud);
                    this->lidarObjectDetector = param08.c;
                }
                // ParamStruct23 param23 = {this->lidarObjectDetector};
                // InputStruct23 input23 = {};
                // OutputStruct23 output23;
                // classicLidarDetectoRun(param23, input23, output23);
                // DualArray = output23.output;

                // 将run打开
                auto start_1 = std::chrono::steady_clock::now();
                auto end_1 = std::chrono::steady_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_1 - start_1);

                std::vector<int> Pos_array_run;
                std::vector<Frame> Obstacle_array_run;
                std::cout << "in run" << std::endl;
                YAML::Node config;
                config = YAML::LoadFile("../config/perception.yaml");
                double map_resolution = config["map_resolution"].as<double>();
                double z_down = config["PassThroughFilter_z_down"].as<double>();
                double z_up = config["PassThroughFilter_z_up"].as<double>();
                double PassThroughFilter_z_down = config["PassThroughFilter_z_down0"].as<double>();
                double PassThroughFilter_z_up = config["PassThroughFilter_z_up0"].as<double>();
                double PassThroughFilter_x_min = config["PassThroughFilter_x_min"].as<double>();
                double PassThroughFilter_x_max = config["PassThroughFilter_x_max"].as<double>();
                double PassThroughFilter_y_min = config["PassThroughFilter_y_min"].as<double>();
                double PassThroughFilter_y_max = config["PassThroughFilter_y_max"].as<double>();
                double DownsampleLeafSize_xy = config["DownsampleLeafSize_xy"].as<double>();
                double DownsampleLeafSize_z = config["DownsampleLeafSize_z"].as<double>();
                int NormalEatimationKSearch = config["NormalEatimationKSearch"].as<int>();
                double SlopeRate = config["SlopeRate"].as<double>();
                double NormalThereshold = config["NormalThereshold"].as<double>();
                int GridHeightComputeMode = config["GridHeightComputeMode"].as<int>();
                double GroundHeight = config["GroundHeight"].as<double>();
                double clusterDis = config["clusterDis"].as<double>();
                double trackingDis = config["trackingDis"].as<double>();
                double GridSize = config["GridSize"].as<double>();
                g_mtxCloud.lock();
                // 获取输入点云
                pcl::PointCloud<pcl::PointXYZI>::Ptr t_inputCloud;

                t_inputCloud = this->lidarObjectDetector->input_cloud;
                g_mtxCloud.unlock();
                if (t_inputCloud->points.empty())
                {
                    DualArray = std::make_pair(Pos_array_run, Obstacle_array_run);
                }
                else
                {
                    auto startTime_ = std::chrono::high_resolution_clock::now();
                    // 裁剪自车
                    typename pcl::CropBox<pcl::PointXYZI> croped;
                    croped.setInputCloud(t_inputCloud);
                    croped.setMin(Eigen::Vector4f(-1, -12, -10, 1));
                    croped.setMax(Eigen::Vector4f(1.3, 12, 10, 1));
                    croped.setNegative(true);
                    croped.filter(*t_inputCloud);

                    // 点云预处理(裁剪和降采样)
                    // pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud =
                    //     this->lidarObjectDetector->filterCloud(t_inputCloud, DownsampleLeafSize_xy, DownsampleLeafSize_xy, DownsampleLeafSize_z, Eigen::Vector4f(PassThroughFilter_x_min,
                    //     PassThroughFilter_y_min, PassThroughFilter_z_down, 1),
                    //                 Eigen::Vector4f(PassThroughFilter_x_max, PassThroughFilter_y_max, PassThroughFilter_z_up, 1));
                    perception::ParamStruct16 param16 = {this->lidarObjectDetector};
                    perception::InputStruct16 input16 = {t_inputCloud,
                                             DownsampleLeafSize_xy,
                                             DownsampleLeafSize_xy,
                                             DownsampleLeafSize_z,
                                             Eigen::Vector4f(PassThroughFilter_x_min, PassThroughFilter_y_min, PassThroughFilter_z_down, 1),
                                             Eigen::Vector4f(PassThroughFilter_x_max, PassThroughFilter_y_max, PassThroughFilter_z_up, 1)};
                    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud;
                    perception::OutputStruct16 output16 = {filtered_cloud};
                    classicLidarDetectorFilterCloud(param16, input16, output16);
                    this->lidarObjectDetector = param16.c;
                    filtered_cloud = output16.output;
                    // 降采样输出的点云是 filtered_cloud
                    // 打印降采样的耗时
                    auto endTime_preprocess = std::chrono::high_resolution_clock::now();
                    auto duration_preprocess = std::chrono::duration_cast<std::chrono::milliseconds>(endTime_preprocess - startTime_).count();
                    std::cout << "预处理——总耗时:" << duration_preprocess << " ms" << std::endl;
                    std::cout << "点云的个数：" << filtered_cloud->size() << std::endl;

                    auto startTime_groudfilter = std::chrono::high_resolution_clock::now();
                    // 去除地面
                    // Pos_array_run = this->lidarObjectDetector->GridGroundFilterSimple(filtered_cloud, NormalEatimationKSearch, SlopeRate, NormalThereshold, GridHeightComputeMode,
                    // GroundHeight,GridSize, PassThroughFilter_y_max, -PassThroughFilter_x_min, PassThroughFilter_x_max);

                    // GridGroundFilterSimple
                    // ParamStruct20 param20 = {this->lidarObjectDetector};
                    // InputStruct20 input20 = {filtered_cloud,           NormalEatimationKSearch, SlopeRate, NormalThereshold, GridHeightComputeMode, GroundHeight, GridSize, PassThroughFilter_y_max,
                    //                          -PassThroughFilter_x_min, PassThroughFilter_x_max};
                    // OutputStruct20 output20 = {Pos_array_run};
                    // classicLidarDetectorGridGroundFilterSimple(param20, input20, output20);
                    // this->lidarObjectDetector = param20.c;
                    // Pos_array_run = output20.output;

                    // RANSAC
                    float ransac_threshold = 0.3;
                    perception::ParamStruct25 param25 = {this->lidarObjectDetector};
                    perception::InputStruct25 input25 = {filtered_cloud,          ransac_threshold,        NormalEatimationKSearch, SlopeRate, NormalThereshold, GridHeightComputeMode, GroundHeight, GridSize,
                                             PassThroughFilter_y_max, PassThroughFilter_x_min, PassThroughFilter_x_max};
                                             perception::OutputStruct25 output25 = {Pos_array_run};
                    classicLidarDetectoGridRANSAC(param25, input25, output25);
                    this->lidarObjectDetector = param25.c;
                    Pos_array_run = output25.output;

                    // Patchwork
                    // ParamStruct26 param26 = {this->lidarObjectDetector};
                    // InputStruct26 input26 = {filtered_cloud,           NormalEatimationKSearch, SlopeRate, NormalThereshold, GridHeightComputeMode, GroundHeight, GridSize, PassThroughFilter_y_max,
                    //                          -PassThroughFilter_x_min, PassThroughFilter_x_max};
                    // OutputStruct26 output26 = {Pos_array_run};
                    // classicLidarDetectoGridPatch(param26, input26, output26);
                    // this->lidarObjectDetector = param26.c;
                    // Pos_array_run = output26.output;

                    // 去除地面后 地面点云存储在全局变量'ground_point'当中
                    //           非地面点云存储在全局变量'nonground_point'当中
                    // 打印去地面时间
                    auto endTime_groundfilter = std::chrono::high_resolution_clock::now();
                    auto duration_groundfilter = std::chrono::duration_cast<std::chrono::milliseconds>(endTime_groundfilter - startTime_groudfilter).count();
                    std::cout << "去地面处理时间:" << duration_groundfilter << " ms" << std::endl;
                    g_mtxViewer.lock();
                    this->lidarObjectDetector->viewer->removeAllPointClouds();
                    this->lidarObjectDetector->viewer->removeAllShapes();
                    this->lidarObjectDetector->viewer->removeAllCoordinateSystems();
                    if (1)
                    {
                        perception::ParamStruct10 param10_1 = {this->lidarObjectDetector};
                        perception::InputStruct10 input10_1 = {this->lidarObjectDetector->ground_point, "cloud_origin", Color(1, 1, 1)};
                        perception::OutputStruct10 output10_1;
                        classicLidarDetectorRenderPointCloud(param10_1, input10_1, output10_1);
                        this->lidarObjectDetector = param10_1.c;
                        // this->lidarObjectDetector->renderPointCloud(this->lidarObjectDetector->ground_point, "cloud_origin", Color(1, 1, 1));
                        perception::ParamStruct10 param10_2 = {this->lidarObjectDetector};
                        perception::InputStruct10 input10_2 = {this->lidarObjectDetector->nonground_point, "cloud", Color(0, 1, 0)};
                        perception::OutputStruct10 output10_2;
                        classicLidarDetectorRenderPointCloud(param10_2, input10_2, output10_2);
                        this->lidarObjectDetector = param10_2.c;
                        // this->lidarObjectDetector->renderPointCloud(this->lidarObjectDetector->nonground_point, "cloud", Color(0, 1, 0));
                    }

                    std::cout << "test!!!!!!!!!!" << std::endl;

                    g_mtxViewer.unlock();
                    std::cout << "test----------" << std::endl;
                    end = std::chrono::steady_clock::now();
                    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
                    cout << "duration process: " << duration.count() << endl;
                    DualArray = std::make_pair(Pos_array_run, Obstacle_array_run);
                }
                // run打开结束

                // DualArray = this->lidarObjectDetector->run();

                Pos_array = DualArray.first;
                Obstacle_array = DualArray.second;
                std::cout << "*****结束点云处理*****" << std::endl;
            }
            else
            {
                cout << "Lidar Detection Fail" << endl;
            }

            if ((Pos_array.size() == 0))
            {
                continue;
            }
            std::cout << "发送数据" << std::endl;
            auto startTime_pub = std::chrono::high_resolution_clock::now();

            prediction::ObjectList msg1;
            prediction::Object *msg2;
            prediction::PredictPoint *msg3;

            int count_obstacle = 0;

            for (int i = 0; i < Pos_array.size(); i += 3)
            {
                msg2 = msg1.add_object();
                msg2->set_h(Pos_array[i + 2]);
                msg2->set_w(GridSize);
                msg2->set_l(GridSize);
                msg3 = msg2->add_predictpoint();
                msg3->set_x((Pos_array[i] - origin_in_grid_x) * GridSize + 0.5 * GridSize);
                msg3->set_y((Pos_array[i + 1] - origin_in_grid_y) * GridSize + 0.5 * GridSize);
                ++count_obstacle;
            }

            // for (int i = 1; i <= (Pos_array.size() - 2) / 2; i++)
            // // for (int i = 1; i <= 10; i++)
            // {
            //     msg2 = msg1.add_object();
            //     msg2->set_w(map_resolution);
            //     msg2->set_l(map_resolution);
            //     msg3 = msg2->add_predictpoint();
            //     // msg3->set_y(-((Pos_array[(2 * i + 1)] * map_resolution + 0.5 * map_resolution) - 0.5 * Pos_array[1]));
            //     //  msg3->set_x((Pos_array[(2 * i)] * map_resolution + 0.5 * map_resolution) - 0.5 * Pos_array[0]);
            //     msg3->set_x(((Pos_array[(2 * i + 1)] * map_resolution + 0.5 * map_resolution) - 0.5 * Pos_array[1]));
            //     msg3->set_y(((Pos_array[(2 * i)] * map_resolution + 0.5 * map_resolution) - 0.5 * Pos_array[0]));
            // }

            auto endTime_pub = std::chrono::high_resolution_clock::now();

            std::cout << "planning count: " << count_obstacle << std::endl;

            size_t predSize = msg1.ByteSize();
            cout << "predSize" << predSize << endl;
            void *predBuf = malloc(predSize);
            if (!msg1.SerializeToArray(predBuf, predSize))
            {
                std::cerr << "Failed to write msg." << std::endl;
            }

            // 发送
            zmq_send(this->pubObjectList, predBuf, predSize, 0);
            free(predBuf);

            // 发送unity数据
            // cout << "\033[31m"
            //      << "predSize:"
            //      << "\033[0m" << predSize << endl;
            prediction::ObjectList msg4;
            prediction::Object *msg5;
            prediction::PredictPoint *msg6;

            int count__ = 0;
            for (int i = 0; i < Pos_array.size(); i += 3)
            // for (int i = 1; i <= 10; i++)
            {
                count__++;
                msg5 = msg4.add_object();
                msg5->set_h(Pos_array[i + 2]);
                msg5->set_w(GridSize);
                msg5->set_l(GridSize);
                msg6 = msg5->add_predictpoint();
                msg6->set_x((Pos_array[i] - origin_in_grid_x) * GridSize + 0.5 * GridSize);
                msg6->set_y((Pos_array[i + 1] - origin_in_grid_y) * GridSize + 0.5 * GridSize);
            }
            std::cout << "unity count: " << count__ << std::endl;
            size_t predSize_unity = msg4.ByteSize();
            cout << "predSize_unity:" << predSize_unity << endl;
            void *predBuf_unity = malloc(predSize_unity);
            if (!msg4.SerializeToArray(predBuf_unity, predSize_unity))
            {
                std::cerr << "Failed to write msg." << std::endl;
            }

            // 发送
            zmq_send(this->pubObjectList_unity, predBuf_unity, predSize_unity, 0);
            free(predBuf_unity);
            end = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
            std::this_thread::sleep_for(std::chrono::milliseconds(100 - duration.count()));

            if ((Obstacle_array.size() == 0))
            {
                continue;
            }

            // 发送聚类信息
            prediction::ObjectList msg7;
            prediction::Object *msg8;
            prediction::PredictPoint *msg9;
            for (int i = 1; i < Obstacle_array.size(); i++)
            // for (int i = 1; i <= 10; i++)
            {
                msg8 = msg7.add_object();
                msg8->set_z(Obstacle_array[i].time.lidar_z);
                msg8->set_w(Obstacle_array[i].time.w);
                msg8->set_h(Obstacle_array[i].time.h);
                msg8->set_l(Obstacle_array[i].time.l);
                msg8->set_type(Obstacle_array[i].time.type);
                msg8->set_trackid(Obstacle_array[i].time.objectId);
                msg9 = msg8->add_predictpoint();
                msg9->set_x(Obstacle_array[i].time.x);
                msg9->set_y(Obstacle_array[i].time.y);
                msg9->set_vx(Obstacle_array[i].time.vel);
                msg9->set_vy(Obstacle_array[i].time.yaw);
            }

            size_t predSize_cluster = msg7.ByteSize();
            cout << "predSize_cluster:" << predSize_cluster << endl;
            void *predBuf_cluster = malloc(predSize_cluster);
            if (!msg7.SerializeToArray(predBuf_cluster, predSize_cluster))
            {
                std::cerr << "Failed to write msg." << std::endl;
            }

            // 发送
            zmq_send(this->pubObjectList_cluster, predBuf_cluster, predSize_cluster, 0);
            free(predBuf_cluster);

            end = std::chrono::steady_clock::now();
            auto duration_cluster = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
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
};

int main()
{
    ThreadJobs threadJob;
    pcl::visualization::PCLVisualizer::Ptr mainViewer;
    mainViewer = threadJob.getViewer();
    // mainViewer->spinOnce();
    thread thread1(&ThreadJobs::recvPointCloud128, &threadJob);
    thread thread2(&ThreadJobs::recvPointCloud16_0, &threadJob);
    thread thread3(&ThreadJobs::recvPointCloud16_1, &threadJob);
    thread thread4(&ThreadJobs::processCloudPoints, &threadJob);
    thread thread5(&ThreadJobs::recvImu, &threadJob);

    thread::id threadID4 = thread4.get_id();

    thread1.detach();
    thread2.detach();
    thread3.detach();
    thread4.detach();
    thread5.detach();

    while (1)
    {
        //=========== do what you want in main===============

        g_mtxViewer.lock();

        mainViewer->spinOnce();
        // mainViewer->removeAllPointClouds();
        //  mainViewer->removeAllShapes();
        //  mainViewer->removeAllCoordinateSystems();
        //  std::this_thread::sleep_for(std::chrono::milliseconds(100));
        g_mtxViewer.unlock();
        // int thread1Status = pthread_kill(cvtThreadId2Long(threadID1), 0);
        // checkThreadStatus(thread1Status, threadID1);

        // int thread2Status = pthread_kill(cvtThreadId2Long(threadID2), 0);
        // checkThreadStatus(thread2Status, threadID2);
        std::this_thread::sleep_for(std::chrono::milliseconds(60));
    }
}
