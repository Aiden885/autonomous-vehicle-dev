/*
 *
 * Created on: Aug 25, 2022 15:03
 * Description:
 *
 * Copyright (c) 2022 YH. WANG
 */

#ifndef LIDAR_PROCESS_H
#define LIDAR_PROCESS_H 

// #include <Eigen/Core>
// #include <Eigen/Dense>
#include <experimental/filesystem>
#include <pcl/kdtree/kdtree_flann.h>

#include "baseLidarObjectDetector.h"
#include "patchworkpp.h"
#include "yaml-cpp/yaml.h"
#include <chrono>
#include <ctime>
#include <fstream>
#include <iostream>
#include <cloudData.pb.h>
#include <mutex>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/plane_clipper3D.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <perception.pb.h>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

using namespace std;
using namespace pcl;
using namespace io;
using namespace cv;

//==========================
// some struct here
//==========================
struct Color
{

    float r, g, b;

    Color(float setR, float setG, float setB) : r(setR), g(setG), b(setB) {}
};

struct Vect3
{

    double x, y, z;

    Vect3(double setX, double setY, double setZ) : x(setX), y(setY), z(setZ) {}

    Vect3 operator+(const Vect3 &vec)
    {
        Vect3 result(x + vec.x, y + vec.y, z + vec.z);
        return result;
    }
};

struct Box
{
    float x_min;
    float y_min;
    float z_min;
    float x_max;
    float y_max;
    float z_max;
};

// struct State
// {
//     double x;       //高斯x
//     double y;       //高斯y
//     double yaw; //横摆角，航向角
//     double v;       //车辆当前速度
//     double rtkMode;     //IMU的定位状态
// };
// std::vector<State> state12;
struct PcaBox
{
    // Eigen::Quaternionf bboxQuaternion;
    // Eigen::Vector3f bboxTransform;
    Eigen::Vector3f center;
    Eigen::Quaternionf rotation;
    float boxLength;
    float boxWidth;
    float boxHeight;

    pcl::PointXYZI minPoint;
    pcl::PointXYZI maxPoint;
};

enum CameraAngle
{
    XY,
    TopDown,
    Side,
    FPS
};

// ======================class here===========================

class PointCloudsDetector : public LidarBaseObjectDetector
{
  private:
    // pcl related

    // pcl::PointCloud<pcl::PointXYZI> input;

  public:
    PointCloudsDetector();
    ~PointCloudsDetector();
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    void find_Z_value(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    pcl::PointCloud<pcl::PointXYZI>::Ptr PassThroughFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, double z_max, double z_min);
    vector<int> Pointcloud_to_grid(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, double map_resolution);
    double max_z;
    double min_z;
    int wi = 0;
    int frame_id = 0;
    int time_id = 0;
    int id = 0;

    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_point;
    pcl::PointCloud<pcl::PointXYZI>::Ptr nonground_point;

    struct GridInfos
    {
        int point_num = 0;
        int isvisited = 0;
        int property = 0;
        float mean_height = -1000.0;
        float max_height = -1000.0;
        float min_height = 1000.0;
        float plane_height = -1000.0;
        float normal_vector_x = 0.0;
        float normal_vector_y = 0.0;
        float normal_vector_z = 0.0;
        std::vector<int> point_indices;
        std::vector<int> ground_indices;
        std::vector<int> nonground_indices;
    };

    std::vector<Frame> frames;

    pcl::PointCloud<pcl::PointXYZI>::Ptr inputcloud(int i);

    void initCamera(CameraAngle setAngle);
    void initView();
    bool checkAlive();
    void receiveCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr input) override;
    // void ReadFile();
    void rPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud);
    void renderPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, std::string name, Color color);
    Box setBoundingBox(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster);
    PcaBox setPcaBox(Frame frame);

    Frame object(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster, double trackingDis);
    std::vector<Frame> ObstacleBounding(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusterCloud, double trackingDis);

    void renderBox(Box box, int id, Color color = Color(0, 1, 0), float opacity = 1);
    void renderPcaBox(PcaBox box, int id, Color color = Color(0, 1, 0), float opacity = 1);

    void swapXAndY(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud); //
    void swapYAndX(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud); //

    void numPoints(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

    pcl::visualization::PCLVisualizer::Ptr getViewer() override;

    pcl::PointCloud<pcl::PointXYZI>::Ptr clipPlane(pcl::PointCloud<pcl::PointXYZI>::Ptr src_cloud, const Eigen::Vector4f plane, bool negative);

    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float filterRes_x, float filterRes_y, float filterRes_z, Eigen::Vector4f minPoint,
                                                     Eigen::Vector4f maxPoint);

    pcl::PointCloud<pcl::PointXYZI>::Ptr rmvNearPoints(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

    pcl::PointCloud<pcl::PointXYZI>::Ptr RANSAC3d(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int maxIterations, float distanceThreshold);

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> Clustering(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);
    std::vector<int> GridGroundFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud, int normal_estimation_Ksearch, float slope_rate, float normal_thereshold, int grid_height_mode,
                                      float car_height, float grid_size, float right_left_distance, float back_distance,
                                      float front_distance); // 定义车周地面高度
    std::vector<int> GridGroundFilterSimple(const pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud, int normal_estimation_Ksearch, float slope_rate, float normal_thereshold, int grid_height_mode,
                                            float car_height, float grid_size, float right_left_distance, float back_distance,
                                            float front_distance); // 定义车周地面高度
    std::vector<int> GridRANSAC(const pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud, float ransac_threshold, int normal_estimation_Ksearch, float slope_rate, float normal_thereshold,
                                int grid_height_mode, float car_height, float grid_size, float right_left_distance, float back_distance,
                                float front_distance); // 定义车周地面高度
    void read_pcd_to_eigen(pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloudi, Eigen::MatrixXf &cloud, float right_left_distance, float back_distance, float front_distance);
    void processPointClouds(pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloudi, float right_left_distance, float back_distance, float front_distance);
    std::vector<int> GridPatchworkpp(pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloudi, int normal_estimation_Ksearch, float slope_rate, float normal_thereshold, int grid_height_mode,
                                     float car_height,          // 定义车周地面高度
                                     float grid_size,           // 栅格大小
                                     float right_left_distance, // 左或右距离
                                     float back_distance,       // 后方距离
                                     float front_distance);     // 前方距离
    // Run callback function.
    std::pair<std::vector<int>, std::vector<Frame>> run() override;
    void test();
};

#endif /* LIDAR_PROCESS_H */
