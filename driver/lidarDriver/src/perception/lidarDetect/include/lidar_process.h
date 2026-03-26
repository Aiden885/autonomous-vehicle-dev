#ifndef LIDAR_PROCESS_H
#define LIDAR_PROCESS_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <chrono>
#include <iostream>
#include <string>
#include <unordered_set>
#include <vector>

#include "PtCloudCommunicator.h"

using namespace std;

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

enum CameraAngle
{
    XY,
    TopDown,
    Side,
    FPS
};

template <typename PointT>
class PointCloudsDetector
{
  private:
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr;
    pcl::visualization::PCLVisualizer::Ptr viewer;

    PtCloudCommunicator communicator;

  public:
    PointCloudsDetector();
    ~PointCloudsDetector();

    void initCamera(CameraAngle setAngle);
    void InitView();
    void renderPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, std::string name, Color color);
    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);
    void renderBox(Box box, int id, Color color = Color(1, 0, 0), float opacity = 1);

    typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud,
                                                      float filterRes, Eigen::Vector4f minPoint,
                                                      Eigen::Vector4f maxPoint);
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>
    RANSAC3d(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);
    std::vector<typename pcl::PointCloud<PointT>::Ptr>
    Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

    // Run callback function.
    void recvPointCloud();
    void detect();
    void run();
};

#endif /* LIDAR_PROCESS_H */
