/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2023-07-22 14:18:53
 * @LastEditors: wangzhenhui 965127065@qq.com
 * @LastEditTime: 2023-08-29 15:12:57
 * @FilePath: /perceptionLidarObjectDetection_128_zl(together)/perceptionLidarObjectDetection_128_zl/include/baseLidarObjectDetector.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef _LIDAR_BASE_OBJECT_ 
#define _LIDAR_BASE_OBJECT_
#include <iostream>
#include <zmq.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/plane_clipper3D.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <mutex>

#include <perception.pb.h>

using namespace std;
struct Frame
{
    time_t time_stamp; // 时间戳ms
    int frameId;       // 点云帧id
    int ld;            // 计数器（聚类到的物体个数）
    double imu_x;      // 自车高斯坐标x
    double imu_y;      // 自车高斯坐标y
    double imu_yaw;    // 自车航向角rad
    struct Time
    {
        int timeld;   // 当前帧点云下的第几个物体
        int objectId; // 物体跟踪id(新增的id为计数器当前的值)
        int type;     // 物体类型，0汽车，1卡车，2行人，3自行车, -1未知
        double x;     // 物体中心的高斯坐标x
        double y;     // 物体中心的高斯坐标y
        double z;     // 物体中心的高斯坐标z
        double yaw;   // 物体的航向角rad   Guass
        double vel;   // 速度

        double lidar_x;   // 物体中心的lidar坐标x
        double lidar_y;   // 物体中心的lidar坐标y
        double lidar_z;   // 物体中心的lidar坐标z
        double lidar_yaw; // 物体yaw

        double w; // 物体的宽
        double l; // 物体的长
        double h; // 物体的高
    } time;
};
extern std::mutex g_mtxViewer;
extern std::mutex g_mtxObjList;
extern std::mutex g_mtxCloud;

class LidarBaseObjectDetector
{
public:
    // pc::PointVec input;
    perception::ObjectList objResultList;
    pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud;

    double imu_x;       // 高斯x
    double imu_y;       // 高斯y
    double imu_yaw;     // 横摆角，航向角
    double imu_v;       // 车辆当前速度
    double imu_rtkMode; // IMU的定位状态
    void *publisher;
    LidarBaseObjectDetector() : input_cloud(new pcl::PointCloud<pcl::PointXYZI>)
    {
    }
    virtual ~LidarBaseObjectDetector() {}
    virtual std::pair<std::vector<int>, std::vector<Frame> > run() = 0;
    virtual void receiveCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr input) = 0;
    virtual pcl::visualization::PCLVisualizer::Ptr getViewer() = 0;

    // virtual bool display();
    bool sendResultInit();
    void sendResult();
};
#endif
