/**
 * @brief 文件说明
 * @file functionLidarDetector.h
 * @version 0.0.1
 * @author Tianqi Ke (ktq23@mails.tsinghua.edu.cn)
 * @date 2023-12-12
 */
#pragma once

#include "classicLidarDetector.h"
#include <cmath>
#include <iostream>
#include <string>

/**
 * \namespace perception
 * \brief perception namespace
 */

namespace perception{

typedef struct
{
    PointCloudsDetector *c;
} ParamStruct01;


typedef struct
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
} InputStruct01;


typedef struct
{

} OutputStruct01;

/**
 * @brief classicLidarDetectorFindZ 函数
 * @param[IN] param1, PointCloudsDetector类的对象的指针
 * @param[IN] input1, 点云指针
 * @param[OUT] output1, 空输出
 
 * @cn_name 点云高度查询
 
 * @granularity atomic
 
 * @tag perception
 */
void classicLidarDetectorFindZ(ParamStruct01 &, InputStruct01 &, OutputStruct01 &);
// void find_Z_value(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);




typedef struct
{
    PointCloudsDetector *c;
} ParamStruct02;


typedef struct
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    double z_max;
    double z_min;
} InputStruct02;


typedef struct
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr output;
} OutputStruct02;

/**
 * @brief classicLidarDetectorPassThroughFilter 函数
 * @param[IN] param2, PointCloudsDetector类的对象的指针
 * @param[IN] input2, 点云指针和z的最大最小值
 * @param[OUT] output2, 点云指针
 
 * @cn_name 直通滤波

 * @granularity atomic

 * @tag perception
 */
void classicLidarDetectorPassThroughFilter(ParamStruct02 &, InputStruct02 &, OutputStruct02 &);
//     pcl::PointCloud<pcl::PointXYZI>::Ptr PassThroughFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, double z_max, double z_min);


typedef struct
{
    PointCloudsDetector *c;
} ParamStruct03;


typedef struct
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    double map_resolution;
} InputStruct03;


typedef struct
{
    vector<int> Pos_array;
} OutputStruct03;

/**
 * @brief classicLidarDetectorPointCloudToGrid 函数
 * @param[IN] param3, PointCloudsDetector类的对象的指针
 * @param[IN] input3, 点云指针和分辨率
 * @param[OUT] 栅格地图
 
 * @cn_name 栅格化

 * @granularity atomic

 * @tag perception
 */
void classicLidarDetectorPointCloudToGrid(ParamStruct03 &, InputStruct03 &, OutputStruct03 &);
//     vector<int> Pointcloud_to_grid(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, double map_resolution);




typedef struct
{
    PointCloudsDetector *c;
} ParamStruct04;


typedef struct
{
    int i;
    // std::string folderPath;
} InputStruct04;


typedef struct
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr output;
} OutputStruct04;

/**
 * @brief classicLidarDetectorInputcloud 函数
 * @param[IN] param4, PointCloudsDetector类的对象的指针
 * @param[IN] input4, 文件编号
 * @param[OUT] output4, 点云指针
 
 * @cn_name pcd文件加载

 * @granularity atomic

 * @tag perception
 */
void classicLidarDetectorInputcloud(ParamStruct04 &, InputStruct04 &, OutputStruct04 &);




typedef struct
{
    PointCloudsDetector *c;
} ParamStruct05;


typedef struct
{
    CameraAngle setAngle;
} InputStruct05;


typedef struct
{

} OutputStruct05;

/**
 * @brief classicLidarDetectorInitCamera 函数
 * @param[IN] param5, PointCloudsDetector类的对象的指针
 * @param[IN] input5, 姿态参数结构体
 * @param[OUT] output5, 空输出
 
 * @cn_name 3D视图的相机位姿初始化

 * @granularity atomic

 * @tag perception
 */
void classicLidarDetectorInitCamera(ParamStruct05 &, InputStruct05 &, OutputStruct05 &);
// void initCamera(CameraAngle setAngle);




typedef struct
{
    PointCloudsDetector *c;
} ParamStruct06;


typedef struct
{

} InputStruct06;


typedef struct
{

} OutputStruct06;

/**
 * @brief classicLidarDetectorInitView 函数
 * @param[IN] param6, PointCloudsDetector类的对象的指针
 * @param[IN] input6, 空输入
 * @param[OUT] output6, 空输出
 
 * @cn_name 3D视图初始化

 * @granularity atomic

 * @tag perception
 */
void classicLidarDetectorInitView(ParamStruct06 &, InputStruct06 &, OutputStruct06 &);
// void initView();





typedef struct
{
    PointCloudsDetector *c;
} ParamStruct07;


typedef struct
{

} InputStruct07;


typedef struct
{
    bool isAlive;
} OutputStruct07;

/**
 * @brief classicLidarDetectorCheckAlive 函数
 * @param[IN] param7, PointCloudsDetector类的对象的指针
 * @param[IN] input7, 空输入
 * @param[OUT] output7, bool类型，判断是否正常工作
 
 * @cn_name 3D视图状态检测

 * @granularity atomic

 * @tag perception
 */
void classicLidarDetectorCheckAlive(ParamStruct07 &, InputStruct07 &, OutputStruct07 &);
// bool checkAlive();




typedef struct
{
    PointCloudsDetector *c;
} ParamStruct08;


typedef struct
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr input;
} InputStruct08;


typedef struct
{

} OutputStruct08;

/**
 * @brief classicLidarDetectorReceiveCloud 函数
 * @param[IN] param8, PointCloudsDetector类的对象的指针
 * @param[IN] input8, 点云指针
 * @param[OUT] output8, 空输出
 
 * @cn_name 点云数据接收

 * @granularity atomic

 * @tag perception
 */
void classicLidarDetectorReceiveCloud(ParamStruct08 &param08, InputStruct08 &input08, OutputStruct08 &output08);
// void receiveCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr input) override;





typedef struct
{
    PointCloudsDetector *c;
} ParamStruct09;


typedef struct
{
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud;
} InputStruct09;


typedef struct
{

} OutputStruct09;

/**
 * @brief classicLidarDetectorRPointCloud 函数
 * @param[IN] param9, PointCloudsDetector类的对象的指针
 * @param[IN] input9, 点云指针
 * @param[OUT] output9, 空输出
 
 * @cn_name 3D视图点云渲染

 * @granularity atomic

 * @tag perception
 */
void classicLidarDetectorRPointCloud(ParamStruct09 &, InputStruct09 &, OutputStruct09 &);
// void rPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud);





typedef struct
{
    PointCloudsDetector *c;
} ParamStruct10;


typedef struct
{
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud;
    std::string name;
    Color color;
} InputStruct10;


typedef struct
{

} OutputStruct10;

/**
 * @brief classicLidarDetectorRenderPointCloud 函数
 * @param[IN] param10, PointCloudsDetector类的对象的指针
 * @param[IN] input10, 渲染参数设置
 * @param[OUT] output10, 空输出
 
 * @cn_name 基于点云强度的渲染

 * @granularity atomic

 * @tag perception
 */
void classicLidarDetectorRenderPointCloud(ParamStruct10 &, InputStruct10 &, OutputStruct10 &);
// void renderPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, std::string name, Color color);





typedef struct
{
    PointCloudsDetector *c;
} ParamStruct11;


typedef struct
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
} InputStruct11;

typedef struct
{

} OutputStruct11;

/**
 * @brief classicLidarDetectorSwapXAndY 函数
 * @param[IN] param11, PointCloudsDetector类的对象的指针
 * @param[IN] input11,  点云输入
 * @param[OUT] output11, 空输出
 
 * @cn_name 点云XY坐标转换

 * @granularity atomic

 * @tag perception
 */
void classicLidarDetectorSwapXAndY(ParamStruct11 &, InputStruct11 &, OutputStruct11 &);
// void swapXAndY(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);





typedef struct
{
    PointCloudsDetector *c;
} ParamStruct13;


typedef struct
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
} InputStruct13;


typedef struct
{

} OutputStruct13;

/**
 * @brief classicLidarDetectorNumPoints 函数
 * @param[IN] param13, PointCloudsDetector类的对象的指针
 * @param[IN] input13, 点云输入
 * @param[OUT] output13, 空输出
 
 * @cn_name 点云数量统计

 * @granularity atomic

 * @tag perception
 */
void classicLidarDetectorNumPoints(ParamStruct13 &, InputStruct13 &, OutputStruct13 &);
// void numPoints(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);





typedef struct
{
    PointCloudsDetector *c;
} ParamStruct14;


typedef struct
{

} InputStruct14;


typedef struct
{
    pcl::visualization::PCLVisualizer::Ptr output;
} OutputStruct14;

/**
 * @brief classicLidarDetectorGetViewer 函数
 * @param[IN] param14, PointCloudsDetector类的对象的指针
 * @param[IN] input14, 空输入
 * @param[OUT] output14, 点云指针输出
 
 * @cn_name PCL可视化器初始化

 * @granularity atomic

 * @tag perception
 */
void classicLidarDetectorGetViewer(ParamStruct14 &, InputStruct14 &, OutputStruct14 &);
// pcl::visualization::PCLVisualizer::Ptr getViewer() override;




typedef struct
{
    PointCloudsDetector *c;
} ParamStruct15;


typedef struct
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr src_cloud;
    const Eigen::Vector4f plane;
    bool negative;
} InputStruct15;


typedef struct
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr output;
} OutputStruct15;

/**
 * @brief classicLidarDetectorClipPlane 函数
 * @param[IN] param15, PointCloudsDetector类的对象的指针
 * @param[IN] input15, 点云指针、裁剪平面
 * @param[OUT] output15, 裁减后点云输出

 * @cn_name 点云裁剪

 * @granularity atomic

 * @tag perception
 */
void classicLidarDetectorClipPlane(ParamStruct15 &, InputStruct15 &, OutputStruct15 &);
// pcl::PointCloud<pcl::PointXYZI>::Ptr clipPlane(pcl::PointCloud<pcl::PointXYZI>::Ptr src_cloud, const Eigen::Vector4f plane, bool negative);


typedef struct
{
    PointCloudsDetector *c;
} ParamStruct16;


typedef struct
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    float filterRes_x, filterRes_y, filterRes_z;
    Eigen::Vector4f minPoint;
    Eigen::Vector4f maxPoint;
} InputStruct16;


typedef struct
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr output;
} OutputStruct16;

/**
 * @brief classicLidarDetectorFilterCloud 函数
 * @param[IN] param16, PointCloudsDetector类的对象的指针
 * @param[IN] input16, classicLidarDetectorFilterCloud 的输出参数struct
 * @param[OUT] output16, classicLidarDetectorFilterCloud 的输出参数struct
 
 * @cn_name 降采样

 * @granularity atomic

 * @tag perception
 */
void classicLidarDetectorFilterCloud(ParamStruct16 &, InputStruct16 &, OutputStruct16 &);
// pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
//                                                      float filterRes_x, float filterRes_y, float filterRes_z,
//                                                      Eigen::Vector4f minPoint,
//                                                      Eigen::Vector4f maxPoint);



typedef struct
{
    PointCloudsDetector *c;
} ParamStruct19;

typedef struct
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    float clusterTolerance;
    int minSize;
    int maxSize;
} InputStruct19;


typedef struct
{
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> output;
} OutputStruct19;

/**
 * @brief classicLidarDetectorClustering 函数
 * @param[IN] param19, PointCloudsDetector类的对象的指针
 * @param[IN] input19, 聚类输入参数
 * @param[OUT] output19, 聚类后的点云输出
 
 * @cn_name 欧式聚类方法

 * @granularity atomic

 * @tag perception
 */
void classicLidarDetectorClustering(ParamStruct19 &, InputStruct19 &, OutputStruct19 &);
// std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> Clustering(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
//                                                                  float clusterTolerance, int minSize, int maxSize);


typedef struct
{
    PointCloudsDetector *c;
} ParamStruct20;


typedef struct
{
    const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    int normal_estimation_Ksearch;
    float slope_rate;
    float normal_thereshold;
    int grid_height_mode;
    float car_height;
    float grid_size;
    float right_left_distance;
    float back_distance;
    float front_distance;
} InputStruct20;


typedef struct
{
    std::vector<int> output;
} OutputStruct20;

/**
 * @brief classicLidarDetectorGridGroundFilter 函数
 * @param[IN] param20, PointCloudsDetector类的对象的指针
 * @param[IN] input20, 过滤参数输入
 * @param[OUT] output20, 过滤后的点云矩阵输出
 
 * @cn_name CropBox过滤

 * @granularity atomic

 * @tag perception
 */
void classicLidarDetectorGridGroundFilter(ParamStruct20 &, InputStruct20 &, OutputStruct20 &);
// std::vector<int> GridGroundFilterSimple(const pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud,
//                                     int normal_estimation_Ksearch,
//                                     float slope_rate,
//                                     float normal_thereshold,
//                                     int grid_height_mode,
//                                     float car_height,
//                                     float grid_size,
//                                     float right_left_distance,
//                                     float back_distance,
//                                     float front_distance); // 定义车周地面高度


typedef struct
{
    PointCloudsDetector *c;
} ParamStruct21;


typedef struct
{
    Box box;
    int id;
    Color color = Color(0, 1, 0);
    float opacity = 1;
} InputStruct21;

typedef struct
{

} OutputStruct21;

/**
 * @brief classicLidarDetectorRenderBox 函数
 * @param[IN] param21, PointCloudsDetector类的对象的指针
 * @param[IN] input21, 渲染设置输入
 * @param[OUT] output21, 空输出
 
 * @cn_name 3D视图立方体渲染

 * @granularity atomic

 * @tag perception
 */
void classicLidarDetectorRenderBox(ParamStruct21 &, InputStruct21 &, OutputStruct21 &);
// void renderBox(Box box, int id, Color color = Color(0, 1, 0), float opacity = 1);


typedef struct
{
    PointCloudsDetector *c;
} ParamStruct22;


typedef struct
{
    PcaBox box;
    int id;
    Color color = Color(0, 1, 0);
    float opacity = 1;
} InputStruct22;


typedef struct
{

} OutputStruct22;

/**
 * @brief classicLidarDetectorRenderPcaBox 函数
 * @param[IN] param22, PointCloudsDetector类的对象的指针
 * @param[IN] input22, 渲染参数输入
 * @param[OUT] output22, 空输出
 
 * @cn_name PCA立方体渲染

 * @granularity atomic

 * @tag perception
 */
void classicLidarDetectorRenderPcaBox(ParamStruct22 &, InputStruct22 &, OutputStruct22 &);
// void renderPcaBox(PcaBox box, int id, Color color = Color(0, 1, 0), float opacity = 1);


typedef struct
{
    PointCloudsDetector *c;
} ParamStruct25;


typedef struct
{
    const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    float ransac_threshold;
    int normal_estimation_Ksearch;
    float slope_rate;
    float normal_thereshold;
    int grid_height_mode;
    float car_height;
    float grid_size;
    float right_left_distance;
    float back_distance;
    float front_distance;
} InputStruct25;


typedef struct
{
    std::vector<int> output;
} OutputStruct25;

/**
 * @brief classicLidarDetectoGridRANSAC 函数
 * @param[IN] param25, PointCloudsDetector类的对象的指针
 * @param[IN] input25, 点云指针和RANSAC算法参数
 * @param[OUT] output25,地面点id数组
 
 * @cn_name 随机抽样一致方法

 * @granularity atomic

 * @tag perception
 */
void classicLidarDetectoGridRANSAC(ParamStruct25 &, InputStruct25 &, OutputStruct25 &);
// std::vector<int> GridRANSAC(const pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud,
//                                     float ransac_threshold,
//                                     int normal_estimation_Ksearch,
//                                     float slope_rate,
//                                     float normal_thereshold,
//                                     int grid_height_mode,
//                                     float car_height,
//                                     float grid_size,
//                                     float right_left_distance,
//                                     float back_distance,
//                                     float front_distance);


typedef struct
{
    PointCloudsDetector *c;
} ParamStruct26;


typedef struct
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloudi;
    int normal_estimation_Ksearch;
    float slope_rate;
    float normal_thereshold;
    int grid_height_mode;
    float car_height;
    float grid_size;
    float right_left_distance;
    float back_distance;
    float front_distance;
} InputStruct26;


typedef struct
{
    std::vector<int> output;
} OutputStruct26;

/**
 * @brief classicLidarDetectoGridPatch 函数
 * @param[IN] param26, PointCloudsDetector类的对象的指针
 * @param[IN] input26，点云指针和Patchwork算法参数
 * @param[OUT] output26，地面点id数组
 
 * @cn_name patchwork方法

 * @granularity atomic

 * @tag perception
 */
void classicLidarDetectoGridPatch(ParamStruct26 &, InputStruct26 &, OutputStruct26 &);

}