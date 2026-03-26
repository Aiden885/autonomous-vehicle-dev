/**
 * @brief 文件说明
 * @file functionalLidarDetector.h
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

namespace perception
{

    /**
     * @brief classicLidarDetectorFind_Z_value 函数组件可以配置的参数， 用struct 封装
     */
    typedef struct
    {
        PointCloudsDetector c;
    } ParamStruct01;

    /**
     * @brief classicLidarDetectorFind_Z_value 函数组件输入的参数， 用struct 封装
     */
    typedef struct
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    } InputStruct01;

    /**
     * @brief classicLidarDetectorFind_Z_value 函数组件输出的参数， 用struct 封装
     */
    typedef struct
    {

    } OutputStruct01;

    /**
     * @brief classicLidarDetectorFind_Z_value 函数
     * @param[IN] param1, PointCloudsDetector类的对象
     * @param[IN] input1, 点云指针
     * @param[OUT] output1, 空输出
     * @cn_name 点云查找z坐标极值
     * @granularity atomic
     * @tag perception
     */
    void classicLidarDetectorFind_Z_value(ParamStruct01 &, InputStruct01 &, OutputStruct01 &);
    // void find_Z_value(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

    /**
     * @brief classicLidarDetectorPassThroughFilter 函数组件可以配置的参数， 用struct 封装
     */
    typedef struct
    {
        PointCloudsDetector c;
    } ParamStruct02;

    /**
     * @brief classicLidarDetectorPassThroughFilter 函数组件输入的参数， 用struct 封装
     */
    typedef struct
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
        double z_max;
        double z_min;
    } InputStruct02;

    /**
     * @brief classicLidarDetectorPassThroughFilter 函数组件输出的参数， 用struct 封装
     */
    typedef struct
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr output;
    } OutputStruct02;

    /**
     * @brief classicLidarDetectorPassThroughFilter 函数
     * @param[IN] param2, PointCloudsDetector类的对象
     * @param[IN] input2, 点云指针和z的界限
     * @param[OUT] output2, 点云指针
     * @cn_name 点云直通滤波
     * @granularity atomic
     * @tag perception
     */
    void classicLidarDetectorPassThroughFilter(ParamStruct02 &, InputStruct02 &, OutputStruct02 &);
    //     pcl::PointCloud<pcl::PointXYZI>::Ptr PassThroughFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, double z_max, double z_min);

    /**
     * @brief classicLidarDetectorPointcloud_to_grid 函数组件可以配置的参数， 用struct 封装
     */
    typedef struct
    {
        PointCloudsDetector c;
    } ParamStruct03;

    /**
     * @brief classicLidarDetectorPointcloud_to_grid 函数组件输入的参数， 用struct 封装
     */
    typedef struct
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
        double map_resolution;
    } InputStruct03;

    /**
     * @brief classicLidarDetectorPointcloud_to_grid 函数组件输出的参数， 用struct 封装
     */
    typedef struct
    {
        vector<int> Pos_array;
    } OutputStruct03;

    /**
     * @brief classicLidarDetectorPointcloud_to_grid 函数
     * @param[IN] param3, PointCloudsDetector类的对象
     * @param[IN] input3, 点云指针和分辨率
     * @param[OUT] 栅格地图
     * @cn_name 栅格化点云
     * @granularity atomic
     * @tag perception
     */
    void classicLidarDetectorPointcloud_to_grid(ParamStruct03 &, InputStruct03 &, OutputStruct03 &);
    //     vector<int> Pointcloud_to_grid(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, double map_resolution);

    /**
     * @brief classicLidarDetectorInputcloud 函数组件可以配置的参数， 用struct 封装
     */
    typedef struct
    {
        PointCloudsDetector c;
    } ParamStruct04;

    /**
     * @brief classicLidarDetectorInputcloud 函数组件输入的参数， 用struct 封装
     */
    typedef struct
    {
        int i;
        // std::string folderPath;
    } InputStruct04;

    /**
     * @brief classicLidarDetectorInputcloud 函数组件输出的参数， 用struct 封装
     */
    typedef struct
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr output;
    } OutputStruct04;

    /**
     * @brief classicLidarDetectorInputcloud 函数
     * @param[IN] param4, PointCloudsDetector类的对象
     * @param[IN] input4, 文件编号和路径
     * @param[OUT] output4, 点云指针
     * @cn_name 读取点云数据
     * @granularity atomic
     * @tag perception
     */
    void classicLidarDetectorInputcloud(ParamStruct04 &, InputStruct04 &, OutputStruct04 &);
    // pcl::PointCloud<pcl::PointXYZI>::Ptr inputcloud(int i,std::string folderPath);

    /**
     * @brief classicLidarDetectorInitCamera 函数组件可以配置的参数， 用struct 封装
     */
    typedef struct
    {
        PointCloudsDetector c;
    } ParamStruct05;

    /**
     * @brief classicLidarDetectorInitCamera 函数组件输入的参数， 用struct 封装
     */
    typedef struct
    {
        CameraAngle setAngle;
    } InputStruct05;

    /**
     * @brief classicLidarDetectorInitCamera 函数组件输出的参数， 用struct 封装
     */
    typedef struct
    {

    } OutputStruct05;

    /**
     * @brief classicLidarDetectorInitCamera 函数
     * @param[IN] param5, PointCloudsDetector类的对象
     * @param[IN] input5, 姿态参数结构体
     * @param[OUT] output5, 空输出
     * @cn_name 相机初始化姿态
     * @granularity atomic
     * @tag perception
     */
    void classicLidarDetectorInitCamera(ParamStruct05 &, InputStruct05 &, OutputStruct05 &);
    // void initCamera(CameraAngle setAngle);

    /**
     * @brief classicLidarDetectorFind_Z_value 函数组件可以配置的参数， 用struct 封装
     */
    typedef struct
    {
        PointCloudsDetector c;
    } ParamStruct06;

    /**
     * @brief classicLidarDetectorInitView 函数组件输入的参数， 用struct 封装
     */
    typedef struct
    {

    } InputStruct06;

    /**
     * @brief classicLidarDetectorInitView 函数组件输出的参数， 用struct 封装
     */
    typedef struct
    {

    } OutputStruct06;

    /**
     * @brief classicLidarDetectorInitView 函数
     * @param[IN] param6, PointCloudsDetector类的对象
     * @param[IN] input6, 空输入
     * @param[OUT] output6, 空输出
     * @cn_name 点云初始化视图
     * @granularity atomic
     * @tag perception
     */
    void classicLidarDetectorInitView(ParamStruct06 &, InputStruct06 &, OutputStruct06 &);
    // void initView();

    /**
     * @brief classicLidarDetectorCheckAlive 函数组件可以配置的参数， 用struct 封装
     */
    typedef struct
    {
        PointCloudsDetector c;
    } ParamStruct07;

    /**
     * @brief classicLidarDetectorCheckAlive 函数组件输入的参数， 用struct 封装
     */
    typedef struct
    {

    } InputStruct07;

    /**
     * @brief classicLidarDetectorCheckAlive 函数组件输出的参数， 用struct 封装
     */
    typedef struct
    {
        bool isAlive;
    } OutputStruct07;

    /**
     * @brief classicLidarDetectorCheckAlive 函数
     * @param[IN] param7, PointCloudsDetector类的对象
     * @param[IN] input7, 空输入
     * @param[OUT] output7, bool类型，判断是否正常工作
     * @cn_name 点云判定可视窗口活动
     * @granularity atomic
     * @tag perception
     */
    void classicLidarDetectorCheckAlive(ParamStruct07 &, InputStruct07 &, OutputStruct07 &);
    // bool checkAlive();

    /**
     * @brief classicLidarDetectorReceiveCloud 函数组件可以配置的参数， 用struct 封装
     */
    typedef struct
    {
        PointCloudsDetector *c;
    } ParamStruct08;

    /**
     * @brief classicLidarDetectorReceiveCloud 函数组件输入的参数， 用struct 封装
     */
    typedef struct
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr input;
    } InputStruct08;

    /**
     * @brief classicLidarDetectorReceiveCloud 函数组件输出的参数， 用struct 封装
     */
    typedef struct
    {

    } OutputStruct08;

    /**
     * @brief classicLidarDetectorReceiveCloud 函数
     * @param[IN] param8, PointCloudsDetector类的对象
     * @param[IN] input8, 点云指针
     * @param[OUT] output8, 空输出
     * @cn_name 存储点云数据
     * @granularity atomic
     * @tag perception
     */
    void classicLidarDetectorReceiveCloud(ParamStruct08 &param08, InputStruct08 &input08, OutputStruct08 &output08);
    // void receiveCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr input) override;

    /**
     * @brief classicLidarDetectorRPointCloud 函数组件可以配置的参数， 用struct 封装
     */
    typedef struct
    {
        PointCloudsDetector c;
    } ParamStruct09;

    /**
     * @brief classicLidarDetectorRPointCloud 函数组件输入的参数， 用struct 封装
     */
    typedef struct
    {
        const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud;
    } InputStruct09;

    /**
     * @brief classicLidarDetectorRPointCloud 函数组件输出的参数， 用struct 封装
     */
    typedef struct
    {

    } OutputStruct09;

    /**
     * @brief classicLidarDetectorRPointCloud 函数
     * @param[IN] param9, PointCloudsDetector类的对象
     * @param[IN] input9, 点云指针
     * @param[OUT] output9, 空输出
     * @cn_name 渲染点云数据
     * @granularity atomic
     * @tag perception
     */
    void classicLidarDetectorRPointCloud(ParamStruct09 &, InputStruct09 &, OutputStruct09 &);
    // void rPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud);

    /**
     * @brief classicLidarDetectorRenderPointCloud 函数组件可以配置的参数， 用struct 封装
     */
    typedef struct
    {
        PointCloudsDetector *c;
    } ParamStruct10;

    /**
     * @brief classicLidarDetectorRenderPointCloud 函数组件输入的参数， 用struct 封装
     */
    typedef struct
    {
        const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud;
        std::string name;
        Color color;
    } InputStruct10;

    /**
     * @brief classicLidarDetectorRenderPointCloud 函数组件输出的参数， 用struct 封装
     */
    typedef struct
    {

    } OutputStruct10;

    /**
     * @brief classicLidarDetectorRenderPointCloud 函数
     * @param[IN] param10, PointCloudsDetector类的对象
     * @param[IN] input10, 渲染设置
     * @param[OUT] output10, 空输出
     * @cn_name 显示点云数据
     * @granularity atomic
     * @tag perception
     */
    void classicLidarDetectorRenderPointCloud(ParamStruct10 &, InputStruct10 &, OutputStruct10 &);
    // void renderPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, std::string name, Color color);

    /**
     * @brief classicLidarDetectorSwapXAndY 函数组件可以配置的参数， 用struct 封装
     */
    typedef struct
    {
        PointCloudsDetector c;
    } ParamStruct11;

    /**
     * @brief classicLidarDetectorSwapXAndY 函数组件输入的参数， 用struct 封装
     */
    typedef struct
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    } InputStruct11;

    /**
     * @brief classicLidarDetectorSwapXAndY 函数组件输出的参数， 用struct 封装
     */
    typedef struct
    {

    } OutputStruct11;

    /**
     * @brief classicLidarDetectorSwapXAndY 函数
     * @param[IN] param11, PointCloudsDetector类的对象
     * @param[IN] input11,  点云输入
     * @param[OUT] output11, 空输出
     * @cn_name 点云互换xy
     * @granularity atomic
     * @tag perception
     */
    void classicLidarDetectorSwapXAndY(ParamStruct11 &, InputStruct11 &, OutputStruct11 &);
    // void swapXAndY(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

    /**
     * @brief classicLidarDetectorSwapYAndX 函数组件可以配置的参数， 用struct 封装
     */
    typedef struct
    {
        PointCloudsDetector c;
    } ParamStruct12;

    /**
     * @brief classicLidarDetectorSwapYAndX 函数组件输入的参数， 用struct 封装
     */
    typedef struct
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    } InputStruct12;

    /**
     * @brief classicLidarDetectorSwapYAndX 函数组件输出的参数， 用struct 封装
     */
    typedef struct
    {

    } OutputStruct12;

    /**
     * @brief classicLidarDetectorSwapYAndX 函数
     * @param[IN] param12, PointCloudsDetector类的对象
     * @param[IN] input12, 点云输入
     * @param[OUT] output12, 空输出
     * @cn_name 点云互换xy
     * @granularity atomic
     * @tag perception
     */
    void classicLidarDetectorSwapYAndX(ParamStruct12 &, InputStruct12 &, OutputStruct12 &);
    // void swapYAndX(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud); //

    /**
     * @brief classicLidarDetectorNumPoints 函数组件可以配置的参数， 用struct 封装
     */
    typedef struct
    {
        PointCloudsDetector c;
    } ParamStruct13;

    /**
     * @brief classicLidarDetectorNumPoints 函数组件输入的参数， 用struct 封装
     */
    typedef struct
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    } InputStruct13;

    /**
     * @brief classicLidarDetectorNumPoints 函数组件输出的参数， 用struct 封装
     */
    typedef struct
    {

    } OutputStruct13;

    /**
     * @brief classicLidarDetectorNumPoints 函数
     * @param[IN] param13, PointCloudsDetector类的对象
     * @param[IN] input13, 点云输入
     * @param[OUT] output13, 空输出
     * @cn_name 统计点云数量
     * @granularity atomic
     * @tag perception
     */
    void classicLidarDetectorNumPoints(ParamStruct13 &, InputStruct13 &, OutputStruct13 &);
    // void numPoints(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

    /**
     * @brief classicLidarDetectorGetViewer 函数组件可以配置的参数， 用struct 封装
     */
    typedef struct
    {
        PointCloudsDetector c;
    } ParamStruct14;

    /**
     * @brief classicLidarDetectorGetViewer 函数组件输入的参数， 用struct 封装
     */
    typedef struct
    {

    } InputStruct14;

    /**
     * @brief classicLidarDetectorGetViewer 函数组件输出的参数， 用struct 封装
     */
    typedef struct
    {
        pcl::visualization::PCLVisualizer::Ptr output;
    } OutputStruct14;

    /**
     * @brief classicLidarDetectorGetViewer 函数
     * @param[IN] param14, PointCloudsDetector类的对象
     * @param[IN] input14, 空输入
     * @param[OUT] output14, 点云输出
     * @cn_name 获取可视化指针
     * @granularity atomic
     * @tag perception
     */
    void classicLidarDetectorGetViewer(ParamStruct14 &, InputStruct14 &, OutputStruct14 &);
    // pcl::visualization::PCLVisualizer::Ptr getViewer() override;

    /**
     * @brief classicLidarDetectorClipPlane 函数组件可以配置的参数， 用struct 封装
     */
    typedef struct
    {
        PointCloudsDetector c;
    } ParamStruct15;

    /**
     * @brief classicLidarDetectorClipPlane 函数组件输入的参数， 用struct 封装
     */
    typedef struct
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr src_cloud;
        const Eigen::Vector4f plane;
        bool negative;
    } InputStruct15;

    /**
     * @brief classicLidarDetectorClipPlane 函数组件输出的参数， 用struct 封装
     */
    typedef struct
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr output;
    } OutputStruct15;

    /**
     * @brief classicLidarDetectorClipPlane 函数
     * @param[IN] param15, PointCloudsDetector类的对象
     * @param[IN] input15, 点云裁减的输入
     * @param[OUT] output15, 裁减后点云输出
     * @cn_name 点云剪裁
     * @granularity atomic
     * @tag perception
     */
    void classicLidarDetectorClipPlane(ParamStruct15 &, InputStruct15 &, OutputStruct15 &);
    // pcl::PointCloud<pcl::PointXYZI>::Ptr clipPlane(pcl::PointCloud<pcl::PointXYZI>::Ptr src_cloud, const Eigen::Vector4f plane, bool negative);

    /**
     * @brief classicLidarDetectorFilterCloud 函数组件可以配置的参数， 用struct 封装
     */
    typedef struct
    {
        PointCloudsDetector *c;
    } ParamStruct16;

    /**
     * @brief classicLidarDetectorFilterCloud 函数组件输入的参数， 用struct 封装
     */
    typedef struct
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
        float filterRes_x, filterRes_y, filterRes_z;
        Eigen::Vector4f minPoint;
        Eigen::Vector4f maxPoint;
    } InputStruct16;

    /**
     * @brief classicLidarDetectorFilterCloud 函数组件输出的参数， 用struct 封装
     */
    typedef struct
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr output;
    } OutputStruct16;

    /**
     * @brief classicLidarDetectorFilterCloud 函数
     * @param[IN] param16, PointCloudsDetector类的对象
     * @param[IN] input16, classicLidarDetectorFilterCloud 的输出参数struct
     * @param[OUT] output16, classicLidarDetectorFilterCloud 的输出参数struct
     * @cn_name 预处理点云数据
     * @granularity atomic
     * @tag perception
     */
    void classicLidarDetectorFilterCloud(ParamStruct16 &, InputStruct16 &, OutputStruct16 &);
    // pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
    //                                                      float filterRes_x, float filterRes_y, float filterRes_z,
    //                                                      Eigen::Vector4f minPoint,
    //                                                      Eigen::Vector4f maxPoint);

    /**
     * @brief classicLidarDetectorRmvNearPoints 函数组件可以配置的参数， 用struct 封装
     */
    typedef struct
    {
        PointCloudsDetector c;
    } ParamStruct17;

    /**
     * @brief classicLidarDetectorRmvNearPoints 函数组件输入的参数， 用struct 封装
     */
    typedef struct
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
        Eigen::Vector4f minPoint;
        Eigen::Vector4f maxPoint;
    } InputStruct17;

    /**
     * @brief classicLidarDetectorRmvNearPoints 函数组件输出的参数， 用struct 封装
     */
    typedef struct
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr output;
    } OutputStruct17;

    /**
     * @brief classicLidarDetectorRmvNearPoints 函数
     * @param[IN] param17, PointCloudsDetector类的对象
     * @param[IN] input17, 点云和剔除范围
     * @param[OUT] output17, 点云输出
     * @cn_name 点云剔除
     * @granularity atomic
     * @tag perception
     */
    void classicLidarDetectorRmvNearPoints(ParamStruct17 &, InputStruct17 &, OutputStruct17 &);
    // pcl::PointCloud<pcl::PointXYZI>::Ptr rmvNearPoints(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
    //                                                        Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

    /**
     * @brief classicLidarDetectorRANSAC3d 函数组件可以配置的参数， 用struct 封装
     */
    typedef struct
    {
        PointCloudsDetector c;
    } ParamStruct18;

    /**
     * @brief classicLidarDetectorRANSAC3d 函数组件输入的参数， 用struct 封装
     */
    typedef struct
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
        int maxIterations;
        float distanceThreshold;
    } InputStruct18;

    /**
     * @brief classicLidarDetectorRANSAC3d 函数组件输出的参数， 用struct 封装
     */
    typedef struct
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr output;
    } OutputStruct18;

    /**
     * @brief classicLidarDetectorRANSAC3d 函数
     * @param[IN] param18, PointCloudsDetector类的对象
     * @param[IN] input18, 点云分割输入
     * @param[OUT] output18, 分割后点云输出
     * @cn_name 点云分割
     * @granularity atomic
     * @tag perception
     */
    void classicLidarDetectorRANSAC3d(ParamStruct18 &, InputStruct18 &, OutputStruct18 &);
    // pcl::PointCloud<pcl::PointXYZI>::Ptr RANSAC3d(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int maxIterations, float distanceThreshold);

    /**
     * @brief classicLidarDetectorClustering 函数组件可以配置的参数， 用struct 封装
     */
    typedef struct
    {
        PointCloudsDetector c;
    } ParamStruct19;

    /**
     * @brief classicLidarDetectorClustering 函数组件输入的参数， 用struct 封装
     */
    typedef struct
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
        float clusterTolerance;
        int minSize;
        int maxSize;
    } InputStruct19;

    /**
     * @brief classicLidarDetectorClustering 函数组件输出的参数， 用struct 封装
     */
    typedef struct
    {
        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> output;
    } OutputStruct19;

    /**
     * @brief classicLidarDetectorClustering 函数
     * @param[IN] param19, PointCloudsDetector类的对象
     * @param[IN] input19, 聚类输入参数
     * @param[OUT] output19, 聚类后的点云输出
     * @cn_name 点云聚类
     * @granularity atomic
     * @tag perception
     */
    void classicLidarDetectorClustering(ParamStruct19 &, InputStruct19 &, OutputStruct19 &);
    // std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> Clustering(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
    //                                                                  float clusterTolerance, int minSize, int maxSize);

    /**
     * @brief classicLidarDetectorGridGroundFilter 函数组件可以配置的参数， 用struct 封装
     */
    typedef struct
    {
        PointCloudsDetector *c;
    } ParamStruct20;

    /**
     * @brief classicLidarDetectorGridGroundFilter 函数组件输入的参数， 用struct 封装
     */
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

    /**
     * @brief classicLidarDetectorGridGroundFilterSimple 函数组件输出的参数， 用struct 封装
     */
    typedef struct
    {
        std::vector<int> output;
    } OutputStruct20;

    /**
     * @brief classicLidarDetectorGridGroundFilterSimple 函数
     * @param[IN] param20, PointCloudsDetector类的对象
     * @param[IN] input20, 过滤参数输入
     * @param[OUT] output20, 过滤后的点云矩阵输出
     * @cn_name 点云地面过滤
     * @granularity atomic
     * @tag perception
     */
    void classicLidarDetectorGridGroundFilterSimple(ParamStruct20 &, InputStruct20 &, OutputStruct20 &);
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

    /**
     * @brief classicLidarDetectorRenderBox 函数组件可以配置的参数， 用struct 封装
     */
    typedef struct
    {
        PointCloudsDetector c;
    } ParamStruct21;

    /**
     * @brief classicLidarDetectorRenderBox 函数组件输入的参数， 用struct 封装
     */
    typedef struct
    {
        Box box;
        int id;
        Color color = Color(0, 1, 0);
        float opacity = 1;
    } InputStruct21;

    /**
     * @brief classicLidarDetectorRenderBox 函数组件输出的参数， 用struct 封装
     */
    typedef struct
    {

    } OutputStruct21;

    /**
     * @brief classicLidarDetectorRenderBox 函数
     * @param[IN] param21, PointCloudsDetector类的对象
     * @param[IN] input21, 渲染设置输入
     * @param[OUT] output21, 空输出
     * @cn_name PCL渲染立方体
     * @granularity atomic
     * @tag perception
     */
    void classicLidarDetectorRenderBox(ParamStruct21 &, InputStruct21 &, OutputStruct21 &);
    // void renderBox(Box box, int id, Color color = Color(0, 1, 0), float opacity = 1);

    /**
     * @brief classicLidarDetectorRenderPcaBox 函数组件可以配置的参数， 用struct 封装
     */
    typedef struct
    {
        PointCloudsDetector c;
    } ParamStruct22;

    /**
     * @brief classicLidarDetectorRenderPcaBox 函数组件输入的参数， 用struct 封装
     */
    typedef struct
    {
        PcaBox box;
        int id;
        Color color = Color(0, 1, 0);
        float opacity = 1;
    } InputStruct22;

    /**
     * @brief classicLidarDetectorRenderPcaBox 函数组件输出的参数， 用struct 封装
     */
    typedef struct
    {

    } OutputStruct22;

    /**
     * @brief classicLidarDetectorRenderPcaBox 函数
     * @param[IN] param22, PointCloudsDetector类的对象
     * @param[IN] input22, 渲染参数输入
     * @param[OUT] output22, 空输出
     * @cn_name PCA渲染
     * @granularity atomic
     * @tag perception
     */
    void classicLidarDetectorRenderPcaBox(ParamStruct22 &, InputStruct22 &, OutputStruct22 &);
    // void renderPcaBox(PcaBox box, int id, Color color = Color(0, 1, 0), float opacity = 1);

    /**
     * @brief classicLidarDetectoRun 函数组件可以配置的参数， 用struct 封装
     */
    typedef struct
    {
        PointCloudsDetector *c;
    } ParamStruct23;

    /**
     * @brief classicLidarDetectoRun 函数组件输入的参数， 用struct 封装
     */
    typedef struct
    {

    } InputStruct23;

    /**
     * @brief classicLidarDetectoRun 函数组件输出的参数， 用struct 封装
     */
    typedef struct
    {
        std::pair<std::vector<int>, std::vector<Frame>> output;
    } OutputStruct23;

    /**
     * @brief classicLidarDetectoRun 函数
     * @param[IN] param23, PointCloudsDetector类的对象
     * @param[IN] input23, 空输入
     * @param[OUT] output23, 点云类运行输出
     * @cn_name 运行点云类
     * @granularity atomic
     * @tag perception
     */
    void classicLidarDetectoRun(ParamStruct23 &, InputStruct23 &, OutputStruct23 &);
    // std::pair<std::vector<int>, std::vector<Frame> > run() override;

    /**
     * @brief classicLidarDetectoTest 函数组件可以配置的参数， 用struct 封装
     */
    typedef struct
    {
        PointCloudsDetector c;
    } ParamStruct24;

    /**
     * @brief classicLidarDetectoTest 函数组件输入的参数， 用struct 封装
     */
    typedef struct
    {

    } InputStruct24;

    /**
     * @brief classicLidarDetectoTest 函数组件输出的参数， 用struct 封装
     */
    typedef struct
    {

    } OutputStruct24;

    /**
     * @brief classicLidarDetectoTest 函数
     * @param[IN] param24, PointCloudsDetector类的对象
     * @param[IN] input24, 空输入
     * @param[OUT] output24,空输出
     * @cn_name 测试点云类
     * @granularity atomic
     * @tag perception
     */
    void classicLidarDetectoTest(ParamStruct24 &, InputStruct24 &, OutputStruct24 &);
    // void test();

    /**
     * @brief classicLidarDetectoGridRANSAC 函数组件可以配置的参数， 用struct 封装
     */
    typedef struct
    {
        PointCloudsDetector *c;
    } ParamStruct25;

    /**
     * @brief classicLidarDetectoGridRANSAC 函数组件输入的参数， 用struct 封装
     */
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

    /**
     * @brief classicLidarDetectoGridRANSAC 函数组件输出的参数， 用struct 封装
     */
    typedef struct
    {
        std::vector<int> output;
    } OutputStruct25;

    /**
     * @brief classicLidarDetectoGridRANSAC 函数
     * @param[IN] param25, PointCloudsDetector类的对象
     * @param[IN] input25, 空输入
     * @param[OUT] output25,空输出
     * @cn_name RANSAC地面提取
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

    /**
     * @brief classicLidarDetectoGridRANSAC 函数组件可以配置的参数， 用struct 封装
     */
    typedef struct
    {
        PointCloudsDetector *c;
    } ParamStruct26;

    /**
     * @brief classicLidarDetectoGridRANSAC 函数组件输入的参数， 用struct 封装
     */
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

    /**
     * @brief classicLidarDetectoGridRANSAC 函数组件输出的参数， 用struct 封装
     */
    typedef struct
    {
        std::vector<int> output;
    } OutputStruct26;

    /**
     * @brief classicLidarDetectoGridPatch 函数
     * @param[IN] param26, PointCloudsDetector类的对象
     * @param[IN] input26
     * @param[OUT] output26
     * @cn_name Patchwork地面提取
     * @granularity atomic
     * @tag perception
     */
    void classicLidarDetectoGridPatch(ParamStruct26 &, InputStruct26 &, OutputStruct26 &);
}