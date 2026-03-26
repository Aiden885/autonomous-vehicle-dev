/**
 * @brief LidarDetector功能
 * @file functionLidarDetector.cpp
 * @version 0.0.1
 * @author Tianqi Ke (ktq23@mails.tsinghua.edu.cn)
 * @date 2023-12-11
 */
#include "functionLidarDetector.h"

namespace perception{

/**
 * @brief classicLidarDetectorFindZ 函数
 * @param[IN] param1, PointCloudsDetector类的对象的指针
 * @param[IN] input1, 点云指针
 * @param[OUT] output1, 空输出
 
 * @cn_name: 点云高度查询
 
 * @granularity: atomic
 
 * @tag: perception
 */
void classicLidarDetectorFindZ(ParamStruct01 &param01, InputStruct01 &input01, OutputStruct01 &output01)
{
    PointCloudsDetector *c = param01.c;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = input01.cloud;
    c->find_Z_value(cloud);
    return;
}

/**
 * @brief classicLidarDetectorPassThroughFilter 函数
 * @param[IN] param2, PointCloudsDetector类的对象的指针
 * @param[IN] input2, 点云指针和z的最大最小值
 * @param[OUT] output2, 点云指针
 
 * @cn_name: 直通滤波

 * @granularity: atomic

 * @tag: perception
 */
void classicLidarDetectorPassThroughFilter(ParamStruct02 &param02, InputStruct02 &input02, OutputStruct02 &output02)
{
    PointCloudsDetector *c = param02.c;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = input02.cloud;
    double z_max = input02.z_max;
    double z_min = input02.z_min;
    output02.output = c->PassThroughFilter(cloud, z_max, z_min);
    return;
}

/**
 * @brief classicLidarDetectorPointCloudToGrid 函数
 * @param[IN] param3, PointCloudsDetector类的对象
 * @param[IN] input3, 点云指针和分辨率
 * @param[OUT] 栅格地图
 
 * @cn_name: 栅格化

 * @granularity: atomic
 
 * @tag: perception
 */
void classicLidarDetectorPointCloudToGrid(ParamStruct03 &param03, InputStruct03 &input03, OutputStruct03 &output03)
{
    PointCloudsDetector *c = param03.c;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = input03.cloud;
    double map_resolution = input03.map_resolution;
    output03.Pos_array = c->Pointcloud_to_grid(cloud, map_resolution);
    return;
}

/**
 * @brief classicLidarDetectorInputcloud 函数
 * @param[IN] param4, PointCloudsDetector类的对象的指针
 * @param[IN] input4, 文件编号
 * @param[OUT] output4, 点云指针
 
 * @cn_name: pcd文件加载

 * @granularity: atomic

 * @tag: perception
 */
void classicLidarDetectorInputcloud(ParamStruct04 &param04, InputStruct04 &input04, OutputStruct04 &output04)
{
    PointCloudsDetector *c = param04.c;
    int i = input04.i;
    // std::string folderPath = input04.folderPath;
    output04.output = c->inputcloud(i);
    return;
}

/**
 * @brief classicLidarDetectorInitCamera 函数
 * @param[IN] param5, PointCloudsDetector类的对象
 * @param[IN] input5, 姿态参数结构体
 * @param[OUT] output5, 空输出
 
 * @cn_name: 3D视图的相机位姿初始化

 * @granularity: atomic
 
 * @tag: perception
 */
void classicLidarDetectorInitCamera(ParamStruct05 &param05, InputStruct05 &input05, OutputStruct05 &output05)
{
    PointCloudsDetector *c = param05.c;
    c->initCamera(input05.setAngle);
    return;
}

/**
 * @brief classicLidarDetectorInitView 函数
 * @param[IN] param6, PointCloudsDetector类的对象
 * @param[IN] input6, 空输入
 * @param[OUT] output6, 空输出
 
 * @cn_name: 3D视图初始化

 * @granularity: atomic
 
 * @tag: perception
 */
void classicLidarDetectorInitView(ParamStruct06 &param06, InputStruct06 &input06, OutputStruct06 &output06)
{
    PointCloudsDetector *c = param06.c;
    c->initView();
    return;
}

/**
 * @brief classicLidarDetectorCheckAlive 函数
 * @param[IN] param7, PointCloudsDetector类的对象
 * @param[IN] input7, 空输入
 * @param[OUT] output7, bool类型，判断是否正常工作
 
 * @cn_name: 3D视图状态检测

 * @granularity: atomic
 
 * @tag: perception
 */
void classicLidarDetectorCheckAlive(ParamStruct07 &param07, InputStruct07 &input07, OutputStruct07 &output07)
{
    PointCloudsDetector *c = param07.c;
    output07.isAlive = c->checkAlive();
    return;
}

/**
 * @brief classicLidarDetectorReceiveCloud 函数
 * @param[IN] param8, PointCloudsDetector类的对象
 * @param[IN] input8, 点云指针
 * @param[OUT] output8, 空输出
 
 * @cn_name: 点云数据接收

 * @granularity: atomic
 
 * @tag: perception
 */
void classicLidarDetectorReceiveCloud(ParamStruct08 &param08, InputStruct08 &input08, OutputStruct08 &output08)
{
    PointCloudsDetector *c = param08.c;
    pcl::PointCloud<pcl::PointXYZI>::Ptr input = input08.input;
    c->receiveCloud(input);
    return;
}

/**
 * @brief classicLidarDetectorRPointCloud 函数
 * @param[IN] param9, PointCloudsDetector类的对象
 * @param[IN] input9, 点云指针
 * @param[OUT] output9, 空输出
 
 * @cn_name: 3D视图点云渲染

 * @granularity: atomic
 
 * @tag: perception
 */
void classicLidarDetectorRPointCloud(ParamStruct09 &param09, InputStruct09 &input09, OutputStruct09 &output09)
{
    PointCloudsDetector *c = param09.c;
    c->rPointCloud(input09.cloud);
    return;
}

/**
 * @brief classicLidarDetectorRenderPointCloud 函数
 * @param[IN] param10, PointCloudsDetector类的对象的指针
 * @param[IN] input10, 渲染参数设置
 * @param[OUT] output10, 空输出
 
 * @cn_name: 基于点云强度的渲染

 * @granularity: atomic

 * @tag: perception
 */
void classicLidarDetectorRenderPointCloud(ParamStruct10 &param10, InputStruct10 &input10, OutputStruct10 &output10)
{
    PointCloudsDetector *c = param10.c;
    std::string name = input10.name;
    c->renderPointCloud(input10.cloud, name, input10.color);
    return;
}

/**
 * @brief classicLidarDetectorSwapXAndY 函数
 * @param[IN] param11, PointCloudsDetector类的对象
 * @param[IN] input11,  点云输入
 * @param[OUT] output11, 空输出
 
 * @cn_name: 点云XY坐标转换

 * @granularity: atomic
 
 * @tag: perception
 */
void classicLidarDetectorSwapXAndY(ParamStruct11 &param11, InputStruct11 &input11, OutputStruct11 &output11)
{
    PointCloudsDetector *c = param11.c;
    c->swapXAndY(input11.cloud);
    return;
}


/**
 * @brief classicLidarDetectorNumPoints 函数
 * @param[IN] param13, PointCloudsDetector类的对象
 * @param[IN] input13, 点云输入
 * @param[OUT] output13, 空输出
 
 * @cn_name: 点云数量统计

 * @granularity: atomic
 
 * @tag: perception
 */
void classicLidarDetectorNumPoints(ParamStruct13 &param13, InputStruct13 &input13, OutputStruct13 &output13)
{
    PointCloudsDetector *c = param13.c;
    c->numPoints(input13.cloud);
    return;
}

/**
 * @brief classicLidarDetectorGetViewer 函数
 * @param[IN] param14, PointCloudsDetector类的对象的指针
 * @param[IN] input14, 空输入
 * @param[OUT] output14, 点云指针输出
 
 * @cn_name: PCL可视化器初始化

 * @granularity: atomic

 * @tag: perception
 */
void classicLidarDetectorGetViewer(ParamStruct14 &param14, InputStruct14 &input14, OutputStruct14 &output14)
{
    PointCloudsDetector *c = param14.c;
    output14.output = c->getViewer();
    return;
}

/**
 * @brief classicLidarDetectorClipPlane 函数
 * @param[IN] param15, PointCloudsDetector类的对象的指针
 * @param[IN] input15, 点云指针、裁剪平面
 * @param[OUT] output15, 裁减后点云输出

 * @cn_name: 点云裁剪

 * @granularity: atomic

 * @tag: perception
 */
void classicLidarDetectorClipPlane(ParamStruct15 &param15, InputStruct15 &input15, OutputStruct15 &output15)
{
    PointCloudsDetector *c = param15.c;
    pcl::PointCloud<pcl::PointXYZI>::Ptr src_cloud = input15.src_cloud;
    const Eigen::Vector4f plane = input15.plane;
    bool negative = input15.negative;
    output15.output = c->clipPlane(src_cloud, plane, negative);
    return;
}

/**
 * @brief classicLidarDetectorFilterCloud 函数
 * @param[IN] param16, PointCloudsDetector类的对象
 * @param[IN] input16, classicLidarDetectorFilterCloud 的输出参数struct
 * @param[OUT] output16, classicLidarDetectorFilterCloud 的输出参数struct
 
 * @cn_name: 降采样

 * @granularity: atomic
 
 * @tag: perception
 */
void classicLidarDetectorFilterCloud(ParamStruct16 &param16, InputStruct16 &input16, OutputStruct16 &output16)
{
    PointCloudsDetector *c = param16.c;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = input16.cloud;
    float filterRes_x = input16.filterRes_x;
    float filterRes_y = input16.filterRes_y;
    float filterRes_z = input16.filterRes_z;
    output16.output = c->filterCloud(cloud, filterRes_x, filterRes_y, filterRes_z, input16.minPoint, input16.maxPoint);
    return;
}



/**
 * @brief classicLidarDetectorClustering 函数
 * @param[IN] param19, PointCloudsDetector类的对象
 * @param[IN] input19, 聚类输入参数
 * @param[OUT] output19, 聚类后的点云输出
 
 * @cn_name: 欧式聚类方法

 * @granularity: atomic
 
 * @tag: perception
 */
void classicLidarDetectorClustering(ParamStruct19 &param19, InputStruct19 &input19, OutputStruct19 &output19)
{
    PointCloudsDetector *c = param19.c;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = input19.cloud;
    float clusterTolerance = input19.clusterTolerance;
    int minSize = input19.minSize;
    int maxSize = input19.maxSize;
    output19.output = c->Clustering(cloud, clusterTolerance, minSize, maxSize);
    return;
}

/**
 * @brief classicLidarDetectorGridGroundFilter 函数
 * @param[IN] param20, PointCloudsDetector类的对象的指针
 * @param[IN] input20, 过滤参数输入
 * @param[OUT] output20, 过滤后的点云矩阵输出
 
 * @cn_name: CropBox过滤

 * @granularity: atomic

 * @tag: perception
 */
void classicLidarDetectorGridGroundFilter(ParamStruct20 &param20, InputStruct20 &input20, OutputStruct20 &output20)
{
    PointCloudsDetector *c = param20.c;
    const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = input20.cloud;
    int normal_estimation_Ksearch = input20.normal_estimation_Ksearch;
    float slope_rate = input20.slope_rate;
    float normal_thereshold = input20.normal_thereshold;
    int grid_height_mode = input20.grid_height_mode;
    float car_height = input20.car_height;
    float grid_size = input20.grid_size;
    float right_left_distance = input20.right_left_distance;
    float back_distance = input20.back_distance;
    float front_distance = input20.front_distance;
    output20.output =
        c->GridGroundFilterSimple(cloud, normal_estimation_Ksearch, slope_rate, normal_thereshold, grid_height_mode, car_height, grid_size, right_left_distance, back_distance, front_distance);
    return;
}

/**
 * @brief classicLidarDetectorRenderBox 函数
 * @param[IN] param21, PointCloudsDetector类的对象
 * @param[IN] input21, 渲染设置输入
 * @param[OUT] output21, 空输出
 
 * @cn_name: 3D视图立方体渲染

 * @granularity: atomic
 
 * @tag: perception
 */
void classicLidarDetectorRenderBox(ParamStruct21 &param21, InputStruct21 &input21, OutputStruct21 &output21)
{
    PointCloudsDetector *c = param21.c;
    int id = input21.id;
    Color color = input21.color;
    float opacity = input21.opacity;
    c->renderBox(input21.box, id, color, opacity);
    return;
}

/**
 * @brief classicLidarDetectorRenderPcaBox 函数
 * @param[IN] param22, PointCloudsDetector类的对象
 * @param[IN] input22, 渲染参数输入
 * @param[OUT] output22, 空输出
 
 * @cn_name: PCA立方体渲染

 * @granularity: atomic
 
 * @tag: perception
 */
void classicLidarDetectorRenderPcaBox(ParamStruct22 &param22, InputStruct22 &input22, OutputStruct22 &output22)
{
    PointCloudsDetector *c = param22.c;
    int id = input22.id;
    Color color = input22.color;
    float opacity = input22.opacity;
    c->renderPcaBox(input22.box, id, color, opacity);
    return;
}


/**
 * @brief classicLidarDetectoGridRANSAC 函数
 * @param[IN] param25, PointCloudsDetector类的对象的指针
 * @param[IN] input25, 点云指针和RANSAC算法参数
 * @param[OUT] output25,地面点id数组
 
 * @cn_name: 随机抽样一致方法

 * @granularity: atomic

 * @tag: perception
 */
void classicLidarDetectoGridRANSAC(ParamStruct25 &param25, InputStruct25 &input25, OutputStruct25 &output25)
{
    PointCloudsDetector *c = param25.c;
    float ransac_threshold = input25.ransac_threshold;
    const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = input25.cloud;
    int normal_estimation_Ksearch = input25.normal_estimation_Ksearch;
    float slope_rate = input25.slope_rate;
    float normal_thereshold = input25.normal_thereshold;
    int grid_height_mode = input25.grid_height_mode;
    float car_height = input25.car_height;
    float grid_size = input25.grid_size;
    float right_left_distance = input25.right_left_distance;
    float back_distance = input25.back_distance;
    float front_distance = input25.front_distance;
    output25.output =
        c->GridRANSAC(cloud, ransac_threshold, normal_estimation_Ksearch, slope_rate, normal_thereshold, grid_height_mode, car_height, grid_size, right_left_distance, back_distance, front_distance);
    return;
}

/**
 * @brief classicLidarDetectoGridPatch 函数
 * @param[IN] param26, PointCloudsDetector类的对象的指针
 * @param[IN] input26，点云指针和Patchwork算法参数
 * @param[OUT] output26，地面点id数组
 
 * @cn_name: patchwork方法

 * @granularity: atomic

 * @tag: perception
 */
void classicLidarDetectoGridPatch(ParamStruct26 &param26, InputStruct26 &input26, OutputStruct26 &output26)
{
    PointCloudsDetector *c = param26.c;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = input26.point_cloudi;
    int normal_estimation_Ksearch = input26.normal_estimation_Ksearch;
    float slope_rate = input26.slope_rate;
    float normal_thereshold = input26.normal_thereshold;
    int grid_height_mode = input26.grid_height_mode;
    float car_height = input26.car_height;
    float grid_size = input26.grid_size;
    float right_left_distance = input26.right_left_distance;
    float back_distance = input26.back_distance;
    float front_distance = input26.front_distance;
    output26.output = c->GridPatchworkpp(cloud, normal_estimation_Ksearch, slope_rate, normal_thereshold, grid_height_mode, car_height, grid_size, right_left_distance, back_distance, front_distance);
    return;
}
}