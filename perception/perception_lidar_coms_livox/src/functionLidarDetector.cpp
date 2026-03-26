/**
 * @brief LidarDetector功能
 * @file functionalLidarDetector功能.cpp
 * @version 0.0.1
 * @author Tianqi Ke (ktq23@mails.tsinghua.edu.cn)
 * @date 2023-12-11
 */
#include "functionLidarDetector.h"

namespace perception{

/**
 * @brief classicLidarDetectorFind_Z_value 函数
 * @param[IN] param1, PointCloudsDetector类的对象
 * @param[IN] input1, 点云指针
 * @param[OUT] output1, 空输出
 * @cn_name 点云查找z坐标极值
 * @granularity atomic
 * @tag perception
 */
void classicLidarDetectorFind_Z_value(ParamStruct01 &param01, InputStruct01 &input01, OutputStruct01 &output01)
{
    PointCloudsDetector &c = param01.c;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = input01.cloud;
    c.find_Z_value(cloud);
    return;
}

/**
 * @brief classicLidarDetectorPassThroughFilter 函数
 * @param[IN] param2, PointCloudsDetector类的对象
 * @param[IN] input2, 点云指针和z的界限
 * @param[OUT] output2, 点云指针
 * @cn_name 点云直通滤波
 * @granularity atomic
 * @tag perception
 */
void classicLidarDetectorPassThroughFilter(ParamStruct02 &param02, InputStruct02 &input02, OutputStruct02 &output02)
{
    PointCloudsDetector &c = param02.c;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = input02.cloud;
    double z_max = input02.z_max;
    double z_min = input02.z_min;
    output02.output = c.PassThroughFilter(cloud, z_max, z_min);
    return;
}

/**
 * @brief classicLidarDetectorPointcloud_to_grid 函数
 * @param[IN] param3, PointCloudsDetector类的对象
 * @param[IN] input3, 点云指针和分辨率
 * @param[OUT] 栅格地图
 * @cn_name 栅格化点云
 * @granularity atomic
 * @tag perception
 */
void classicLidarDetectorPointcloud_to_grid(ParamStruct03 &param03, InputStruct03 &input03, OutputStruct03 &output03)
{
    PointCloudsDetector &c = param03.c;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = input03.cloud;
    double map_resolution = input03.map_resolution;
    output03.Pos_array = c.Pointcloud_to_grid(cloud, map_resolution);
    return;
}

/**
 * @brief classicLidarDetectorInputcloud 函数
 * @param[IN] param4, PointCloudsDetector类的对象
 * @param[IN] input4, 文件编号和路径
 * @param[OUT] output4, 点云指针
 * @cn_name 读取点云数据
 * @granularity atomic
 * @tag perception
 */
void classicLidarDetectorInputcloud(ParamStruct04 &param04, InputStruct04 &input04, OutputStruct04 &output04)
{
    PointCloudsDetector &c = param04.c;
    int i = input04.i;
    // std::string folderPath = input04.folderPath;
    output04.output = c.inputcloud(i);
    return;
}

/**
 * @brief classicLidarDetectorInitCamera 函数
 * @param[IN] param5, PointCloudsDetector类的对象
 * @param[IN] input5, 姿态参数结构体
 * @param[OUT] output5, 空输出
 * @cn_name 相机初始化姿态
 * @granularity atomic
 * @tag perception
 */
void classicLidarDetectorInitCamera(ParamStruct05 &param05, InputStruct05 &input05, OutputStruct05 &output05)
{
    PointCloudsDetector &c = param05.c;
    c.initCamera(input05.setAngle);
    return;
}

/**
 * @brief classicLidarDetectorInitView 函数
 * @param[IN] param6, PointCloudsDetector类的对象
 * @param[IN] input6, 空输入
 * @param[OUT] output6, 空输出
 * @cn_name 点云初始化视图
 * @granularity atomic
 * @tag perception
 */
void classicLidarDetectorInitView(ParamStruct06 &param06, InputStruct06 &input06, OutputStruct06 &output06)
{
    PointCloudsDetector &c = param06.c;
    c.initView();
    return;
}

/**
 * @brief classicLidarDetectorCheckAlive 函数
 * @param[IN] param7, PointCloudsDetector类的对象
 * @param[IN] input7, 空输入
 * @param[OUT] output7, bool类型，判断是否正常工作
 * @cn_name 点云判定可视窗口活动
 * @granularity atomic
 * @tag perception
 */
void classicLidarDetectorCheckAlive(ParamStruct07 &param07, InputStruct07 &input07, OutputStruct07 &output07)
{
    PointCloudsDetector &c = param07.c;
    output07.isAlive = c.checkAlive();
    return;
}

/**
 * @brief classicLidarDetectorReceiveCloud 函数
 * @param[IN] param8, PointCloudsDetector类的对象
 * @param[IN] input8, 点云指针
 * @param[OUT] output8, 空输出
 * @cn_name 存储点云数据
 * @granularity atomic
 * @tag perception
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
 * @cn_name 渲染点云数据
 * @granularity atomic
 * @tag perception
 */
void classicLidarDetectorRPointCloud(ParamStruct09 &param09, InputStruct09 &input09, OutputStruct09 &output09)
{
    PointCloudsDetector &c = param09.c;
    c.rPointCloud(input09.cloud);
    return;
}

/**
 * @brief classicLidarDetectorRenderPointCloud 函数
 * @param[IN] param10, PointCloudsDetector类的对象
 * @param[IN] input10, 渲染设置
 * @param[OUT] output10, 空输出
 * @cn_name 显示点云数据
 * @granularity atomic
 * @tag perception
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
 * @cn_name 点云互换xy
 * @granularity atomic
 * @tag perception
 */
void classicLidarDetectorSwapXAndY(ParamStruct11 &param11, InputStruct11 &input11, OutputStruct11 &output11)
{
    PointCloudsDetector &c = param11.c;
    c.swapXAndY(input11.cloud);
    return;
}

/**
 * @brief classicLidarDetectorSwapYAndX 函数
 * @param[IN] param12, PointCloudsDetector类的对象
 * @param[IN] input12, 点云输入
 * @param[OUT] output12, 空输出
 * @cn_name 点云互换xy
 * @granularity atomic
 * @tag perception
 */
void classicLidarDetectorSwapYAndX(ParamStruct12 &param12, InputStruct12 &input12, OutputStruct12 &output12)
{
    PointCloudsDetector &c = param12.c;
    c.swapYAndX(input12.cloud);
    return;
}

/**
 * @brief classicLidarDetectorNumPoints 函数
 * @param[IN] param13, PointCloudsDetector类的对象
 * @param[IN] input13, 点云输入
 * @param[OUT] output13, 空输出
 * @cn_name 统计点云数量
 * @granularity atomic
 * @tag perception
 */
void classicLidarDetectorNumPoints(ParamStruct13 &param13, InputStruct13 &input13, OutputStruct13 &output13)
{
    PointCloudsDetector &c = param13.c;
    c.numPoints(input13.cloud);
    return;
}

/**
 * @brief classicLidarDetectorGetViewer 函数
 * @param[IN] param14, PointCloudsDetector类的对象
 * @param[IN] input14, 空输入
 * @param[OUT] output14, 点云输出
 * @cn_name 获取可视化指针
 * @granularity atomic
 * @tag perception
 */
void classicLidarDetectorGetViewer(ParamStruct14 &param14, InputStruct14 &input14, OutputStruct14 &output14)
{
    PointCloudsDetector &c = param14.c;
    output14.output = c.getViewer();
    return;
}

/**
 * @brief classicLidarDetectorClipPlane 函数
 * @param[IN] param15, PointCloudsDetector类的对象
 * @param[IN] input15, 点云裁减的输入
 * @param[OUT] output15, 裁减后点云输出
 * @cn_name 点云剪裁
 * @granularity atomic
 * @tag perception
 */
void classicLidarDetectorClipPlane(ParamStruct15 &param15, InputStruct15 &input15, OutputStruct15 &output15)
{
    PointCloudsDetector &c = param15.c;
    pcl::PointCloud<pcl::PointXYZI>::Ptr src_cloud = input15.src_cloud;
    const Eigen::Vector4f plane = input15.plane;
    bool negative = input15.negative;
    output15.output = c.clipPlane(src_cloud, plane, negative);
    return;
}

/**
 * @brief classicLidarDetectorFilterCloud 函数
 * @param[IN] param16, PointCloudsDetector类的对象
 * @param[IN] input16, classicLidarDetectorFilterCloud 的输出参数struct
 * @param[OUT] output16, classicLidarDetectorFilterCloud 的输出参数struct
 * @cn_name 预处理点云数据
 * @granularity atomic
 * @tag perception
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
 * @brief classicLidarDetectorRmvNearPoints 函数
 * @param[IN] param17, PointCloudsDetector类的对象
 * @param[IN] input17, 点云和剔除范围
 * @param[OUT] output17, 点云输出
 * @cn_name 点云剔除
 * @granularity atomic
 * @tag perception
 */
void classicLidarDetectorRmvNearPoints(ParamStruct17 &param17, InputStruct17 &input17, OutputStruct17 &output17)
{
    PointCloudsDetector &c = param17.c;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = input17.cloud;
    output17.output = c.rmvNearPoints(cloud, input17.minPoint, input17.maxPoint);
    return;
}

/**
 * @brief classicLidarDetectorRANSAC3d 函数
 * @param[IN] param18, PointCloudsDetector类的对象
 * @param[IN] input18, 点云分割输入
 * @param[OUT] output18, 分割后点云输出
 * @cn_name 点云分割
 * @granularity atomic
 * @tag perception
 */
void classicLidarDetectorRANSAC3d(ParamStruct18 &param18, InputStruct18 &input18, OutputStruct18 &output18)
{
    PointCloudsDetector &c = param18.c;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = input18.cloud;
    int maxIterations = input18.maxIterations;
    float distanceThreshold = input18.distanceThreshold;
    output18.output = c.RANSAC3d(cloud, maxIterations, distanceThreshold);
    return;
}

/**
 * @brief classicLidarDetectorClustering 函数
 * @param[IN] param19, PointCloudsDetector类的对象
 * @param[IN] input19, 聚类输入参数
 * @param[OUT] output19, 聚类后的点云输出
 * @cn_name 点云聚类
 * @granularity atomic
 * @tag perception
 */
void classicLidarDetectorClustering(ParamStruct19 &param19, InputStruct19 &input19, OutputStruct19 &output19)
{
    PointCloudsDetector &c = param19.c;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = input19.cloud;
    float clusterTolerance = input19.clusterTolerance;
    int minSize = input19.minSize;
    int maxSize = input19.maxSize;
    output19.output = c.Clustering(cloud, clusterTolerance, minSize, maxSize);
    return;
}

/**
 * @brief classicLidarDetectorGridGroundFilter 函数
 * @param[IN] param20, PointCloudsDetector类的对象
 * @param[IN] input20, 过滤参数输入
 * @param[OUT] output20, 过滤后的点云矩阵输出
 * @cn_name 点云地面过滤
 * @granularity atomic
 * @tag perception
 */
void classicLidarDetectorGridGroundFilterSimple(ParamStruct20 &param20, InputStruct20 &input20, OutputStruct20 &output20)
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
 * @cn_name PCL渲染立方体
 * @granularity atomic
 * @tag perception
 */
void classicLidarDetectorRenderBox(ParamStruct21 &param21, InputStruct21 &input21, OutputStruct21 &output21)
{
    PointCloudsDetector &c = param21.c;
    int id = input21.id;
    Color color = input21.color;
    float opacity = input21.opacity;
    c.renderBox(input21.box, id, color, opacity);
    return;
}

/**
 * @brief classicLidarDetectorRenderPcaBox 函数
 * @param[IN] param22, PointCloudsDetector类的对象
 * @param[IN] input22, 渲染参数输入
 * @param[OUT] output22, 空输出
 * @cn_name PCA渲染
 * @granularity atomic
 * @tag perception
 */
void classicLidarDetectorRenderPcaBox(ParamStruct22 &param22, InputStruct22 &input22, OutputStruct22 &output22)
{
    PointCloudsDetector &c = param22.c;
    int id = input22.id;
    Color color = input22.color;
    float opacity = input22.opacity;
    c.renderPcaBox(input22.box, id, color, opacity);
    return;
}

/**
 * @brief classicLidarDetectoRun 函数
 * @param[IN] param23, PointCloudsDetector类的对象
 * @param[IN] input23, 空输入
 * @param[OUT] output23, 点云类运行输出
 * @cn_name 运行点云类
 * @granularity atomic
 * @tag perception
 */
void classicLidarDetectoRun(ParamStruct23 &param23, InputStruct23 &input23, OutputStruct23 &output23)
{
    PointCloudsDetector *c = param23.c;
    output23.output = c->run();
    return;
}

/**
 * @brief classicLidarDetectoTest 函数
 * @param[IN] param24, PointCloudsDetector类的对象
 * @param[IN] input24, 空输入
 * @param[OUT] output24,空输出
 * @cn_name 测试点云类
 * @granularity atomic
 * @tag perception
 */
void classicLidarDetectoTest(ParamStruct24 &param24, InputStruct24 &input24, OutputStruct24 &output24)
{
    PointCloudsDetector &c = param24.c;
    c.test();
    return;
}

/**
 * @brief classicLidarDetectoGridRANSAC 函数
 * @param[IN] param25, PointCloudsDetector类的对象
 * @param[IN] input25, 空输入
 * @param[OUT] output25,空输出
 * @cn_name RANSAC地面提取
 * @granularity atomic
 * @tag perception
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
 * @param[IN] param26, PointCloudsDetector类的对象
 * @param[IN] input26
 * @param[OUT] output26
 * @cn_name Patchwork地面提取
 * @granularity atomic
 * @tag perception
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