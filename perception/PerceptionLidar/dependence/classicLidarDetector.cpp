/*
 *
 * Created on: Aug 25, 2022 15:03
 * Description:
 *
 * Copyright (c) 2022 YH. WANG
 */

#include <algorithm>
#include <classicLidarDetector.h>

//===================global variables=====================
std::mutex g_mtxCloud;
std::mutex g_mtxObjList;
std::mutex g_mtxViewer;
//===================================global tools functions====================================================
bool cmpCloudSize(pcl::PointCloud<pcl::PointXYZI>::Ptr cld1, pcl::PointCloud<pcl::PointXYZI>::Ptr cld2) { return (cld1->points.size()) > (cld2->points.size()); }

PcaBox getBetterBox(PcaBox box1, Box box2)
{
    float pcaBoxRange = box1.boxLength * box1.boxWidth;
    float boxRange = (box2.x_max - box2.x_min) * (box2.y_max - box2.y_min);

    if (pcaBoxRange / boxRange <= 1)
    {
        return box1;
    }
    else
    {
        PcaBox t_box;
        return t_box;
        // t_box.bboxQuaternion
    }
}

//========================PointCloudsDetector Functions==============================================================
PointCloudsDetector::PointCloudsDetector() : viewer(new pcl::visualization::PCLVisualizer("3D Viewer"))
{
    g_mtxViewer.lock();
    this->initView();
    g_mtxViewer.unlock();
}

PointCloudsDetector::~PointCloudsDetector() {}

void PointCloudsDetector::numPoints(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) { std::cout << cloud->points.size() << std::endl; }

pcl::visualization::PCLVisualizer::Ptr PointCloudsDetector::getViewer() { return this->viewer; }

// --- INIT START
void PointCloudsDetector::initCamera(CameraAngle setAngle)
{

    viewer->setBackgroundColor(0, 0, 0);

    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 5;

    switch (setAngle)
    {
    case XY:
        viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
        break;
    case TopDown:
        viewer->setCameraPosition(0, 0, distance, 1, 0, 1);
        break;
    case Side:
        viewer->setCameraPosition(0, -distance, 0, 0, 0, 1);
        break;
    case FPS:
        viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if (setAngle != FPS)
        viewer->addCoordinateSystem(1.0);
}

void PointCloudsDetector::initView()
{
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();

    CameraAngle setAngle = XY;
    initCamera(setAngle);
}
// --- INIT END

bool PointCloudsDetector::checkAlive()
{
    if (!viewer->wasStopped())
        return false;
    else
        return true;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr PointCloudsDetector::inputcloud(int i)
{

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    std::string folder_path = "/home/wang/data/cs55_0630(toqiyan-slope)/"; // pcds

    std::vector<std::string> file_paths;
    boost::filesystem::directory_iterator end_itr;
    for (boost::filesystem::directory_iterator itr(folder_path); itr != end_itr; ++itr)
    {
        if (boost::filesystem::is_regular_file(itr->path()) && itr->path().extension() == ".pcd")
        {
            file_paths.push_back(itr->path().string());
        }
    }
    std::sort(file_paths.begin(), file_paths.end());

    std::string file_name = file_paths[i];
    std::string file_path = file_name;

    char strfilepath[256];
    strcpy(strfilepath, file_path.c_str());
    if (-1 == io::loadPCDFile(strfilepath, *cloud))
    { // 读取.pcd文件
        PCL_ERROR("Could not read the file\n ");
    }
    return cloud;
}

void PointCloudsDetector::receiveCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr input)
{
    g_mtxCloud.lock();
    this->input_cloud = input;
    g_mtxCloud.unlock();
}

// 622
void PointCloudsDetector::swapXAndY(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    // 遍历输入点云，交换每个点的 x 和 y 坐标
    for (auto &point : cloud->points)
    {
        float temp = point.x;
        point.x = -point.y;
        point.y = temp;
    }
}
// 换回来
void PointCloudsDetector::swapYAndX(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    // 遍历输入点云，交换每个点的 x 和 y 坐标
    for (auto &point : cloud->points)
    {
        float temp = point.y;
        point.y = -point.x;
        point.x = temp;
    }
}

pcl::PointCloud<pcl::PointXYZI>::Ptr PointCloudsDetector::clipPlane(pcl::PointCloud<pcl::PointXYZI>::Ptr src_cloud, const Eigen::Vector4f plane, bool negative)
{
    pcl::PlaneClipper3D<pcl::PointXYZI> clipper(plane);
    pcl::PointIndices::Ptr indices(new pcl::PointIndices);
    clipper.clipPointCloud3D(*src_cloud, indices->indices);
    pcl::PointCloud<pcl::PointXYZI>::Ptr dst_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(src_cloud);
    extract.setIndices(indices);
    extract.setNegative(negative);
    extract.filter(*dst_cloud);
    return dst_cloud;
}

// set cloud range of interest
pcl::PointCloud<pcl::PointXYZI>::Ptr PointCloudsDetector::filterCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float filterRes_x, float filterRes_y, float filterRes_z, Eigen::Vector4f minPoint,
                                                                      Eigen::Vector4f maxPoint)
{
    /*   pcl::PointCloud<pcl::PointXYZI>::Ptr turn_cloud(new pcl::PointCloud<pcl::PointXYZI>());
      for (int i=0;i<cloud->points.size();i++)
      {
          pcl::PointXYZI pointTemp;
      pointTemp.x=cloud->points[i].y;
      pointTemp.y=cloud->points[i].x;
      pointTemp.z=cloud->points[i].z;
      pointTemp.intensity=cloud->points[i].intensity;
      turn_cloud->points.push_back(pointTemp);
      } */
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudDownsampled(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudCroped(new pcl::PointCloud<pcl::PointXYZI>());

    auto startTime = std::chrono::steady_clock::now();

    typename pcl::CropBox<pcl::PointXYZI> croped(true);
    croped.setInputCloud(cloud);
    croped.setMin(minPoint);
    croped.setMax(maxPoint);
    croped.filter(*cloudCroped);

    auto midTime = std::chrono::steady_clock::now();
    auto crop_time = std::chrono::duration_cast<std::chrono::milliseconds>(midTime - startTime).count();
    std::cout << "预处理——裁剪时间:" << crop_time << " ms" << std::endl;

    pcl::VoxelGrid<pcl::PointXYZI> sor;
    sor.setInputCloud(cloudCroped);
    sor.setLeafSize(filterRes_x, filterRes_y, filterRes_z);
    sor.filter(*cloudDownsampled);

    auto endTime = std::chrono::steady_clock::now();
    auto downsample_time = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - midTime).count();

    std::cout << "预处理——降采样时间:" << downsample_time << " ms" << std::endl;
    // std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudDownsampled;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr PointCloudsDetector::rmvNearPoints(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    pcl::PointCloud<pcl::PointXYZI>::Ptr nearRemovedCloud(new pcl::PointCloud<pcl::PointXYZI>());

    typename pcl::CropBox<pcl::PointXYZI> croped(true);
    croped.setInputCloud(cloud);
    croped.setMin(minPoint);
    croped.setMax(maxPoint);
    croped.setNegative(true);
    croped.filter(*nearRemovedCloud);

    return nearRemovedCloud;
}

// std::pair<typename pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr>
pcl::PointCloud<pcl::PointXYZI>::Ptr PointCloudsDetector::RANSAC3d(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // input: cloud, maxitr, thr
    std::unordered_set<int> inliersResult;

    srand(time(NULL));

    while (maxIterations--)
    {
        std::unordered_set<int> inliers;

        while (inliers.size() < 3)
        {

            //=======================================
            // add 1 is to avoid divied by zero in some cases
            //=======================================
            inliers.insert(rand() % (cloud->points.size()) + 1);
            auto t_iter = inliers.begin();
            if (cloud->points[*t_iter].z > -0.87 || cloud->points[*t_iter + 1].z > -0.87 || cloud->points[*t_iter + 2].z > -0.87)
            {
                continue;
            }
        }

        float x1, y1, z1, x2, y2, z2, x3, y3, z3;
        auto iter = inliers.begin();
        x1 = cloud->points[*iter].x;
        y1 = cloud->points[*iter].y;
        z1 = cloud->points[*iter].z;

        iter++;
        x2 = cloud->points[*iter].x;
        y2 = cloud->points[*iter].y;
        z2 = cloud->points[*iter].z;

        iter++;
        x3 = cloud->points[*iter].x;
        y3 = cloud->points[*iter].y;
        z3 = cloud->points[*iter].z;

        float a = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
        float b = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
        float c = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
        float d = -(a * x1 + b * y1 + c * z1);

        for (int index = 0; index < cloud->points.size(); index++) // calclulate other points' dists belong inliers or outliers
        {
            if (inliers.count(index) > 0)
            {
                continue;
            }
            pcl::PointXYZI point = cloud->points[index];
            float x4 = point.x;
            float y4 = point.y;
            float z4 = point.z;

            float dist = fabs(a * x4 + b * y4 + c * z4 + d) / sqrt(a * a + b * b + c * c);

            if (dist <= distanceThreshold)
            {
                inliers.insert(index);
            }
        }
        if (inliers.size() > inliersResult.size())
            inliersResult = inliers;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudInliers(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZI>());

    for (int index = 0; index < cloud->points.size(); index++)
    {
        pcl::PointXYZI point = cloud->points[index];
        if (inliersResult.count(index))
            cloudInliers->points.push_back(point);
        else
            cloudOutliers->points.push_back(point);
    }

    // typename pcl::PointCloud<NormalType>::Ptr model_normals (new pcl::PointCloud<NormalType> ()); //normal  vec, to be global
    // typedef pcl::PointXYZRGBA PointType;            //PointXYZRGBA数据结构
    // typedef pcl::Normal NormalType;                      //法线结构
    // typename pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
    // norm_est.setKSearch (10);                                   //设置K邻域搜索阀值为10个点
    // norm_est.setInputCloud (cloudInliers);          //设置输入点云
    // norm_est.compute (*model_normals);          //计算点云法线

    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segResult(cloudOutliers, cloudInliers);
    std::cout << "finish ransac" << std::endl;
    // return segResult;
    return cloudOutliers;
}

void PointCloudsDetector::renderBox(Box box, int id, Color color, float opacity)
{
    if (opacity > 1.0)
        opacity = 1.0;
    if (opacity < 0.0)
        opacity = 0.0;

    std::string cube = "box" + std::to_string(id);
    // viewer->addCube(box.bboxTransform, box.bboxQuaternion, box.cube_length, box.cube_width, box.cube_height, cube);
    viewer->addCube(box.x_min, box.x_max, box.y_min, box.y_max, box.z_min, box.z_max, color.r, color.g, color.b, cube);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, cube);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, cube);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, cube);

    std::string cubeFill = "boxFill" + std::to_string(id);
    // viewer->addCube(box.bboxTransform, box.bboxQuaternion, box.cube_length, box.cube_width, box.cube_height, cubeFill);
    viewer->addCube(box.x_min, box.x_max, box.y_min, box.y_max, box.z_min, box.z_max, color.r, color.g, color.b, cubeFill);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, cubeFill);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, cubeFill);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity * 0.3, cubeFill);
}

void PointCloudsDetector::renderPcaBox(PcaBox box, int id, Color color, float opacity)
{
    if (opacity > 1.0)
        opacity = 1.0;
    if (opacity < 0.0)
        opacity = 0.0;

    std::string cube = "box" + std::to_string(id);
    viewer->addCube(box.center, box.rotation, box.boxLength, box.boxWidth, box.boxHeight, cube);
    // viewer->addCube(box.bboxTransform, box.bboxQuaternion, box.boxLength, box.boxWidth, box.boxHeight, cube);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, cube);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, cube);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, cube);

    // std::string cubeFill = "boxFill" + std::to_string(id);
    // viewer->addCube(box.center, box.rotation, box.boxLength, box.boxWidth, box.boxHeight, cubeFill);
    // // viewer->addCube(box.bboxTransform, box.bboxQuaternion, box.boxLength, box.boxWidth, box.boxHeight, cubeFill);
    // viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
    //                                     pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, cubeFill);
    // viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, cubeFill);
    // viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity * 0.3, cubeFill);
}

void PointCloudsDetector::rPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud)
{
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> single_cloud(cloud, 0, 205, 205);
    viewer->addPointCloud<pcl::PointXYZI>(cloud, single_cloud, "cloud");
}

void PointCloudsDetector::renderPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, std::string name, Color color)
{
    if (cloud->points.size())
    {
        if (color.r == -1)
        {
            // Select color based off of cloud intensity根据点云强度选择颜色
            pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(cloud, "intensity");
            viewer->addPointCloud<pcl::PointXYZI>(cloud, intensity_distribution, name);
        }
        else
        {
            // Select color based off input value根据输入值选择颜色
            viewer->addPointCloud<pcl::PointXYZI>(cloud, name);
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, name);
        }

        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, name);
    }
}

void PointCloudsDetector::find_Z_value(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    std::cout << "点云数据点数：" << cloud->points.size() << std::endl;
    /*
      for(int i=0;i<cloud->points.size();i++)
    {
    double z=-cloud->points[i].z;
    cloud->points[i].z=z;
    }
    */

    max_z = cloud->points[0].z;
    min_z = cloud->points[0].z;
    for (int i = 0; i < cloud->points.size() - 1; i++)
    {
        if (cloud->points[i].z > max_z)
        {
            max_z = cloud->points[i].z;
        }
        if (cloud->points[i].z < min_z)
        {
            min_z = cloud->points[i].z;
        }
    }
    std::cout << "orig max_z=" << max_z << ",min_z=" << min_z << std::endl;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr PointCloudsDetector::PassThroughFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, double z_max, double z_min)
{

    /*
       std::cout << "直通滤波前点云数据点数：" << cloud->points.size() << std::endl;
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_after_PassThrough(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PassThrough<pcl::PointXYZI> passthrough;
        passthrough.setInputCloud(cloud);//输入点云
        passthrough.setFilterFieldName("z");//对z轴进行操作
        passthrough.setFilterLimits(min_z + z_min, max_z - z_max);//设置直通滤波器操作范围
        passthrough.setFilterLimitsNegative(false);//true表示保留范围外，false表示保留范围内
        passthrough.filter(*cloud_after_PassThrough);//执行滤波，过滤结果保存在 cloud_after_PassThrough
        std::cout << "直通滤波后点云数据点数：" << cloud_after_PassThrough->points.size() << std::endl;
        return cloud_after_PassThrough;
    */
    return cloud;
}

vector<int> PointCloudsDetector::Pointcloud_to_grid(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, double map_resolution)
{

    int repeat_num = 0;
    std::vector<int> Pos_array;
    std::vector<std::pair<int, int>> coordinates;
    if (cloud->points.empty())
    {
        std::cout << "pcd is empty!\n" << std::endl;
    }
    double x_length = 30;
    double y_length = 16;
    int width = int(x_length / map_resolution);
    int height = int(y_length / map_resolution);
    // 存入栅格地图长和宽
    Pos_array.push_back(y_length);
    Pos_array.push_back(x_length);
    // 计算激光点在栅格地图中的坐标
    for (int iter = 0; iter < cloud->points.size(); iter++)
    {
        int i = int((cloud->points[iter].y - (-y_length / 2)) / map_resolution);
        int j = int((cloud->points[iter].x - (-x_length / 2)) / map_resolution);
        std::pair<int, int> p = {i, j};
        if (std::find(coordinates.begin(), coordinates.end(), p) == coordinates.end())
        {
            coordinates.push_back(p);
            Pos_array.push_back(i);
            Pos_array.push_back(j);
            repeat_num++;
        }
    }

    std::cout << "去重后的栅格数:" << Pos_array.size() / 2 - 1 << std::endl;
    std::cout << "重复的栅格数:" << repeat_num << std::endl;
    return Pos_array;
}

// --- Grid Ground Filter -----------------------------------------------------------
std::vector<int> PointCloudsDetector::GridGroundFilterSimple(const pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud, int normal_estimation_Ksearch = 10, float slope_rate = 1,
                                                             float normal_thereshold = 0.9, int grid_height_mode = 1,
                                                             float car_height = -0.3,          // 定义车周地面高度
                                                             float grid_size = 0.4,            // 栅格大小
                                                             float right_left_distance = 20.0, // 左或右距离
                                                             float back_distance = 20.0,       // 后方距离
                                                             float front_distance = 60.0)      // 前方距离
{
    int ViewOption = 1;
    int normal_estimation = 1;
    // 输出数据集
    ground_point.reset(new pcl::PointCloud<pcl::PointXYZI>());
    typename pcl::CropBox<pcl::PointXYZI> croped;
    croped.setInputCloud(filtered_cloud);
    croped.setMin(Eigen::Vector4f(-back_distance, -right_left_distance, car_height, 1));
    croped.setMax(Eigen::Vector4f(front_distance, right_left_distance, 3, 1));
    croped.setNegative(true);
    croped.filter(*ground_point);

    nonground_point.reset(new pcl::PointCloud<pcl::PointXYZI>());
    typename pcl::CropBox<pcl::PointXYZI> croped_;
    croped_.setInputCloud(filtered_cloud);
    croped_.setMin(Eigen::Vector4f(-back_distance, -right_left_distance, car_height, 1));
    croped_.setMax(Eigen::Vector4f(front_distance, right_left_distance, 3, 1));
    croped_.setNegative(false);
    croped_.filter(*nonground_point);

    // 定义最大梯度变化
    float slope = slope_rate * grid_size;

    int origin_in_grid_x = back_distance / grid_size;
    int origin_in_grid_y = right_left_distance / grid_size;
    int num_cells_x = (back_distance + front_distance) / grid_size;
    int num_cells_y = (right_left_distance + right_left_distance) / grid_size;
    std::vector<std::vector<GridInfos>> pointcloud_grid(num_cells_x, std::vector<GridInfos>(num_cells_y));

    std::vector<int> grid_map_road;
    std::cout << "grid_size: " << grid_size << std::endl;

    int point_indice = 0;
    // 遍历点云中的每个点
    for (const auto &point : nonground_point->points)
    {
        int cell_x = static_cast<int>((point.x + back_distance) / grid_size);
        int cell_y = static_cast<int>((point.y + right_left_distance) / grid_size);
        if (0 <= cell_x && cell_x < num_cells_x && 0 <= cell_y && cell_y < num_cells_y)
        {
            pointcloud_grid[cell_x][cell_y].point_indices.push_back(point_indice);
            ++pointcloud_grid[cell_x][cell_y].point_num;
        }
        ++point_indice;
    }
    std::cout << "总的输入点云数量：" << point_indice << std::endl;

    for (int i = 0; i < num_cells_x; ++i)
    {
        for (int j = 0; j < num_cells_y; ++j)
        {
            if (pointcloud_grid[i][j].point_num >= 2)
            {
                grid_map_road.push_back(i);
                grid_map_road.push_back(j);
                grid_map_road.push_back(1);
                // grid_map_road.push_back(i - origin_in_grid_x) * grid_size + 0.5 * grid_size);
                // grid_map_road.push_back((j - origin_in_grid_y) * grid_size + 0.5 * grid_size);
            }
        }
    }

    return grid_map_road;
}

std::vector<int> PointCloudsDetector::GridRANSAC(const pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud, float ransac_threshold, int normal_estimation_Ksearch, float slope_rate,
                                                 float normal_thereshold, int grid_height_mode, float car_height, float grid_size, float right_left_distance, float back_distance,
                                                 float front_distance) // 定义车周地面高度
{
    ground_point.reset(new pcl::PointCloud<pcl::PointXYZI>());
    nonground_point.reset(new pcl::PointCloud<pcl::PointXYZI>());

    typename pcl::SACSegmentation<pcl::PointXYZI> seg;
    typename pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    typename pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    seg.setInputCloud(filtered_cloud);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(ransac_threshold); // 设置 RANSAC 阈值，用于确定离平面多远的点应被视为离群点
    seg.segment(*inliers, *coefficients);

    // Extract the inliers (ground points)
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(filtered_cloud);
    extract.setIndices(inliers);
    extract.setNegative(false); // 设置为 false 以提取平面内的点
    extract.filter(*ground_point);

    // Extract the outliers (non-ground points)
    extract.setNegative(true); // 设置为 true 以提取平面外的点
    extract.filter(*nonground_point);

    // 定义最大梯度变化
    float slope = slope_rate * grid_size;

    int origin_in_grid_x = back_distance / grid_size;
    int origin_in_grid_y = right_left_distance / grid_size;
    int num_cells_x = (back_distance + front_distance) / grid_size;
    int num_cells_y = (right_left_distance + right_left_distance) / grid_size;
    std::vector<std::vector<GridInfos>> pointcloud_grid(num_cells_x, std::vector<GridInfos>(num_cells_y));

    std::vector<int> grid_map_road;
    std::cout << "grid_size: " << grid_size << std::endl;

    int point_indice = 0;
    // 遍历点云中的每个点
    for (const auto &point : nonground_point->points)
    {
        int cell_x = static_cast<int>((point.x + back_distance) / grid_size);
        int cell_y = static_cast<int>((point.y + right_left_distance) / grid_size);
        if (0 <= cell_x && cell_x < num_cells_x && 0 <= cell_y && cell_y < num_cells_y)
        {
            pointcloud_grid[cell_x][cell_y].point_indices.push_back(point_indice);
            ++pointcloud_grid[cell_x][cell_y].point_num;
        }
        ++point_indice;
    }
    std::cout << "总的输入点云数量：" << point_indice << std::endl;

    for (int i = 0; i < num_cells_x; ++i)
    {
        for (int j = 0; j < num_cells_y; ++j)
        {
            if (pointcloud_grid[i][j].point_num >= 2)
            {
                grid_map_road.push_back(i);
                grid_map_road.push_back(j);
                grid_map_road.push_back(1);
                // grid_map_road.push_back(i - origin_in_grid_x) * grid_size + 0.5 * grid_size);
                // grid_map_road.push_back((j - origin_in_grid_y) * grid_size + 0.5 * grid_size);
            }
        }
    }

    return grid_map_road;
}

void PointCloudsDetector::read_pcd_to_eigen(pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloudi, Eigen::MatrixXf &cloud, float right_left_distance, float back_distance, float front_distance)
{
    cloud.resize(point_cloudi->size(), 4);
    cloud.resize(point_cloudi->size(), 4);
    size_t j = 0;
    for (size_t i = 0; i < point_cloudi->size(); ++i)
    {
        if ((point_cloudi->points[i].y < right_left_distance) && ((point_cloudi->points[i].y + right_left_distance) > 0) && (point_cloudi->points[i].x > back_distance) &&
            ((point_cloudi->points[i].x < front_distance)))
        {
            cloud.row(j) << point_cloudi->points[i].x, point_cloudi->points[i].y, point_cloudi->points[i].z, point_cloudi->points[i].intensity;
            j++;
        }
    }
    Eigen::MatrixXf new_cloud = cloud.block(0, 0, j, cloud.cols());
    cloud.resize(0, 0);
    cloud.resize(j, 4);
    cloud = new_cloud;
    Eigen::VectorXf mean_values = cloud.colwise().mean();
    Eigen::VectorXf max_values = cloud.colwise().maxCoeff();
    Eigen::VectorXf min_values = cloud.colwise().minCoeff();
    std::cout << "Mean values of each column:" << std::endl;
    std::cout << mean_values << std::endl;
    std::cout << "Max values of each column:" << std::endl;
    std::cout << max_values << std::endl;
    std::cout << "Min values of each column:" << std::endl;
    std::cout << min_values << std::endl;
}

void PointCloudsDetector::processPointClouds(pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloudi, float right_left_distance, float back_distance, float front_distance)
{
    Eigen::MatrixXf cloud;
    read_pcd_to_eigen(point_cloudi, cloud, right_left_distance, back_distance, front_distance);

    patchwork::Params patchwork_parameters;
    patchwork_parameters.verbose = true;
    patchwork::PatchWorkpp Patchworkpp(patchwork_parameters);

    Patchworkpp.estimateGround(cloud);
    // Get Ground and Nonground
    Eigen::MatrixX3f ground = Patchworkpp.getGround();
    Eigen::MatrixX3f nonground = Patchworkpp.getNonground();
    double time_taken = Patchworkpp.getTimeTaken();

    Eigen::VectorXi ground_idx = Patchworkpp.getGroundIndices();
    Eigen::VectorXi nonground_idx = Patchworkpp.getNongroundIndices();

    // Get centers and normals for patches
    Eigen::MatrixX3f centers = Patchworkpp.getCenters();
    Eigen::MatrixX3f normals = Patchworkpp.getNormals();

    std::cout << "Origianl Points  #: " << cloud.rows() << endl;
    std::cout << "Ground Points    #: " << ground.rows() << endl;
    std::cout << "Nonground Points #: " << nonground.rows() << endl;
    std::cout << "Time Taken : " << time_taken / 1000000 << "(sec)" << endl;
    std::cout << "Press ... \n" << endl;
    std::cout << "\t H  : help" << endl;
    std::cout << "\t N  : visualize the surface normals" << endl;
    std::cout << "\tESC : close the Open3D window" << endl;
    // pcl::PointCloud<pcl::PointXYZ> my_cloud;
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud(point_cloudi);
    ground_point.reset(new pcl::PointCloud<pcl::PointXYZI>());
    nonground_point.reset(new pcl::PointCloud<pcl::PointXYZI>());
    for (int i = 0; i < nonground.rows(); ++i)
    {
        // 给定某个点的 x 值
        float query_x = nonground(i, 0);

        // 定义一个搜索点
        pcl::PointXYZI search_point;
        search_point.x = query_x;

        // 在 KD 树中搜索最近邻点
        std::vector<int> pointIdxNKNSearch(1); // 只搜索一个最近邻点
        std::vector<float> pointNKNSquaredDistance(1);
        if (kdtree.nearestKSearch(search_point, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
        {
            // 打印找到的最近邻点的 i 值
            int nearest_point_index = pointIdxNKNSearch[0];
            float nearest_point_i_value = (*point_cloudi)[nearest_point_index].intensity;
            pcl::PointXYZI non_point;
            non_point.x = nonground(i, 0);
            non_point.y = nonground(i, 1);
            non_point.z = nonground(i, 2);
            non_point.intensity = nearest_point_i_value;
            nonground_point->push_back(non_point);
        }
        else
        {
            std::cerr << "No nearest neighbor found." << std::endl;
        }
    }
    // std::string non_pcd_name = "/home/mayc/patchwork-plusplus/data/0130test/other_pcds/1641634400.939636.pcd";
    // pcl::io::savePCDFileASCII(non_pcd_name, *nonground_point);
    for (int i = 0; i < ground.rows(); ++i)
    {
        float query_x = ground(i, 0);

        // 定义一个搜索点
        pcl::PointXYZI search_point;
        search_point.x = query_x;

        // 在 KD 树中搜索最近邻点
        std::vector<int> pointIdxNKNSearch(1); // 只搜索一个最近邻点
        std::vector<float> pointNKNSquaredDistance(1);
        if (kdtree.nearestKSearch(search_point, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
        {
            // 打印找到的最近邻点的 i 值
            int nearest_point_index = pointIdxNKNSearch[0];
            float nearest_point_i_value = (*point_cloudi)[nearest_point_index].intensity;
            pcl::PointXYZI point;
            point.x = ground(i, 0);
            point.y = ground(i, 1);
            point.z = ground(i, 2);
            point.intensity = nearest_point_i_value;
            ground_point->push_back(point);
            // std::cout << "The intensity value of the nearest neighbor point is: " << nearest_point_i_value << std::endl;
        }
        else
        {
            std::cerr << "No nearest neighbor found." << std::endl;
        }
    }
    // std::string pcd_name = "/home/mayc/patchwork-plusplus/data/0130test/ground_pcds/1641634400.939636.pcd";
    // pcl::io::savePCDFileASCII(pcd_name, *ground_point);
}

std::vector<int> PointCloudsDetector::GridPatchworkpp(pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloudi, int normal_estimation_Ksearch, float slope_rate, float normal_thereshold, int grid_height_mode,
                                                      float car_height,          // 定义车周地面高度
                                                      float grid_size,           // 栅格大小
                                                      float right_left_distance, // 左或右距离
                                                      float back_distance,       // 后方距离
                                                      float front_distance)      // 前方距离
{
    // PatchworkProcessor processor;
    processPointClouds(point_cloudi, right_left_distance, back_distance, front_distance);
    // 定义最大梯度变化
    float slope = slope_rate * grid_size;

    int origin_in_grid_x = back_distance / grid_size;
    int origin_in_grid_y = right_left_distance / grid_size;
    int num_cells_x = (back_distance + front_distance) / grid_size;
    int num_cells_y = (right_left_distance + right_left_distance) / grid_size;
    std::vector<std::vector<GridInfos>> pointcloud_grid(num_cells_x, std::vector<GridInfos>(num_cells_y));

    std::vector<int> grid_map_road;
    std::cout << "grid_size: " << grid_size << std::endl;

    int point_indice = 0;
    // 遍历点云中的每个点
    for (const auto &point : nonground_point->points)
    {
        int cell_x = static_cast<int>((point.x + back_distance) / grid_size);
        int cell_y = static_cast<int>((point.y + right_left_distance) / grid_size);
        if (0 <= cell_x && cell_x < num_cells_x && 0 <= cell_y && cell_y < num_cells_y)
        {
            pointcloud_grid[cell_x][cell_y].point_indices.push_back(point_indice);
            ++pointcloud_grid[cell_x][cell_y].point_num;
        }
        ++point_indice;
    }
    std::cout << "总的输入点云数量：" << point_indice << std::endl;

    for (int i = 0; i < num_cells_x; ++i)
    {
        for (int j = 0; j < num_cells_y; ++j)
        {
            if (pointcloud_grid[i][j].point_num >= 2)
            {
                grid_map_road.push_back(i);
                grid_map_road.push_back(j);
                grid_map_road.push_back(1);
                // grid_map_road.push_back(i - origin_in_grid_x) * grid_size + 0.5 * grid_size);
                // grid_map_road.push_back((j - origin_in_grid_y) * grid_size + 0.5 * grid_size);
            }
        }
    }

    return grid_map_road;
}

// --- Grid Ground Filter -----------------------------------------------------------
std::vector<int> PointCloudsDetector::GridGroundFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud, int normal_estimation_Ksearch = 10, float slope_rate = 1,
                                                       float normal_thereshold = 0.9, int grid_height_mode = 1,
                                                       float car_height = -0.3,          // 定义车周地面高度
                                                       float grid_size = 0.4,            // 栅格大小
                                                       float right_left_distance = 20.0, // 左或右距离
                                                       float back_distance = 20.0,       // 后方距离
                                                       float front_distance = 60.0)      // 前方距离
{
    int ViewOption = 1;
    int normal_estimation = 1;
    // 输出数据集
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

    if (normal_estimation)
    {
        // 估计法线
        // ---------------------------------------------------
        pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
        ne.setInputCloud(filtered_cloud);
        // 定义视点
        ne.setViewPoint(0.0, 0.0, 1000.0);
        // 创建一个空的kdtree对象，并把它传递给法线估计对象
        // 基于给出的输入数据集，kdtree将被建立
        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
        ne.setSearchMethod(tree);

        // 查询元素
        ne.setKSearch(normal_estimation_Ksearch);
        // 计算特征值
        ne.compute(*cloud_normals);
    }

    // 定义最大梯度变化
    float slope = slope_rate * grid_size;

    int origin_in_grid_x = back_distance / grid_size;
    int origin_in_grid_y = right_left_distance / grid_size;
    int num_cells_x = (back_distance + front_distance) / grid_size;
    int num_cells_y = (right_left_distance + right_left_distance) / grid_size;
    std::vector<std::vector<GridInfos>> pointcloud_grid(num_cells_x, std::vector<GridInfos>(num_cells_y));

    std::vector<int> grid_map_road;
    std::cout << "grid_size: " << grid_size << std::endl;

    int point_indice = 0;
    // 遍历点云中的每个点
    for (const auto &point : filtered_cloud->points)
    {
        int cell_x = static_cast<int>((point.x + back_distance) / grid_size);
        int cell_y = static_cast<int>((point.y + right_left_distance) / grid_size);
        if (0 <= cell_x && cell_x < num_cells_x && 0 <= cell_y && cell_y < num_cells_y)
        {
            pointcloud_grid[cell_x][cell_y].point_indices.push_back(point_indice);
            ++pointcloud_grid[cell_x][cell_y].point_num;
        }
        ++point_indice;
    }
    std::cout << "总的输入点云数量：" << point_indice << std::endl;

    // 计算每个栅格的信息
    // ---------------------------------------
    for (int i = 0; i < num_cells_x; ++i)
    {
        for (int j = 0; j < num_cells_y; ++j)
        {
            if (pointcloud_grid[i][j].point_num)
            {
                // 平均值·最大值·最小值
                float sum_height = 0.0;
                float max_height = -1000.0;
                float min_height = 1000.0;
                // 平均法向量
                float sum_vector_x = 0.0;
                float sum_vector_y = 0.0;
                float sum_vector_z = 0.0;

                for (int k = 0; k < pointcloud_grid[i][j].point_num; ++k)
                {
                    // 平均值·最大值·最小值
                    sum_height += filtered_cloud->points[pointcloud_grid[i][j].point_indices[k]].z;
                    if (filtered_cloud->points[pointcloud_grid[i][j].point_indices[k]].z > max_height)
                    {
                        max_height = filtered_cloud->points[pointcloud_grid[i][j].point_indices[k]].z;
                    }
                    if (filtered_cloud->points[pointcloud_grid[i][j].point_indices[k]].z < min_height)
                    {
                        min_height = filtered_cloud->points[pointcloud_grid[i][j].point_indices[k]].z;
                    }
                    // 平均法向量
                    sum_vector_x += cloud_normals->points[pointcloud_grid[i][j].point_indices[k]].normal_x;
                    sum_vector_y += cloud_normals->points[pointcloud_grid[i][j].point_indices[k]].normal_y;
                    sum_vector_z += cloud_normals->points[pointcloud_grid[i][j].point_indices[k]].normal_z;
                }
                // 平均值·最大值·最小值
                pointcloud_grid[i][j].mean_height = sum_height / pointcloud_grid[i][j].point_num;
                pointcloud_grid[i][j].max_height = max_height;
                pointcloud_grid[i][j].min_height = min_height;
                // 平均法向量
                float sum_vector = pow(sum_vector_x * sum_vector_x + sum_vector_y * sum_vector_y + sum_vector_z * sum_vector_z, 0.5);
                pointcloud_grid[i][j].normal_vector_x = sum_vector_x / sum_vector;
                pointcloud_grid[i][j].normal_vector_y = sum_vector_y / sum_vector;
                pointcloud_grid[i][j].normal_vector_z = sum_vector_z / sum_vector;

                // 路边树木
                // if((pointcloud_grid[i][j].min_height > 0.7) && (pointcloud_grid[i][j].point_num > 0)){
                //     pointcloud_grid[i][j].property = 1;
                //     pointcloud_grid[i][j].isvisited = 1;
                //     pointcloud_grid[i][j].plane_height = car_height;
                // }
            }
            else
            {
                // 栅格内无点记为空
                pointcloud_grid[i][j].property = -1;
            }
        }
    }

    //  g_mtxViewer.lock();
    // viewer->removeAllPointClouds();
    // viewer->removeAllShapes();
    // 给盲区赋值
    int blind_region_start_x = origin_in_grid_x - 2 / grid_size;
    int blind_region_start_y = origin_in_grid_y - 1.6 / grid_size;
    int blind_region_end_x = origin_in_grid_x + 4 / grid_size;
    int blind_region_end_y = origin_in_grid_y + 1.6 / grid_size;

    // 赋值高度和属性
    for (int x = blind_region_start_x; x <= blind_region_end_x; ++x)
    {
        for (int y = blind_region_start_y; y <= blind_region_end_y; ++y)
        {
            if (pointcloud_grid[x][y].point_num < 1)
            {
                pointcloud_grid[x][y].mean_height = car_height;
                pointcloud_grid[x][y].max_height = car_height;
                pointcloud_grid[x][y].min_height = car_height;
                pointcloud_grid[x][y].plane_height = car_height;
                pointcloud_grid[x][y].property = 0;
            }
        }
    }

    // 创建地面点云索引对象
    pcl::PointIndices::Ptr groundindices(new pcl::PointIndices);

    int start_x = origin_in_grid_x;
    int start_y = origin_in_grid_y;

    // 采用广度优先搜索的方式，进行连通域的搜索
    // 进行八方向搜索，以梯度变化作为判断准则
    // 定义方向数组，用于遍历相邻的位置
    const int dx[] = {-1, 1, 0, 0, 1, 1, -1, -1};
    const int dy[] = {0, 0, -1, 1, 1, -1, 1, -1};

    // 创建一个队列，用于存储待访问的栅格位置
    std::queue<std::pair<int, int>> q;
    q.push(std::make_pair(start_x, start_y));
    // 将当前位置标记为已访问
    pointcloud_grid[start_x][start_y].property = 1;
    pointcloud_grid[start_x][start_y].isvisited = 1;
    pointcloud_grid[start_x][start_y].plane_height = car_height;
    while (!q.empty())
    {
        std::pair<int, int> curr = q.front();
        q.pop();
        int x = curr.first;
        int y = curr.second;

        // 遍历当前位置的四个相邻位置
        for (int i = 0; i < 4; i++)
        {
            int nx = x + dx[i];
            int ny = y + dy[i];
            // 判断是否在盲区内
            if ((nx > blind_region_start_x) && (nx < blind_region_end_x) && (ny > blind_region_start_y) && (ny < blind_region_end_y) && pointcloud_grid[nx][ny].isvisited == 0)
            {
                q.push(std::make_pair(nx, ny));
                pointcloud_grid[nx][ny].isvisited = 1;
                for (int k = 0; k < pointcloud_grid[nx][ny].point_num; ++k)
                {
                    if ((filtered_cloud->points[pointcloud_grid[nx][ny].point_indices[k]]).z < car_height + slope)
                    {
                        pointcloud_grid[nx][ny].ground_indices.push_back(pointcloud_grid[nx][ny].point_indices[k]);
                        groundindices->indices.push_back(pointcloud_grid[nx][ny].point_indices[k]);
                    }
                }
                if (pointcloud_grid[nx][ny].property == 0)
                {
                    pointcloud_grid[nx][ny].property = 1;
                }
            }
            // 判断相邻位置是否在有效范围内,且未被访问
            else if ((nx >= 0) && (nx < num_cells_x) && (ny >= 0) && (ny < num_cells_y) && pointcloud_grid[nx][ny].isvisited == 0)
            {
                if ((pointcloud_grid[nx][ny].mean_height - pointcloud_grid[x][y].plane_height > -slope) && (pointcloud_grid[nx][ny].mean_height - pointcloud_grid[x][y].plane_height < slope) &&
                    (pointcloud_grid[nx][ny].max_height - pointcloud_grid[nx][ny].min_height < slope))
                {

                    // 高度变化在阈值内
                    if ((pointcloud_grid[x][y].property == 1) || // 路面到路面
                        (pointcloud_grid[x][y].property == -1))  // 空洞到路面
                    {
                        for (int k = 0; k < pointcloud_grid[nx][ny].point_num; ++k)
                        {

                            pointcloud_grid[nx][ny].ground_indices.push_back(pointcloud_grid[nx][ny].point_indices[k]);
                            groundindices->indices.push_back(pointcloud_grid[nx][ny].point_indices[k]);
                        }
                        pointcloud_grid[nx][ny].plane_height = pointcloud_grid[nx][ny].mean_height;
                        pointcloud_grid[nx][ny].property = 1;
                        pointcloud_grid[nx][ny].isvisited = 1;
                        q.push(std::make_pair(nx, ny));
                    }
                }
                else if ((pointcloud_grid[nx][ny].max_height - pointcloud_grid[nx][ny].min_height < slope) && (pointcloud_grid[nx][ny].point_num > 0))
                {
                    // 高度变化超出阈值，但是极差在阈值内，说明当前搜索的栅格是平面，放宽差值搜索的阈值
                    if ((pointcloud_grid[nx][ny].mean_height - pointcloud_grid[x][y].plane_height > -slope * 3) &&
                        (pointcloud_grid[nx][ny].mean_height - pointcloud_grid[x][y].plane_height < slope) * 3)
                    {
                        // viewer->addCube(nx * grid_size - back_distance, nx * grid_size - back_distance + grid_size,
                        //                 ny * grid_size - right_left_distance, ny * grid_size - right_left_distance + grid_size,
                        //                 pointcloud_grid[nx][ny].max_height, pointcloud_grid[nx][ny].max_height + 0.1,
                        //                 1, 0, 0, "1_" + std::to_string(nx) + "_" + std::to_string(ny));

                        for (int k = 0; k < pointcloud_grid[nx][ny].point_num; ++k)
                        {
                            pointcloud_grid[nx][ny].ground_indices.push_back(pointcloud_grid[nx][ny].point_indices[k]);
                            groundindices->indices.push_back(pointcloud_grid[nx][ny].point_indices[k]);
                        }
                        pointcloud_grid[nx][ny].plane_height = pointcloud_grid[nx][ny].mean_height;
                        pointcloud_grid[nx][ny].property = 1;
                        pointcloud_grid[nx][ny].isvisited = 1;
                        q.push(std::make_pair(nx, ny));
                    }
                    else
                    {
                        for (int k = 0; k < pointcloud_grid[nx][ny].point_num; ++k)
                        {
                            if ((filtered_cloud->points[pointcloud_grid[nx][ny].point_indices[k]].z - pointcloud_grid[x][y].plane_height > -slope) &&
                                (filtered_cloud->points[pointcloud_grid[nx][ny].point_indices[k]].z - pointcloud_grid[x][y].plane_height < slope))
                            {
                                pointcloud_grid[nx][ny].ground_indices.push_back(pointcloud_grid[nx][ny].point_indices[k]);
                                groundindices->indices.push_back(pointcloud_grid[nx][ny].point_indices[k]);
                            }
                        }
                        // 障碍物
                        pointcloud_grid[nx][ny].plane_height = pointcloud_grid[nx][ny].max_height;
                        pointcloud_grid[nx][ny].property = 3;
                        pointcloud_grid[nx][ny].isvisited = 1;
                    }
                }
                else
                {
                    if (pointcloud_grid[nx][ny].max_height - pointcloud_grid[nx][ny].min_height > slope)
                    {
                        // viewer->addCube(nx * grid_size - back_distance, nx * grid_size - back_distance + grid_size,
                        //                 ny * grid_size - right_left_distance, ny * grid_size - right_left_distance + grid_size,
                        //                 pointcloud_grid[nx][ny].max_height, pointcloud_grid[nx][ny].max_height + 0.1,
                        //                 1, 0, 0, "1_" + std::to_string(nx) + "_" + std::to_string(ny));
                    }
                    // 高度变化超出阈值
                    // 先解决空洞
                    if (pointcloud_grid[nx][ny].point_num == 0)
                    {

                        pointcloud_grid[nx][ny].plane_height = pointcloud_grid[x][y].plane_height;

                        pointcloud_grid[nx][ny].isvisited = 1;
                        pointcloud_grid[nx][ny].property = -1;
                        q.push(std::make_pair(nx, ny));
                    }
                    else
                    {
                        // 非空洞
                        if ((pointcloud_grid[x][y].property == 1) && (pointcloud_grid[nx][ny].max_height - pointcloud_grid[nx][ny].min_height > slope))
                        {
                            // viewer->addCube(nx * grid_size - back_distance, nx * grid_size - back_distance + grid_size,
                            //                 ny * grid_size - right_left_distance, ny * grid_size - right_left_distance + grid_size,
                            //                 pointcloud_grid[nx][ny].max_height, pointcloud_grid[nx][ny].max_height + 0.1,
                            //                 0, 0, 1, "1_" + std::to_string(nx) + "_" + std::to_string(ny));
                            // 判定为障碍物和地面交界处
                            // 首先排除噪点影响
                            int ground_num = 0;
                            int nonground_num = 0;

                            std::queue<std::pair<int, int>> q_obstacle;
                            q_obstacle.push(std::make_pair(nx, ny));
                            // 将交界处标记为已访问
                            pointcloud_grid[nx][ny].property = 2;
                            pointcloud_grid[nx][ny].isvisited = 1;
                            pointcloud_grid[nx][ny].plane_height = pointcloud_grid[nx][ny].max_height;
                            while (!q_obstacle.empty())
                            {
                                std::pair<int, int> curr_obstacle = q_obstacle.front();
                                q_obstacle.pop();
                                int ox = curr_obstacle.first;
                                int oy = curr_obstacle.second;
                                for (int j = 0; j < 4; ++j)
                                {
                                    int nox = ox + dx[j];
                                    int noy = oy + dy[j];
                                    if ((nox >= 0) && (nox < num_cells_x) && (noy >= 0) && (noy < num_cells_y) && pointcloud_grid[nox][noy].isvisited == 0 && pointcloud_grid[nox][noy].point_num != 0)
                                    {
                                        if (pointcloud_grid[ox][oy].plane_height - pointcloud_grid[nox][noy].min_height < slope * 0.1)
                                        {
                                            pointcloud_grid[nox][noy].plane_height = pointcloud_grid[nox][noy].max_height;
                                            pointcloud_grid[nox][noy].property = 3; // 3为障碍物
                                            pointcloud_grid[nox][noy].isvisited = 1;
                                            q_obstacle.push(std::make_pair(nox, noy));
                                        }
                                    }
                                }
                            }
                        }

                        float sum_height = 0.0;
                        int ground_num = 0;
                        for (int k = 0; k < pointcloud_grid[nx][ny].point_num; ++k)
                        {
                            if ((filtered_cloud->points[pointcloud_grid[nx][ny].point_indices[k]].z - pointcloud_grid[x][y].plane_height < slope * 1.5))
                            {
                                sum_height += filtered_cloud->points[pointcloud_grid[nx][ny].point_indices[k]].z;
                                ++ground_num;
                                pointcloud_grid[nx][ny].ground_indices.push_back(pointcloud_grid[nx][ny].point_indices[k]);
                                groundindices->indices.push_back(pointcloud_grid[nx][ny].point_indices[k]);
                            }
                            else
                            {
                                pointcloud_grid[nx][ny].nonground_indices.push_back(pointcloud_grid[nx][ny].point_indices[k]);
                            }
                        }
                        pointcloud_grid[nx][ny].plane_height = sum_height / ground_num;
                        if (pointcloud_grid[nx][ny].point_num - ground_num < 5)
                        {
                            for (int ii = 0; ii < pointcloud_grid[nx][ny].nonground_indices.size(); ++ii)
                            {
                                pointcloud_grid[nx][ny].ground_indices.push_back(pointcloud_grid[nx][ny].nonground_indices[ii]);
                                groundindices->indices.push_back(pointcloud_grid[nx][ny].nonground_indices[ii]);
                            }
                            pointcloud_grid[nx][ny].property = 1;
                            q.push(std::make_pair(nx, ny));
                        }
                        else
                        {
                            pointcloud_grid[nx][ny].property = 2;
                        }

                        pointcloud_grid[nx][ny].isvisited = 1;
                        if (ViewOption)
                        {
                            // viewer->addCube(nx * grid_size - back_distance, nx * grid_size - back_distance + grid_size,
                            //                 ny * grid_size - right_left_distance, ny * grid_size - right_left_distance + grid_size,
                            //                 pointcloud_grid[nx][ny].plane_height, pointcloud_grid[nx][ny].plane_height + 0.1,
                            //                 1, 0, 0, "1_" + std::to_string(nx) + "_" + std::to_string(ny));
                        }
                        // q.push(std::make_pair(nx, ny));
                    }
                }
            }
        }
    }

    // 车身
    int car_region_start_x = origin_in_grid_x - 2 / grid_size;
    int car_region_start_y = origin_in_grid_y - 1.6 / grid_size;
    int car_region_end_x = origin_in_grid_x + 2 / grid_size;
    int car_region_end_y = origin_in_grid_y + 1.6 / grid_size;

    for (int i = car_region_start_x; i <= car_region_end_x; ++i)
    {
        for (int j = car_region_start_y; j <= car_region_end_y; ++j)
        {
            pointcloud_grid[i][j].property = 1;
        }
    }

    for (int i = 0; i < num_cells_x; ++i)
    {
        for (int j = 0; j < num_cells_y; ++j)
        {
            if ((pointcloud_grid[i][j].property >= 2 || pointcloud_grid[i][j].property == 0) && (pointcloud_grid[i][j].point_num - pointcloud_grid[i][j].ground_indices.size() > 3))
            {
                grid_map_road.push_back(i);
                grid_map_road.push_back(j);
                grid_map_road.push_back(pointcloud_grid[i][j].max_height);
                // grid_map_road.push_back(i - origin_in_grid_x) * grid_size + 0.5 * grid_size);
                // grid_map_road.push_back((j - origin_in_grid_y) * grid_size + 0.5 * grid_size);
            }
        }
    }

    // 使用索引提取滤波器提取点云
    // 提取地面点云
    ground_point.reset(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(filtered_cloud);
    extract.setIndices(groundindices);
    extract.setNegative(false);
    extract.filter(*ground_point);

    // 提取非地面点云
    nonground_point.reset(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::ExtractIndices<pcl::PointXYZI> extract_;
    extract_.setInputCloud(filtered_cloud);
    extract_.setIndices(groundindices);
    extract_.setNegative(true);
    extract_.filter(*nonground_point);

    return grid_map_road;
}

//-----------------------------------------------------------------------------------

std::vector<typename pcl::PointCloud<pcl::PointXYZI>::Ptr>
PointCloudsDetector::Clustering(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float clusterTolerance, int minSize,
                                int maxSize) // clusterTolerance 欧式距离的阈值，minsize小于最小点数当成外点，maxsize大于最大点数完成当前点的搜索
{
    // // zaodian process
    // std::vector<int> zaodian_flag;
    // for (int i = 0; i < cloud->points.size(); i++)
    // {

    //     int zaodian_count = 0;
    //     for (int j = 0; j < cloud->points.size(); j++)
    //     {
    //         if (sqrt((cloud->points[i].x - cloud->points[j].x) * (cloud->points[i].x - cloud->points[j].x) + (cloud->points[i].y - cloud->points[j].y) * (cloud->points[i].y - cloud->points[j].y) +
    //         (cloud->points[i].z - cloud->points[j].z) * (cloud->points[i].z - cloud->points[j].z)) > 1)
    //         {
    //             zaodian_count++;
    //         }
    //     }
    //     if (zaodian_count == cloud->points.size() - 1)
    //     {
    //         zaodian_flag.push_back(i);
    //     }
    // }
    // std::cout << "zaodian_number:" << zaodian_flag.size() << std::endl;
    // std::cout << "zaodian11111111111111:" << cloud->points.size() << std::endl;
    // if (zaodian_flag.size() > 0)
    // {
    //     for (int i = 0; i < zaodian_flag.size(); i++)
    //     {
    //         cloud->erase(cloud->begin() + zaodian_flag[i]);
    //     }
    // }

    // std::cout << "zaodian222222222222222222:" << cloud->points.size() << std::endl;

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;
    if (cloud->points.empty())
    {
        return clusters;
    }
    // perform euclidean clustering to group detected obstacles创建用于提取搜索方法的kdtree树对象
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(cloud);
    // 被分割出来的点云团（标号队列）
    std::vector<pcl::PointIndices> cluster_indices;
    // 欧式聚类分割器
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(clusterTolerance); // 判定半径
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    // 搜索决策树
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);
    // 保存聚类后的点
    for (pcl::PointIndices getIndices : cluster_indices)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloudCluster(new pcl::PointCloud<pcl::PointXYZI>);
        for (int index : getIndices.indices)
        {
            cloudCluster->points.push_back(cloud->points[index]);
        }
        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

        clusters.push_back(cloudCluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

// 最小和最大的两点组成立方体
Box PointCloudsDetector::setBoundingBox(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    pcl::PointXYZI minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}

PcaBox PointCloudsDetector::setPcaBox(Frame frame)
{
    PcaBox pcaBox;

    // Eigen::Vector4f pcaCentroid;
    // pcl::compute3DCentroid(*cluster, pcaCentroid);
    // Eigen::Matrix3f covariance;

    // computeCovarianceMatrixNormalized(*cluster, pcaCentroid, covariance);
    // //======================
    // // remove the effects of z-axis
    // covariance(0, 2) = 0;
    // covariance(1, 2) = 0;
    // covariance(2, 0) = 0;
    // covariance(2, 1) = 0;
    // //======================

    // Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    // Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();

    // eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));

    // Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    // projectionTransform.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();
    // projectionTransform.block<3, 1>(0, 3) = -1.f * (projectionTransform.block<3, 3>(0, 0) * pcaCentroid.head<3>());
    // typename pcl::PointCloud<pcl::PointXYZI>::Ptr cloudPointsProjected(new pcl::PointCloud<pcl::PointXYZI>);
    // pcl::transformPointCloud(*cluster, *cloudPointsProjected, projectionTransform);

    // // Get the minimum and maximum points of the transformed cloud.
    // pcl::PointXYZI minPoint, maxPoint;
    // pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
    // const Eigen::Vector3f meanDiagonal = 0.5f * (maxPoint.getVector3fMap() + minPoint.getVector3fMap());
    Eigen::Vector3f center(frame.time.lidar_x, frame.time.lidar_y, frame.time.lidar_z);
    // Eigen::Vector3f center(0, 0, 0);
    // Eigen::Quaternionf rotation;
    Eigen::Quaternionf rotation(1, 0, 0, 0);
    Eigen::Quaternionf yaw_rotation(Eigen::AngleAxisf(frame.time.lidar_yaw, Eigen::Vector3f(0, 0, 1)));
    rotation = yaw_rotation * rotation;
    // rotation = Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(frame.time.lidar_yaw, Eigen::Vector3f::UnitZ());
    // // Final transform

    pcaBox.center = center;
    pcaBox.rotation = rotation;
    // pcaBox.bboxQuaternion = eigenVectorsPCA;
    // pcaBox.bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();
    pcaBox.boxLength = frame.time.l;
    pcaBox.boxWidth = frame.time.w;
    pcaBox.boxHeight = frame.time.h;

    //     double rotation_angle = M_PI / 4; // 旋转角度为45度，这里使用弧度表示
    //     Eigen::Vector3f rotation_center(1, 1, 1); // 旋转中心点为(1, 1, 1)

    //     // 定义长方体的尺寸
    //     float length = 2.0;
    //     float width = 1.0;
    //     float height = 0.5;

    //     // 计算旋转的变换矩阵和四元数
    //     Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    // transform(0, 3) = 1; // x为平移的x轴坐标值
    // transform(1, 3) = 1; // y为平移的y轴坐标值
    // transform(2, 3) =1; // z为平移的z轴坐标值

    // Eigen::Quaternionf quat;
    // quat = Eigen::AngleAxisf(0.0f * M_PI / 180.0f, Eigen::Vector3f::UnitZ());

    //     pcaBox.bboxTransform = transform;

    //         pcaBox.bboxQuaternion =quat;
    //     pcaBox.boxLength = length;
    //     pcaBox.boxWidth = width;
    //     pcaBox.boxHeight = height;

    return pcaBox;
}

double calculateDistance(const Point &p1, const Point &p2)
{
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    return std::sqrt(dx * dx + dy * dy);
}
double calculateSlope(double x1, double y1, double x2, double y2) { return (y2 - y1) / (x2 - x1); }

// 计算直线的截距
double calculateIntercept(double x1, double y1, double slope) { return y1 - slope * x1; }
pair<double, double> calculateProjection(double x1, double y1, double x2, double y2, double x3, double y3)
{
    double slope = calculateSlope(x1, y1, x2, y2);
    double intercept = calculateIntercept(x1, y1, slope);
    double projectionX = (slope * (y3 - intercept) + x3) / (slope * slope + 1);
    double projectionY = slope * projectionX + intercept;
    return make_pair(projectionX, projectionY);
}
double calculateDistance_pair(const std::pair<double, double> &point1, const std::pair<double, double> &point2)
{
    double x1 = point1.first;
    double y1 = point1.second;
    double x2 = point2.first;
    double y2 = point2.second;

    double distance = std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
    return distance;
}
double calculateDistanceToLine(double x1, double y1, double x2, double y2, double x3, double y3)
{
    double slope = calculateSlope(x1, y1, x2, y2);
    double intercept = calculateIntercept(x1, y1, slope);

    double distance = fabs(slope * x3 - y3 + intercept) / sqrt(slope * slope + 1);

    return distance;
}

pair<double, double> p3p4calculateProjection(double x0, double y0, double k, double b)
{

    double xp = (k * (y0 - b) + x0) / (k * k + 1);
    double yp = k * xp + b;
    return make_pair(xp, yp);
}

Frame PointCloudsDetector::object(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster, double trackingDis)
{
    // if(cluster->points.size()>200)
    // {
    // 聚类点云的高度
    // double cluster_zmax = cluster->points[0].z;
    // double cluster_zmin = cluster->points[0].z;
    // for (int i = 0; i < cluster->points.size() - 1; i++)
    // {
    //     if (cluster->points[i].z > cluster_zmax)
    //     {
    //         cluster_zmax = cluster->points[i].z;
    //     }
    //     if (cluster->points[i].z < cluster_zmin)
    //     {
    //         cluster_zmin = cluster->points[i].z;
    //     }
    //     if (cluster->points[i].z < 0)
    //     {
    //         cluster_zmin = 0;
    //     }
    // }
    double car_vel;
    std::vector<cv::Point2d> imu_points;
    cv::Point2d imu_point(this->imu_x, this->imu_y);
    imu_points.push_back(imu_point);
    pcl::PointXYZI minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    // double cluster_height = abs(cluster_zmax - cluster_zmin);
    double cluster_height = abs(maxPoint.z - minPoint.z);
    std::vector<cv::Point2f> points;
    std::pair<double, double> point1all;
    std::pair<double, double> point2all;

    // 遍历每个聚类中的点
    double minDistance_pairall = 100000.0;
    for (int i = 0; i < cluster->points.size(); i++)
    {
        // if (cluster->points[i].z > -1)
        // {
        cv::Point2f p;
        p.x = cluster->points[i].x;
        p.y = cluster->points[i].y;
        points.push_back(p);
        // }
    }
    std::vector<cv::Point2f> hull;
    cv::convexHull(points, hull);
    std::vector<std::pair<double, double>> pair_length;
    double maxdistance_hull = 0;

    // for (size_t i = 0; i < hull.size(); i++)
    // {
    //     Point lineStart = hull[i];
    //     Point lineEnd = hull[(i + 1) % hull.size()];
    //     double distance_hull = calculateDistance(hull[i], hull[(i + 1) % hull.size()]);
    //     if (distance_hull > maxdistance_hull)
    //     {
    //         maxdistance_hull = distance_hull;
    //         point1all.first = hull[i].x;
    //         point1all.second = hull[i].y;
    //         point2all.first = hull[(i + 1) % hull.size()].x;
    //         point2all.second = hull[(i + 1) % hull.size()].y;
    //     }
    // }

    double maxwucha_plus = 100000;
    for (size_t i = 0; i < hull.size(); i++)
    {
        Point lineStart = hull[i];
        Point lineEnd = hull[(i + 1) % hull.size()];

        double wucha_plus = 0;
        for (size_t j = 0; j < points.size(); j++)
        {
            double wucha = calculateDistanceToLine(hull[i].x, hull[i].y, hull[(i + 1) % hull.size()].x, hull[(i + 1) % hull.size()].y, points[j].x, points[j].y);
            wucha_plus += wucha;
            // pair<double, double> projection = calculateProjection(hull[i].x, hull[i].y, hull[(i + 1) % hull.size()].x, hull[(i + 1) % hull.size()].y, hull[j].x, hull[j].y);
            // std::cout << "投影点的坐标为："
            //           << "(" << projection.first << ", " << projection.second << ")" << std::endl;
            // projections.push_back(projection);
        }
        if (wucha_plus < maxwucha_plus)
        {
            maxwucha_plus = wucha_plus;
            point1all.first = hull[i].x;
            point1all.second = hull[i].y;
            point2all.first = hull[(i + 1) % hull.size()].x;
            point2all.second = hull[(i + 1) % hull.size()].y;
        }
        // for (size_t zi = 0; zi < projections.size(); ++zi)
        // {
        //     double distance_pairH = calculateDistanceToLine(hull[i].x, hull[i].y, hull[(i + 1) % hull.size()].x, hull[(i + 1) % hull.size()].y, hull[zi].x, hull[zi].y);
        //     if (distance_pairH > maxDistance_pairH)
        //     {
        //         maxDistance_pairH = distance_pairH;
        //     }

        //     for (size_t zj = zi + 1; zj < projections.size(); ++zj)
        //     {
        //         double distance_pair = calculateDistance_pair(projections[zi], projections[zj]);
        //         if (distance_pair > maxDistance_pair)
        //         {
        //             maxDistance_pair = distance_pair;
        //             point1 = projections[zi];
        //             point2 = projections[zj];
        //         }
        //     }
        // }
        // std::cout << "最大距离的两个点坐标为：" << std::endl;
        // std::cout << "(" << point1.first << ", " << point1.second << ")" << std::endl;
        // std::cout << "(" << point2.first << ", " << point2.second << ")" << std::endl;
        // pair_length.push_back(point1);
        // pair_length.push_back(point2);
        // double area_all = maxDistance_pair * maxDistance_pairH;
        // if (area_all < minDistance_pairall)
        // {
        //     minDistance_pairall = area_all;
        //     point1all = pair_length[2 * i];
        //     point2all = pair_length[2 * i + 1];
        // }
    }
    // std::cout << "最小面积的两个相邻点坐标:" << std::endl;
    // std::cout << "(" << point1all.first << ", " << point1all.second << ")" << std::endl;
    // std::cout << "(" << point2all.first << ", " << point2all.second << ")" << std::endl;
    // std::cout << minDistance_pairall << std::endl;
    std::vector<std::pair<double, double>> projections;
    double maxDistance_pairH = 0.0;
    int pairH_flag = -1;
    for (int i = 0; i < points.size(); i++)
    {
        pair<double, double> projection = calculateProjection(point1all.first, point1all.second, point2all.first, point2all.second, points[i].x, points[i].y);
        // std::cout << "投影点的坐标为："
        //           << "(" << projection.first << ", " << projection.second << ")" << std::endl;
        projections.push_back(projection);

        double distance_pairH = calculateDistanceToLine(point1all.first, point1all.second, point2all.first, point2all.second, points[i].x, points[i].y);
        if (distance_pairH > maxDistance_pairH)
        {
            maxDistance_pairH = distance_pairH;
            pairH_flag = i;
        }
    }

    double maxDistance_pair = 0.0;
    std::pair<double, double> point1;
    std::pair<double, double> point2;
    std::pair<double, double> point3;
    std::pair<double, double> point4;
    std::pair<double, double> point5;
    for (size_t zi = 0; zi < projections.size(); zi++)
    {

        for (size_t zj = zi + 1; zj < projections.size(); zj++)
        {
            double distance_pair = calculateDistance_pair(projections[zi], projections[zj]);
            if (distance_pair > maxDistance_pair)
            {
                maxDistance_pair = distance_pair;
                point1 = projections[zi];
                point2 = projections[zj];
            }
        }
    }
    point5.first = points[pairH_flag].x;
    point5.second = points[pairH_flag].y;

    double k_p1p2 = (point2.second - point1.second) / (point2.first - point1.first);
    double b_p1p2 = point1.second - k_p1p2 * point1.first;
    double b_p3p4 = point5.second - k_p1p2 * point5.first;

    point3 = p3p4calculateProjection(point2.first, point2.second, k_p1p2, b_p3p4);
    point4 = p3p4calculateProjection(point1.first, point1.second, k_p1p2, b_p3p4);

    // std::cout << "())))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))" << std::endl;
    // std::cout << "(" << point1.first << ", " << point1.second << ")" << std::endl;
    // std::cout << "(" << point2.first << ", " << point2.second << ")" << std::endl;
    // std::cout << "(" << point3.first << ", " << point3.second << ")" << std::endl;
    // std::cout << "(" << point4.first << ", " << point4.second << ")" << std::endl;

    // double longlength_angle = std::atan((point1all.second - point2all.second) / (point1all.first - point2all.first));
    // std::cout << "angle********************************************:" << longlength_angle << std::endl;
    // cv::RotatedRect boundingRect = cv::minAreaRect(cv::Mat(hull));

    // 打印凸包的点
    for (const auto &point : hull)
    {
        // std::cout << "x: " << point.x << ", y: " << point.y << std::endl;
    }
    // std::cout << "zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz" << std::endl;

    // 创建一个空白图像
    cv::Mat image = cv::Mat::zeros(1000, 1000, CV_8UC3);

    // 绘制点云
    for (const auto &point : points)
    {
        cv::circle(image, point * 40, 2, cv::Scalar(0, 0, 255), cv::FILLED);
    }

    // 转换凸包坐标为整数
    std::vector<std::vector<cv::Point>> hulls = {{}};
    std::transform(hull.begin(), hull.end(), std::back_inserter(hulls[0]), [](const cv::Point2f &point) { return cv::Point(point.x * 40, point.y * 40); });

    // 绘制凸包
    cv::drawContours(image, hulls, 0, cv::Scalar(0, 255, 0), 2);
    cv::Point2f rectpair[2];
    rectpair[0] = cv::Point2f(point1all.first, point1all.second);
    rectpair[1] = cv::Point2f(point2all.first, point2all.second);
    cv::Point2f rectPoints[4];
    // boundingRect.points(rectPoints);
    rectPoints[0] = cv::Point2f(point1.first, point1.second);
    rectPoints[1] = cv::Point2f(point2.first, point2.second);
    rectPoints[2] = cv::Point2f(point3.first, point3.second);
    rectPoints[3] = cv::Point2f(point4.first, point4.second);
    for (int i = 0; i < 4; i++)
    {
        cv::line(image, rectPoints[i] * 40, rectPoints[(i + 1) % 4] * 40, cv::Scalar(255, 0, 0), 2);
    }
    cv::line(image, rectpair[0] * 40, rectpair[1] * 40, cv::Scalar(255, 0, 0), 2);
    double x_center, y_center;
    x_center = (rectPoints[0].x + rectPoints[1].x + rectPoints[2].x + rectPoints[3].x) / 4;
    y_center = (rectPoints[0].y + rectPoints[1].y + rectPoints[2].y + rectPoints[3].y) / 4;

    // x_center = (minPoint.x + minPoint.x + maxPoint.x + maxPoint.x) / 4;
    // y_center = (minPoint.y + minPoint.y + maxPoint.y + maxPoint.y) / 4;
    // std::cout << "x_center:" << x_center << "         y_center:" << y_center << std::endl;
    double cluster_w = (abs(rectPoints[0].x) + abs(rectPoints[1].x) + abs(rectPoints[2].x) + abs(rectPoints[3].x)) / 2;
    double cluster_l = (abs(rectPoints[0].y) + abs(rectPoints[1].y) + abs(rectPoints[2].y) + abs(rectPoints[3].y)) / 2;
    double longlength_max = 0;
    double longlength_min = 100000;

    for (int i = 0; i < 3; i++)
    {

        double longlengthx = rectPoints[3].x - rectPoints[i].x;
        double longlengthy = rectPoints[3].y - rectPoints[i].y;
        double longlength = std::sqrt(longlengthx * longlengthx + longlengthy * longlengthy);
        if (longlength > longlength_max)
        {
            longlength_max = longlength;
        }
        if (longlength < longlength_min)
        {
            longlength_min = longlength;
        }
    }

    int longlength_flag = -1;
    double longlength_middle;
    for (int i = 0; i < 3; i++)
    {

        double longlengthx = rectPoints[3].x - rectPoints[i].x;
        double longlengthy = rectPoints[3].y - rectPoints[i].y;
        double longlength = std::sqrt(longlengthx * longlengthx + longlengthy * longlengthy);
        if (longlength != longlength_max && longlength != longlength_min)
        {
            longlength_flag = i;
            longlength_middle = longlength;
        }
    }
    double longlength_angle = std::atan((rectPoints[3].y - rectPoints[longlength_flag].y) / (rectPoints[3].x - rectPoints[longlength_flag].x));
    int type_id = -1; // 0小汽车,1卡车,2行人,3自行车
    if (longlength_middle > 3 && longlength_middle < 6 && longlength_min > 1 && longlength_min < 2 && cluster_height < 4)
    {
        type_id = 0;
    }
    // if (longlength_middle * longlength_min > 2 && longlength_middle - longlength_min > 4)
    // {
    //     type_id = 1;
    // }
    if (longlength_middle > 0.1 && longlength_middle < 0.5 && longlength_min > 0.1 && longlength_min < 0.3 && cluster_height < 2.2)
    {
        type_id = 2;
    }
    // if (longlength_middle * longlength_min < 0.3 && longlength_middle - longlength_min > 0.2)
    // {
    //     type_id = 3;
    // }

    // cv::line(image, rectPoints[3] * 100, rectPoints[longlength_flag] * 100, cv::Scalar(0, 0, 255), 2);

    // double longlength_angle = std::atan((rectPoints[3].y - rectPoints[longlength_flag].y) / (rectPoints[3].x - rectPoints[longlength_flag].x));

    // double longlength_angle = 0;
    // longlength_angle = std::atan((rectPoints[3].y - rectPoints[longlength_flag].y) / (rectPoints[3].x - rectPoints[longlength_flag].x));

    // std::vector<Frame> frames;
    // std::time_t currentTime = std::time(nullptr);
    auto currentTime = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());

    double x_guass = x_center * cos(this->imu_yaw) - y_center * sin(this->imu_yaw) + this->imu_x;
    double y_guass = x_center * sin(this->imu_yaw) + y_center * cos(this->imu_yaw) + this->imu_y;
    Frame frame1;
    if (frame_id == 0)
    {
        frame1.time_stamp = currentTime.time_since_epoch().count(); // ms
        frame1.frameId = frame_id;                                  // 点云帧数
        frame1.ld = id;                                             // 聚类数计数
        frame1.imu_x = this->imu_x;
        frame1.imu_y = this->imu_y;
        frame1.imu_yaw = this->imu_yaw;

        frame1.time.timeld = time_id; // 点云当前帧的聚类个数
        frame1.time.objectId = time_id;
        frame1.time.type = type_id;

        frame1.time.x = x_guass;
        frame1.time.y = y_guass;
        frame1.time.z = (maxPoint.z + minPoint.z) / 2;
        frame1.time.yaw = longlength_angle + this->imu_yaw;

        frame1.time.lidar_x = x_center;
        frame1.time.lidar_y = y_center;
        frame1.time.lidar_z = (maxPoint.z + minPoint.z) / 2;
        frame1.time.lidar_yaw = longlength_angle;

        frame1.time.w = longlength_min;
        frame1.time.l = longlength_middle;
        frame1.time.h = cluster_height;
        frames.push_back(frame1);
    }
    else
    {

        std::vector<cv::Point2d> lastFrame;
        std::vector<int> lastFrame_id;
        std::vector<time_t> lastFrame_time;
        for (int i = 0; i < frames.size(); i++)
        {
            if (frames[i].frameId == frame_id - 1)
            {
                // std::cout << "x = " << frames[i].time.x
                //           << ", y = " << frames[i].time.y
                //           << ", z = " << frames[i].time.z << std::endl;
                cv::Point2d p1{frames[i].time.x + (this->imu_x - frames[i].imu_x), frames[i].time.y + (this->imu_y - frames[i].imu_y)};
                // cv::Point2d p1{frames[i].time.x, frames[i].time.y};
                lastFrame.push_back(p1);
                lastFrame_id.push_back(frames[i].time.objectId);
                lastFrame_time.push_back(frames[i].time_stamp);
            }
        }

        cv::Point2d p2{x_center, y_center};
        std::vector<cv::Point2d> currentFrame;
        currentFrame.push_back(p2);
        double minDistance = std::numeric_limits<double>::max();
        int closestIndex = -1;
        for (int i = 0; i < lastFrame.size(); i++)
        {
            double distance = calculateDistance(lastFrame[i], currentFrame[0]);
            // double imu_distance = calculateDistance(imu_points[imu_points.size() - 1], imu_points[imu_points.size() - 2]);
            if (distance < minDistance)
            {
                minDistance = distance;
                closestIndex = i;
            }
        }
        if (minDistance < trackingDis && closestIndex != -1)
        {
            frame1.time.objectId = lastFrame_id[closestIndex];
            // longlength_angle = std::atan((y_center - lastFrame[closestIndex].y) / (x_center - lastFrame[closestIndex].x)); //* (180 / M_PI)
            // longlength_angle = std::atan((rectPoints[3].y - rectPoints[longlength_flag].y) / (rectPoints[3].x - rectPoints[longlength_flag].x));
            car_vel = minDistance / (currentTime.time_since_epoch().count() - lastFrame_time[closestIndex]) * 1000;
        }
        else
        {
            frame1.time.objectId = id;
            // longlength_angle = 0;
            // longlength_angle = std::atan((rectPoints[3].y - rectPoints[longlength_flag].y) / (rectPoints[3].x - rectPoints[longlength_flag].x));
        }
        frame1.time_stamp = currentTime.time_since_epoch().count();
        frame1.frameId = frame_id;
        frame1.ld = id;
        frame1.imu_x = this->imu_x;
        frame1.imu_y = this->imu_y;
        frame1.imu_yaw = this->imu_yaw;

        frame1.time.timeld = time_id;
        frame1.time.type = type_id;

        frame1.time.x = x_guass;
        frame1.time.y = y_guass;
        frame1.time.z = (maxPoint.z + minPoint.z) / 2;
        frame1.time.yaw = longlength_angle + this->imu_yaw;
        frame1.time.vel = car_vel;

        frame1.time.lidar_x = x_center;
        frame1.time.lidar_y = y_center;
        frame1.time.lidar_z = (maxPoint.z + minPoint.z) / 2;
        frame1.time.lidar_yaw = longlength_angle;

        frame1.time.w = longlength_min;
        frame1.time.l = longlength_middle;
        frame1.time.h = cluster_height;
        frames.push_back(frame1);
    }
    Frame firstFrame = frames[id];

    // std::cout << "-----------------------------------------------------------------------------------------------------------------------" << std::endl;
    // std::cout << "time_stamp: " << firstFrame.time_stamp << std::endl;
    // std::cout << "cloud_frame_id: " << firstFrame.frameId << std::endl;
    // std::cout << "imu_x_y_yaw_Guass: (" << firstFrame.imu_x << ", " << firstFrame.imu_y << ", " << firstFrame.imu_yaw << ")" << std::endl;
    // std::cout << "object_id: " << firstFrame.time.objectId << std::endl;
    // std::cout << "type_id: " << firstFrame.time.type << std::endl;
    // std::cout << "Position_x_y_z_lidar: (" << firstFrame.time.lidar_x << ", " << firstFrame.time.lidar_y << ", " << firstFrame.time.lidar_z << ", " << firstFrame.time.lidar_yaw << ")" << std::endl;
    // std::cout << "Position_x_y_z_yaw_Guass: (" << firstFrame.time.x << ", " << firstFrame.time.y << ", " << firstFrame.time.z << ", " << firstFrame.time.yaw << ")" << std::endl;
    // std::cout << "vel: " << firstFrame.time.vel << std::endl;
    // std::cout << "WLH: (" << firstFrame.time.w << ", " << firstFrame.time.l << ", " << firstFrame.time.h << ")" << std::endl;
    // std::cout << "             " << std::endl;
    // std::cout << "-----------------------------------------------------------------------------------------------------------------------" << std::endl;

    if (sqrt((firstFrame.time.x - firstFrame.imu_x) * (firstFrame.time.x - firstFrame.imu_x) + (firstFrame.time.y - firstFrame.imu_y) * (firstFrame.time.y - firstFrame.imu_y)) < 10)
    {
        std::ofstream file("output.txt", std::ios_base::app);

        // std::ofstream file("data.txt");

        // 将信息写入文件
        // file << "time_stamp: " << firstFrame.time_stamp << std::endl;
        // file << "cloud_frame_id: " << firstFrame.frameId << std::endl;
        // file << "imu_x_y_yaw_Guass: (" << firstFrame.imu_x << ", " << firstFrame.imu_y << ", " << firstFrame.imu_yaw << ")" << std::endl;
        // file << "object_id: " << firstFrame.time.objectId << std::endl;
        // file << "type_id: " << firstFrame.time.type << std::endl;

        // file << "Position_x_y_z_yaw_Guass: (" << firstFrame.time.x << ", " << firstFrame.time.y << ", " << firstFrame.time.z << ", " << firstFrame.time.yaw << ")" << std::endl;
        // file << "WLH: (" << firstFrame.time.w << ", " << firstFrame.time.l << ", " << firstFrame.time.h << ")" << std::endl;
        // file << "             " << std::endl;
        file << firstFrame.time.type << std::endl;
        file << firstFrame.time.objectId << std::endl;
        file << firstFrame.time.x << std::endl;
        file << firstFrame.time.y << std::endl;
        file << firstFrame.time.yaw << std::endl;
        file << "0" << std::endl;
        file << firstFrame.time.l << std::endl;
        file << firstFrame.time.w << std::endl;
        file << firstFrame.time.h << std::endl;
        file << firstFrame.imu_x << std::endl;
        file << firstFrame.imu_y << std::endl;
        file << firstFrame.imu_yaw << std::endl;
        file << "             " << std::endl;

        // 关闭文件

        //     file<< firstFrame.frameId << std::endl;
        // file <<  firstFrame.time.objectId <<  std::endl;
        // file <<  firstFrame.time.type <<  std::endl;
        //    file <<firstFrame.time.x << " " << firstFrame.time.y << " " << firstFrame.time.z  <<  std::endl;

        file.close();
    }
    return frame1;
}

std::vector<Frame> PointCloudsDetector::ObstacleBounding(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusterCloud, double trackingDis)
{
    std::vector<Frame> Obstacle_array;
    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(1, 1, 0), Color(0, 0, 1)};
    if (clusterCloud.size() > 0)
    {
        sort(clusterCloud.begin(), clusterCloud.end(), cmpCloudSize);
        this->objResultList.clear_object();
        // this->rPointCloud(nonground_point);
        for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : clusterCloud)
        {

            Frame frame2 = object(cluster, trackingDis);

            time_id++;
            id++;
            PcaBox box1 = setPcaBox(frame2);
            Box box2 = setBoundingBox(cluster);
            perception::Object *objectInfo;
            // double(box1.boxHeight) > 0.3
            if (abs(frame2.time.lidar_x) < 10 && box1.boxLength < 6 && box1.boxHeight < 4 && box1.boxLength > 0.2 && box1.boxHeight > 0.2)
            {
                Obstacle_array.push_back(frame2);
                objectInfo = this->objResultList.add_object();
                objectInfo->set_x(frame2.time.lidar_x);
                objectInfo->set_y(frame2.time.lidar_y);
                objectInfo->set_z(frame2.time.lidar_z);
                objectInfo->set_w(double(box1.boxWidth));
                objectInfo->set_h(double(box1.boxHeight));
                objectInfo->set_l(double(box1.boxLength));
                objectInfo->set_type(-1);
                objectInfo->set_trackid(-1);
                objectInfo->set_vx(0.0);
                objectInfo->set_vy(0.0);
                this->renderPointCloud(cluster, "obstCloud" + std::to_string(clusterId), Color(1, 0, 0));
                this->renderPcaBox(box1, clusterId + 1, Color(0, 1, 0));
                std::string textId = "id_" + std::to_string(frame2.time.objectId);
                pcl::PointXYZ position;
                position.x = frame2.time.lidar_x;
                position.y = frame2.time.lidar_y;
                position.z = frame2.time.lidar_z + box1.boxLength / 2 + 0.5;
                //   viewer->addText3D(textId, position, 0.1, 1.0, 1.0, 1.0);
                position.z = frame2.time.lidar_z + box1.boxLength / 2 + 1;

                pcl::PointCloud<pcl::PointXYZI>::iterator it = nonground_point->begin();

                // 遍历点云中的每个点
                while (it != nonground_point->end())
                {
                    // 判断点的坐标是否在指定范围内
                    if (it->x >= frame2.time.lidar_x - box1.boxWidth / 2 && it->x <= frame2.time.lidar_x + box1.boxWidth / 2 && it->y >= frame2.time.lidar_y - box1.boxLength / 2 &&
                        it->y <= frame2.time.lidar_y + box1.boxLength / 2)

                    {
                        // 删除满足条件的点
                        it = nonground_point->erase(it);
                    }
                    else
                    {
                        // 继续迭代到下一个点
                        ++it;
                    }
                }

                switch (frame2.time.type)
                {
                case 0:
                    // viewer->addText3D("car", position, 0.1, 1.0, 1.0, 1.0);
                    break;
                case 2:
                    // viewer->addText3D("person", position, 0.1, 1.0, 1.0, 1.0);
                    break;
                }
                ++clusterId;
            }
        }
    }
    return Obstacle_array;
}

std::pair<std::vector<int>, std::vector<Frame>> PointCloudsDetector::run()
{
    auto start = std::chrono::steady_clock::now();
    auto end = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    std::vector<int> Pos_array;
    std::vector<Frame> Obstacle_array;
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
    t_inputCloud = this->input_cloud;
    g_mtxCloud.unlock();
    if (t_inputCloud->points.empty())
    {
        return std::make_pair(Pos_array, Obstacle_array);
    }
    auto startTime_ = std::chrono::high_resolution_clock::now();

    // 裁剪自车
    typename pcl::CropBox<pcl::PointXYZI> croped;
    croped.setInputCloud(t_inputCloud);
    croped.setMin(Eigen::Vector4f(-1, -12, -10, 1));
    croped.setMax(Eigen::Vector4f(1.3, 12, 10, 1));
    croped.setNegative(true);
    croped.filter(*t_inputCloud);

    // 点云预处理(裁剪和降采样)
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud =
        filterCloud(t_inputCloud, DownsampleLeafSize_xy, DownsampleLeafSize_xy, DownsampleLeafSize_z, Eigen::Vector4f(PassThroughFilter_x_min, PassThroughFilter_y_min, PassThroughFilter_z_down, 1),
                    Eigen::Vector4f(PassThroughFilter_x_max, PassThroughFilter_y_max, PassThroughFilter_z_up, 1));
    // 降采样输出的点云是 filtered_cloud
    // 打印降采样的耗时
    auto endTime_preprocess = std::chrono::high_resolution_clock::now();
    auto duration_preprocess = std::chrono::duration_cast<std::chrono::milliseconds>(endTime_preprocess - startTime_).count();
    std::cout << "预处理——总耗时:" << duration_preprocess << " ms" << std::endl;
    std::cout << "点云的个数：" << filtered_cloud->size() << std::endl;

    auto startTime_groudfilter = std::chrono::high_resolution_clock::now();
    // 去除地面
    Pos_array = GridGroundFilterSimple(filtered_cloud, NormalEatimationKSearch, SlopeRate, NormalThereshold, GridHeightComputeMode, GroundHeight, GridSize, PassThroughFilter_y_max,
                                       -PassThroughFilter_x_min, PassThroughFilter_x_max);

    // 去除地面后 地面点云存储在全局变量'ground_point'当中
    //           非地面点云存储在全局变量'nonground_point'当中
    // 打印去地面时间
    auto endTime_groundfilter = std::chrono::high_resolution_clock::now();
    auto duration_groundfilter = std::chrono::duration_cast<std::chrono::milliseconds>(endTime_groundfilter - startTime_groudfilter).count();
    std::cout << "去地面处理时间:" << duration_groundfilter << " ms" << std::endl;
    g_mtxViewer.lock();
    this->viewer->removeAllPointClouds();
    this->viewer->removeAllShapes();
    this->viewer->removeAllCoordinateSystems();
    if (1)
    {

        this->renderPointCloud(ground_point, "cloud_origin", Color(1, 1, 1));

        this->renderPointCloud(nonground_point, "cloud", Color(0, 1, 0));
    }
    // g_mtxViewer.unlock();

    // // 聚类
    // std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusterCloud = Clustering(nonground_point, 0.6, 20, 5000); // obstacle clustering 0.3

    // if (clusterCloud.empty())
    // {
    //     return std::make_pair(Pos_array, Obstacle_array);
    // }

    // 画框
    // Obstacle_array = ObstacleBounding(clusterCloud, trackingDis);
    // // this->rPointCloud(t_inputCloud);

    // // this->renderPointCloud(t_inputCloud, "cloud", Color(-1, 1, 0));
    // frame_id++;
    std::cout << "test!!!!!!!!!!" << std::endl;
    // viewer->spin();
    g_mtxViewer.unlock();
    std::cout << "test----------" << std::endl;
    // Pos_array = Pointcloud_to_grid(nonground_point, map_resolution);

    end = std::chrono::steady_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    cout << "duration process: " << duration.count() << endl;

    return std::make_pair(Pos_array, Obstacle_array);
}

// --- OBJECT TEST

void PointCloudsDetector ::test() { cout << "test func in PointCloudsDetector class ok" << endl; }
