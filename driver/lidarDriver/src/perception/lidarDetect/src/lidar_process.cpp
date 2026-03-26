#include "lidar_process.h"

#include <pcl/common/common.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <zmq.h>

#include <jsoncpp/json/json.h>

template <typename PointT>
PointCloudsDetector<PointT>::PointCloudsDetector()
    : cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>),
      viewer(new pcl::visualization::PCLVisualizer("3D Viewer"))
{
    communicator.InitSubscriber(5005);
    communicator.InitPublisher(5008);
}

template <typename PointT>
PointCloudsDetector<PointT>::~PointCloudsDetector()
{
}

template <typename PointT>
void PointCloudsDetector<PointT>::detect()
{
    if (!viewer->wasStopped())
    {
        // clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
        pcl::PointCloud<pcl::PointXYZI>::Ptr filter_cloud =
            FilterCloud(cloud_ptr, 0.2, Eigen::Vector4f(0, -10, -1.6, 1), Eigen::Vector4f(30, 10, 5, 1));
        // std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segment_cloud
        // =
        //     RANSAC3d(filter_cloud, 100, 0.2);
        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cluster_cloud =
            Clustering(filter_cloud, 0.5, 30, 300);

        int clusterId = 0;
        // std::vector<Color> colors = {Color(1, 0, 0), Color(1, 1, 0), Color(0, 0, 1)};
        Json::Value object;
        Json::FastWriter writer;
        for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cluster_cloud)
        {
            std::cout << "cluster size " << cluster->points.size() << std::endl;
            this->renderPointCloud(cluster, "obstCloud" + std::to_string(clusterId), Color(1, 0, 1));

            Box box = BoundingBox(cluster);
            this->renderBox(box, clusterId);

            object[std::to_string(clusterId)]["position"]["x"] = box.x_min;
            object[std::to_string(clusterId)]["position"]["y"] = box.y_min;
            object[std::to_string(clusterId)]["position"]["z"] = box.z_min;
            object[std::to_string(clusterId)]["size"]["l"] = box.x_max - box.x_min;
            object[std::to_string(clusterId)]["size"]["w"] = box.y_max - box.y_min;
            object[std::to_string(clusterId)]["size"]["h"] = box.z_max - box.z_min;
            object[std::to_string(clusterId)]["scene"] = 1;

            clusterId++;
        }

        this->renderPointCloud(filter_cloud, "cloud", Color(-1, 1, 0));
        viewer->spinOnce();

        std::string sendBuf = writer.write(object);
        communicator.publish(sendBuf);
    }
}

template <typename PointT>
void PointCloudsDetector<PointT>::InitView()
{
    CameraAngle setAngle = XY;
    initCamera(setAngle);
}

template <typename PointT>
Box PointCloudsDetector<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{
    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
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

template <typename PointT>
void PointCloudsDetector<PointT>::renderBox(Box box, int id, Color color, float opacity)
{
    if (opacity > 1.0)
        opacity = 1.0;
    if (opacity < 0.0)
        opacity = 0.0;

    std::string cube = "box" + std::to_string(id);
    // viewer->addCube(box.bboxTransform, box.bboxQuaternion, box.cube_length, box.cube_width,
    // box.cube_height, cube);
    viewer->addCube(box.x_min, box.x_max, box.y_min, box.y_max, box.z_min, box.z_max, color.r, color.g,
                    color.b, cube);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                                        pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, cube);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b,
                                        cube);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, cube);

    std::string cubeFill = "boxFill" + std::to_string(id);
    // viewer->addCube(box.bboxTransform, box.bboxQuaternion, box.cube_length, box.cube_width,
    // box.cube_height, cubeFill);
    viewer->addCube(box.x_min, box.x_max, box.y_min, box.y_max, box.z_min, box.z_max, color.r, color.g,
                    color.b, cubeFill);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                                        pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, cubeFill);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b,
                                        cubeFill);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity * 0.3, cubeFill);
}

template <typename PointT>
void PointCloudsDetector<PointT>::renderPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                                                   std::string name, Color color)
{
    if (color.r == -1)
    {
        // Select color based off of cloud intensity
        pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(
            cloud, "intensity");
        viewer->addPointCloud<pcl::PointXYZI>(cloud, intensity_distribution, name);
    }
    else
    {
        // Select color based off input value
        viewer->addPointCloud<pcl::PointXYZI>(cloud, name);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g,
                                                 color.b, name);
    }

    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, name);
}

template <typename PointT>
void PointCloudsDetector<PointT>::initCamera(CameraAngle setAngle)
{
    viewer->setBackgroundColor(0, 0, 0);

    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 10;

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

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr
PointCloudsDetector<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes,
                                         Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
    typename pcl::PointCloud<PointT>::Ptr cloud_downsampled(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloud_nearremoved(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>());

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // do voxel grid point reduction and region based filtering
    std::cout << "bf:" << cloud->points.size() << std::endl;
    // std::cout << "filter cloud 1st: " << cloud->points.at(0).x << " " << cloud->points.at(0).y <<
    // std::endl;
    typename pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(filterRes, filterRes, filterRes);
    sor.filter(*cloud_downsampled);
    std::cout << "down:" << cloud_downsampled->points.size() << std::endl;

    typename pcl::CropBox<PointT> croped(true);
    croped.setInputCloud(cloud_downsampled);
    croped.setMin(minPoint);
    croped.setMax(maxPoint);
    // croped.setNegative(true);
    croped.filter(*cloud_filtered);
    std::cout << "crop1:" << cloud_filtered->points.size() << std::endl;
    // getchar();

    typename pcl::CropBox<PointT> crop1(true);
    crop1.setInputCloud(cloud_filtered);
    crop1.setMin(Eigen::Vector4f(-0.7, -1.5, -1.5, 1));
    crop1.setMax(Eigen::Vector4f(0.3, 3, 1.5, 1));
    crop1.setNegative(true);
    crop1.filter(*cloud_nearremoved);
    std::cout << "crop2:" << cloud_nearremoved->points.size() << std::endl;

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_nearremoved;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>
PointCloudsDetector<PointT>::RANSAC3d(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations,
                                      float distanceThreshold)
{
    // input: cloud, maxitr, thr
    std::unordered_set<int> inliersResult;

    srand(time(NULL));

    while (maxIterations--)
    {
        std::unordered_set<int> inliers;

        while (inliers.size() < 3)
        {
            // std::cout << "rand0" << std::endl;
            // std::cout << cloud->points.size() << std::endl;
            inliers.insert(rand() % (cloud->points.size()));
            // std::cout << "rand1" << std::endl;
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

        for (int index = 0; index < cloud->points.size(); index++)
        {
            if (inliers.count(index) > 0)
            {
                continue;
            }
            PointT point = cloud->points[index];
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

    typename pcl::PointCloud<PointT>::Ptr cloudInliers(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

    for (int index = 0; index < cloud->points.size(); index++)
    {
        PointT point = cloud->points[index];
        if (inliersResult.count(index))
            cloudInliers->points.push_back(point);
        else
            cloudOutliers->points.push_back(point);
    }
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(
        cloudOutliers, cloudInliers);
    std::cout << "finish ransac" << std::endl;
    return segResult;
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr>
PointCloudsDetector<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance,
                                        int minSize, int maxSize)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // perform euclidean clustering to group detected obstacles
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance); // 2cm
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    for (pcl::PointIndices getIndices : cluster_indices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>);
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
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size()
              << " clusters" << std::endl;

    return clusters;
}

template <typename PointT>
void PointCloudsDetector<PointT>::recvPointCloud()
{
    communicator.subscribe();
    cloud_ptr = communicator.getPointCloud();

    cout << cloud_ptr->points.at(0).x << "  " << cloud_ptr->points.at(0).y << " " << cloud_ptr->points.size()
         << endl;
}

template <typename PointT>
void PointCloudsDetector<PointT>::run()
{
    while (true)
    {
        recvPointCloud();
        detect();
    }
}
