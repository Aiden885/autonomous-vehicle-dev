#include "lidarTools.h"
#include <iostream>

using namespace lidarTools;

void generatePCD(std::string filename) {
    // Create point cloud object
    pcl::PointCloud<pcl::PointXYZI> cloud;

    // Set basic parameters
    cloud.width = 400;
    cloud.height = 1;
    cloud.is_dense = false;
    cloud.points.resize(cloud.width * cloud.height);

    // Random number generators
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> x_dist(-10.0, 10.0);
    std::uniform_real_distribution<> y_dist(-10.0, 10.0); 
    std::uniform_real_distribution<> z_dist(-1.0, 1.0);
    std::uniform_real_distribution<> i_dist(0, 255.0);

    // Generate random points
    for (auto& point : cloud.points) {
        point.x = x_dist(gen);
        point.y = y_dist(gen);
        point.z = z_dist(gen);
        point.intensity = i_dist(gen);
    }

    // Save to PCD file
    pcl::io::savePCDFileASCII(filename, cloud);
    std::cout << "Generated and saved " << cloud.points.size() << " points to " << filename << std::endl;
}

int main()
{
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::io::loadPCDFile<pcl::PointXYZ>("/home/ran/Pictures/PCDFiles/chef.pcd", *pointcloud);
    


    std::vector<pcl::PointIndices> cluster_indices;
    pcdAffinityPropagationCluster(pointcloud, cluster_indices);
    // Create colored point clouds for each cluster
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> colored_clusters;
    
    // Random color generator
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> color_dist(0, 255);

    // Process each cluster
    for (const auto& cluster : cluster_indices) {
        // Create new colored point cloud for this cluster
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        
        // Generate random color for this cluster
        uint8_t r = color_dist(gen);
        uint8_t g = color_dist(gen);
        uint8_t b = color_dist(gen);

        // Copy points from original cloud to colored cloud
        for (const auto& idx : cluster.indices) {
            pcl::PointXYZRGB colored_point;
            colored_point.x = pointcloud->points[idx].x;
            colored_point.y = pointcloud->points[idx].y;
            colored_point.z = pointcloud->points[idx].z;
            colored_point.r = r;
            colored_point.g = g;
            colored_point.b = b;
            colored_cloud->points.push_back(colored_point);
        }

        colored_cloud->width = colored_cloud->points.size();
        colored_cloud->height = 1;
        colored_cloud->is_dense = true;
        
        colored_clusters.push_back(colored_cloud);
    }

    // Create a PCL visualizer
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Clustered Cloud"));
    viewer->setBackgroundColor(0, 0, 0);

    // Add each colored cluster to viewer
    for (size_t i = 0; i < colored_clusters.size(); i++) {
        std::string cloud_name = "cloud_" + std::to_string(i);
        viewer->addPointCloud<pcl::PointXYZRGB>(colored_clusters[i], cloud_name);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, cloud_name);
    }

    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
    }
    std::cout << cluster_indices.size() << std::endl;

// Create a PCL visualizer
    // pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    // viewer->setBackgroundColor(0, 0, 0);
    // viewer->addPointCloud<pcl::PointXYZI>(pointcloud, "cloud");
    // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
    // viewer->addCoordinateSystem(1.0);
    // viewer->initCameraParameters();

    // // Keep window open until 'q' is pressed
    // while (!viewer->wasStopped())
    // {
    //     viewer->spinOnce(100);
    // }

    return 0;
}
