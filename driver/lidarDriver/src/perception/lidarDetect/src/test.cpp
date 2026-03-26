#include <iostream>
// #include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main(int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/yuyang/program/livox/ws_livox/box/save-1.pcd", *cloud) ==
        -1) //*打开点云文件
    {
        PCL_ERROR("Couldn't read file save-1.pcd\n");
        return -1;
    }

    std::cout << "cloud size: " << cloud->points.size() << std::endl;

    // pcl::PointXYZ minPt, maxPt;
    // int x, y, z;

    // pcl::getMinMax3D(*cloud, x, y, z, minPt, maxPt);

    // std::cout << minPt.x << "  " << minPt.y << "  " << minPt.z << std::endl;
    // std::cout << maxPt.x << "  " << maxPt.y << "  " << maxPt.z << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>());

    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.3, 0.3, 0.3);
    sor.filter(*cloud_downsampled);
    std::cout << "down:" << cloud_downsampled->points.size() << std::endl;

    return 0;
}
