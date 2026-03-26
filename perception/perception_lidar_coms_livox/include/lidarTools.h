#ifndef LIDAR_TOOLS_H
#define LIDAR_TOOLS_H

#include <experimental/filesystem>
#include <pcl/kdtree/kdtree_flann.h>

#include "baseLidarObjectDetector.h"
#include "patchworkpp.h"
#include "yaml-cpp/yaml.h"
#include <chrono>
#include <ctime>
#include <fstream>
#include <iostream>
#include <lidarDataProto/cloudData.pb.h>
#include <mutex>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/convolution_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/plane_clipper3D.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ml/kmeans.h>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
// #include <pcl/cluster/dbscan.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/search/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/normal_3d.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/ndt.h>
#include <pcl/features/pfh.h>
#include <pcl/features/normal_3d.h>
#include <perception/perception.pb.h>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <random>
#include <cmath>
#include <memory>

using namespace std;
using namespace pcl;
using namespace io;
using namespace cv;

/**
 * \namespace perception
 * \brief perception namespace。
 */
namespace perception{

static const int UNCLASSIFIED = -1;
static const int NOISE = -2;

//==========================
// some struct here
//==========================
struct ClusterResult {
    std::vector<int> labels;                     // Cluster label for each point
    typename pcl::PointCloud<pcl::PointXYZI>::Ptr centroids = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);  // Cluster centroids
    float total_error;                           // Sum of squared distances to centroids
    int num_clusters;
    std::vector<pcl::PointIndices> cluster_indices;
    // std::vector<int> points;
    // Eigen::Vector3f centroid;
};

struct ClusterDistance {
    int cluster1;
    int cluster2;
    float distance;

    bool operator>(const ClusterDistance& other) const {
        return distance > other.distance;
    }
};

void loadPCD(std::string filename, pcl::PointCloud<pcl::PointXYZI>::Ptr output);
//Filter
void pcdIntensityThresholdLowerFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr output, int intensity_thres);
void pcdVoxelFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr output, const double size_x, const double size_y, const double size_z);
void pcdRadiusFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr output, const double radius, const int min_neighbors);
void pcdStatisticFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr output, const double mean_K, const double std);
float gaussian(float x, float sigma);
void pcdGaussianFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr output, const double sigma=1.0, const double search_radius=0.1);

//sample
void updateDistances(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,  int point_idx,std::vector<float>& min_distances, pcl::KdTreeFLANN<PointXYZI>::Ptr kdtree_);
int findFarthestPoint(std::vector<float>& distances);
void pcdFarthestPointSample(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr output, int num_samples, bool random_seed = true);
void pcdRandomSample(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr output, const int num_samples);

//Kmeans
void initializeCentroids(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr centroids);
void assignPointsToClusters(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr centroids, std::vector<int>& labels) ;
void updateCentroids(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr centroids, std::vector<int>& labels, int k) ;
float calculateDistance(pcl::PointXYZI& p1, pcl::PointXYZI& p2);
float calculateTotalError(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr centroids, std::vector<int>& labels);
void pcdKmeansCluster(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, ClusterResult &result, int k, const int max_iterations_=100, const double convergence_threshold=0.001);

// DBScanCluster
// bool expandCluster(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, size_t point_idx, int cluster_id, std::vector<int>& labels, int min_points, float epsilon, pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_);
// std::vector<int> regionQuery(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, size_t point_idx, float epsilon, pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_);
// void pcdDBScanCluster(pcl::PointCloud<pcl::PointXYZI>::Ptr input,
//                       std::vector<pcl::PointIndices> &cluster_indices,
//                       double eps = 0.3,
//                       int min_pts = 5);
void pcdDBScanCluster(pcl::PointCloud<pcl::PointXYZI>::Ptr input, 
                      std::vector<pcl::PointIndices> &cluster_indices,
                      double epsilon = 0.3,
                      int min_points = 10) ;


// pcdHierarchicalCluster
// void updateCentroid(ClusterResult& cluster,  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
// void pcdHierarchicalCluster(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, ClusterResult &result, float distance_threshold_);


// key points
double computeCloudResolution(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
void ISSKeyPointExtract(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints, double salient_radius = 0.06,
                        double non_max_radius = 0.04, double normal_radius = 0.04, double threshold21 = 0.975, double threshold32 = 0.975, double min_neighbors = 5, int threads = 4);
void SIFTKeyPointExtract(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints, float min_scale_ = 0.01f, int nr_octaves = 3, int nr_scales_per_octave = 4, float min_contrast = 0.01f);
void harrisKeyPointKeyPointExtract(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints, float threshold = 1e-6, float radius = 0.01, bool non_maxima = true, int method = 1);

//特征提取
void PFHExtractFeature(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, double radius=0.05);



//
void ICPPcMatch(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud,  Eigen::Matrix<float, 4, 4> &transformation_matrix);
void GICPPcMatch(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud,  Eigen::Matrix<float, 4, 4> &transformation_matrix);
void NTDPcMatch(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud,  Eigen::Matrix<float, 4, 4> &transformation_matrix, double thre=0.01, double step=0.1, double res=1.0);


void saveToBin(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, std::string & out_file);
bool get_filelist_from_dir(std::string _path, std::vector<std::string>& _files);
void convertPCDtoBin(std::string & in_file, std::string & out_file);
void pcd2Bin(string &pcd_dir, string &bin_dir);


bool convertBin2PCD(const std::string& input_path, const std::string& output_path);
std::vector<pcl::PointXYZI> readBinaryFile(const std::string& filename);
void bin2Pcd(string &bin_dir, string &pcd_dir);




void pcdClip(pcl::PointCloud<pcl::PointXYZI>::Ptr input, pcl::PointCloud<pcl::PointXYZI>::Ptr output, 
            double min_x = -99999, double max_x = 99999, double min_y = -99999, double max_y = 99999, 
            double min_z = -99999, double max_z = 99999);
void pcdRotateWithMat(pcl::PointCloud<pcl::PointXYZI>::Ptr input, pcl::PointCloud<pcl::PointXYZI>::Ptr output, Eigen::Matrix3f rotationMatrix);
void pcdRotateWithEuler(pcl::PointCloud<pcl::PointXYZI>::Ptr input, pcl::PointCloud<pcl::PointXYZI>::Ptr output,
                        double roll=0, double pitch=0, double yaw=0); // rool->x, pitch->y, yaw->z
void pcdRotateWithQuaternions(pcl::PointCloud<pcl::PointXYZI>::Ptr input, pcl::PointCloud<pcl::PointXYZI>::Ptr output, Eigen::Quaterniond q);
void pcdTrans(pcl::PointCloud<pcl::PointXYZI>::Ptr input, pcl::PointCloud<pcl::PointXYZI>::Ptr output, 
                double trans_x=0, double trans_y=0, double trans_z=0);

//PCA              
void pcdComputeCentroid(const pcl::PointCloud<pcl::PointXYZI>::Ptr input, Eigen::Vector3f &centroid_);
void pcdComputeCovarianceMatrix(const pcl::PointCloud<pcl::PointXYZI>::Ptr input, Eigen::Vector3f &centroid_, Eigen::Matrix3f &covariance_matrix_);
void pcdComputeEigenVectors(Eigen::Matrix3f &covariance_matrix_, Eigen::Matrix3f &eigenvectors_, Eigen::Vector3f &eigenvalues_);
void pcdComputePCA(pcl::PointCloud<pcl::PointXYZI>::Ptr input, Eigen::Matrix3f &eigenvectors_, Eigen::Vector3f &eigenvalues_);

// //Kmeans
// double computeDistance(const pcl::PointXYZI& p1, const pcl::PointXYZI& p2);
// bool checkConvergence(const std::vector<pcl::PointXYZI>& centroids, const std::vector<pcl::PointXYZI>& new_centroids, int k_clusters);
// void updateCentroids(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, std::vector<int>& labels, std::vector<pcl::PointXYZI> &new_centroids, int k_clusters);
// void assignPointsToClusters(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, std::vector<pcl::PointXYZI> &centroids, std::vector<int> &labels, int k_clusters);
// void initializeCentroids(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, std::vector<pcl::PointXYZI> &centroids, int k_clusters);
// void pcdKmeansCluster(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int K_clusters, int max_iterations=100);

void pcdToKDTree(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree);

void pcdGetNearestNeighbor(pcl::PointCloud<pcl::PointXYZI>::Ptr input, pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree, pcl::PointXYZI& result, pcl::PointXYZI& point, double radius = 1.0);


//Clustering
// void pcdEuclideanClustering(pcl::PointCloud<pcl::PointXYZI>::Ptr input, std::vector<pcl::PointIndices> &cluster_indices, double radius = 1.0, int min_cluster_size = 10, int max_cluster_size = 100000);
void pcdEuclideanClustering(pcl::PointCloud<pcl::PointXYZI>::Ptr input, 
                           std::vector<pcl::PointIndices> &cluster_indices,
                           double radius = 1.0,
                           int min_cluster_size = 3,
                           int max_cluster_size = 10000);


void pcdSpectralClustering(pcl::PointCloud<pcl::PointXYZI>::Ptr input,
                          std::vector<pcl::PointIndices> &cluster_indices,
                          int n_clusters = 2,
                          double sigma = 1.0,
                          int k_neighbors = 10);

void pcdAffinityPropagationCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr input,
                                  std::vector<pcl::PointIndices>& cluster_indices,
                                  float damping = 0.9f,
                                  int max_iterations = 10,
                                  int convergence_threshold = 10);
//need to be rewritted
void pcdMeanShiftCluster(pcl::PointCloud<pcl::PointXYZI>::Ptr input, std::vector<pcl::PointIndices> &cluster_indices, float bandwidth = 1.0, int max_iter = 100);

void pcdWardCluster(pcl::PointCloud<pcl::PointXYZI>::Ptr input,
                    std::vector<pcl::PointIndices>& cluster_indices,
                    int num_clusters = 2);
//need to be rewritted
void pcdOPTICSCluster(pcl::PointCloud<pcl::PointXYZI>::Ptr input,
                      std::vector<pcl::PointIndices>& cluster_indices,
                      float eps = 0.2f,
                      int minPts = 5);

void pcdBIRCHCluster(pcl::PointCloud<pcl::PointXYZI>::Ptr input,
                     std::vector<pcl::PointIndices>& cluster_indices,
                     int branching_factor = 50,
                     float max_distance = 0.1f);

struct bbox {
    double x1, y1, x2, y2;
};

// write a function whoes name is pcdBIRCHCluster to perform BIRCH clustering algorithm with pcl, and it's input is pcl::PointCloud<pcl::PointXYZI>::Ptr input, output is std::vector<pcl::PointIndices> &cluster_indices, and you can add other input you need, and return void. And add note with doxygen format
}
#endif /* LIDAR_TOOLS_H */