#include "lidarTools.h"

namespace perception{



/**
 * @brief Load point cloud from PCD file
 * @param[in] filename Path to PCD file to load
 * @param[out] output Output point cloud containing loaded points
 */
void loadPCD(std::string filename, pcl::PointCloud<pcl::PointXYZI>::Ptr output)
{
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(filename, *output) == -1)
    {
        PCL_ERROR("Couldn't read file %s\n", filename.c_str());
        return;
    }
}


/**
 * @brief Filter points based on intensity threshold
 * @param[in] in_cloud Input point cloud
 * @param[out] output Output point cloud containing filtered points
 * @param[in] intensity_thres Intensity threshold value - points with intensity above this are kept
 */
void pcdIntensityThresholdLowerFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr output, int intensity_thres)
{
    for(int i = 0; i < in_cloud->size(); i++)
    {
        pcl::PointXYZI pointTemp;
        if(in_cloud->points[i].intensity > intensity_thres)
        {
            pointTemp.x=in_cloud->points[i].y;
            pointTemp.y=in_cloud->points[i].x;
            pointTemp.z=in_cloud->points[i].z;
            pointTemp.intensity=in_cloud->points[i].intensity;
            output->points.push_back(pointTemp);
        }
    }
}

/**
 * @brief Apply voxel grid filter to downsample point cloud
 * @param[in] in_cloud Input point cloud
 * @param[out] output Output downsampled point cloud
 * @param[in] size_x Voxel size in x dimension
 * @param[in] size_y Voxel size in y dimension 
 * @param[in] size_z Voxel size in z dimension
 */
void pcdVoxelFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr output, const double size_x, const double size_y, const double size_z)
{
    pcl::VoxelGrid<pcl::PointXYZI> sor;
    sor.setInputCloud(in_cloud);
    sor.setLeafSize(size_x, size_y, size_y);
    sor.filter(*output);
}

/**
 * @brief Apply radius outlier removal filter
 * @param[in] in_cloud Input point cloud
 * @param[out] output Output filtered point cloud
 * @param[in] radius Search radius for finding neighbors
 * @param[in] min_neighbors Minimum number of neighbors required to keep point
 */
void pcdRadiusFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr output, const double radius, const int min_neighbors)
{
    pcl::RadiusOutlierRemoval<pcl::PointXYZI> sor;
    sor.setInputCloud(in_cloud);
    sor.setRadiusSearch(radius);
    sor.setMinNeighborsInRadius(min_neighbors);
    sor.filter(*output);
}

/**
 * @brief Apply statistical outlier removal filter
 * @param[in] in_cloud Input point cloud
 * @param[out] output Output filtered point cloud
 * @param[in] mean_K Number of nearest neighbors to use for mean distance estimation
 * @param[in] std Standard deviation multiplier threshold
 */
void pcdStatisticFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr output, const double mean_K, const double std)
{
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    sor.setInputCloud(in_cloud);
    sor.setMeanK(mean_K); // 设置统计滤波器的K值
    sor.setStddevMulThresh(std); // 设置统计滤波器的标准差倍数阈值

    sor.filter(*output);
}

/**
 * @brief Calculate Gaussian weight based on distance
 * @param[in] x Distance value
 * @param[in] sigma Standard deviation parameter
 * @return Gaussian weight value
 */
float gaussian(float x, float sigma) {
    return std::exp(-(x * x) / (2 * sigma * sigma));
}

/**
 * @brief Apply Gaussian filter to smooth point cloud
 * @param[in] in_cloud Input point cloud
 * @param[out] output Output smoothed point cloud
 * @param[in] sigma Controls smoothing strength
 * @param[in] search_radius Controls neighborhood size for smoothing
 */
void pcdGaussianFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr output, const double sigma, const double search_radius)
{
    typename pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_ = typename pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZI>);;
    if (in_cloud->empty()) {
        std::cerr << "Error: Empty input cloud" << std::endl;
        output = nullptr;
        return;
    }

    output->header = in_cloud->header;
    output->sensor_origin_ = in_cloud->sensor_origin_;
    output->sensor_orientation_ = in_cloud->sensor_orientation_;
    output->resize(in_cloud->size());
    // Build KD-tree
    kdtree_->setInputCloud(in_cloud);

    // For each point in the cloud
    //OpenMp  parallelization for better performance
    #pragma omp parallel for
    for (int i = 0; i < static_cast<int>(in_cloud->size()); ++i) {
        // Find neighbors
        std::vector<int> neighbor_indices;
        std::vector<float> neighbor_distances;
        kdtree_->radiusSearch(in_cloud->points[i], search_radius, 
                            neighbor_indices, neighbor_distances);

        if (neighbor_indices.empty()) {
            output->points[i] = in_cloud->points[i];
            continue;
        }

        // Apply Gaussian weighting
        float sum_weights = 0.0f;
        float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
        float sum_intensity = 0.0f;

        for (size_t j = 0; j < neighbor_indices.size(); ++j) {
            float dist = std::sqrt(neighbor_distances[j]);
            float weight = gaussian(dist, sigma);

            const auto& neighbor = in_cloud->points[neighbor_indices[j]];
            sum_x += weight * neighbor.x;
            sum_y += weight * neighbor.y;
            sum_z += weight * neighbor.z;
            sum_intensity += weight * neighbor.intensity;
            sum_weights += weight;
        }

        // Normalize
        if (sum_weights > 0) {
            output->points[i].x = sum_x / sum_weights;
            output->points[i].y = sum_y / sum_weights;
            output->points[i].z = sum_z / sum_weights;
            output->points[i].intensity = sum_intensity / sum_weights;
        } else {
            output->points[i] = in_cloud->points[i];
        }
    }

}

/**
 * @brief Update minimum distances between points and clusters
 * @param[in] cloud Input point cloud
 * @param[in] point_idx Index of current point
 * @param[in,out] min_distances Vector of minimum distances to update
 * @param[in] kdtree KD-tree for neighbor search
 */
void updateDistances(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,  int point_idx,
                    std::vector<float>& min_distances, pcl::KdTreeFLANN<PointXYZI>::Ptr kdtree) {
    std::vector<int> indices;
    std::vector<float> distances;
    
    // Find all points within the maximum current distance
    float max_dist = *std::max_element(min_distances.begin(), min_distances.end());
    kdtree->radiusSearch(cloud->points[point_idx], max_dist, indices, distances);

    // Update minimum distances
    for (size_t i = 0; i < indices.size(); ++i) {
        min_distances[indices[i]] = std::min(min_distances[indices[i]], distances[i]);
    }
}

/**
 * @brief Find point with maximum distance in distance vector
 * @param[in] distances Vector of distances
 * @return Index of point with maximum distance
 */
int findFarthestPoint(std::vector<float>& distances) {
    float max_dist = -1;
    int max_idx = -1;

    for (size_t i = 0; i < distances.size(); ++i) {
        if (distances[i] > max_dist) {
            max_dist = distances[i];
            max_idx = i;
        }
    }

    return max_idx;
}

/**
 * @brief Perform farthest point sampling on point cloud
 * @param[in] in_cloud Input point cloud
 * @param[out] output Output sampled point cloud
 * @param[in] num_samples Number of points to sample
 * @param[in] random_seed Whether to use random initial point
 */
void pcdFarthestPointSample(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr output, int num_samples, bool random_seed)
{
    pcl::KdTreeFLANN<PointXYZI>::Ptr kdtree_ = pcl::KdTreeFLANN<PointXYZI>::Ptr(new pcl::KdTreeFLANN<PointXYZI>);
    std::random_device rd_;
    std::mt19937 gen_;
    
    if (in_cloud->empty() || num_samples <= 0) {
        std::cerr << "Error: Invalid input parameters" << std::endl;
        output = nullptr;
        return ;
    }

    if (num_samples > in_cloud->size()) {
        std::cerr << "Warning: Requested samples exceed input size. Using entire cloud." << std::endl;
        *output = *in_cloud;
        return;
    }

    // Build KD-tree for the input cloud
    kdtree_->setInputCloud(in_cloud);

    // Initialize distance vector to track distances to the sampled set
    std::vector<float> min_distances(in_cloud->size(), std::numeric_limits<float>::max());

    // Vector to store indices of sampled points
    std::vector<int> sampled_indices;
    sampled_indices.reserve(num_samples);

    // Select first point
    int first_point;
    if (random_seed) {
        std::uniform_int_distribution<> dis(0, in_cloud->size() - 1);
        first_point = dis(gen_);
    } else {
        first_point = 0;
    }
    sampled_indices.push_back(first_point);

    // Update distances for all points to the first sampled point
    updateDistances(in_cloud, first_point, min_distances, kdtree_);

    // Select remaining points
    for (int i = 1; i < num_samples; ++i) {
        // Find the point with maximum distance to the sampled set
        int farthest_point = findFarthestPoint(min_distances);
        
        if (farthest_point == -1) break;

        // Add the farthest point to the sampled set
        sampled_indices.push_back(farthest_point);

        // Update distances
        updateDistances(in_cloud, farthest_point, min_distances, kdtree_);
    }
    
    // Create output cloud with sampled points
    output->header = in_cloud->header;
    output->sensor_origin_ = in_cloud->sensor_origin_;
    output->sensor_orientation_ = in_cloud->sensor_orientation_;
    output->reserve(sampled_indices.size());

    for (const auto& idx : sampled_indices) {
        output->push_back(in_cloud->points[idx]);
    }

}

/**
 * @brief Perform random sampling on point cloud
 * @param[in] in_cloud Input point cloud
 * @param[out] output Output sampled point cloud
 * @param[in] num_samples Number of points to sample
 */
void pcdRandomSample(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr output, const int num_samples)
{
    std::mt19937 gen_;

    if (in_cloud->empty() || num_samples <= 0) {
        std::cerr << "Error: Invalid input parameters" << std::endl;
        output = nullptr;
        return;
    }

    if (num_samples >= in_cloud->size()) {
        std::cerr << "Warning: Requested samples exceed input size. Returning entire cloud." << std::endl;
        *output = *in_cloud;
        return;
    }

    // Create index vector
    std::vector<int> indices(in_cloud->size());
    std::iota(indices.begin(), indices.end(), 0);

    // Shuffle indices
    std::shuffle(indices.begin(), indices.end(), gen_);

    // Resize vectors to hold selected points
    indices.resize(num_samples);
    // output->reserve(num_samples);

    // Copy selected points
    for (int i = 0; i < num_samples; ++i) {
        output->push_back(in_cloud->points[indices[i]]);
    }

    // Copy metadata
    output->header = in_cloud->header;
    output->sensor_origin_ = in_cloud->sensor_origin_;
    output->sensor_orientation_ = in_cloud->sensor_orientation_;
    output->is_dense = in_cloud->is_dense;

}

/**
 * @brief Initialize centroids for k-means clustering
 * @param[in] cloud Input point cloud
 * @param[out] centroids Output centroid points
 */
void initializeCentroids(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr centroids)
{
    std::mt19937 gen_;
    std::uniform_int_distribution<> dis(0, cloud->size() - 1);
    std::vector<int> used_indices;
    
    // K-means++ initialization
    for (size_t i = 0; i < centroids->size(); ++i) {
        if (i == 0) {
            // Choose first centroid randomly
            int idx = dis(gen_);
            centroids->points[i] = cloud->points[idx];
            used_indices.push_back(idx);
        } else {
            // Choose subsequent centroids with probability proportional to distance
            std::vector<float> distances(cloud->size(), std::numeric_limits<float>::max());
            float total_distance = 0;

            for (size_t j = 0; j < cloud->size(); ++j) {
                for (const auto& used_idx : used_indices) {
                    float dist = calculateDistance(cloud->points[j], cloud->points[used_idx]);
                    distances[j] = std::min(distances[j], dist);
                }
                total_distance += distances[j];
            }

            // Choose next centroid
            std::uniform_real_distribution<float> dis_float(0, total_distance);
            float rand_val = dis_float(gen_);
            float cumsum = 0;
            int selected_idx = 0;

            for (size_t j = 0; j < cloud->size(); ++j) {
                cumsum += distances[j];
                if (cumsum >= rand_val) {
                    selected_idx = j;
                    break;
                }
            }

            centroids->points[i] = cloud->points[selected_idx];
            used_indices.push_back(selected_idx);
        }
    }
}

/**
 * @brief Assign points to nearest cluster centers
 * @param[in] cloud Input point cloud
 * @param[in] centroids Cluster centroid points
 * @param[out] labels Point cluster assignments
 */
void assignPointsToClusters(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                               pcl::PointCloud<pcl::PointXYZI>::Ptr centroids,
                               std::vector<int>& labels) 
{
    #pragma omp parallel for
    for (int i = 0; i < static_cast<int>(cloud->size()); ++i) {
        float min_dist = std::numeric_limits<float>::max();
        int min_cluster = 0;

        for (size_t j = 0; j < centroids->size(); ++j) {
            float dist = calculateDistance(cloud->points[i], centroids->points[j]);
            if (dist < min_dist) {
                min_dist = dist;
                min_cluster = j;
            }
        }

        labels[i] = min_cluster;
    }
}

/**
 * @brief Update cluster centroids based on point assignments
 * @param[in] cloud Input point cloud
 * @param[in,out] centroids Cluster centroid points to update
 * @param[in] labels Point cluster assignments
 * @param[in] k Number of clusters
 */
void updateCentroids(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr centroids,
                    std::vector<int>& labels, int k) 
{
    std::vector<int> cluster_sizes(k, 0);
        
    // Reset centroids
    for (auto& centroid : centroids->points) {
        centroid.x = 0;
        centroid.y = 0;
        centroid.z = 0;
    }

    // Sum points
    for (size_t i = 0; i < cloud->size(); ++i) {
        int cluster = labels[i];
        centroids->points[cluster].x += cloud->points[i].x;
        centroids->points[cluster].y += cloud->points[i].y;
        centroids->points[cluster].z += cloud->points[i].z;
        cluster_sizes[cluster]++;
    }

    // Calculate means
    for (int i = 0; i < k; ++i) {
        if (cluster_sizes[i] > 0) {
            centroids->points[i].x /= cluster_sizes[i];
            centroids->points[i].y /= cluster_sizes[i];
            centroids->points[i].z /= cluster_sizes[i];
        }
    }
}

/**
 * @brief Calculate Euclidean distance between two points
 * @param[in] p1 First point
 * @param[in] p2 Second point
 * @return Squared Euclidean distance between points
 */
float calculateDistance(pcl::PointXYZI& p1, pcl::PointXYZI& p2) {
    float dx = p1.x - p2.x;
    float dy = p1.y - p2.y;
    float dz = p1.z - p2.z;
    return dx*dx + dy*dy + dz*dz;
}

/**
 * @brief Calculate total clustering error
 * @param[in] cloud Input point cloud
 * @param[in] centroids Cluster centroid points
 * @param[in] labels Point cluster assignments
 * @return Total sum of squared distances to cluster centers
 */
float calculateTotalError(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                        pcl::PointCloud<pcl::PointXYZI>::Ptr centroids,
                        std::vector<int>& labels) {
    float total_error = 0;
    #pragma omp parallel for reduction(+:total_error)
    for (int i = 0; i < static_cast<int>(cloud->size()); ++i) {
        total_error += calculateDistance(cloud->points[i], centroids->points[labels[i]]);
    }
    return total_error;
}

/**
 * @brief Perform k-means clustering on point cloud
 * @param[in] in_cloud Input point cloud
 * @param[out] result Clustering result containing labels and centroids
 * @param[in] k Number of clusters
 * @param[in] max_iterations_ Maximum number of iterations
 * @param[in] convergence_threshold Error threshold for convergence
 */
void pcdKmeansCluster(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, ClusterResult &result, int k, const int max_iterations_, const double convergence_threshold)
{
    if (in_cloud->empty() || k <= 0 || k > in_cloud->size()) {
        std::cerr << "Error: Invalid input parameters" << std::endl;
        return ;
    }

    // Initialize result structure
    result.labels.resize(in_cloud->size());
    result.centroids->resize(k);

    // Initialize centroids randomly
    initializeCentroids(in_cloud, result.centroids);

    float prev_error = std::numeric_limits<float>::max();
    
    // Main k-means loop
    for (int iteration = 0; iteration < max_iterations_; ++iteration) {
        // Assign points to nearest centroid
        assignPointsToClusters(in_cloud, result.centroids, result.labels);

        // Update centroids
        updateCentroids(in_cloud, result.centroids, result.labels, k);

        // Calculate total error
        result.total_error = calculateTotalError(in_cloud, result.centroids, result.labels);

        // Check convergence
        if (std::abs(prev_error - result.total_error) < convergence_threshold) {
            std::cout << "Converged after " << iteration + 1 << " iterations" << std::endl;
            break;
        }

        prev_error = result.total_error;
    }
}

// /**
//  * @brief Expand DBSCAN cluster from seed point
//  * @param[in] cloud Input point cloud
//  * @param[in] point_idx Index of seed point
//  * @param[in] cluster_id Current cluster ID
//  * @param[in,out] labels Point cluster assignments
//  * @param[in] min_points Minimum points for core point
//  * @param[in] epsilon Search radius
//  * @param[in] kdtree KD-tree for neighbor search
//  * @return True if cluster was expanded successfully
//  */
// bool expandCluster(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, size_t point_idx, int cluster_id, std::vector<int>& labels, int min_points, float epsilon, pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_)
// {
//     std::vector<int> seeds = regionQuery(cloud, point_idx, epsilon, kdtree_);

//     // Check if this point is a core point
//     if (seeds.size() < static_cast<size_t>(min_points)) {
//         labels[point_idx] = NOISE;
//         return false;
//     }

//     // Mark as part of cluster
//     labels[point_idx] = cluster_id;

//     // Process all seeds
//     std::queue<int> seed_queue;
//     for (const auto& seed_idx : seeds) {
//         if (labels[seed_idx] != UNCLASSIFIED) {
//             continue;
//         }
//         seed_queue.push(seed_idx);
//         labels[seed_idx] = cluster_id;
//     }

//     // Process queue
//     while (!seed_queue.empty()) {
//         int current_point = seed_queue.front();
//         seed_queue.pop();

//         // Find neighbors
//         std::vector<int> neighbors = regionQuery(cloud, current_point, epsilon, kdtree_);

//         // If this is a core point
//         if (neighbors.size() >= static_cast<size_t>(min_points)) {
//             // Process each neighbor
//             for (const auto& neighbor_idx : neighbors) {
//                 int& neighbor_label = labels[neighbor_idx];
                
//                 // If unclassified or noise, add to cluster
//                 if (neighbor_label == UNCLASSIFIED || neighbor_label == NOISE) {
//                     if (neighbor_label == UNCLASSIFIED) {
//                         seed_queue.push(neighbor_idx);
//                     }
//                     neighbor_label = cluster_id;
//                 }
//             }
//         }
//     }

//     return true;
// }

// /**
//  * @brief Find points within radius of query point
//  * @param[in] cloud Input point cloud
//  * @param[in] point_idx Index of query point
//  * @param[in] epsilon_ Search radius
//  * @param[in] kdtree_ KD-tree for neighbor search
//  * @return Vector of neighbor point indices
//  */
// std::vector<int> regionQuery(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, size_t point_idx, float epsilon_, pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_)
// {
//     std::vector<int> neighbor_indices;
//     std::vector<float> distances;

//     kdtree_->radiusSearch(cloud->points[point_idx], 
//                             epsilon_,
//                             neighbor_indices,
//                             distances);

//     return neighbor_indices;
// }

// /**
//  * @brief Perform DBSCAN clustering on point cloud
//  * @param[in] input Input point cloud
//  * @param[out] cluster_indices Vector of point indices for each cluster
//  * @param[in] epsilon Search radius for finding neighbors
//  * @param[in] min_points Minimum number of points to form a cluster
//  */
// void pcdDBScanCluster(pcl::PointCloud<pcl::PointXYZI>::Ptr input, 
//                       std::vector<pcl::PointIndices> &cluster_indices,
//                       double epsilon,
//                       int min_points)
// {
//     if (input->empty()) {
//         return;
//     }

//     // Create KdTree for searching
//     pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_(new pcl::KdTreeFLANN<pcl::PointXYZI>);
//     kdtree_->setInputCloud(input);

//     // Initialize labels
//     std::vector<int> labels(input->size(), UNCLASSIFIED);
//     int cluster_id = 0;

//     // Process each point
//     for (size_t i = 0; i < input->size(); ++i) {
//         // Skip already labeled points
//         if (labels[i] != UNCLASSIFIED) {
//             continue;
//         }

//         // Try to expand cluster
//         if (expandCluster(input, i, cluster_id, epsilon, min_points, labels, kdtree_)) {
//             // Create new cluster
//             pcl::PointIndices cluster;
//             for (size_t j = 0; j < labels.size(); ++j) {
//                 if (labels[j] == cluster_id) {
//                     cluster.indices.push_back(j);
//                 }
//             }
//             cluster_indices.push_back(cluster);
//             ++cluster_id;
//         }
//     }
// }


/**
 * @brief Perform DBSCAN clustering on point cloud
 * @param[in] input Input point cloud
 * @param[out] cluster_indices Vector of point indices for each cluster
 * @param[in] epsilon Search radius for finding neighbors
 * @param[in] min_points Minimum number of points to form a cluster
 */
void pcdDBScanCluster(pcl::PointCloud<pcl::PointXYZI>::Ptr input, 
                      std::vector<pcl::PointIndices> &cluster_indices,
                      double epsilon,
                      int min_points) 
{
    if (input->empty()) {
        return;
    }

    // Create KdTree for searching
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(input);

    std::vector<bool> processed(input->size(), false);
    std::vector<int> cluster_labels(input->size(), -1);
    int cluster_id = 0;

    // Process each point
    for (size_t i = 0; i < input->size(); i++) {
        if (processed[i]) {
            continue;
        }

        std::vector<int> seed_queue;
        std::vector<int> neighbors;
        std::vector<float> distances;

        // Find neighbors
        tree->radiusSearch(input->points[i], epsilon, neighbors, distances);

        if (neighbors.size() >= min_points) {
            // Start new cluster
            seed_queue.push_back(i);
            cluster_labels[i] = cluster_id;
            processed[i] = true;

            // Process all points in seed queue
            for (size_t j = 0; j < seed_queue.size(); j++) {
                int current_point = seed_queue[j];
                
                // Find neighbors of current point
                neighbors.clear();
                distances.clear();
                tree->radiusSearch(input->points[current_point], epsilon, neighbors, distances);

                if (neighbors.size() >= min_points) {
                    for (const auto& neighbor_idx : neighbors) {
                        if (!processed[neighbor_idx]) {
                            seed_queue.push_back(neighbor_idx);
                            cluster_labels[neighbor_idx] = cluster_id;
                            processed[neighbor_idx] = true;
                        }
                    }
                }
            }

            // Create point indices for this cluster
            pcl::PointIndices cluster;
            for (size_t j = 0; j < cluster_labels.size(); j++) {
                if (cluster_labels[j] == cluster_id) {
                    cluster.indices.push_back(j);
                }
            }
            cluster_indices.push_back(cluster);
            cluster_id++;
        } else {
            processed[i] = true;
        }
    }
}


/**
 * @brief Compute resolution of point cloud
 * @param[in] cloud Input point cloud
 * @return Average distance between points
 */
double computeCloudResolution(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
    double resolution = 0.0;
    int n_points = 0;
    int nres;
    std::vector<int> indices(2);
    std::vector<float> sqr_distances(2);
    pcl::search::KdTree<pcl::PointXYZI> tree;
    tree.setInputCloud(cloud);
    
    for (size_t i = 0; i < cloud->size(); ++i) {
        if (!std::isfinite((*cloud)[i].x))
            continue;
        
        // Find the second nearest neighbor
        nres = tree.nearestKSearch(i, 2, indices, sqr_distances);
        if (nres == 2) {
            resolution += sqrt(sqr_distances[1]);
            ++n_points;
        }
    }
    
    if (n_points != 0)
        resolution /= n_points;
    
    return resolution;
}

/**
 * @brief Extract ISS keypoints from point cloud
 * @param[in] in_cloud Input point cloud
 * @param[out] keypoints Output keypoint cloud
 * @param[in] salient_radius Radius for computing covariance matrix
 * @param[in] non_max_radius Radius for non-maximum suppression
 * @param[in] normal_radius Radius for normal estimation
 * @param[in] threshold21 First eigenvalue threshold
 * @param[in] threshold32 Second eigenvalue threshold
 * @param[in] min_neighbors Minimum number of neighbors
 * @param[in] threads Number of threads to use
 */
void ISSKeyPointExtract(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints, double salient_radius,
                        double non_max_radius, double normal_radius, double threshold21, double threshold32, double min_neighbors, int threads)
{
    // Output cloud for keypoints
    
    // Compute cloud resolution
    double cloud_resolution = computeCloudResolution(in_cloud);
    
    // Setup ISS detector
    pcl::ISSKeypoint3D<pcl::PointXYZI, pcl::PointXYZI> iss_detector;
    iss_detector.setInputCloud(in_cloud);
    
    // Set ISS parameters relative to cloud resolution
    iss_detector.setSalientRadius(salient_radius * cloud_resolution);
    iss_detector.setNonMaxRadius(non_max_radius * cloud_resolution);
    
    iss_detector.setThreshold21(threshold21);
    iss_detector.setThreshold32(threshold32);
    iss_detector.setMinNeighbors(min_neighbors);
    iss_detector.setNumberOfThreads(threads);
    
    // Compute keypoints
    iss_detector.compute(*keypoints);
}

/**
 * @brief Extract SIFT keypoints from point cloud
 * @param[in] in_cloud Input point cloud
 * @param[out] keypoints Output keypoint cloud
 * @param[in] min_scale_ Minimum scale for keypoint detection
 * @param[in] nr_octaves Number of octaves
 * @param[in] nr_scales_per_octave Number of scales per octave
 * @param[in] min_contrast Minimum contrast threshold
 */
void SIFTKeyPointExtract(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints, float min_scale_, int nr_octaves, int nr_scales_per_octave, float min_contrast)
{   
    // Estimate cloud resolution for scale parameters
    float resolution = computeCloudResolution(in_cloud);
    float min_scale = min_scale_ * resolution;
    
    // Setup SIFT detector
    pcl::SIFTKeypoint<pcl::PointXYZI, pcl::PointWithScale> sift_detector;
    sift_detector.setInputCloud(in_cloud);
    
    // Create KdTree for searching
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
    sift_detector.setSearchMethod(tree);
    
    // Set SIFT parameters
    sift_detector.setScales(min_scale, nr_octaves, nr_scales_per_octave);
    sift_detector.setMinimumContrast(min_contrast);
    
    // Compute keypoints
    sift_detector.compute(*keypoints);
}

/**
 * @brief Extract Harris keypoints from point cloud
 * @param[in] in_cloud Input point cloud
 * @param[out] keypoints Output keypoint cloud
 * @param[in] threshold Response threshold
 * @param[in] radius Neighborhood size
 * @param[in] non_maxima Enable/disable non-maxima suppression
 * @param[in] method Response calculation method
 */

// threshold: Response threshold for keypoint detection
// radius: The neighborhood size for keypoint detection
// non_maxima: Enable/disable non-maxima suppression
// method: Selection of response calculation method
void harrisKeyPointKeyPointExtract(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints, float threshold, float radius, bool non_maxima, int method)
{ 
    // Estimate cloud resolution for radius
    float resolution = computeCloudResolution(in_cloud);
    float adjusted_radius = radius * resolution;
    
    // Setup Harris detector
    pcl::HarrisKeypoint3D<pcl::PointXYZI, pcl::PointXYZI> harris_detector;
    harris_detector.setInputCloud(in_cloud);
    
    // Set Harris parameters
    harris_detector.setRadius(adjusted_radius);
    harris_detector.setThreshold(threshold);
    harris_detector.setNonMaxSupression(non_maxima);
    harris_detector.setRefine(true);
    
    // Set method
    switch(method) {
        case 1:
            harris_detector.setMethod(pcl::HarrisKeypoint3D<pcl::PointXYZI, pcl::PointXYZI>::HARRIS);
            break;
        case 2:
            harris_detector.setMethod(pcl::HarrisKeypoint3D<pcl::PointXYZI, pcl::PointXYZI>::NOBLE);
            break;
        case 3:
            harris_detector.setMethod(pcl::HarrisKeypoint3D<pcl::PointXYZI, pcl::PointXYZI>::LOWE);
            break;
        case 4:
            harris_detector.setMethod(pcl::HarrisKeypoint3D<pcl::PointXYZI, pcl::PointXYZI>::TOMASI);
            break;
        case 5:
            harris_detector.setMethod(pcl::HarrisKeypoint3D<pcl::PointXYZI, pcl::PointXYZI>::CURVATURE);
            break;
        default:
            harris_detector.setMethod(pcl::HarrisKeypoint3D<pcl::PointXYZI, pcl::PointXYZI>::HARRIS);
    }
    
    // Compute keypoints
    harris_detector.compute(*keypoints);
}

/**
 * @brief Extracts Point Feature Histogram (PFH) features from a point cloud
 * @param in_cloud Input point cloud
 * @param radius Search radius for feature computation
 */
void PFHExtractFeature(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, double radius)
{
    // 计算点云的法线
    pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    ne.setInputCloud(in_cloud);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.03); // 搜索半径，根据点云的尺度调整
    ne.compute(*normals);

    // PFH特征提取
    pcl::PFHEstimation<pcl::PointXYZI, pcl::Normal, pcl::PFHSignature125> pfh;
    pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhs(new pcl::PointCloud<pcl::PFHSignature125>());
    pfh.setInputCloud(in_cloud);
    pfh.setInputNormals(normals);
    pfh.setSearchMethod(tree);
    pfh.setRadiusSearch(0.05); // 搜索半径，根据点云的尺度调整
    pfh.compute(*pfhs);

    // 输出PFH特征信息
    std::cout << "PFH feature size: " << pfhs->points.size() << std::endl;
    for (size_t i = 0; i < pfhs->points.size(); ++i) {
        std::cout << "PFH feature " << i << ": ";
        for (size_t j = 0; j < 125; ++j) {
            std::cout << pfhs->points[i].histogram[j] << " ";
        }
        std::cout << std::endl;
    }
}

/**
 * @brief Performs Iterative Closest Point (ICP) matching between two point clouds
 * @param in_cloud Source point cloud
 * @param target_cloud Target point cloud
 * @param[out] transformation_matrix The resulting transformation matrix
 */
void ICPPcMatch(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud, Eigen::Matrix<float, 4, 4> &transformation_matrix)
{
    pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
    icp.setInputSource (in_cloud);
    icp.setInputTarget (target_cloud);
    pcl::PointCloud<pcl::PointXYZI> FinalCloud;
    icp.align (FinalCloud);
    
    transformation_matrix = icp.getFinalTransformation();
}

/**
 * @brief Performs Generalized Iterative Closest Point (GICP) matching between two point clouds
 * @param in_cloud Source point cloud
 * @param target_cloud Target point cloud
 * @param[out] transformation_matrix The resulting transformation matrix
 */
void GICPPcMatch(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud,  Eigen::Matrix<float, 4, 4> &transformation_matrix)
{
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> gicp;
    gicp.setInputSource(in_cloud);
    gicp.setInputTarget(target_cloud);
    pcl::PointCloud<pcl::PointXYZI> FinalCloud;
    gicp.align(FinalCloud);

    transformation_matrix = gicp.getFinalTransformation();
}

/**
 * @brief Performs Normal Distributions Transform (NDT) matching between two point clouds
 * @param in_cloud Source point cloud
 * @param target_cloud Target point cloud
 * @param[out] transformation_matrix The resulting transformation matrix
 * @param thre Transformation epsilon threshold
 * @param step Step size for optimization
 * @param res Resolution of the voxel grid
 */
void NTDPcMatch(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud,  Eigen::Matrix<float, 4, 4> &transformation_matrix, double thre, double step, double res)
{
    // 创建NDT对象
    pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;
    ndt.setInputSource(in_cloud);
    ndt.setInputTarget(target_cloud);

    // 设置NDT参数
    ndt.setTransformationEpsilon(thre); // 变换收敛阈值
    ndt.setStepSize(step); // 优化步长
    ndt.setResolution(res); // 体素网格的分辨率

    // 输出点云存储
    pcl::PointCloud<pcl::PointXYZI> FinalCloud;
    ndt.align(FinalCloud);

    transformation_matrix = ndt.getFinalTransformation();
}

/**
 * @brief Gets list of files from a directory
 * @param _path Directory path
 * @param[out] _files Vector to store filenames
 * @return true if successful
 */
bool get_filelist_from_dir(std::string _path, std::vector<std::string>& _files)
{
    DIR* dir = opendir(_path.c_str());
    struct dirent* ptr;
    std::vector<std::string> file;
    while((ptr = readdir(dir)) != NULL)
    {
        if(ptr->d_name[0] == '.')	continue;
        _files.push_back(ptr->d_name);
    }
    closedir(dir);
    sort(_files.begin(), _files.end());
}

/**
 * @brief Converts a PCD file to binary format
 * @param in_file Input PCD file path
 * @param out_file Output binary file path
 */
void convertPCDtoBin(std::string & in_file, std::string & out_file)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

    if (pcl::io::loadPCDFile<pcl::PointXYZI>(in_file, *cloud) == -1)
    {
        std::string err = "Couldn't read file " + in_file;
        PCL_ERROR(err.c_str());
        return;
    }
    saveToBin(cloud, out_file);
    
}

/**
 * @brief Saves point cloud to binary format
 * @param cloud Input point cloud
 * @param out_file Output binary file path
 */
void saveToBin(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, std::string & out_file)
{
    std::ofstream output(out_file.c_str(), std::ios::out | std::ios::binary);
    float intensity = 1.0;
    for (int j = 0; j < cloud->size(); j++)
    {
        output.write((char*)& cloud->at(j).x, sizeof(cloud->at(j).x));
        output.write((char*)& cloud->at(j).y, sizeof(cloud->at(j).y));
        output.write((char*)& cloud->at(j).z, sizeof(cloud->at(j).z));
        intensity = cloud->at(j).intensity / 255.0;
        output.write((char*)& intensity, sizeof(intensity));
        /*std::cout << cloud->at(j).x << " "
                  << cloud->at(j).y << " "
                  << cloud->at(j).z << " "
                  << intensity << std::endl;*/
    }
    output.close();
}

/**
 * @brief Converts all PCD files in a directory to binary format
 * @param pcd_dir Input PCD directory path
 * @param bin_dir Output binary directory path
 */
void pcd2Bin(string &pcd_dir, string &bin_dir)
{
    std::vector<std::string> pcdNames;
    get_filelist_from_dir(pcd_dir, pcdNames);
    std::cout << "converting..." << std::endl;
    for(int i = 0; i < pcdNames.size(); i++) {
        //std::cout << "<<<<<<<<<<<<<<<<<<<<<<<<< " << i << " >>>>>>>>>>>>>>>>>>>>>>>>>" << std::endl;
        std::string pcdfile = pcd_dir + pcdNames[i].c_str();
        std::string frameName =  pcdNames[i].substr(0,pcdNames[i].find('.',0));
        std::string binfile = bin_dir + frameName +".bin";
        convertPCDtoBin(pcdfile, binfile);
    }
}

/**
 * @brief Converts a binary point cloud file to PCD format
 * @param input_path Input binary file path
 * @param output_path Output PCD file path
 * @return true if conversion successful
 */
bool convertBin2PCD(const std::string& input_path, const std::string& output_path)
{
    // Check if input file exists
        if (!boost::filesystem::exists(input_path)) {
            std::cerr << "Error: Input file does not exist: " << input_path << std::endl;
            return false;
        }

        // Read binary file
        std::vector<pcl::PointXYZI> points = readBinaryFile(input_path);
        if (points.empty()) {
            std::cerr << "Error: No points read from file or file is empty" << std::endl;
            return false;
        }

        // Convert to PCL point cloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        cloud->points.resize(points.size());

        for (size_t i = 0; i < points.size(); i++) {
            cloud->points[i].x = points[i].x;
            cloud->points[i].y = points[i].y;
            cloud->points[i].z = points[i].z;
            cloud->points[i].intensity = points[i].intensity;
        }

        // Set point cloud metadata
        cloud->width = points.size();
        cloud->height = 1; // unorganized point cloud
        cloud->is_dense = true;

        // Save PCD file
        if (pcl::io::savePCDFile(output_path, *cloud) == -1) {
            std::cerr << "Error: Failed to save PCD file: " << output_path << std::endl;
            return false;
        }

        std::cout << "Successfully converted " << input_path << " to " << output_path << std::endl;
        std::cout << "Number of points: " << cloud->size() << std::endl;
        return true;
}

/**
 * @brief Reads points from a binary file
 * @param filename Input binary file path
 * @return Vector of points read from file
 */
std::vector<pcl::PointXYZI> readBinaryFile(const std::string& filename) {
    // Open file in binary mode
    std::ifstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file: " << filename << std::endl;
        return std::vector<pcl::PointXYZI>();
    }

    // Get file size
    file.seekg(0, std::ios::end);
    const size_t num_elements = file.tellg() / sizeof(pcl::PointXYZI);
    file.seekg(0, std::ios::beg);

    // Read data
    std::vector<pcl::PointXYZI> points(num_elements);
    file.read(reinterpret_cast<char*>(points.data()), num_elements * sizeof(pcl::PointXYZI));

    if (!file) {
        std::cerr << "Error: Only " << file.gcount() << " bytes could be read" << std::endl;
        return std::vector<pcl::PointXYZI>();
    }

    file.close();
    return points;
}

/**
 * @brief Converts all binary files in a directory to PCD format
 * @param bin_dir Input binary directory path
 * @param pcd_dir Output PCD directory path
 */
void bin2Pcd(string &bin_dir, string &pcd_dir)
{
    std::vector<std::string> bin_names;
    get_filelist_from_dir(bin_dir, bin_names);
    std::cout << "converting..." << std::endl;
    for(int i = 0; i < bin_names.size(); i++) {
        //std::cout << "<<<<<<<<<<<<<<<<<<<<<<<<< " << i << " >>>>>>>>>>>>>>>>>>>>>>>>>" << std::endl;
        std::string binfile = bin_dir + bin_names[i].c_str();
        std::string frameName =  bin_names[i].substr(0,bin_names[i].find('.',0));
        std::string pcdfile = pcd_dir + frameName +".pcd";
        convertBin2PCD(binfile, pcdfile);
    }
}

/**
 * @brief Clips a point cloud to specified boundaries
 * @param input Input point cloud
 * @param output Output point cloud after clipping
 * @param min_x Minimum X boundary
 * @param max_x Maximum X boundary
 * @param min_y Minimum Y boundary
 * @param max_y Maximum Y boundary
 * @param min_z Minimum Z boundary
 * @param max_z Maximum Z boundary
 */
void pcdClip(pcl::PointCloud<pcl::PointXYZI>::Ptr input, pcl::PointCloud<pcl::PointXYZI>::Ptr output, double min_x, double max_x, double min_y, double max_y, double min_z, double max_z)
{
    pcl::CropBox<pcl::PointXYZI> crop;
    crop.setInputCloud(input);
    crop.setMin(Eigen::Vector4f(min_x, min_y, min_z, -400));
    crop.setMax(Eigen::Vector4f(max_x, max_y, max_z, 400));
    crop.filter(*output);

}

/**
 * @brief Rotates a point cloud using a rotation matrix
 * @param input Input point cloud
 * @param output Output rotated point cloud
 * @param rotationMatrix 3x3 rotation matrix
 */
void pcdRotateWithMat(pcl::PointCloud<pcl::PointXYZI>::Ptr input, pcl::PointCloud<pcl::PointXYZI>::Ptr output, Eigen::Matrix3f rotationMatrix)
{
    pcl::PointXYZI result;
    for (int i = 0; i < input->size(); ++i) {
        Eigen::Vector3f point(input->points[i].x, input->points[i].y, input->points[i].z);
        point = rotationMatrix * point;
        result.x = point(0);
        result.y = point(1);
        result.z = point(2);
        result.intensity = input->points[i].intensity;
        output->points.push_back(result);
    }
}

/**
 * @brief Rotates a point cloud using Euler angles
 * @param input Input point cloud
 * @param output Output rotated point cloud
 * @param roll Roll angle in radians
 * @param pitch Pitch angle in radians
 * @param yaw Yaw angle in radians
 */
void pcdRotateWithEuler(pcl::PointCloud<pcl::PointXYZI>::Ptr input, pcl::PointCloud<pcl::PointXYZI>::Ptr output, double roll, double pitch, double yaw)
{
    pcl::PointXYZI result;
    Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitZ());

    Eigen::Quaternionf q = yawAngle * pitchAngle * rollAngle;
    Eigen::Matrix3f rotationMatrix = q.matrix();

    // 对每个点进行旋转
    for (size_t i = 0; i < input->points.size(); ++i) {
        Eigen::Vector3f point(input->points[i].x, input->points[i].y, input->points[i].z);
        point = rotationMatrix * point;
        result.x = point.x();
        result.y = point.y();
        result.z = point.z();
        result.intensity = input->points[i].intensity;
        output->points.push_back(result);
    }
}

/**
 * @brief Rotates a point cloud using quaternions
 * @param input Input point cloud
 * @param output Output rotated point cloud
 * @param q Quaternion for rotation
 */
void pcdRotateWithQuaternions(pcl::PointCloud<pcl::PointXYZI>::Ptr input, pcl::PointCloud<pcl::PointXYZI>::Ptr output, Eigen::Quaterniond q)
{
    pcl::PointXYZI result;
    Eigen::Matrix3d rotationMatrix = q.toRotationMatrix();
    for (int i = 0; i < input->size(); ++i) {
        Eigen::Vector3d point(input->points[i].x, input->points[i].y, input->points[i].z);
        point = rotationMatrix * point;
        result.x = point(0);
        result.y = point(1);
        result.z = point(2);
        result.intensity = input->points[i].intensity;
        output->points.push_back(result);
    }
}

/**
 * @brief Translates a point cloud
 * @param input Input point cloud
 * @param output Output translated point cloud
 * @param trans_x Translation in X direction
 * @param trans_y Translation in Y direction
 * @param trans_z Translation in Z direction
 */
void pcdTrans(pcl::PointCloud<pcl::PointXYZI>::Ptr input, pcl::PointCloud<pcl::PointXYZI>::Ptr output, double trans_x, double trans_y, double trans_z)
{
    pcl::PointXYZI result;
    // 对每个点进行平移
    for (size_t i = 0; i < input->points.size(); ++i) {
        result.x = input->points[i].x + trans_x;
        result.y = input->points[i].y + trans_y;
        result.z = input->points[i].z + trans_z;
        result.intensity = input->points[i].intensity;
        output->points.push_back(result);
    }
}

/**
 * @brief Computes the centroid of a point cloud
 * @param input Input point cloud
 * @param[out] centroid_ Computed centroid vector
 */
void pcdComputeCentroid(const pcl::PointCloud<pcl::PointXYZI>::Ptr input, Eigen::Vector3f &centroid_) {
    centroid_.setZero();
    for (const auto& point : input->points) {
        centroid_[0] += point.x;
        centroid_[1] += point.y;
        centroid_[2] += point.z;
    }
    centroid_ /= static_cast<float>(input->size());
}

/**
 * @brief Computes the covariance matrix of a point cloud
 * @param input Input point cloud
 * @param centroid_ Centroid of the point cloud
 * @param[out] covariance_matrix_ Computed covariance matrix
 */
void pcdComputeCovarianceMatrix(const pcl::PointCloud<pcl::PointXYZI>::Ptr input, Eigen::Vector3f &centroid_, Eigen::Matrix3f &covariance_matrix_) {
    covariance_matrix_.setZero();
    
    for (const auto& point : input->points) {
        Eigen::Vector3f pt(point.x - centroid_[0],
                            point.y - centroid_[1],
                            point.z - centroid_[2]);
        
        covariance_matrix_ += pt * pt.transpose();
    }
    
    covariance_matrix_ /= static_cast<float>(input->size() - 1);
}

/**
 * @brief Computes eigenvectors and eigenvalues from a covariance matrix
 * @param covariance_matrix_ Input covariance matrix
 * @param[out] eigenvectors_ Computed eigenvectors
 * @param[out] eigenvalues_ Computed eigenvalues
 */
void pcdComputeEigenVectors(Eigen::Matrix3f &covariance_matrix_, Eigen::Matrix3f &eigenvectors_, Eigen::Vector3f &eigenvalues_) {
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance_matrix_);
    eigenvalues_ = solver.eigenvalues();
    eigenvectors_ = solver.eigenvectors();
    
    // Sort eigenvalues and eigenvectors in descending order
    for (int i = 0; i < 2; ++i) {
        int max_idx = i;
        for (int j = i + 1; j < 3; ++j) {
            if (eigenvalues_[j] > eigenvalues_[max_idx]) {
                max_idx = j;
            }
        }
        if (max_idx != i) {
            std::swap(eigenvalues_[i], eigenvalues_[max_idx]);
            Eigen::Vector3f temp = eigenvectors_.col(i);
            eigenvectors_.col(i) = eigenvectors_.col(max_idx);
            eigenvectors_.col(max_idx) = temp;
        }
    }
}

/**
 * @brief Performs Principal Component Analysis (PCA) on a point cloud
 * @param input Input point cloud
 * @param[out] eigenvectors_ Computed eigenvectors
 * @param[out] eigenvalues_ Computed eigenvalues
 */
void pcdComputePCA(pcl::PointCloud<pcl::PointXYZI>::Ptr input, Eigen::Matrix3f &eigenvectors_, Eigen::Vector3f &eigenvalues_)
{
    if (input->empty()) {
        std::cerr << "Error: Empty point cloud" << std::endl;
        return;
    }
    Eigen::Vector3f centroid_;
    Eigen::Matrix3f covariance_matrix_;
    // Step 1: Compute centroid
    pcdComputeCentroid(input, centroid_);

    // Step 2: Compute covariance matrix
    pcdComputeCovarianceMatrix(input, centroid_, covariance_matrix_);

    // Step 3: Compute eigenvalues and eigenvectors
    pcdComputeEigenVectors(covariance_matrix_, eigenvectors_, eigenvalues_);
}

// void pcdKmeansCluster(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int K_clusters, int max_iterations)
// {
//     std::vector<pcl::PointXYZI> centroids;
//     std::vector<int> point_labels;
//     // Initialize centroids randomly from the input points
//     initializeCentroids(cloud, centroids, K_clusters);
    
//     bool converged = false;
//     int iteration = 0;
    
//     while (!converged && iteration < max_iterations) {
//         // Assign points to nearest centroid
//         std::vector<int> new_labels(cloud->points.size());
//         assignPointsToClusters(cloud, centroids, new_labels, K_clusters);
        
//         // Update centroids based on mean of assigned points
//         std::vector<pcl::PointXYZI> new_centroids(K_clusters);
//         updateCentroids(cloud, new_labels, new_centroids, K_clusters);
        
//         // Check for convergence
//         converged = checkConvergence(centroids, new_centroids, K_clusters);
        
//         centroids = new_centroids;
//         point_labels = new_labels;
//         iteration++;
//     }
    
//     std::cout << "K-means converged after " << iteration << " iterations." << std::endl;

// }

// double computeDistance(const pcl::PointXYZI& p1, const pcl::PointXYZI& p2) {
//     return std::sqrt(
//         std::pow(p1.x - p2.x, 2) + 
//         std::pow(p1.y - p2.y, 2) + 
//         std::pow(p1.z - p2.z, 2)
//     );
// }

// bool checkConvergence(const std::vector<pcl::PointXYZI>& centroids, const std::vector<pcl::PointXYZI>& new_centroids, int k_clusters) {
//     const double threshold = 1e-4;
    
//     for (int i = 0; i < k_clusters; i++) {
//         if (computeDistance(centroids[i], new_centroids[i]) > threshold) {
//             return false;
//         }
//     }
    
//     return true;
// }

// void updateCentroids(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, 
//                     std::vector<int>& labels,
//                     std::vector<pcl::PointXYZI> &new_centroids,
//                     int k_clusters) {
    
//     std::vector<int> cluster_sizes(k_clusters, 0);
    
//     // Sum up all points in each cluster
//     for (size_t i = 0; i < cloud->points.size(); i++) {
//         int label = labels[i];
//         new_centroids[label].x += cloud->points[i].x;
//         new_centroids[label].y += cloud->points[i].y;
//         new_centroids[label].z += cloud->points[i].z;
//         cluster_sizes[label]++;
//     }
    
//     // Compute mean position for each cluster
//     for (int i = 0; i < k_clusters; i++) {
//         if (cluster_sizes[i] > 0) {
//             new_centroids[i].x /= cluster_sizes[i];
//             new_centroids[i].y /= cluster_sizes[i];
//             new_centroids[i].z /= cluster_sizes[i];
//         }
//     }
// }

// void assignPointsToClusters(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, std::vector<pcl::PointXYZI> &centroids, std::vector<int> &labels, int k_clusters)
// {
//     for (size_t i = 0; i < cloud->points.size(); i++) {
//         double min_dist = std::numeric_limits<double>::max();
//         int label = 0;
        
//         // Find nearest centroid
//         for (int j = 0; j < k_clusters; j++) {
//             double dist = computeDistance(cloud->points[i], centroids[j]);
//             if (dist < min_dist) {
//                 min_dist = dist;
//                 label = j;
//             }
//         }
        
//         labels[i] = label;
//     }
// }

// void initializeCentroids(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, std::vector<pcl::PointXYZI> &centroids, int k_clusters) {
//     centroids.clear();
//     std::random_device rd;
//     std::mt19937 gen(rd());
//     std::uniform_int_distribution<> dis(0, cloud->points.size() - 1);

//     // Randomly select k points as initial centroids
//     for (int i = 0; i < k_clusters; i++) {
//         centroids.push_back(cloud->points[dis(gen)]);
//     }
// }

/**
 * @brief Creates a KD-tree structure from a point cloud for efficient nearest neighbor searches
 * @details Builds a KD-tree data structure from the input point cloud to enable fast spatial queries
 * 
 * @param[in] cloud Input point cloud to build the KD-tree from
 * @param[out] kdtree Output KD-tree structure
 */
void pcdToKDTree(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree)
{
    kdtree->setInputCloud(cloud);   
}

/**
 * @brief Finds the nearest neighbor point to a query point within a given radius
 * @details Uses KD-tree to efficiently search for the closest point to the query point
 * 
 * @param[in] input Input point cloud containing all points
 * @param[in] kdtree KD-tree structure built from the input cloud
 * @param[out] result The nearest neighbor point found
 * @param[in] point Query point to find neighbors for
 * @param[in] radius Search radius limit
 */
void pcdGetNearestNeighbor(pcl::PointCloud<pcl::PointXYZI>::Ptr input, pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree, pcl::PointXYZI& result, pcl::PointXYZI& point, double radius)
{
    std::vector<int> indices(1);
    std::vector<float> distances(1);
    kdtree->nearestKSearch(point, radius, indices, distances);      
    result = input->points[indices[0]];
}

// /**
//  * @brief Performs Euclidean clustering on a point cloud
//  * @details Groups points into clusters based on Euclidean distance between points
//  * 
//  * @param[in] input Input point cloud to be clustered
//  * @param[out] cluster_indices Vector of point indices for each cluster found
//  * @param[in] radius Maximum distance between points in a cluster
//  * @param[in] min_cluster_size Minimum number of points required to form a cluster
//  * @param[in] max_cluster_size Maximum number of points allowed in a cluster
//  */
// void pcdEuclideanClustering(pcl::PointCloud<pcl::PointXYZI>::Ptr input, std::vector<pcl::PointIndices> &cluster_indices, double radius, int min_cluster_size, int max_cluster_size)
// {
//     pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
//     tree->setInputCloud(input);

//     pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
//     ec.setClusterTolerance(radius);
//     ec.setMinClusterSize(min_cluster_size);
//     ec.setMaxClusterSize(max_cluster_size);
//     ec.setSearchMethod(tree);
//     ec.setInputCloud(input);
//     ec.extract(cluster_indices);
// }

/**
 * @brief Performs Euclidean clustering on a point cloud
 * @details Groups points into clusters based on Euclidean distance between points
 * 
 * @param[in] input Input point cloud to be clustered
 * @param[out] cluster_indices Vector of point indices for each cluster found
 * @param[in] radius Maximum distance between points in a cluster
 * @param[in] min_cluster_size Minimum number of points required to form a cluster
 * @param[in] max_cluster_size Maximum number of points allowed in a cluster
 */
void pcdEuclideanClustering(pcl::PointCloud<pcl::PointXYZI>::Ptr input, 
                           std::vector<pcl::PointIndices> &cluster_indices,
                           double radius,
                           int min_cluster_size,
                           int max_cluster_size)
{
    if (input->empty()) {
        return;
    }

    // Create KdTree object for search
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(input);

    // Create vector for storing processed point flags
    std::vector<bool> processed(input->size(), false);
    
    // Process each point in the cloud
    for (size_t i = 0; i < input->size(); ++i) {
        if (processed[i]) {
            continue;
        }

        // Create vector for current cluster
        std::vector<int> cluster_indices_temp;
        std::vector<int> seed_queue;
        seed_queue.push_back(i);
        processed[i] = true;

        // Process all points in the seed queue
        for (size_t j = 0; j < seed_queue.size(); ++j) {
            // Find neighbors within radius
            std::vector<int> neighbors;
            std::vector<float> distances;
            tree->radiusSearch(input->points[seed_queue[j]], radius, neighbors, distances);

            // Add neighbors to cluster
            for (const auto& neighbor_idx : neighbors) {
                if (!processed[neighbor_idx]) {
                    seed_queue.push_back(neighbor_idx);
                    processed[neighbor_idx] = true;
                }
            }
        }

        // If cluster meets size requirements, add it to results
        if (seed_queue.size() >= min_cluster_size && seed_queue.size() <= max_cluster_size) {
            pcl::PointIndices cluster;
            cluster.indices = seed_queue;
            cluster_indices.push_back(cluster);
        }
    }
}


/**
 * @brief Performs spectral clustering on a point cloud
 * @param input Input point cloud to be clustered
 * @param cluster_indices Output vector containing the indices of points in each cluster
 * @param n_clusters Number of clusters to generate
 * @param sigma Gaussian kernel bandwidth parameter
 * @param k_neighbors Number of nearest neighbors for similarity graph construction
 */
void pcdSpectralClustering(pcl::PointCloud<pcl::PointXYZI>::Ptr input,
                          std::vector<pcl::PointIndices> &cluster_indices,
                          int n_clusters,
                          double sigma,
                          int k_neighbors) {
    if (input->empty()) {
        return;
    }

    int n_points = input->size();

    // Build KD-tree for nearest neighbor search
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(input);

    // Construct similarity matrix using k-nearest neighbors
    Eigen::MatrixXf W = Eigen::MatrixXf::Zero(n_points, n_points);
    std::vector<int> neighbors;
    std::vector<float> distances;

    for (int i = 0; i < n_points; i++) {
        tree->nearestKSearch(input->points[i], k_neighbors, neighbors, distances);
        for (int j = 0; j < neighbors.size(); j++) {
            float dist = sqrt(distances[j]);
            float weight = exp(-dist * dist / (2 * sigma * sigma));
            W(i, neighbors[j]) = weight;
            W(neighbors[j], i) = weight;
        }
    }

    // Compute degree matrix
    Eigen::VectorXf D = W.rowwise().sum();

    // Compute normalized Laplacian matrix
    Eigen::MatrixXf L = Eigen::MatrixXf::Identity(n_points, n_points);
    for (int i = 0; i < n_points; i++) {
        for (int j = 0; j < n_points; j++) {
            if (D(i) > 0 && D(j) > 0) {
                L(i,j) -= W(i,j) / sqrt(D(i) * D(j));
            }
        }
    }

    // Compute eigenvectors
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> eigen_solver(L);
    Eigen::MatrixXf eigenvectors = eigen_solver.eigenvectors();

    // Take first n_clusters eigenvectors
    Eigen::MatrixXf features = eigenvectors.leftCols(n_clusters);

    // Normalize rows to unit length
    for (int i = 0; i < n_points; i++) {
        float norm = features.row(i).norm();
        if (norm > 0) {
            features.row(i) /= norm;
        }
    }

    // Perform k-means clustering on the features
    std::vector<int> labels(n_points);
    std::vector<Eigen::VectorXf> centroids(n_clusters);

    // Initialize centroids randomly
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, n_points - 1);
    for (int i = 0; i < n_clusters; i++) {
        centroids[i] = features.row(dis(gen));
    }

    // K-means iterations
    bool converged = false;
    int max_iter = 100;
    int iter = 0;

    while (!converged && iter < max_iter) {
        converged = true;
        
        // Assign points to nearest centroid
        for (int i = 0; i < n_points; i++) {
            float min_dist = std::numeric_limits<float>::max();
            int best_cluster = 0;
            
            for (int j = 0; j < n_clusters; j++) {
                // Convert centroid vector to row vector for comparison
                Eigen::RowVectorXf centroid_row = centroids[j].transpose();
                float dist = (features.row(i) - centroid_row).squaredNorm();
                if (dist < min_dist) {
                    min_dist = dist;
                    best_cluster = j;
                }
            }
            
            if (labels[i] != best_cluster) {
                labels[i] = best_cluster;
                converged = false;
            }
        }
        
        // Update centroids
        // Initialize new centroids with correct dimensions
        std::vector<Eigen::VectorXf> new_centroids(n_clusters, Eigen::VectorXf::Zero(features.cols()));
        std::vector<int> counts(n_clusters, 0);
        
        for (int i = 0; i < n_points; i++) {
            // Convert row to column vector for accumulation
            new_centroids[labels[i]] += features.row(i).transpose();
            counts[labels[i]]++;
        }
        
        for (int i = 0; i < n_clusters; i++) {
            if (counts[i] > 0) {
                centroids[i] = new_centroids[i] / counts[i];
            }
        }
        
        iter++;
    }

    // Convert labels to cluster indices
    cluster_indices.clear();
    cluster_indices.resize(n_clusters);
    for (int i = 0; i < n_points; i++) {
        cluster_indices[labels[i]].indices.push_back(i);
    }
}

/**
 * @brief Performs Affinity Propagation clustering on a PCL point cloud.
 *
 * This function implements the Affinity Propagation algorithm, which identifies
 * clusters by selecting exemplars through message passing (responsibility and
 * availability) based on point similarity. It uses squared Euclidean distance
 * as the similarity metric and outputs clusters as point indices.
 *
 * @param[in] input Pointer to the input point cloud (XYZI format).
 * @param[out] cluster_indices Vector of PointIndices containing the resulting clusters.
 * @param[in] damping Damping factor (0.5 to 1.0) to stabilize convergence.
 * @param[in] max_iterations Maximum number of iterations to run the algorithm.
 * @param[in] convergence_threshold Number of iterations with no exemplar change to stop early.
 * @note Similarity is computed as negative squared Euclidean distance; no pre-set cluster count needed.
 * @note Requires a non-empty input cloud; otherwise, cluster_indices will be empty.
 */
void pcdAffinityPropagationCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr input,
                                  std::vector<pcl::PointIndices>& cluster_indices,
                                  float damping,
                                  int max_iterations,
                                  int convergence_threshold)
{
    // Clear output
    cluster_indices.clear();

    // Validate input
    if (!input || input->empty()) {
        return;
    }

    const size_t n = input->size();

    // Compute similarity matrix (negative squared Euclidean distance)
    std::vector<std::vector<float>> similarity(n, std::vector<float>(n, 0.0f));
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = 0; j < n; ++j) {
            if (i == j) {
                continue; // Diagonal set later as preference
            }
            float dx = input->points[i].x - input->points[j].x;
            float dy = input->points[i].y - input->points[j].y;
            float dz = input->points[i].z - input->points[j].z;
            similarity[i][j] = -(dx * dx + dy * dy + dz * dz);
        }
    }

    // Set preference (diagonal) as median of similarities (self-preference influences cluster count)
    std::vector<float> off_diagonal;
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = 0; j < n; ++j) {
            if (i != j) {
                off_diagonal.push_back(similarity[i][j]);
            }
        }
    }
    std::sort(off_diagonal.begin(), off_diagonal.end());
    float median_preference = off_diagonal[off_diagonal.size() / 2];
    for (size_t i = 0; i < n; ++i) {
        similarity[i][i] = median_preference; // Preference affects number of clusters
    }

    // Initialize responsibility and availability matrices
    std::vector<std::vector<float>> responsibility(n, std::vector<float>(n, 0.0f));
    std::vector<std::vector<float>> availability(n, std::vector<float>(n, 0.0f));

    // Track exemplars and convergence
    std::vector<int> exemplars(n, -1);
    int unchanged_count = 0;

    // Main AP loop
    for (int iter = 0; iter < max_iterations && unchanged_count < convergence_threshold; ++iter) {
        // Update responsibility: r(i,k) = s(i,k) - max_{k'!=k} {a(i,k') + s(i,k')}
        for (size_t i = 0; i < n; ++i) {
            for (size_t k = 0; k < n; ++k) {
                float max_ak_sk = -std::numeric_limits<float>::max();
                for (size_t kp = 0; kp < n; ++kp) {
                    if (kp != k) {
                        float val = availability[i][kp] + similarity[i][kp];
                        max_ak_sk = std::max(max_ak_sk, val);
                    }
                }
                float new_r = similarity[i][k] - max_ak_sk;
                responsibility[i][k] = (1.0f - damping) * new_r + damping * responsibility[i][k];
            }
        }

        // Update availability: a(i,k) reflects how suitable k is as an exemplar for i
        for (size_t i = 0; i < n; ++i) {
            for (size_t k = 0; k < n; ++k) {
                if (i == k) {
                    // Self-availability: a(k,k) = sum_{i'!=k, r(i',k)>0} r(i',k)
                    float sum_r = 0.0f;
                    for (size_t ip = 0; ip < n; ++ip) {
                        if (ip != k) {
                            sum_r += std::max(0.0f, responsibility[ip][k]);
                        }
                    }
                    availability[i][k] = (1.0f - damping) * sum_r + damping * availability[i][k];
                } else {
                    // a(i,k) = min(0, r(k,k) + sum_{i'!=i,k, r(i',k)>0} r(i',k))
                    float sum_r = 0.0f;
                    for (size_t ip = 0; ip < n; ++ip) {
                        if (ip != i && ip != k) {
                            sum_r += std::max(0.0f, responsibility[ip][k]);
                        }
                    }
                    float new_a = std::min(0.0f, responsibility[k][k] + sum_r);
                    availability[i][k] = (1.0f - damping) * new_a + damping * availability[i][k];
                }
            }
        }

        // Check exemplars: point i chooses k where r(i,k) + a(i,k) is maximized
        std::vector<int> new_exemplars(n);
        bool changed = false;
        for (size_t i = 0; i < n; ++i) {
            float max_ra = -std::numeric_limits<float>::max();
            int best_k = -1;
            for (size_t k = 0; k < n; ++k) {
                float ra = responsibility[i][k] + availability[i][k];
                if (ra > max_ra) {
                    max_ra = ra;
                    best_k = k;
                }
            }
            new_exemplars[i] = best_k;
            if (new_exemplars[i] != exemplars[i]) {
                changed = true;
            }
        }

        exemplars = new_exemplars;
        unchanged_count = changed ? 0 : unchanged_count + 1;
    }

    // Extract clusters from exemplars
    std::vector<pcl::PointIndices> temp_clusters(n);
    for (size_t i = 0; i < n; ++i) {
        if (exemplars[i] >= 0) {
            temp_clusters[exemplars[i]].indices.push_back(static_cast<int>(i));
        }
    }
    for (size_t i = 0; i < n; ++i) {
        if (!temp_clusters[i].indices.empty()) {
            cluster_indices.push_back(temp_clusters[i]);
        }
    }
}


/**
 * @brief Performs mean-shift clustering on a point cloud
 * @details Uses mean-shift algorithm to cluster points by iteratively shifting points towards local density maxima
 * 
 * @param input Input point cloud to be clustered
 * @param cluster_indices Output vector containing indices for each cluster
 * @param bandwidth Kernel bandwidth parameter that determines the size of the window used to compute mean shift
 * @param max_iter Maximum number of iterations for mean shift convergence
 * 
 * @note The algorithm works by:
 *       1. For each point, calculating weighted mean of nearby points within bandwidth
 *       2. Shifting the point towards the weighted mean
 *       3. Repeating until convergence or max iterations reached
 *       4. Points that converge to the same mode are grouped into clusters
 */
void pcdMeanShiftCluster(pcl::PointCloud<pcl::PointXYZI>::Ptr input, std::vector<pcl::PointIndices> &cluster_indices, float bandwidth, int max_iter) {
    if (input->empty()) {
        return;
    }

    int n_points = input->points.size();
    std::vector<bool> converged(n_points, false);
    std::vector<pcl::PointXYZI, Eigen::aligned_allocator<pcl::PointXYZI>> shifted_points(input->points);
    
    // For each point
    for (int i = 0; i < n_points; i++) {
        if (converged[i]) continue;
        
        pcl::PointXYZI current_mean = input->points[i];
        pcl::PointXYZI previous_mean;
        
        int iter = 0;
        do {
            previous_mean = current_mean;
            pcl::PointXYZI new_mean;
            new_mean.x = 0;
            new_mean.y = 0; 
            new_mean.z = 0;
            new_mean.intensity = 0;
            float total_weight = 0;
            
            // For each point, calculate weighted mean
            for (int j = 0; j < n_points; j++) {
                float dist = sqrt(
                    pow(previous_mean.x - input->points[j].x, 2) +
                    pow(previous_mean.y - input->points[j].y, 2) +
                    pow(previous_mean.z - input->points[j].z, 2)
                );
                
                if (dist < bandwidth) {
                    float weight = exp(-(dist * dist) / (2 * bandwidth * bandwidth));
                    new_mean.x += weight * input->points[j].x;
                    new_mean.y += weight * input->points[j].y;
                    new_mean.z += weight * input->points[j].z;
                    new_mean.intensity += weight * input->points[j].intensity;
                    total_weight += weight;
                }
            }
            
            if (total_weight > 0) {
                current_mean.x = new_mean.x / total_weight;
                current_mean.y = new_mean.y / total_weight;
                current_mean.z = new_mean.z / total_weight;
                current_mean.intensity = new_mean.intensity / total_weight;
            }
            
            iter++;
        } while (iter < max_iter && 
                sqrt(pow(current_mean.x - previous_mean.x, 2) +
                     pow(current_mean.y - previous_mean.y, 2) +
                     pow(current_mean.z - previous_mean.z, 2)) > 0.001);
        
        shifted_points[i] = current_mean;
        converged[i] = true;
        
        // Mark other points that converged to the same mode
        for (int j = i + 1; j < n_points; j++) {
            if (!converged[j]) {
                float dist = sqrt(
                    pow(current_mean.x - input->points[j].x, 2) +
                    pow(current_mean.y - input->points[j].y, 2) +
                    pow(current_mean.z - input->points[j].z, 2)
                );
                if (dist < bandwidth/2) {
                    shifted_points[j] = current_mean;
                    converged[j] = true;
                }
            }
        }
    }
    
    // Group points with similar shifted positions into clusters
    std::map<std::vector<float>, pcl::PointIndices> clusters;
    for (int i = 0; i < n_points; i++) {
        std::vector<float> rounded_point = {
            round(shifted_points[i].x * 100) / 100,
            round(shifted_points[i].y * 100) / 100,
            round(shifted_points[i].z * 100) / 100
        };
        clusters[rounded_point].indices.push_back(i);
    }
    
    // Convert map to vector of clusters
    cluster_indices.clear();
    for (const auto& cluster : clusters) {
        if (cluster.second.indices.size() > 0) {
            cluster_indices.push_back(cluster.second);
        }
    }
}

/**
 * @brief Performs Ward's hierarchical clustering on a PCL point cloud.
 *
 * This function implements Ward's method, a hierarchical clustering algorithm that
 * minimizes the increase in within-cluster variance when merging clusters. It uses
 * squared Euclidean distance and a KdTree for efficient distance calculations.
 * The process continues until the desired number of clusters is reached.
 *
 * @param[in] input Pointer to the input point cloud (XYZI format).
 * @param[out] cluster_indices Vector of PointIndices containing the resulting clusters.
 * @param[in] num_clusters Desired number of final clusters (must be >= 1 and <= point count).
 * @note This is a simplified implementation; full Ward's method typically builds a dendrogram,
 *       but here we stop at a specified number of clusters.
 * @note Requires a non-empty input cloud; otherwise, cluster_indices will be empty.
 */
void pcdWardCluster(pcl::PointCloud<pcl::PointXYZI>::Ptr input,
                    std::vector<pcl::PointIndices>& cluster_indices,
                    int num_clusters)
{
    // Clear output
    cluster_indices.clear();

    // Validate input
    if (!input || input->empty() || num_clusters < 1 || num_clusters > static_cast<int>(input->size())) {
        return;
    }

    // Special case: if num_clusters equals point count, return singletons
    if (num_clusters == static_cast<int>(input->size())) {
        for (size_t i = 0; i < input->size(); ++i) {
            pcl::PointIndices singleton;
            singleton.indices.push_back(static_cast<int>(i));
            cluster_indices.push_back(singleton);
        }
        return;
    }

    // Structure to represent a cluster
    struct Cluster {
        pcl::PointIndices indices;  // Point indices in this cluster
        pcl::PointXYZI centroid;    // Cluster centroid
        float variance;             // Within-cluster variance
    };

    // Initialize each point as its own cluster
    std::vector<Cluster> clusters(input->size());
    for (size_t i = 0; i < input->size(); ++i) {
        clusters[i].indices.indices.push_back(static_cast<int>(i));
        clusters[i].centroid = input->points[i];
        clusters[i].variance = 0.0f;
    }

    // Use KdTree for efficient nearest-neighbor searches (optional optimization)
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud(input);

    // Merge clusters until we reach num_clusters
    while (clusters.size() > static_cast<size_t>(num_clusters)) {
        float min_variance_increase = std::numeric_limits<float>::max();
        std::pair<size_t, size_t> merge_pair;

        // Find the pair of clusters with the smallest variance increase
        for (size_t i = 0; i < clusters.size(); ++i) {
            for (size_t j = i + 1; j < clusters.size(); ++j) {
                // Compute new centroid if clusters i and j are merged
                pcl::PointXYZI new_centroid;
                int n1 = clusters[i].indices.indices.size();
                int n2 = clusters[j].indices.indices.size();
                int n = n1 + n2;

                new_centroid.x = (clusters[i].centroid.x * n1 + clusters[j].centroid.x * n2) / n;
                new_centroid.y = (clusters[i].centroid.y * n1 + clusters[j].centroid.y * n2) / n;
                new_centroid.z = (clusters[i].centroid.z * n1 + clusters[j].centroid.z * n2) / n;
                new_centroid.intensity = (clusters[i].centroid.intensity * n1 + clusters[j].centroid.intensity * n2) / n;

                // Compute variance increase (Ward's criterion)
                float variance_increase = 0.0f;
                for (int idx : clusters[i].indices.indices) {
                    float dx = input->points[idx].x - new_centroid.x;
                    float dy = input->points[idx].y - new_centroid.y;
                    float dz = input->points[idx].z - new_centroid.z;
                    variance_increase += dx * dx + dy * dy + dz * dz;
                }
                for (int idx : clusters[j].indices.indices) {
                    float dx = input->points[idx].x - new_centroid.x;
                    float dy = input->points[idx].y - new_centroid.y;
                    float dz = input->points[idx].z - new_centroid.z;
                    variance_increase += dx * dx + dy * dy + dz * dz;
                }
                variance_increase *= static_cast<float>(n1 * n2) / n; // Ward's update formula

                if (variance_increase < min_variance_increase) {
                    min_variance_increase = variance_increase;
                    merge_pair = {i, j};
                }
            }
        }

        // Merge the selected clusters
        Cluster new_cluster;
        new_cluster.indices.indices.insert(new_cluster.indices.indices.end(),
                                           clusters[merge_pair.first].indices.indices.begin(),
                                           clusters[merge_pair.first].indices.indices.end());
        new_cluster.indices.indices.insert(new_cluster.indices.indices.end(),
                                           clusters[merge_pair.second].indices.indices.begin(),
                                           clusters[merge_pair.second].indices.indices.end());
        int n1 = clusters[merge_pair.first].indices.indices.size();
        int n2 = clusters[merge_pair.second].indices.indices.size();
        int n = n1 + n2;
        new_cluster.centroid.x = (clusters[merge_pair.first].centroid.x * n1 + clusters[merge_pair.second].centroid.x * n2) / n;
        new_cluster.centroid.y = (clusters[merge_pair.first].centroid.y * n1 + clusters[merge_pair.second].centroid.y * n2) / n;
        new_cluster.centroid.z = (clusters[merge_pair.first].centroid.z * n1 + clusters[merge_pair.second].centroid.z * n2) / n;
        new_cluster.centroid.intensity = (clusters[merge_pair.first].centroid.intensity * n1 + clusters[merge_pair.second].centroid.intensity * n2) / n;
        new_cluster.variance = min_variance_increase;

        // Remove old clusters and add the new one
        clusters.erase(clusters.begin() + std::max(merge_pair.first, merge_pair.second));
        clusters.erase(clusters.begin() + std::min(merge_pair.first, merge_pair.second));
        clusters.push_back(new_cluster);
    }

    // Populate output with final clusters
    for (const auto& cluster : clusters) {
        cluster_indices.push_back(cluster.indices);
    }
}

/**
 * @brief Performs OPTICS clustering on a PCL point cloud.
 *
 * This function implements the OPTICS (Ordering Points To Identify the Clustering Structure)
 * algorithm, a density-based clustering method that orders points based on reachability
 * distances and identifies clusters of varying densities. It uses a KdTree for efficient
 * nearest-neighbor searches and outputs clusters as point indices.
 *
 * @param[in] input Pointer to the input point cloud (XYZI format).
 * @param[out] cluster_indices Vector of PointIndices containing the resulting clusters.
 * @param[in] eps Maximum distance (epsilon) for points to be considered neighbors.
 * @param[in] minPts Minimum number of points required to form a core point.
 * @note This is a simplified OPTICS implementation; full OPTICS typically produces a reachability plot,
 *       but here we extract clusters directly using a basic threshold approach.
 * @note Requires a non-empty input cloud; otherwise, cluster_indices will be empty.
 */
void pcdOPTICSCluster(pcl::PointCloud<pcl::PointXYZI>::Ptr input,
                      std::vector<pcl::PointIndices>& cluster_indices,
                      float eps,
                      int minPts)
{
    // Clear output
    cluster_indices.clear();

    // Validate input
    if (!input || input->empty()) {
        return;
    }

    // Initialize KdTree for nearest-neighbor searches
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud(input);

    // Track processed points and core distances
    std::vector<bool> processed(input->size(), false);
    std::vector<float> core_distances(input->size(), std::numeric_limits<float>::max());
    std::vector<float> reachability_distances(input->size(), std::numeric_limits<float>::max());

    // Helper structure for priority queue (smallest reachability first)
    auto cmp = [](std::pair<int, float> left, std::pair<int, float> right) {
        return left.second > right.second;
    };
    std::set<std::pair<int, float>, decltype(cmp)> seeds(cmp);

    // Process all points
    for (size_t i = 0; i < input->size(); ++i) {
        if (processed[i]) {
            continue;
        }

        // Start a new cluster
        pcl::PointIndices current_cluster;
        processed[i] = true;

        // Get neighbors and compute core distance
        std::vector<int> neighbor_indices;
        std::vector<float> neighbor_distances;
        if (kdtree.radiusSearch(input->points[i], eps, neighbor_indices, neighbor_distances) >= minPts) {
            // Point is a core point
            core_distances[i] = neighbor_distances[minPts - 1]; // Distance to (minPts-1)th neighbor
            current_cluster.indices.push_back(static_cast<int>(i));

            // Initialize seeds with neighbors
            seeds.clear();
            for (size_t j = 0; j < neighbor_indices.size(); ++j) {
                int idx = neighbor_indices[j];
                if (!processed[idx]) {
                    reachability_distances[idx] = std::max(core_distances[i], neighbor_distances[j]);
                    seeds.insert({idx, reachability_distances[idx]});
                }
            }

            // Expand cluster using seeds
            while (!seeds.empty()) {
                // Get point with smallest reachability distance
                int current_idx = seeds.begin()->first;
                float current_reach = seeds.begin()->second;
                seeds.erase(seeds.begin());

                if (processed[current_idx]) {
                    continue;
                }

                processed[current_idx] = true;
                current_cluster.indices.push_back(current_idx);

                // Check if this is a core point
                neighbor_indices.clear();
                neighbor_distances.clear();
                if (kdtree.radiusSearch(input->points[current_idx], eps, neighbor_indices, neighbor_distances) >= minPts) {
                    core_distances[current_idx] = neighbor_distances[minPts - 1];
                    for (size_t j = 0; j < neighbor_indices.size(); ++j) {
                        int idx = neighbor_indices[j];
                        if (!processed[idx]) {
                            float new_reach = std::max(core_distances[current_idx], neighbor_distances[j]);
                            if (new_reach < reachability_distances[idx]) {
                                reachability_distances[idx] = new_reach;
                                seeds.insert({idx, new_reach});
                            }
                        }
                    }
                }
            }
        }

        // Add non-empty clusters to output
        if (!current_cluster.indices.empty()) {
            cluster_indices.push_back(current_cluster);
        }
    }

    // Handle noise points (optional: points not in any cluster)
    pcl::PointIndices noise;
    for (size_t i = 0; i < input->size(); ++i) {
        if (!processed[i]) {
            noise.indices.push_back(static_cast<int>(i));
        }
    }
    if (!noise.indices.empty()) {
        cluster_indices.push_back(noise); // Noise as the last cluster
    }
}

/**
 * @brief Performs BIRCH-like clustering on a PCL point cloud.
 *
 * This function implements a simplified version of the BIRCH clustering algorithm,
 * which hierarchically clusters points based on spatial proximity. It uses a
 * KdTree for efficient nearest-neighbor searches and groups points into clusters
 * based on a branching factor and distance threshold. The output is a vector of
 * point indices representing the clusters.
 *
 * @param[in] input Pointer to the input point cloud (XYZI format).
 * @param[out] cluster_indices Vector of PointIndices containing the resulting clusters.
 * @param[in] branching_factor Maximum number of subclusters in a node (controls granularity).
 * @param[in] max_distance Maximum distance between points to be in the same cluster.
 * @note BIRCH typically uses a CF-tree, but this is a simplified adaptation using PCL tools.
 * @note Requires a non-empty input cloud; otherwise, cluster_indices will be empty.
 */
void pcdBIRCHCluster(pcl::PointCloud<pcl::PointXYZI>::Ptr input,
                     std::vector<pcl::PointIndices>& cluster_indices,
                     int branching_factor,
                     float max_distance)
{
    // Clear output in case it's not empty
    cluster_indices.clear();

    // Check if input is valid
    if (!input || input->empty()) {
        return;
    }

    // Initialize KdTree for efficient nearest-neighbor search
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud(input);

    // Track which points have been clustered
    std::vector<bool> processed(input->size(), false);

    // Iterate through all points to form clusters
    for (size_t i = 0; i < input->size(); ++i) {
        // Skip if point is already clustered
        if (processed[i]) {
            continue;
        }

        // Start a new cluster
        pcl::PointIndices current_cluster;
        current_cluster.indices.push_back(static_cast<int>(i));
        processed[i] = true;

        // Use a queue to grow the cluster (mimicking BIRCH's subtree expansion)
        std::vector<int> queue;
        queue.push_back(i);

        size_t queue_idx = 0;
        while (queue_idx < queue.size() && current_cluster.indices.size() < static_cast<size_t>(branching_factor)) {
            int seed_idx = queue[queue_idx++];

            // Find neighbors within max_distance
            std::vector<int> neighbor_indices;
            std::vector<float> neighbor_distances;
            pcl::PointXYZI seed_point = input->points[seed_idx];
            if (kdtree.radiusSearch(seed_point, max_distance, neighbor_indices, neighbor_distances) > 0) {
                for (size_t j = 0; j < neighbor_indices.size(); ++j) {
                    int neighbor_idx = neighbor_indices[j];
                    if (!processed[neighbor_idx]) {
                        processed[neighbor_idx] = true;
                        current_cluster.indices.push_back(neighbor_idx);
                        queue.push_back(neighbor_idx);

                        // Enforce branching factor limit
                        if (current_cluster.indices.size() >= static_cast<size_t>(branching_factor)) {
                            break;
                        }
                    }
                }
            }
        }

        // Add the cluster to the output if it has points
        if (!current_cluster.indices.empty()) {
            cluster_indices.push_back(current_cluster);
        }
    }
}

/**
 * @brief Represents a tracked object's state
 */
struct KalmanState {
    int id;                     // Unique track ID
    cv::KalmanFilter kf;       // Kalman filter for this track
    bbox lastBBox;             // Last known bounding box
    int age;                   // Number of frames track has existed
    int totalVisibleCount;     // Number of frames track was detected
    int consecutiveInvisibleCount; // Number of consecutive frames without detection
};

/**
 * @brief Calculate IOU between two bounding boxes
 * @param box1 First bounding box
 * @param box2 Second bounding box
 * @return IOU score between 0 and 1
 */
float calculateIOU(const bbox& box1, const bbox& box2) {
    float x_left = std::max(box1.x1, box2.x1);
    float y_top = std::max(box1.y1, box2.y1);
    float x_right = std::min(box1.x2, box2.x2);
    float y_bottom = std::min(box1.y2, box2.y2);

    if (x_right < x_left || y_bottom < y_top)
        return 0.0f;

    float intersection_area = (x_right - x_left) * (y_bottom - y_top);
    float box1_area = (box1.x2 - box1.x1) * (box1.y2 - box1.y1);
    float box2_area = (box2.x2 - box2.x1) * (box2.y2 - box2.y1);
    
    return intersection_area / (box1_area + box2_area - intersection_area);
}

/**
 * @brief Initialize a new Kalman filter tracker
 * @param bbox Initial bounding box
 * @param id Track ID to assign
 * @return Initialized KalmanState
 */
KalmanState initializeTrack(const bbox& box, int id) {
    KalmanState state;
    state.id = id;
    state.lastBBox = box;
    state.age = 1;
    state.totalVisibleCount = 1;
    state.consecutiveInvisibleCount = 0;

    // Initialize Kalman filter with state: [x, y, width, height, dx, dy, dw, dh]
    state.kf.init(8, 4, 0);
    state.kf.transitionMatrix = (cv::Mat_<float>(8, 8) << 
        1,0,0,0,1,0,0,0,
        0,1,0,0,0,1,0,0,
        0,0,1,0,0,0,1,0,
        0,0,0,1,0,0,0,1,
        0,0,0,0,1,0,0,0,
        0,0,0,0,0,1,0,0,
        0,0,0,0,0,0,1,0,
        0,0,0,0,0,0,0,1);

    state.kf.measurementMatrix = cv::Mat::zeros(4, 8, CV_32F);
    state.kf.measurementMatrix.at<float>(0,0) = 1;
    state.kf.measurementMatrix.at<float>(1,1) = 1;
    state.kf.measurementMatrix.at<float>(2,2) = 1;
    state.kf.measurementMatrix.at<float>(3,3) = 1;

    // Initialize state
    state.kf.statePost.at<float>(0) = (box.x1 + box.x2) / 2;
    state.kf.statePost.at<float>(1) = (box.y1 + box.y2) / 2;
    state.kf.statePost.at<float>(2) = box.x2 - box.x1;
    state.kf.statePost.at<float>(3) = box.y2 - box.y1;

    return state;
}

/**
 * @brief Update Kalman filter with new measurement
 * @param state KalmanState to update
 * @param bbox New bounding box measurement
 */
void updateKalmanFilter(KalmanState& state, const bbox& box) {
    cv::Mat measurement = (cv::Mat_<float>(4,1) << 
        (box.x1 + box.x2) / 2,
        (box.y1 + box.y2) / 2,
        box.x2 - box.x1,
        box.y2 - box.y1);

    state.kf.correct(measurement);
    state.lastBBox = box;
    state.totalVisibleCount++;
    state.consecutiveInvisibleCount = 0;
}

/**
 * @brief Predict new state using Kalman filter
 * @param state KalmanState to predict
 * @return Predicted bounding box
 */
bbox predictKalmanFilter(KalmanState& state) {
    cv::Mat prediction = state.kf.predict();
    bbox predicted;
    float width = prediction.at<float>(2);
    float height = prediction.at<float>(3);
    float cx = prediction.at<float>(0);
    float cy = prediction.at<float>(1);
    
    predicted.x1 = cx - width/2;
    predicted.y1 = cy - height/2;
    predicted.x2 = cx + width/2;
    predicted.y2 = cy + height/2;
    
    return predicted;
}

/**
 * @brief Perform Kalman filter tracking on bounding boxes
 * @param input Vector of detected bounding boxes
 * @param trackID Output vector of track IDs corresponding to input boxes
 * @param iouThreshold Minimum IOU for track association (default 0.3)
 * @param maxAge Maximum age of track before deletion (default 10)
 * @param minHits Minimum hits needed to confirm track (default 3)
 */
void kalmanTrack(const std::vector<bbox>& input, std::vector<int>& trackID, 
                 float iouThreshold = 0.3f, int maxAge = 10, int minHits = 3) {
    static std::vector<KalmanState> tracks;
    static int nextTrackID = 0;
    trackID.clear();
    trackID.resize(input.size(), -1);

    // Predict new locations for all tracks
    std::vector<bbox> predictions;
    for (auto& track : tracks) {
        predictions.push_back(predictKalmanFilter(track));
    }

    // Associate detections with tracks using IOU
    std::vector<int> assignment(input.size(), -1);
    std::vector<bool> trackUpdated(tracks.size(), false);

    // Calculate IOU matrix
    for (size_t i = 0; i < input.size(); i++) {
        float maxIOU = iouThreshold;
        int bestTrack = -1;

        for (size_t j = 0; j < tracks.size(); j++) {
            if (trackUpdated[j]) continue;
            
            float iou = calculateIOU(input[i], predictions[j]);
            if (iou > maxIOU) {
                maxIOU = iou;
                bestTrack = j;
            }
        }

        if (bestTrack >= 0) {
            assignment[i] = bestTrack;
            trackUpdated[bestTrack] = true;
        }
    }

    // Update matched tracks
    for (size_t i = 0; i < input.size(); i++) {
        if (assignment[i] >= 0) {
            updateKalmanFilter(tracks[assignment[i]], input[i]);
            trackID[i] = tracks[assignment[i]].id;
        }
    }

    // Create new tracks for unmatched detections
    for (size_t i = 0; i < input.size(); i++) {
        if (assignment[i] < 0) {
            tracks.push_back(initializeTrack(input[i], nextTrackID));
            trackID[i] = nextTrackID++;
        }
    }

    // Update unmatched tracks
    for (size_t i = 0; i < tracks.size(); i++) {
        if (!trackUpdated[i]) {
            tracks[i].consecutiveInvisibleCount++;
        }
        tracks[i].age++;
    }

    // Remove old tracks
    tracks.erase(
        std::remove_if(tracks.begin(), tracks.end(),
            [maxAge, minHits](const KalmanState& track) {
                return (track.age >= maxAge && 
                        track.totalVisibleCount < minHits) ||
                       track.consecutiveInvisibleCount >= maxAge;
            }),
        tracks.end());
}


}