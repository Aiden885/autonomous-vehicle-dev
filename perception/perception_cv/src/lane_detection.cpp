#include "lane_detection.hpp"

/**
 * @brief Checks if two lines are similar based on their angle and distance parameters
 * @param l1 First line represented as pair of (angle, distance)
 * @param l2 Second line represented as pair of (angle, distance) 
 * @return True if lines are similar, false otherwise
 * @details Lines are considered similar if their angle difference is <= 20 degrees
 *          or if their angle difference from 180 degrees is <= 20 degrees
 */
bool is_similar(pair<int, int> &l1, pair<int, int> &l2)
{
    /*double theta1 = l1.first * PI / 180;
    double theta2 = l2.first * PI / 180;
    double delta_k = abs(tan(theta1) - tan(theta2));
    double delta_b = abs(l1.second * (tan(theta1) * sin(theta1) + cos(theta1)) -
                         l2.second * (tan(theta2) * sin(theta2) + cos(theta2)));*/
    int delta_angle = abs(l1.first - l2.first);
    int delta_r = abs(l1.second - l2.second);
    // 一般相似直线
    if (delta_angle <= 20 || abs(delta_angle - 180) <= 20)
        return true;
    // if (delta_angle <= 20 && delta_r <= 70)
    //     return true;
    return false;
}

/**
 * @brief Updates a cluster with a new line by updating size and center point
 * @param line New line to add to cluster
 * @param cluster Cluster to update, represented as pair of (center point, size)
 * @details Updates cluster size and recalculates center point as weighted average
 */
void update_cluster(pair<int, int> &line, pair<pair<int, int>, int> &cluster)
{
    // 更新簇大小
    cluster.second++;
    // 更新中心点
    cluster.first.first =
            (cluster.first.first * (cluster.second - 1) + line.first) / cluster.second;
    cluster.first.second =
            (cluster.first.second * (cluster.second - 1) + line.second) / cluster.second;
}

/**
 * @brief Comparison function for sorting clusters by size
 * @param g1 First cluster
 * @param g2 Second cluster
 * @return True if first cluster is larger than second cluster
 */
bool cmp(const pair<pair<int, int>, int> &g1, const pair<pair<int, int>, int> &g2)
{
    return g1.second > g2.second;
}

/**
 * @brief Clusters similar lines together
 * @param lines Vector of lines to cluster, each line represented as pair of (angle, distance)
 * @details Groups similar lines into clusters and updates the input vector with cluster centers.
 *          Lines are considered similar based on is_similar() function.
 *          Merges clusters that are similar to each other.
 *          Sorts clusters by size and keeps only the largest clusters.
 */
void lines_cluster(vector<pair<int, int>> &lines)
{
    if (lines.empty())
        return;
    // 检测结果聚类
    vector<pair<pair<int, int>, int> > clusters;
    clusters.emplace_back(lines[0], 1);

    // 是否创建新的簇
    bool flag;

    // 聚类
    for (int i = 1; i < lines.size(); ++i)
    {
        flag = true;
        for (auto &cluster : clusters)
        {
            // 当前数据点与某簇的中心点相似，则将其添加到簇中
            if (is_similar(lines[i], cluster.first))
            {
                // 更新簇
                // update_cluster(lines[i], cluster);
                cluster.second++;
                flag = false;
                break;
            }
        }
        if (flag)   // 创建新的簇
            clusters.emplace_back(lines[i], 1);
    }


    // 合并相似参数
    for (int i = 0; i < clusters.size(); ++i)
    {
        if (clusters[i].second < 0)
        {
            continue;
        }
        for (int j = i + 1; j < clusters.size(); ++j)
        {
            if (clusters[j].second < 0)
            {
                continue;
            }
            if (is_similar(clusters[i].first, clusters[j].first))
            {
                clusters[i].first.first = (clusters[i].first.first * clusters[i].second +
                                           clusters[j].first.first * clusters[j].second) /
                                          (clusters[i].second + clusters[j].second);
                clusters[i].first.second = (clusters[i].first.second * clusters[i].second +
                                            clusters[j].first.second * clusters[j].second) /
                                           (clusters[i].second + clusters[j].second);
                clusters[i].second += clusters[j].second;
                clusters[j].second = -1;
            }
        }
    }
    auto iter = clusters.begin();
    while (iter != clusters.end())
    {
        if ((*iter).second < 0)
            iter = clusters.erase(iter);
        else
            iter++;
    }

    // 按照簇的大小进行排序
    sort(clusters.begin(), clusters.end(), cmp);

    // 打印簇
    // cout << endl << "num of clusters: " << clusters.size() << " " << endl;
    // for (const auto &cluster : clusters)
    // {
    //     cout << "num: " << cluster.second;
    //     cout << '(' << cluster.first.first << ", " << cluster.first.second << ")  ";
    //     cout << endl;
    // }
    // cout << endl;

    lines.clear();

    // 取最大的前四个簇的中心点（均值点）
    for (int i = 0; i < clusters.size(); ++i)
    {
        lines.push_back(clusters[i].first);
        cout << "final center " << lines[i].first << " " << lines[i].second << endl;
    }
    cout << endl;
}

/**
 * @brief Performs Hough Transform to detect lines in an image
 * @param src Input binary image
 * @param lines Output vector of detected lines, each line represented as pair of (angle, distance)
 * @param threshold Minimum number of votes required for line detection
 * @details Implements standard Hough Transform algorithm:
 *          1. Creates parameter space (theta-r) accumulator array
 *          2. For each edge point, votes for all possible lines through that point
 *          3. Finds peaks in accumulator array above threshold
 *          4. Filters lines based on angle constraints
 *          5. Clusters similar lines together
 */
void HoughTransform(cv::Mat &src, vector<pair<int, int>> &lines, int threshold)
{
    // 参数空间的计数矩阵
    int **count;

    // 计数器初始化
    int rows = src.rows;
    int cols = src.cols;
    int r_max = 2 * (int) sqrt(rows * rows + cols * cols) + 1;
    count = new int *[181];
    for (int i = 0; i < 181; ++i)
    {
        count[i] = new int[r_max];
        memset(count[i], 0, r_max * sizeof(int));
    }

    // 参数空间变量
    int theta, r;

    // 遍历图像为每组参数投票
    for (int row = 0; row < rows - 2; ++row)
    {
        for (int col = 2; col < cols - 2; ++col)
        {
            // 对边缘点进行统计
            if (src.at<char>(row, col) == 255)
            {
                for (theta = 0; theta < 181; ++theta)
                {
                    r = int(row * sin(theta * PI / 180.0) + col * cos(theta * PI / 180.0) + r_max / 2.0);
                    count[theta][r]++;
                }
            }
        }
    }

    // 遍历计数矩阵，选出超出阈值的参数
    for (theta = 0; theta < 181; ++theta)
    {
        for (r = 0; r < r_max; ++r)
        {
            if (count[theta][r] >= threshold && abs(theta - 90) >= 15 && abs(theta) > 10 && 180 - theta > 10)
                // if (count[theta][r] >= threshold)
            {
                if (theta > 90 && (r - r_max / 2) < 0)
                    lines.emplace_back(theta - 180, r_max / 2 - r);
                else
                    lines.emplace_back(theta, r - r_max / 2);
            }
        }
    }

    // 直线参数聚类
    lines_cluster(lines);

    for (int i = 0; i < 181; ++i)
    {
        delete[] count[i];
    }
    delete[] count;
}