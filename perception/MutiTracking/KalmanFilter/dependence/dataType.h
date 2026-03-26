#ifndef _DATATYPE_H_
#define _DATATYPE_H_

#include <Eigen/Core>
#include <cstddef>
#include <vector>

typedef Eigen::Matrix<float, 1, 4, Eigen::RowMajor> DETECTBOX;
typedef Eigen::Matrix<float, -1, 4, Eigen::RowMajor> DETECTBOXSS;

typedef Eigen::Matrix<float, -1, 2, Eigen::RowMajor> POSITION;

// Kalmanfilter
// typedef Eigen::Matrix<float, 8, 8, Eigen::RowMajor> KAL_FILTER;
typedef Eigen::Matrix<float, 1, 4, Eigen::RowMajor> KAL_MEAN;
typedef Eigen::Matrix<float, 4, 4, Eigen::RowMajor> KAL_COVA;
typedef Eigen::Matrix<float, 1, 2, Eigen::RowMajor> KAL_HMEAN;
typedef Eigen::Matrix<float, 2, 2, Eigen::RowMajor> KAL_HCOVA;
using KAL_DATA = std::pair<KAL_MEAN, KAL_COVA>;
using KAL_HDATA = std::pair<KAL_HMEAN, KAL_HCOVA>;

// tracker:
using MATCH_DATA = std::pair<int, int>;
typedef struct t
{
    std::vector<MATCH_DATA> matches;
    std::vector<int> unmatched_tracks;
    std::vector<int> unmatched_detections;
} TRACHER_MATCHD;

// linear_assignment:
typedef Eigen::Matrix<float, -1, -1, Eigen::RowMajor> DYNAMICM;

#endif // DATATYPE_H
