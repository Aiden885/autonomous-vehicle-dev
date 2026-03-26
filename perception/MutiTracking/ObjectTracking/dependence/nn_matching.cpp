#include "nn_matching.h"
#include "errmsg.h"
#include <iostream>

using namespace Eigen;

NearNeighborDisMetric::NearNeighborDisMetric(NearNeighborDisMetric::METRIC_TYPE metric,
                                             float matching_threshold)
{
    if (metric == euclidean)
    {
        _metric = &NearNeighborDisMetric::_nneuclidean_distance;
    }
    else if (metric == cosine)
    {
        _metric = &NearNeighborDisMetric::_nncosine_distance;
    }
    else
    {
        errMsg::getInstance()->out("nn_matching.cpp",
                                   "NearestNeighborDistanceMetric::NearestNeighborDistanceMetric",
                                   "Invalid metric; must be either 'euclidean' or 'cosine'", true);
    }
    this->mating_threshold = matching_threshold;
}

DYNAMICM
NearNeighborDisMetric::distance(const POSITION &dets_pos, const POSITION &tracks_pos)
{
    // DYNAMICM cost_matrix = Eigen::MatrixXf::Zero(dets_pos.size(), tracks_pos.size());

    DYNAMICM cost_matrix = (this->*_metric)(dets_pos, tracks_pos);

    return cost_matrix;
}

// Eigen::VectorXf NearNeighborDisMetric::_nncosine_distance(const POSITION &dets_pos,
//                                                           const POSITION &tracks_pos)
// {
//     MatrixXf distances = _cosine_distance(x, y);
//     VectorXf res = distances.colwise().minCoeff().transpose();
//     return res;
// }

DYNAMICM NearNeighborDisMetric::_nneuclidean_distance(const POSITION &dets_pos, const POSITION &tracks_pos)
{
    // std::cout << "_nneuclidean_distance   dets_pos \n"
    //           << dets_pos << std::endl
    //           << "--------------------    " << std::endl
    //           << tracks_pos << std::endl;

    DYNAMICM tracks_x_broad = tracks_pos.col(0).replicate(1, dets_pos.rows());
    DYNAMICM tracks_y_broad = tracks_pos.col(1).replicate(1, dets_pos.rows());
    DYNAMICM dets_x_broad = dets_pos.col(0).transpose().replicate(tracks_pos.rows(), 1);
    DYNAMICM dets_y_broad = dets_pos.col(1).transpose().replicate(tracks_pos.rows(), 1);
    DYNAMICM delta_x = dets_x_broad - tracks_x_broad;
    DYNAMICM delta_y = dets_y_broad - tracks_y_broad;
    DYNAMICM res = delta_x.array().square() + delta_y.array().square();
    // std::cout << "_nneuclidean_distance    res " << res.rows() << "    " << res.cols() << std::endl;

    // std::cout << "_nneuclidean_distance before  " << std::endl
    //           << res << std::endl;

    res = res.cwiseSqrt();

    // std::cout << "_nneuclidean_distance after   " << std::endl
    //           << res << std::endl;

    return res;
}

DYNAMICM NearNeighborDisMetric::_nncosine_distance(const POSITION &dets_pos, const POSITION &tracks_pos)
{
    DYNAMICM res;
    return res;
}

// Eigen::MatrixXf NearNeighborDisMetric::_pdist(const FEATURESS &x, const FEATURESS &y)
// {
//     int len1 = x.rows(), len2 = y.rows();
//     if (len1 == 0 || len2 == 0)
//     {
//         return Eigen::MatrixXf::Zero(len1, len2);
//     }
//     MatrixXf res = x * y.transpose() * -2;
//     res = res.colwise() + x.rowwise().squaredNorm();
//     res = res.rowwise() + y.rowwise().squaredNorm().transpose();
//     res = res.array().max(MatrixXf::Zero(res.rows(), res.cols()).array());
//     return res;
// }

// Eigen::MatrixXf NearNeighborDisMetric::_cosine_distance(const FEATURESS &a, const FEATURESS &b,
//                                                         bool data_is_normalized)
// {
//     if (data_is_normalized == true)
//     {
//         // undo:
//         assert(false);
//     }
//     MatrixXf res = 1. - (a * b.transpose()).array();
//     return res;
// }
