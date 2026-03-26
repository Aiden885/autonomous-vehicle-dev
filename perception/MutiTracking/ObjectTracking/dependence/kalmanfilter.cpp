#include "kalmanfilter.h"
#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <iostream>

const double KalmanFilter::chi2inv95[10] = {0, 3.8415, 5.9915, 7.8147, 9.4877,
                                            11.070, 12.592, 14.067, 15.507, 16.919};
KalmanFilter::KalmanFilter(const double &dt)
{
    int ndim = 2;

    _motion_mat = Eigen::MatrixXf::Identity(4, 4);
    for (int i = 0; i < ndim; i++)
    {
        _motion_mat(i, ndim + i) = dt;
    }
    _update_mat = Eigen::MatrixXf::Identity(2, 4);

    this->_std_weight_position = 1. / 20;
    this->_std_weight_velocity = 1. / 160;
}

KAL_DATA KalmanFilter::initiate(const DETECTBOX &measurement)
{
    DETECTBOX mean_pos = measurement;
    DETECTBOX mean_vel;
    for (int i = 0; i < 2; i++)
        mean_vel(i) = 0;

    KAL_MEAN mean; // x,y,w,vx,vy
    for (int i = 0; i < 4; i++)
    {
        if (i < 2)
            mean(i) = mean_pos(i);
        else
            mean(i) = mean_vel(i - 2);
    }

    KAL_MEAN std;
    std(0) = 2 * _std_weight_position * measurement[3];
    std(1) = 2 * _std_weight_position * measurement[3];
    std(2) = 10 * _std_weight_velocity * measurement[3];
    std(3) = 10 * _std_weight_velocity * measurement[3];

    KAL_MEAN tmp = std.array().square();
    KAL_COVA var = tmp.asDiagonal();
    return std::make_pair(mean, var);
}

void KalmanFilter::predict(KAL_MEAN &mean, KAL_COVA &covariance)
{
    // revise the data;
    Eigen::Matrix<float, 1, 2, Eigen::RowMajor> std_pos;
    std_pos << _std_weight_position * mean(2), _std_weight_position * mean(2);
    Eigen::Matrix<float, 1, 2, Eigen::RowMajor> std_vel;
    std_vel << _std_weight_velocity * mean(2), _std_weight_velocity * mean(2);
    KAL_MEAN tmp;
    tmp.block<1, 2>(0, 0) = std_pos;
    tmp.block<1, 2>(0, 2) = std_vel;
    tmp = tmp.array().square();
    KAL_COVA motion_cov = tmp.asDiagonal();
    KAL_MEAN mean1 = this->_motion_mat * mean.transpose();
    KAL_COVA covariance1 = this->_motion_mat * covariance * (_motion_mat.transpose());
    covariance1 += motion_cov;

    mean = mean1;
    covariance = covariance1;
}

KAL_HDATA KalmanFilter::project(const KAL_MEAN &mean, const KAL_COVA &covariance)
{
    Eigen::Matrix<float, 1, 2, Eigen::RowMajor> std;
    std << _std_weight_position * mean(2), _std_weight_position * mean(2);
    KAL_HMEAN mean1 = _update_mat * mean.transpose();
    KAL_HCOVA covariance1 = _update_mat * covariance * (_update_mat.transpose());
    Eigen::Matrix<float, 2, 2> diag = std.asDiagonal();
    diag = diag.array().square().matrix();
    covariance1 += diag;
    //    covariance1.diagonal() << diag;
    return std::make_pair(mean1, covariance1);
}

KAL_DATA
KalmanFilter::update(const KAL_MEAN &mean, const KAL_COVA &covariance, const DETECTBOX &measurement)
{
    KAL_HDATA pa = project(mean, covariance);
    // std::cout << "covariance " << covariance << std::endl;

    KAL_HMEAN projected_mean = pa.first;
    KAL_HCOVA projected_cov = pa.second;

    // Eigen::JacobiSVD<Eigen::MatrixXf> svd(projected_cov);
    // std::cout << "rank: " << svd.rank() << std::endl;

    // std::cout << projected_cov.rows() << " " << projected_cov.cols() << " " << std::endl;

    // chol_factor, lower =
    // scipy.linalg.cho_factor(projected_cov, lower=True, check_finite=False)
    // kalmain_gain =
    // scipy.linalg.cho_solve((cho_factor, lower),
    // np.dot(covariance, self._upadte_mat.T).T,
    // check_finite=False).T

    Eigen::Matrix<float, 2, 4> B = (covariance * (_update_mat.transpose())).transpose();
    Eigen::Matrix<float, 4, 2> kalman_gain = (projected_cov.llt().solve(B)).transpose();

    // Eigen::Matrix<float, 5, 3> B = (covariance * (_update_mat.transpose()));
    // Eigen::Matrix<float, 5, 3> kalman_gain = B * (_update_mat * covariance * _update_mat.transpose()).inverse();

    // covariance *_update_mat.transpose();

    Eigen::Matrix<float, 1, 2> innovation = measurement.block<1, 2>(0, 0) - projected_mean;

    auto tmp = innovation * (kalman_gain.transpose());

    // auto tmp = innovation * (kalman_gain.transpose());

    KAL_MEAN new_mean = (mean.array() + tmp.array()).matrix();

    // new_mean(2) = measurement(2); //需要试试

    KAL_COVA new_covariance = covariance - kalman_gain * projected_cov * (kalman_gain.transpose());

    return std::make_pair(new_mean, new_covariance);
}

// Eigen::Matrix<float, 1, -1> KalmanFilter::gating_distance(const KAL_MEAN &mean, const KAL_COVA &covariance,
//                                                           const std::vector<DETECTBOX> &measurements,
//                                                           bool only_position)
// {
//     KAL_HDATA pa = this->project(mean, covariance);

//     KAL_HMEAN mean1 = pa.first;
//     KAL_HCOVA covariance1 = pa.second;

//     DETECTBOXSS d(measurements.size(), 4);
//     int pos = 0;
//     for (DETECTBOX box : measurements)
//     {
//         d.row(pos++) = box - mean1;
//     }
//     Eigen::Matrix<float, -1, -1, Eigen::RowMajor> factor = covariance1.llt().matrixL();
//     Eigen::Matrix<float, -1, -1> z =
//         factor.triangularView<Eigen::Lower>().solve<Eigen::OnTheRight>(d).transpose();
//     auto zz = ((z.array()) * (z.array())).matrix();
//     auto square_maha = zz.colwise().sum();
//     return square_maha;
// }
