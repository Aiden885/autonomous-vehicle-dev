#ifndef NN_MATCHING_H
#define NN_MATCHING_H

#include "dataType.h"

// A tool to calculate distance;
class NearNeighborDisMetric
{
  public:
    enum METRIC_TYPE
    {
        euclidean = 1,
        cosine
    };
    NearNeighborDisMetric(METRIC_TYPE metric, float matching_threshold);
    DYNAMICM distance(const POSITION &dets_pos, const POSITION &tracks_pos);
    float mating_threshold;

  private:
    typedef DYNAMICM (NearNeighborDisMetric::*PTRFUN)(const POSITION &dets_pos, const POSITION &tracks_pos);
    DYNAMICM _nncosine_distance(const POSITION &dets_pos, const POSITION &tracks_pos);
    DYNAMICM _nneuclidean_distance(const POSITION &dets_pos, const POSITION &tracks_pos);

    // Eigen::MatrixXf _pdist(const FEATURESS &x, const FEATURESS &y);
    // Eigen::MatrixXf _cosine_distance(const FEATURESS &a, const FEATURESS &b, bool data_is_normalized =
    // false);

  private:
    PTRFUN _metric;
};

#endif // NN_MATCHING_H
