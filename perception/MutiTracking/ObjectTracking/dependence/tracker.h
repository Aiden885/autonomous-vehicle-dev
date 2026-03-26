#ifndef TRACKER_H
#define TRACKER_H
#include <future>
#include <vector>

#include "kalmanfilter.h"
#include "track.h"
#include "utility.h"

class NearNeighborDisMetric;

class tracker
{
  public:
    std::shared_ptr<NearNeighborDisMetric> metric;
    float max_iou_distance;
    int max_age;
    int n_init;
    std::shared_ptr<KalmanFilter> kf;
    int _next_idx;
    float dt;

  private:
    std::vector<std::string> detection_classes;

  public:
    std::vector<Track> tracks;
    tracker(float max_euclidean_distance = 1.5, float max_iou_distance = 0.9, int max_age = 15,
            int n_init = 5, float dt = 0.1);
    void _init(float max_euclidean_distance = 1.5, float max_iou_distance = 0.9, int max_age = 15,
               int n_init = 5, float dt = 0.1,
               const std::vector<std::string> &classes = {"pedestrian", "vehicle", "cyclist", "others"});
    void predict();
    void update(const DETECTIONS &detections);
    typedef DYNAMICM (tracker::*GATED_METRIC_FUNC)(std::vector<Track> &tracks, const DETECTIONS &dets,
                                                   const std::vector<int> &track_indices,
                                                   const std::vector<int> &detection_indices);

  private:
    void _match(const DETECTIONS &detections, TRACHER_MATCHD &res);
    void _initiate_track(const DETECTION_ROW &detection);
    

  public:
    DYNAMICM gated_matric(std::vector<Track> &tracks, const DETECTIONS &dets,
                          const std::vector<int> &track_indices, const std::vector<int> &detection_indices);
    // DYNAMICM iou_cost(std::vector<Track> &tracks, const DETECTIONS &dets,
    //                   const std::vector<int> &track_indices, const std::vector<int> &detection_indices);
    // Eigen::VectorXf iou(DETECTBOX &bbox, DETECTBOXSS &candidates);
};

#endif // TRACKER_H
