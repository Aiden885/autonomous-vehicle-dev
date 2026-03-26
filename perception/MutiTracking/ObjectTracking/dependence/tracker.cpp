#include "tracker.h"
#include "linear_assignment.h"
#include "nn_matching.h"
#include <iostream>
using namespace std;

//#define MY_inner_DEBUG
#ifdef MY_inner_DEBUG
#include <iostream>
#include <string>
#endif

tracker::tracker(float max_euclidean_distance, float max_iou_distance, int max_age, int n_init, float dt)
{
    _init(max_euclidean_distance, max_iou_distance, max_age, n_init, dt);
}

void tracker::_init(float max_euclidean_distance, float max_iou_distance, int max_age, int n_init, float dt,
                    const std::vector<std::string> &classes)
{
    this->metric = std::shared_ptr<NearNeighborDisMetric>(
        new NearNeighborDisMetric(NearNeighborDisMetric::METRIC_TYPE::euclidean, max_euclidean_distance));
    this->max_iou_distance = max_iou_distance;
    this->max_age = max_age;
    this->n_init = n_init;
    this->dt = dt;

    this->kf = std::shared_ptr<KalmanFilter>(new KalmanFilter(double(dt)));
    this->tracks.clear();
    this->_next_idx = 1;
    this->detection_classes = classes;
}

void tracker::predict()
{
    for (Track &track : tracks)
    {
        // std::cout << "before predict  " << track.mean << std::endl;
        track.predit(kf);
        // std::cout << "after predict  " << track.mean << std::endl;
    }
}

void tracker::update(const DETECTIONS &detections)
{
    TRACHER_MATCHD res;
    cout << "in update "
         << "0000" << endl;
    _match(detections, res);
    cout << "in update "
         << "1111" << endl;

    vector<MATCH_DATA> &matches = res.matches;

    cout << "in update "
         << "7777  " << matches.size() << endl;

    for (MATCH_DATA &data : matches)
    {
        int track_idx = data.first;
        int detection_idx = data.second;
        // cout << track_idx << "    "
        //      << detection_idx << endl;
        tracks[track_idx].update(this->kf, detections[detection_idx]);
        // cout << "in update "
        //      << "8888" << endl;
    }

    cout << "in update "
         << "2222" << endl;
    vector<int> &unmatched_tracks = res.unmatched_tracks;

    for (int &track_idx : unmatched_tracks)
    {
        this->tracks[track_idx].mark_missed();
    }
    vector<int> &unmatched_detections = res.unmatched_detections;

    for (int &detection_idx : unmatched_detections)
    {
        this->_initiate_track(detections[detection_idx]);
    }

    cout << "in update "
         << "3333" << endl;

    vector<Track>::iterator it;
    for (it = tracks.begin(); it != tracks.end();)
    {
        if ((*it).is_deleted())
            it = tracks.erase(it);
        else
            ++it;
    }

    cout << "in update "
         << "44444" << endl;
}

void tracker::_match(const DETECTIONS &detections, TRACHER_MATCHD &res)
{
    vector<int> confirmed_tracks;
    vector<int> unconfirmed_tracks;
    int idx = 0;
    cout << "in match "
         << "0000           " << tracks.size() << endl;
    for (Track &t : tracks)
    {
        cout << "in tracks mean      "
             << t.mean << endl;
        if (t.is_confirmed())
            confirmed_tracks.push_back(idx);
        else
            unconfirmed_tracks.push_back(idx);
        idx++;
    }

    cout << "in match "
         << "1111              " << unconfirmed_tracks.size() << endl;

    TRACHER_MATCHD matcha = linear_assignment::getInstance()->matching_cascade(
        this, &tracker::gated_matric, this->metric->mating_threshold, this->max_age, this->tracks, detections,
        confirmed_tracks);
    cout << "in match "
         << "----------------------" << endl;
    vector<int> second_round_track_candidates;
    second_round_track_candidates.assign(unconfirmed_tracks.begin(), unconfirmed_tracks.end());

    cout << "in match "
         << "2222" << endl;

    vector<int>::iterator it;
    for (it = matcha.unmatched_tracks.begin(); it != matcha.unmatched_tracks.end();)
    {
        int idx = *it;
        if (tracks[idx].time_since_update == 1) // push into unconfirmed
        {
            second_round_track_candidates.push_back(idx);
            it = matcha.unmatched_tracks.erase(it);
            continue;
        }
        ++it;
    }

    cout << "in match "
         << "666666" << endl;

    TRACHER_MATCHD matchb = linear_assignment::getInstance()->min_cost_matching(
        this, &tracker::gated_matric, this->metric->mating_threshold, this->tracks, detections,
        second_round_track_candidates, matcha.unmatched_detections);

    cout << "in match "
         << "3333" << endl;

    // for (size_t i = 0; i < matcha.matches.size(); i++)
    // {
    //     std::cout << "matcha.matches[i].second   " << matcha.matches[i].second << std::endl;
    // }

    std::cout << "---------------------------------------------------------------------------------------------" << std::endl;

    // for (size_t i = 0; i < matchb.matches.size(); i++)
    // {
    //     // std::cout << "matchb.matches[i].second   " << matchb.matches[i].second << std::endl;
    // }

    // get result:
    res.matches.assign(matcha.matches.begin(), matcha.matches.end());
    res.matches.insert(res.matches.end(), matchb.matches.begin(), matchb.matches.end());
    // unmatched_tracks;
    res.unmatched_tracks.assign(matcha.unmatched_tracks.begin(), matcha.unmatched_tracks.end());
    res.unmatched_tracks.insert(res.unmatched_tracks.end(), matchb.unmatched_tracks.begin(),
                                matchb.unmatched_tracks.end());
    res.unmatched_detections.assign(matchb.unmatched_detections.begin(), matchb.unmatched_detections.end());
    cout << "in match "
         << "44444      " << res.matches.size() << endl;
}

void tracker::_initiate_track(const DETECTION_ROW &detection)
{
    KAL_DATA data = kf->initiate(detection.to_xyah());
    KAL_MEAN mean = data.first;
    KAL_COVA covariance = data.second;
    std::string detection_class = "";

    if (detection_classes.size() != 0)
    {
        int size = int(detection_classes.size());
        if (detection.class_num < size)
        {
            detection_class = detection_classes[detection.class_num];
        }
    }

    this->tracks.push_back(
        Track(mean, covariance, this->_next_idx, this->n_init, this->max_age, detection_class));
    _next_idx += 1;
}

DYNAMICM tracker::gated_matric(std::vector<Track> &tracks, const DETECTIONS &dets,
                               const std::vector<int> &track_indices,
                               const std::vector<int> &detection_indices)
{
    POSITION det_xy(detection_indices.size(), 2), track_xy(track_indices.size(), 2);
    int pos = 0;
    for (int i : detection_indices)
    {
        det_xy.row(pos++) = dets[i].xywh.block<1, 2>(0, 0);
    }
    pos = 0;
    std::cout << "in  gated_matric " << std::endl;
    for (int i : track_indices)
    {
        track_xy.row(pos++) = tracks[i].mean.block<1, 2>(0, 0);
        std::cout << tracks[i].mean << std::endl;
    }

    DYNAMICM cost_matrix = this->metric->distance(det_xy, track_xy);
    // DYNAMICM res = linear_assignment::getInstance()->gate_cost_matrix(this->kf, cost_matrix, tracks, dets,
    //   track_indices, detection_indices);

    return cost_matrix;
}

// DYNAMICM
// tracker::iou_cost(std::vector<Track> &tracks, const DETECTIONS &dets, const std::vector<int>
// &track_indices,
//                   const std::vector<int> &detection_indices)
// {
//     int rows = track_indices.size();
//     int cols = detection_indices.size();
//     DYNAMICM cost_matrix = Eigen::MatrixXf::Zero(rows, cols);
//     for (int i = 0; i < rows; i++)
//     {
//         int track_idx = track_indices[i];
//         if (tracks[track_idx].time_since_update > 1)
//         {
//             cost_matrix.row(i) = Eigen::RowVectorXf::Constant(cols, INFTY_COST);
//             continue;
//         }
//         DETECTBOX bbox = tracks[track_idx].to_tlwh();
//         int csize = detection_indices.size();
//         DETECTBOXSS candidates(csize, 4);
//         for (int k = 0; k < csize; k++)
//             candidates.row(k) = dets[detection_indices[k]].xywh;
//         Eigen::RowVectorXf rowV = (1. - iou(bbox, candidates).array()).matrix().transpose();
//         cost_matrix.row(i) = rowV;
//     }
//     return cost_matrix;
// }

// Eigen::VectorXf tracker::iou(DETECTBOX &bbox, DETECTBOXSS &candidates)
// {
//     float bbox_tl_1 = bbox[0];
//     float bbox_tl_2 = bbox[1];
//     float bbox_br_1 = bbox[0] + bbox[2];
//     float bbox_br_2 = bbox[1] + bbox[3];
//     float area_bbox = bbox[2] * bbox[3];

//     Eigen::Matrix<float, -1, 2> candidates_tl;
//     Eigen::Matrix<float, -1, 2> candidates_br;
//     candidates_tl = candidates.leftCols(2);
//     candidates_br = candidates.rightCols(2) + candidates_tl;

//     int size = int(candidates.rows());
//     //    Eigen::VectorXf area_intersection(size);
//     //    Eigen::VectorXf area_candidates(size);
//     Eigen::VectorXf res(size);
//     for (int i = 0; i < size; i++)
//     {
//         float tl_1 = std::max(bbox_tl_1, candidates_tl(i, 0));
//         float tl_2 = std::max(bbox_tl_2, candidates_tl(i, 1));
//         float br_1 = std::min(bbox_br_1, candidates_br(i, 0));
//         float br_2 = std::min(bbox_br_2, candidates_br(i, 1));

//         float w = br_1 - tl_1;
//         w = (w < 0 ? 0 : w);
//         float h = br_2 - tl_2;
//         h = (h < 0 ? 0 : h);
//         float area_intersection = w * h;
//         float area_candidates = candidates(i, 2) * candidates(i, 3);
//         res[i] = area_intersection / (area_bbox + area_candidates - area_intersection);
//     }
//     //#ifdef MY_inner_DEBUG
//     //        std::cout << res << std::endl;
//     //#endif
//     return res;
// }
