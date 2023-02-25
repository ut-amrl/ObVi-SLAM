//
// Created by amanda on 2/20/23.
//

#ifndef UT_VSLAM_TRAJECTORY_METRICS_H
#define UT_VSLAM_TRAJECTORY_METRICS_H

#include <types/timestamped_data_to_frames_utils.h>

#include <vector>

namespace vslam_types_refactor {

struct ATEResults {
  ATEResults() = default;
  ATEResults(const double &rmse_transl_err,
             const double &rmse_rot_err,
             const int &valid_poses_used_in_score,
             const int &lost_poses)
      : rmse_transl_err_(rmse_transl_err),
        rmse_rot_err_(rmse_rot_err),
        valid_poses_used_in_score_(valid_poses_used_in_score),
        lost_poses_(lost_poses){};
  double rmse_transl_err_;
  double rmse_rot_err_;
  int valid_poses_used_in_score_;
  int lost_poses_;
};

struct TrajectoryMetrics {
  std::unordered_map<WaypointId, std::pair<std::vector<double>, std::vector<double>>>
      waypoint_deviations_;
  std::vector<double> all_translation_deviations_;
  std::vector<double> all_rotation_deviations_;
  ATEResults ate_results_;
};

struct FullSequenceMetrics {
  std::vector<TrajectoryMetrics> indiv_trajectory_metrics_;
  TrajectoryMetrics sequence_metrics_;
};

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_TRAJECTORY_METRICS_H
