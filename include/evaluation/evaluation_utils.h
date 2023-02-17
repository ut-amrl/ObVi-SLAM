//
// Created by amanda on 2/17/23.
//

#ifndef UT_VSLAM_EVALUATION_UTILS_H
#define UT_VSLAM_EVALUATION_UTILS_H

#include <base_lib/basic_utils.h>
#include <types/timestamped_data_to_frames_utils.h>
#include <refactoring/types/vslam_basic_types_refactor.h>

#include <utility>
#include <vector>

namespace vslam_types_refactor {

struct SingleTrajectoryATEResults {
  SingleTrajectoryATEResults(const double &rmse_transl_err,
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

struct TrajectorySequenceATEResults {
  TrajectorySequenceATEResults(const double &rmse_transl_err,
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

struct RawWaypointConsistencyResults {
  std::unordered_map<WaypointId, std::vector<double>>
      centroid_deviations_by_waypoint_;
  std::unordered_map<WaypointId, std::vector<double>>
      orientation_deviations_by_waypoint_;
};

TrajectorySequenceATEResults combineSingleTrajectoryResults(
    const std::vector<SingleTrajectoryATEResults> &single_traj_results);

// TODO do we need to find transform that minimizes dist first or should we
// assume they're aligned
SingleTrajectoryATEResults
generateATEforRotAndTranslForSyncedAlignedTrajectories(
    const std::vector<Pose3D<double>> &est_traj,
    const std::vector<Pose3D<double>> &gt_traj);

RawWaypointConsistencyResults computeWaypointConsistencyResults(
    const std::vector<std::vector<WaypointInfo>> &waypoints_by_trajectory,
    const std::vector<util::BoostHashMap<pose::Timestamp, FrameId>>
        &poses_by_timestamp_by_trajectory,
    const std::vector<std::unordered_map<FrameId, Pose3D<double>>>
        &poses_by_frame_by_trajectory);

RawWaypointConsistencyResults
computeWaypointConsistencyResultsForFrameAssociatedWaypoints(
    const std::vector<std::vector<AssociatedWaypointInfo>> &waypoints_by_trajectory,
    const std::vector<std::unordered_map<FrameId, Pose3D<double>>>
        &poses_by_frame_by_trajectory);

Pose3D<double> getMeanPose(const std::vector<Pose3D<double>> &poses);

void getDeviationFromMeanPose(const Pose3D<double> &mean_pose,
                              const Pose3D<double> &compare_pose,
                              double &transl_deviation,
                              double &rot_deviation);

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_EVALUATION_UTILS_H
