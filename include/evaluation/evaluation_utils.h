//
// Created by amanda on 2/17/23.
//

#ifndef UT_VSLAM_EVALUATION_UTILS_H
#define UT_VSLAM_EVALUATION_UTILS_H

#include <base_lib/basic_utils.h>
#include <types/timestamped_data_to_frames_utils.h>
#include <refactoring/types/vslam_basic_types_refactor.h>
#include <evaluation/trajectory_metrics.h>
#include <utility>
#include <vector>

namespace vslam_types_refactor {



// TODO want to move this to metrics class?
struct RawWaypointConsistencyResults {
  std::unordered_map<WaypointId, std::vector<double>>
      centroid_deviations_by_waypoint_;
  std::unordered_map<WaypointId, std::vector<double>>
      orientation_deviations_by_waypoint_;
};

ATEResults combineSingleTrajectoryResults(
    const std::vector<ATEResults> &single_traj_results);

// TODO do we need to find transform that minimizes dist first or should we
// assume they're aligned
ATEResults
generateATEforRotAndTranslForSyncedAlignedTrajectories(
    const std::vector<std::optional<Pose3D<double>>> &est_traj,
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
