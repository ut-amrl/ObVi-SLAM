//
// Created by amanda on 2/17/23.
//

#ifndef UT_VSLAM_EVALUATION_UTILS_H
#define UT_VSLAM_EVALUATION_UTILS_H

#include <base_lib/basic_utils.h>
#include <evaluation/trajectory_metrics.h>
#include <refactoring/types/vslam_basic_types_refactor.h>
#include <refactoring/visualization/ros_visualization.h>
#include <types/timestamped_data_to_frames_utils.h>

#include <utility>
#include <vector>

namespace vslam_types_refactor {

// TODO want to move this to metrics class?
struct RawWaypointConsistencyResults {
  std::unordered_map<WaypointId, std::vector<std::vector<double>>>
      centroid_deviations_by_waypoint_by_trajectory_;
  std::unordered_map<WaypointId, std::vector<std::vector<double>>>
      orientation_deviations_by_waypoint_by_trajectory_;
  // TODO how to handle lost nodes
};

ATEResults combineSingleTrajectoryResults(
    const std::vector<ATEResults> &single_traj_results);

// Assumes we've already transformed to baselink (though probably shouldn't
// matter...?) Also assumes we've already interpolated/aligned timestamps
Pose3D<double> findAlignmentTransformation(
    const std::vector<std::optional<Pose3D<double>>> &est_traj,
    const std::vector<Pose3D<double>> &gt_traj,
    const bool &adjust_translation = true);

// Assumes we've already transformed to baselink (though probably shouldn't
// matter...?) Also assumes we've already interpolated/aligned timestamps
void alignWithGroundTruth(
    const std::vector<Pose3D<double>> &gt_traj,
    const std::vector<std::optional<Pose3D<double>>> &unaligned_est_traj,
    std::vector<std::optional<Pose3D<double>>> &aligned_est_traj,
    const bool &adjust_translation = true);

// TODO do we need to find transform that minimizes dist first or should we
// assume they're aligned
ATEResults generateATEforRotAndTranslForSyncedAlignedTrajectories(
    const std::vector<std::optional<Pose3D<double>>> &est_traj,
    const std::vector<Pose3D<double>> &gt_traj);

RawWaypointConsistencyResults computeWaypointConsistencyResults(
    const std::vector<std::vector<WaypointInfo>> &waypoints_by_trajectory,
    const std::vector<
        std::vector<std::pair<pose::Timestamp, std::optional<Pose3D<double>>>>>
        &comparison_trajectories_rel_baselink,
    const std::vector<util::BoostHashMap<pose::Timestamp, Pose3D<double>>>
        &poses_by_timestamp_by_trajectory,
    const std::vector<std::vector<std::pair<pose::Timestamp, pose::Pose2d>>>
        &odom_poses_by_trajectory,
    const std::shared_ptr<RosVisualization> &vis_manager = nullptr);

Pose3D<double> getMeanPose(const std::vector<Pose3D<double>> &poses);

void getDeviationFromMeanPose(const Pose3D<double> &mean_pose,
                              const Pose3D<double> &compare_pose,
                              double &transl_deviation,
                              double &rot_deviation);

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_EVALUATION_UTILS_H
