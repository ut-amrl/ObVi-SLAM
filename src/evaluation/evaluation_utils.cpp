//
// Created by amanda on 2/17/23.
//

#include <evaluation/evaluation_utils.h>
#include <glog/logging.h>
#include <refactoring/types/vslam_types_math_util.h>

namespace vslam_types_refactor {

TrajectorySequenceATEResults combineSingleTrajectoryResults(
    const std::vector<SingleTrajectoryATEResults> &single_traj_results) {
  double rmse_transl_err = 0;
  double rmse_rot_err = 0;
  int total_valid_poses = 0;
  int total_invalid_poses = 0;

  for (const SingleTrajectoryATEResults &single_traj_ate_result :
       single_traj_results) {
    double squared_sum_transl_err =
        pow(single_traj_ate_result.rmse_transl_err_, 2) *
        single_traj_ate_result.valid_poses_used_in_score_;
    rmse_transl_err += squared_sum_transl_err;
    double squared_sum_rot_err =
        pow(single_traj_ate_result.rmse_rot_err_, 2) *
        single_traj_ate_result.valid_poses_used_in_score_;
    rmse_rot_err += squared_sum_rot_err;
    total_valid_poses += single_traj_ate_result.valid_poses_used_in_score_;
    total_invalid_poses += single_traj_ate_result.lost_poses_;
  }

  rmse_transl_err = sqrt(rmse_transl_err / total_valid_poses);
  rmse_rot_err = sqrt(rmse_rot_err / total_valid_poses);

  return TrajectorySequenceATEResults(
      rmse_transl_err, rmse_rot_err, total_valid_poses, total_invalid_poses);
}

SingleTrajectoryATEResults
generateATEforRotAndTranslForSyncedAlignedTrajectories(
    const std::vector<std::optional<Pose3D<double>>> &est_traj,
    const std::vector<Pose3D<double>> &gt_traj) {
  CHECK_EQ(est_traj.size(), gt_traj.size())
      << "Trajectories must be the same size to calculate ATE";

  // Position error from here (assuming already aligned)
  // http://www2.informatik.uni-freiburg.de/~endres/files/publications/sturm12iros.pdf
  // https://arxiv.org/pdf/1910.04755.pdf

  // Rot error taken approximately from
  // https://rpg.ifi.uzh.ch/docs/IROS18_Zhang.pdf (get delta R as described in
  // the translation ATE references)
  double avg_position_error = 0;
  double avg_rot_error = 0;

  int valid_results_num = 0;
  for (size_t pose_num = 0; pose_num < est_traj.size(); pose_num++) {
    if (est_traj.at(pose_num).has_value()) {
      Pose3D<double> pose_separation = getPose2RelativeToPose1(
          est_traj.at(pose_num).value(), gt_traj.at(pose_num));
      avg_position_error += pose_separation.transl_.squaredNorm();
      avg_rot_error += pow(pose_separation.orientation_.angle(), 2);
      valid_results_num++;
    } else {
      // TODO what do we do with ones that are lost?
    }
  }

  avg_position_error = sqrt(avg_position_error / valid_results_num);
  avg_rot_error = sqrt(avg_rot_error / valid_results_num);
  SingleTrajectoryATEResults single_traj_ate_results(
      avg_position_error,
      avg_rot_error,
      valid_results_num,
      (est_traj.size() - valid_results_num));
  return single_traj_ate_results;
}

RawWaypointConsistencyResults computeWaypointConsistencyResults(
    const std::vector<std::vector<WaypointInfo>> &waypoints_by_trajectory,
    const std::vector<util::BoostHashMap<pose::Timestamp, FrameId>>
        &poses_by_timestamp_by_trajectory,
    const std::vector<std::unordered_map<FrameId, Pose3D<double>>>
        &poses_by_frame_by_trajectory) {
  CHECK_EQ(waypoints_by_trajectory.size(),
           poses_by_timestamp_by_trajectory.size())
      << "Number of trajectories for waypoints and number of trajectories that "
         "we have timestamp-frame mapping must be the same to compute "
         "consistency results";
  CHECK_EQ(waypoints_by_trajectory.size(), poses_by_frame_by_trajectory.size())
      << "Number of trajectories for waypoints and number of trajectories that "
         "we have pose-frame mapping must be the same to compute "
         "consistency results";

  std::vector<std::vector<AssociatedWaypointInfo>>
      associated_waypoints_by_trajectory;  // TODO populate
  for (size_t traj_num = 0; traj_num < waypoints_by_trajectory.size();
       traj_num++) {
    std::vector<AssociatedWaypointInfo> assoc_waypoints_for_traj;
    if (waypoints_by_trajectory.at(traj_num).empty()) {
      associated_waypoints_by_trajectory.emplace_back(assoc_waypoints_for_traj);
      continue;
    }
    std::vector<pose::Timestamp> waypoint_stamps;
    for (const WaypointInfo &waypoint_info :
         waypoints_by_trajectory.at(traj_num)) {
      waypoint_stamps.emplace_back(waypoint_info.waypoint_timestamp_);
    }
    util::BoostHashMap<pose::Timestamp, FrameId> frames_for_req_stamps =
        getFramesForRequiredStamps(
            waypoint_stamps, poses_by_timestamp_by_trajectory.at(traj_num));
    for (const WaypointInfo &waypoint_info :
         waypoints_by_trajectory.at(traj_num)) {
      FrameId frame_for_waypoint =
          frames_for_req_stamps.at(waypoint_info.waypoint_timestamp_);
      AssociatedWaypointInfo assoc_waypoint_info;
      assoc_waypoint_info.waypoint_id_ = waypoint_info.waypoint_id_;
      assoc_waypoint_info.reversed_ = waypoint_info.reversed_;
      assoc_waypoint_info.associated_frame_id_ = frame_for_waypoint;
      assoc_waypoints_for_traj.emplace_back(assoc_waypoint_info);
    }
    associated_waypoints_by_trajectory.emplace_back(assoc_waypoints_for_traj);
  }

  return computeWaypointConsistencyResultsForFrameAssociatedWaypoints(
      associated_waypoints_by_trajectory, poses_by_frame_by_trajectory);
}

RawWaypointConsistencyResults
computeWaypointConsistencyResultsForFrameAssociatedWaypoints(
    const std::vector<std::vector<AssociatedWaypointInfo>>
        &waypoints_by_trajectory,
    const std::vector<std::unordered_map<FrameId, Pose3D<double>>>
        &poses_by_frame_by_trajectory) {
  std::unordered_map<WaypointId, std::vector<Pose3D<double>>> poses_by_waypoint;
  for (size_t traj_num = 0; traj_num < waypoints_by_trajectory.size();
       traj_num++) {
    std::vector<AssociatedWaypointInfo> waypoints_for_traj =
        waypoints_by_trajectory.at(traj_num);
    std::unordered_map<FrameId, Pose3D<double>> poses_by_frame =
        poses_by_frame_by_trajectory.at(traj_num);
    for (const AssociatedWaypointInfo &waypoint_info : waypoints_for_traj) {
      Pose3D<double> pose_at_waypoint =
          poses_by_frame.at(waypoint_info.associated_frame_id_);
      if (waypoint_info.reversed_) {
        pose_at_waypoint = combinePoses(
            pose_at_waypoint,
            Pose3D<double>(
                Position3d<double>(),
                Orientation3D<double>(M_PI, Eigen::Vector3d::UnitZ())));
      }
      poses_by_waypoint[waypoint_info.waypoint_id_].emplace_back(
          pose_at_waypoint);
    }
  }

  RawWaypointConsistencyResults consistency_results;
  for (const auto &waypoint_and_poses : poses_by_waypoint) {
    Pose3D<double> mean_pose = getMeanPose(waypoint_and_poses.second);
    for (const Pose3D<double> &pose_for_waypoint : waypoint_and_poses.second) {
      double transl_dev;
      double rot_dev;
      getDeviationFromMeanPose(
          mean_pose, pose_for_waypoint, transl_dev, rot_dev);
      consistency_results
          .centroid_deviations_by_waypoint_[waypoint_and_poses.first]
          .emplace_back(transl_dev);
      consistency_results
          .orientation_deviations_by_waypoint_[waypoint_and_poses.first]
          .emplace_back(rot_dev);
    }
  }

  return consistency_results;
}

Pose3D<double> getMeanPose(const std::vector<Pose3D<double>> &poses) {
  if (poses.empty()) {
    return Pose3D<double>();
  }
  Position3d<double> position_mean(0, 0, 0);
  // TODO orientation mean
  for (const Pose3D<double> &pose : poses) {
    position_mean = position_mean + pose.transl_;
  }
  position_mean = position_mean / poses.size();

  return Pose3D<double>(position_mean, Orientation3D<double>());  // TODO
}

void getDeviationFromMeanPose(const Pose3D<double> &mean_pose,
                              const Pose3D<double> &compare_pose,
                              double &transl_deviation,
                              double &rot_deviation) {
  Pose3D<double> pose_diff = getPose2RelativeToPose1(mean_pose, compare_pose);
  transl_deviation = pose_diff.transl_.norm();
  rot_deviation = pose_diff.orientation_.angle();
}

}  // namespace vslam_types_refactor