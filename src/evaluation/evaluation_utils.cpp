//
// Created by amanda on 2/17/23.
//

#include <evaluation/evaluation_utils.h>
#include <evaluation/trajectory_interpolation_utils.h>
#include <glog/logging.h>
#include <refactoring/types/vslam_types_math_util.h>

namespace vslam_types_refactor {

ATEResults combineSingleTrajectoryResults(
    const std::vector<ATEResults> &single_traj_results) {
  double rmse_transl_err = 0;
  double rmse_rot_err = 0;
  int total_valid_poses = 0;
  int total_invalid_poses = 0;

  for (const ATEResults &single_traj_ate_result : single_traj_results) {
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

  return ATEResults(
      rmse_transl_err, rmse_rot_err, total_valid_poses, total_invalid_poses);
}

// Assumes we've already transformed to baselink (though probably shouldn't
// matter...?) Also assumes we've already interpolated/aligned timestamps
Pose3D<double> findAlignmentTransformation(
    const std::vector<std::optional<Pose3D<double>>> &est_traj,
    const std::vector<Pose3D<double>> &gt_traj,
    const bool &adjust_translation) {
  // Using algorithm 1 with scale=1 from
  // https://rpg.ifi.uzh.ch/docs/IROS18_Zhang.pdf
  //  A Tutorial on Quantitative Trajectory Evaluation
  //  for Visual(-Inertial) Odometry
  //  Zichao Zhang, Davide Scaramuzza
  Position3d<double> mean_pos_est = Eigen::Vector3d::Zero();
  Position3d<double> mean_pos_gt = Eigen::Vector3d::Zero();

  CHECK_EQ(est_traj.size(), gt_traj.size());

  Covariance<double, 3> traj_cov = Eigen::Matrix3d::Zero();
  size_t non_lost_poses = 0;
  for (size_t pose_idx = 0; pose_idx < gt_traj.size(); pose_idx++) {
    if (!est_traj.at(pose_idx).has_value()) {
      continue;
    }
    non_lost_poses++;
    if (adjust_translation) {
      mean_pos_est += est_traj.at(pose_idx).value().transl_;
      mean_pos_gt += gt_traj.at(pose_idx).transl_;
    }
  }
  mean_pos_est = mean_pos_est / non_lost_poses;
  mean_pos_gt = mean_pos_gt / non_lost_poses;

  non_lost_poses = 0;
  for (size_t pose_idx = 0; pose_idx < gt_traj.size(); pose_idx++) {
    if (!est_traj.at(pose_idx).has_value()) {
      continue;
    }

    non_lost_poses++;
    Position3d<double> gt_deviation =
        gt_traj.at(pose_idx).transl_ - mean_pos_gt;
    Position3d<double> est_deviation =
        est_traj.at(pose_idx).value().transl_ - mean_pos_est;

    traj_cov += (gt_deviation * (est_deviation.transpose()));
  }
  traj_cov = traj_cov / non_lost_poses;

  Eigen::JacobiSVD<Eigen::Matrix<double, 3, 3>> svd;
  svd.compute(traj_cov, Eigen::ComputeFullU | Eigen::ComputeFullV);

  Eigen::Matrix3d uMat = svd.matrixU();
  Eigen::Matrix3d vMat = svd.matrixV();

  Eigen::Matrix3d wMat = Eigen::Matrix3d::Zero();
  wMat(0, 0) = 1;
  wMat(1, 1) = 1;
  if (uMat.determinant() * vMat.determinant() < 0) {
    wMat(2, 2) = -1;
  } else {
    wMat(2, 2) = 1;
  }

  Eigen::Matrix3d rotMat = uMat * wMat * vMat.transpose();

  Position3d<double> transl = Eigen::Vector3d::Zero();
  if (adjust_translation) {
    transl = mean_pos_gt - (rotMat * mean_pos_est);
  }

  return Pose3D<double>(transl,
                        Eigen::AngleAxis<double>(Eigen::Quaterniond(rotMat)));
}

// Assumes we've already transformed to baselink (though probably shouldn't
// matter...?) Also assumes we've already interpolated/aligned timestamps
void alignWithGroundTruth(
    const std::vector<Pose3D<double>> &gt_traj,
    const std::vector<std::optional<Pose3D<double>>> &unaligned_est_traj,
    std::vector<std::optional<Pose3D<double>>> &aligned_est_traj,
    const bool &adjust_translation) {
  Pose3D<double> aligning_transform = findAlignmentTransformation(
      unaligned_est_traj, gt_traj, adjust_translation);

  for (const std::optional<Pose3D<double>> &unaligned_entry :
       unaligned_est_traj) {
    if (!unaligned_entry.has_value()) {
      aligned_est_traj.emplace_back(unaligned_entry);
      continue;
    }
    aligned_est_traj.emplace_back(
        combinePoses(aligning_transform, unaligned_entry.value()));
  }
}

ATEResults generateATEforRotAndTranslForSyncedAlignedTrajectories(
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
  ATEResults single_traj_ate_results(avg_position_error,
                                     avg_rot_error,
                                     valid_results_num,
                                     (est_traj.size() - valid_results_num));
  return single_traj_ate_results;
}

RawWaypointConsistencyResults computeWaypointConsistencyResults(
    const std::vector<std::vector<WaypointInfo>> &waypoints_by_trajectory,
    const std::vector<
        std::vector<std::pair<pose::Timestamp, std::optional<Pose3D<double>>>>>
        &comparison_trajectories_rel_baselink,
    const std::vector<util::BoostHashMap<pose::Timestamp, Pose3D<double>>>
        &poses_by_timestamp_by_trajectory,
    const std::vector<std::vector<std::pair<pose::Timestamp, pose::Pose2d>>>
        &odom_poses_by_trajectory) {
  std::unordered_map<WaypointId, std::vector<std::pair<size_t, Pose3D<double>>>>
      poses_by_waypoint_with_trajectory;

  std::function<void(
      const util::BoostHashMap<pose::Timestamp, Pose3D<double>> &,
      const std::vector<RelativePoseFactorInfo> &)>
      vis_function =
          [](const util::BoostHashMap<pose::Timestamp, Pose3D<double>> &,
             const std::vector<RelativePoseFactorInfo> &) {};

  std::vector<util::BoostHashMap<pose::Timestamp, Pose3D<double>>>
      aligned_poses_by_timestamp_by_trajectory;
  for (size_t traj_num = 0; traj_num < waypoints_by_trajectory.size();
       traj_num++) {
    bool all_found = true;

    std::vector<pose::Timestamp> required_timestamps_for_traj;
    std::vector<WaypointInfo> waypoints_for_traj =
        waypoints_by_trajectory.at(traj_num);
    util::BoostHashMap<pose::Timestamp, Pose3D<double>> raw_poses_by_stamp =
        poses_by_timestamp_by_trajectory.at(traj_num);
    for (const WaypointInfo &waypoint_info : waypoints_for_traj) {
      required_timestamps_for_traj.emplace_back(
          waypoint_info.waypoint_timestamp_);
      if (raw_poses_by_stamp.find(waypoint_info.waypoint_timestamp_) ==
          raw_poses_by_stamp.end()) {
        all_found = false;
      }
    }
    std::sort(required_timestamps_for_traj.begin(),
              required_timestamps_for_traj.end(),
              pose::timestamp_sort());
    if (!all_found) {
      std::vector<std::pair<pose::Timestamp, Pose3D<double>>> est_traj_not_lost;
      for (const std::pair<pose::Timestamp, std::optional<Pose3D<double>>>
               &pose : comparison_trajectories_rel_baselink.at(traj_num)) {
        if (pose.second.has_value()) {
          est_traj_not_lost.emplace_back(
              std::make_pair(pose.first, pose.second.value()));
        }
      }

      util::BoostHashMap<pose::Timestamp, Pose3D<double>> interpolated_poses;
      util::BoostHashMap<pose::Timestamp, Pose3D<double>>
          odom_poses_adjusted_3d;

      // Interpolate to find estimates
      interpolate3dPosesUsingOdom(odom_poses_by_trajectory.at(traj_num),
                                  est_traj_not_lost,
                                  required_timestamps_for_traj,
                                  vis_function,
                                  interpolated_poses,
                                  odom_poses_adjusted_3d);
      aligned_poses_by_timestamp_by_trajectory.emplace_back(interpolated_poses);
    } else {
      aligned_poses_by_timestamp_by_trajectory.emplace_back(
          poses_by_timestamp_by_trajectory.at(traj_num));
    }
  }

  for (size_t traj_num = 0; traj_num < waypoints_by_trajectory.size();
       traj_num++) {
    std::vector<WaypointInfo> waypoints_for_traj =
        waypoints_by_trajectory.at(traj_num);
    util::BoostHashMap<pose::Timestamp, Pose3D<double>> poses_by_stamp =
        aligned_poses_by_timestamp_by_trajectory.at(traj_num);
    for (const WaypointInfo &waypoint_info : waypoints_for_traj) {
      Pose3D<double> pose_at_waypoint =
          poses_by_stamp.at(waypoint_info.waypoint_timestamp_);
      if (waypoint_info.reversed_) {
        pose_at_waypoint = combinePoses(
            pose_at_waypoint,
            Pose3D<double>(
                Position3d<double>(),
                Orientation3D<double>(M_PI, Eigen::Vector3d::UnitZ())));
      }
      poses_by_waypoint_with_trajectory[waypoint_info.waypoint_id_]
          .emplace_back(std::make_pair(traj_num, pose_at_waypoint));
    }
  }

  RawWaypointConsistencyResults consistency_results;
  for (const auto &waypoint_and_poses : poses_by_waypoint_with_trajectory) {
    std::vector<std::vector<double>> centroid_devs;
    centroid_devs.resize(waypoints_by_trajectory.size());
    std::vector<std::vector<double>> orientation_devs;
    orientation_devs.resize(waypoints_by_trajectory.size());

    std::vector<Pose3D<double>> poses_for_waypoint;
    for (const std::pair<size_t, Pose3D<double>>
             &traj_num_and_pose_for_waypoint : waypoint_and_poses.second) {
      poses_for_waypoint.emplace_back(traj_num_and_pose_for_waypoint.second);
    }
    Pose3D<double> mean_pose = getMeanPose(poses_for_waypoint);
    for (const std::pair<size_t, Pose3D<double>> &pose_for_waypoint_with_traj :
         waypoint_and_poses.second) {
      double transl_dev;
      double rot_dev;
      getDeviationFromMeanPose(
          mean_pose, pose_for_waypoint_with_traj.second, transl_dev, rot_dev);
      centroid_devs[pose_for_waypoint_with_traj.first].emplace_back(transl_dev);
      orientation_devs[pose_for_waypoint_with_traj.first].emplace_back(rot_dev);
    }
    consistency_results.centroid_deviations_by_waypoint_by_trajectory_
        [waypoint_and_poses.first] = centroid_devs;
    consistency_results.orientation_deviations_by_waypoint_by_trajectory_
        [waypoint_and_poses.first] = orientation_devs;
  }

  return consistency_results;
}

Pose3D<double> getMeanPose(const std::vector<Pose3D<double>> &poses) {
  if (poses.empty()) {
    return Pose3D<double>();
  }
  Position3d<double> position_mean(0, 0, 0);

  // Using rotation averaging method found here
  // https://stackoverflow.com/a/27410865
  // (References http://www.acsu.buffalo.edu/%7Ejohnc/ave_quat07.pdf)
  Eigen::Matrix<double, 4, Eigen::Dynamic> quat_mat(4, poses.size());
  for (size_t pose_idx = 0; pose_idx < poses.size(); pose_idx++) {
    Pose3D<double> pose = poses.at(pose_idx);
    position_mean = position_mean + pose.transl_;
    quat_mat.col(pose_idx) =
        (1.0 / poses.size()) * Eigen::Quaterniond(pose.orientation_).coeffs();
  }
  Eigen::Matrix<double, 4, 4> quat_mult_mat = quat_mat * quat_mat.transpose();
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver(quat_mult_mat);

  // Eigenvalues are stored in increasing order and the eigenvector that we
  // want corresponds to the largest eigenvalue
  int largest_eigenvalue_idx = 3;
  Eigen::Vector4d avg_quat_as_vec =
      eigen_solver.eigenvectors().col(largest_eigenvalue_idx);

  position_mean = position_mean / poses.size();

  return Pose3D<double>(position_mean, Orientation3D<double>(avg_quat_as_vec));
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