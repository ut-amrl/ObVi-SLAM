#include <ceres/ceres.h>
#include <ceres/problem.h>
#include <evaluation/trajectory_interpolation_utils.h>
#include <refactoring/factors/relative_pose_factor.h>
#include <refactoring/types/vslam_types_conversion.h>
#include <refactoring/types/vslam_types_math_util.h>

#include <unsupported/Eigen/MatrixFunctions>

namespace vslam_types_refactor {

std::vector<RelativePoseFactorInfo> createRelativePoseFactorInfo(
    const std::vector<std::pair<pose::Timestamp, pose::Pose2d>> &odom_poses,
    const std::vector<std::pair<pose::Timestamp, Pose3D<double>>>
        &coarse_fixed_poses,
    const std::vector<pose::Timestamp> &required_timestamps,
    std::optional<Pose3D<double>> &approx_odom_adjustment,
    util::BoostHashMap<pose::Timestamp, pose::Timestamp> &closest_odom_stamp) {
  size_t index_next_required_timestamp = 0;
  size_t index_next_coarse_traj_timestamp = 0;
  std::vector<RelativePoseFactorInfo> pose_factor_infos;

  for (size_t odom_pose_idx = 1; odom_pose_idx < odom_poses.size();
       odom_pose_idx++) {
    RelativePoseFactorInfo odom_pose_factor_info;
    pose::Pose2d relative_odom_pose_2d = pose::getPoseOfObj1RelToObj2(
        odom_poses[odom_pose_idx].second, odom_poses[odom_pose_idx - 1].second);

    pose::Pose3d relative_odom_pose_3d = pose::toPose3d(relative_odom_pose_2d);
    odom_pose_factor_info.before_pose_timestamp_ =
        odom_poses[odom_pose_idx - 1].first;
    odom_pose_factor_info.after_pose_timestamp_ =
        odom_poses[odom_pose_idx].first;

    odom_pose_factor_info.measured_pose_deviation_ =
        Pose3D<double>(relative_odom_pose_3d.first,
                       Orientation3D<double>(relative_odom_pose_3d.second));

    odom_pose_factor_info.pose_deviation_cov_ =
        generateOdomCov(odom_pose_factor_info.measured_pose_deviation_,
                        kTranslErrorMultForTranslError,
                        kTranslErrorMultForRotError,
                        kRotErrorMultForTranslError,
                        kRotErrorMultForRotError);

    pose_factor_infos.emplace_back(odom_pose_factor_info);

    if (index_next_required_timestamp < required_timestamps.size()) {
      pose::Timestamp next_required_timestamp =
          required_timestamps[index_next_required_timestamp];
      if (pose::timestamp_sort()(next_required_timestamp,
                                 odom_poses[odom_pose_idx].first)) {
        if ((next_required_timestamp.first ==
             odom_poses[odom_pose_idx].first.first) &&
            (next_required_timestamp.second ==
             odom_poses[odom_pose_idx].first.second)) {
          // Don't need to do anything?
        } else {
          pose::Pose2d prev_pose = odom_poses[odom_pose_idx - 1].second;
          pose::Timestamp prev_timestamp = odom_poses[odom_pose_idx - 1].first;

          pose::Pose2d curr_pose = odom_poses[odom_pose_idx].second;
          pose::Timestamp curr_timestamp = odom_poses[odom_pose_idx].first;

          pose::Pose2d rel_pose_interp_global =
              pose::interpolatePoses(std::make_pair(prev_timestamp, prev_pose),
                                     std::make_pair(curr_timestamp, curr_pose),
                                     next_required_timestamp);
          if (pose::timestamp_sort()(next_required_timestamp, prev_timestamp)) {
            LOG(ERROR) << "Out of order timestamps";
            exit(1);
          }

          RelativePoseFactorInfo required_pose_factor_1;
          pose::Pose2d required_pose_factor_1_2d =
              pose::getPoseOfObj1RelToObj2(rel_pose_interp_global, prev_pose);

          pose::Pose3d required_pose_factor_1_3d =
              pose::toPose3d(required_pose_factor_1_2d);
          required_pose_factor_1.before_pose_timestamp_ = prev_timestamp;
          required_pose_factor_1.after_pose_timestamp_ =
              next_required_timestamp;
          required_pose_factor_1.measured_pose_deviation_ = Pose3D<double>(
              required_pose_factor_1_3d.first,
              Orientation3D<double>(required_pose_factor_1_3d.second));
          required_pose_factor_1.pose_deviation_cov_ =
              generateOdomCov(required_pose_factor_1.measured_pose_deviation_,
                              kTranslErrorMultForTranslError,
                              kTranslErrorMultForRotError,
                              kRotErrorMultForTranslError,
                              kRotErrorMultForRotError);
          pose_factor_infos.emplace_back(required_pose_factor_1);

          RelativePoseFactorInfo required_pose_factor_2;
          pose::Pose2d required_pose_factor_2_2d =
              pose::getPoseOfObj1RelToObj2(curr_pose, rel_pose_interp_global);

          pose::Pose3d required_pose_factor_2_3d =
              pose::toPose3d(required_pose_factor_2_2d);
          required_pose_factor_2.before_pose_timestamp_ =
              next_required_timestamp;
          required_pose_factor_2.after_pose_timestamp_ = curr_timestamp;
          required_pose_factor_2.measured_pose_deviation_ = Pose3D<double>(
              required_pose_factor_2_3d.first,
              Orientation3D<double>(required_pose_factor_2_3d.second));
          required_pose_factor_2.pose_deviation_cov_ =
              generateOdomCov(required_pose_factor_2.measured_pose_deviation_,
                              kTranslErrorMultForTranslError,
                              kTranslErrorMultForRotError,
                              kRotErrorMultForTranslError,
                              kRotErrorMultForRotError);
          pose_factor_infos.emplace_back(required_pose_factor_2);
          index_next_required_timestamp++;
        }
      }
    }
    if (index_next_coarse_traj_timestamp < coarse_fixed_poses.size()) {
      pose::Timestamp next_coarse_traj_timestamp =
          coarse_fixed_poses[index_next_coarse_traj_timestamp].first;
      if (pose::timestamp_sort()(next_coarse_traj_timestamp,
                                 odom_poses[odom_pose_idx].first)) {
        pose::Timestamp curr_timestamp = odom_poses[odom_pose_idx].first;
        pose::Pose2d curr_pose = odom_poses[odom_pose_idx].second;
        if (!approx_odom_adjustment.has_value()) {
          pose::Pose3d old_curr_odom_pose_3d = pose::toPose3d(curr_pose);
          Pose3D<double> curr_odom_pose_3d(
              old_curr_odom_pose_3d.first,
              Orientation3D<double>(old_curr_odom_pose_3d.second));
          Pose3D<double> closest_fixed_traj_pose =
              coarse_fixed_poses[index_next_coarse_traj_timestamp].second;
          approx_odom_adjustment = getPose2RelativeToPose1(
              closest_fixed_traj_pose, curr_odom_pose_3d);
        }

        closest_odom_stamp[next_coarse_traj_timestamp] = curr_timestamp;
        if ((next_coarse_traj_timestamp.first == curr_timestamp.first) &&
            (next_coarse_traj_timestamp.second == curr_timestamp.second)) {
          // Don't need to do anything?
        } else {
          pose::Pose2d prev_pose = odom_poses[odom_pose_idx - 1].second;
          pose::Timestamp prev_timestamp = odom_poses[odom_pose_idx - 1].first;

          pose::Pose2d rel_pose_interp_global =
              pose::interpolatePoses(std::make_pair(prev_timestamp, prev_pose),
                                     std::make_pair(curr_timestamp, curr_pose),
                                     next_coarse_traj_timestamp);
          if (pose::timestamp_sort()(next_coarse_traj_timestamp,
                                     prev_timestamp)) {
            LOG(ERROR) << "Out of order timestamps";
            exit(1);
          }

          RelativePoseFactorInfo coarse_traj_pose_factor_1;
          pose::Pose2d coarse_traj_pose_factor_1_2d =
              pose::getPoseOfObj1RelToObj2(rel_pose_interp_global, prev_pose);

          pose::Pose3d coarse_traj_pose_factor_1_3d =
              pose::toPose3d(coarse_traj_pose_factor_1_2d);
          coarse_traj_pose_factor_1.before_pose_timestamp_ = prev_timestamp;
          coarse_traj_pose_factor_1.after_pose_timestamp_ =
              next_coarse_traj_timestamp;
          coarse_traj_pose_factor_1.measured_pose_deviation_ = Pose3D<double>(
              coarse_traj_pose_factor_1_3d.first,
              Orientation3D<double>(coarse_traj_pose_factor_1_3d.second));
          coarse_traj_pose_factor_1.pose_deviation_cov_ = generateOdomCov(
              coarse_traj_pose_factor_1.measured_pose_deviation_,
              kTranslErrorMultForTranslError,
              kTranslErrorMultForRotError,
              kRotErrorMultForTranslError,
              kRotErrorMultForRotError);
          pose_factor_infos.emplace_back(coarse_traj_pose_factor_1);

          RelativePoseFactorInfo coarse_traj_pose_factor_2;
          pose::Pose2d coarse_traj_pose_factor_2_2d =
              pose::getPoseOfObj1RelToObj2(curr_pose, rel_pose_interp_global);

          pose::Pose3d coarse_traj_pose_factor_2_3d =
              pose::toPose3d(coarse_traj_pose_factor_2_2d);
          coarse_traj_pose_factor_2.before_pose_timestamp_ =
              next_coarse_traj_timestamp;
          coarse_traj_pose_factor_2.after_pose_timestamp_ = curr_timestamp;
          coarse_traj_pose_factor_2.measured_pose_deviation_ = Pose3D<double>(
              coarse_traj_pose_factor_2_3d.first,
              Orientation3D<double>(coarse_traj_pose_factor_2_3d.second));
          coarse_traj_pose_factor_2.pose_deviation_cov_ = generateOdomCov(
              coarse_traj_pose_factor_2.measured_pose_deviation_,
              kTranslErrorMultForTranslError,
              kTranslErrorMultForRotError,
              kRotErrorMultForTranslError,
              kRotErrorMultForRotError);
          pose_factor_infos.emplace_back(coarse_traj_pose_factor_2);
          index_next_coarse_traj_timestamp++;
        }
      }
    }
  }
  return pose_factor_infos;
}

void interpolate3dPosesUsingOdom(
    const std::vector<std::pair<pose::Timestamp, pose::Pose2d>> &odom_poses,
    const std::vector<std::pair<pose::Timestamp, Pose3D<double>>>
        &coarse_fixed_poses,
    const std::vector<pose::Timestamp> &required_timestamps,
    util::BoostHashMap<pose::Timestamp, Pose3D<double>> &interpolated_poses,
    util::BoostHashMap<pose::Timestamp, Pose3D<double>>
        &odom_poses_adjusted_3d) {
  util::BoostHashMap<pose::Timestamp, pose::Timestamp> closest_odom_stamps;
  std::optional<Pose3D<double>> approx_odom_adjustment;
  std::vector<RelativePoseFactorInfo> pose_factor_infos =
      createRelativePoseFactorInfo(odom_poses,
                                   coarse_fixed_poses,
                                   required_timestamps,
                                   approx_odom_adjustment,
                                   closest_odom_stamps);

  for (const auto &odom_pose : odom_poses) {
    pose::Pose3d old_pose_3d = pose::toPose3d(odom_pose.second);
    Pose3D<double> pose_3d(old_pose_3d.first,
                           Orientation3D<double>(old_pose_3d.second));
    if (approx_odom_adjustment.has_value()) {
      pose_3d = combinePoses(approx_odom_adjustment.value(), pose_3d);
    }

    odom_poses_adjusted_3d[odom_pose.first] = pose_3d;
  }

  // Create data structure with poses that can be updated
  util::BoostHashMap<pose::Timestamp, RawPose3d<double>> raw_poses_by_timestamp;

  // Initialize poses for all timestamps of concern
  for (const pose::Timestamp &required_stamp : required_timestamps) {
    raw_poses_by_timestamp[required_stamp] = convertPoseToArray(
        odom_poses_adjusted_3d.at(closest_odom_stamps.at(required_stamp)));
  }
  for (const auto &odom_stamp_and_time : odom_poses_adjusted_3d) {
    raw_poses_by_timestamp[odom_stamp_and_time.first] =
        convertPoseToArray(odom_stamp_and_time.second);
  }
  for (const std::pair<pose::Timestamp, Pose3D<double>> &stamp_and_coarse_pose :
       coarse_fixed_poses) {
    raw_poses_by_timestamp[stamp_and_coarse_pose.first] =
        convertPoseToArray(stamp_and_coarse_pose.second);
  }

  // Add residuals for each factor
  ceres::Problem problem;
  for (const RelativePoseFactorInfo &factor : pose_factor_infos) {
    problem.AddResidualBlock(
        RelativePoseFactor::createRelativePoseFactor(
            factor.measured_pose_deviation_, factor.pose_deviation_cov_),
        nullptr,
        raw_poses_by_timestamp.at(factor.before_pose_timestamp_).data(),
        raw_poses_by_timestamp.at(factor.after_pose_timestamp_).data());
  }

  // Set the coarse poses constant (just trying to interpolate between them)
  for (const std::pair<pose::Timestamp, Pose3D<double>> &stamp_and_coarse_pose :
       coarse_fixed_poses) {
    problem.SetParameterBlockConstant(
        raw_poses_by_timestamp.at(stamp_and_coarse_pose.first).data());
  }

  ceres::Solver::Summary summary;
  ceres::Solver::Options options;
  ceres::Solve(options, &problem, &summary);
  LOG(INFO) << summary.FullReport();

  for (const auto &timestamp_and_raw_pose : raw_poses_by_timestamp) {
    interpolated_poses[timestamp_and_raw_pose.first] =
        convertToPose3D(timestamp_and_raw_pose.second);
  }
}

}  // namespace vslam_types_refactor