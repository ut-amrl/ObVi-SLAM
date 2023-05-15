#include <base_lib/pose_utils.h>
#include <ceres/ceres.h>
#include <ceres/problem.h>
#include <evaluation/trajectory_interpolation_utils.h>
#include <refactoring/factors/relative_pose_factor.h>
#include <refactoring/factors/relative_pose_factor_utils.h>
#include <refactoring/optimization/low_level_feature_pose_graph.h>
#include <refactoring/types/vslam_types_conversion.h>
#include <refactoring/types/vslam_types_math_util.h>

#include <unsupported/Eigen/MatrixFunctions>

namespace vslam_types_refactor {

bool runOptimization(
    const util::BoostHashMap<pose::Timestamp, Pose3D<double>>
        &initial_estimates,
    const std::vector<RelativePoseFactorInfo> &factors,
    const std::vector<pose::Timestamp> &fixed_timestamps,
    util::BoostHashMap<pose::Timestamp, Pose3D<double>> &results,
    std::vector<pose::Timestamp> &sorted_stamps) {
  LOG(INFO) << "Ready to run optimization. " 
    << "initial_estimates size: " << initial_estimates.size() << std::endl
    << "factors size: " << factors.size() << std::endl
    << "fixed_timestamps size: " << fixed_timestamps.size() << std::endl;
  std::shared_ptr<util::BoostHashMap<pose::Timestamp, RobotPoseNode>>
      robot_pose_nodes = std::make_shared<
          util::BoostHashMap<pose::Timestamp, RobotPoseNode>>();
  sorted_stamps.clear();
  for (const auto &robot_pose : initial_estimates) {
    RawPose3d<double> pose = convertPoseToArray(robot_pose.second);
    (*robot_pose_nodes)[robot_pose.first] = RobotPoseNode(pose);
    sorted_stamps.emplace_back(robot_pose.first);
  }
  std::sort(sorted_stamps.begin(), sorted_stamps.end(), pose::timestamp_sort());

  ceres::Problem problem;
  for (const RelativePoseFactorInfo &factor : factors) {
    double *before_robot_pose_block =
        robot_pose_nodes->at(factor.before_pose_timestamp_).pose_->data();
    double *after_robot_pose_block =
        robot_pose_nodes->at(factor.after_pose_timestamp_).pose_->data();
    if (factor.measured_pose_deviation_.transl_.norm() > 2.0) {
      LOG(WARNING) << "Factor had higher than expected pose deviation";
    }

    problem.AddResidualBlock(
        RelativePoseFactor::createRelativePoseFactor(
            factor.measured_pose_deviation_, factor.pose_deviation_cov_),
        new ceres::HuberLoss(100.0),
        before_robot_pose_block,
        after_robot_pose_block);
  }

  for (const pose::Timestamp &fixed_stamp : fixed_timestamps) {
    double *robot_pose_block = robot_pose_nodes->at(fixed_stamp).pose_->data();
    problem.SetParameterBlockConstant(robot_pose_block);
  }

  ceres::Solver::Options options;
  options.function_tolerance = 1e-12;
  options.gradient_tolerance = 1e-12;
  options.parameter_tolerance = 1e-12;
  options.use_nonmonotonic_steps = true;
  options.max_num_iterations = 600;
  ceres::Solver::Summary summary;
  LOG(INFO) << "Interpolating trajectories";
  ceres::Solve(options, &problem, &summary);

  LOG(INFO) << summary.FullReport();
  if ((summary.termination_type == ceres::TerminationType::FAILURE) ||
      (summary.termination_type == ceres::TerminationType::USER_FAILURE)) {
    LOG(ERROR) << "Ceres optimization failed";
    return false;
  }

  for (const auto &timestamp_and_raw_pose : *robot_pose_nodes) {
    results[timestamp_and_raw_pose.first] =
        convertToPose3D(*(timestamp_and_raw_pose.second.pose_));
  }
  return true;
}

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
  LOG(INFO) << "Beginning to create relative pose factors for interpolation";

  while (index_next_required_timestamp < required_timestamps.size()) {
    pose::Timestamp next_required_timestamp =
        required_timestamps[index_next_required_timestamp];
    if (!pose::timestamp_sort()(next_required_timestamp, odom_poses[0].first)) {
      break;
    }
    RelativePoseFactorInfo factor_info;

    factor_info.before_pose_timestamp_ = next_required_timestamp;
    factor_info.after_pose_timestamp_ = odom_poses[0].first;

    factor_info.measured_pose_deviation_ =
        Pose3D<double>(Position3d<double>::Zero(), Orientation3D<double>());
    Eigen::Matrix<double, 6, 1> std_dev =
        kMinStdDev * Eigen::Matrix<double, 6, 1>::Ones();
    factor_info.pose_deviation_cov_ = createDiagCovFromStdDevs(std_dev);
    pose_factor_infos.emplace_back(factor_info);
    closest_odom_stamp[next_required_timestamp] = odom_poses[0].first;
    index_next_required_timestamp++;
  }

  while (index_next_coarse_traj_timestamp < coarse_fixed_poses.size()) {
    pose::Timestamp next_coarse_traj_timestamp =
        coarse_fixed_poses[index_next_coarse_traj_timestamp].first;
    if (!pose::timestamp_sort()(next_coarse_traj_timestamp,
                                odom_poses[0].first)) {
      break;
    }
    RelativePoseFactorInfo factor_info;

    factor_info.before_pose_timestamp_ = next_coarse_traj_timestamp;
    factor_info.after_pose_timestamp_ = odom_poses[0].first;

    factor_info.measured_pose_deviation_ =
        Pose3D<double>(Position3d<double>::Zero(), Orientation3D<double>());
    Eigen::Matrix<double, 6, 1> std_dev =
        kMinStdDev * Eigen::Matrix<double, 6, 1>::Ones();
    factor_info.pose_deviation_cov_ = createDiagCovFromStdDevs(std_dev);
    pose_factor_infos.emplace_back(factor_info);
    closest_odom_stamp[next_coarse_traj_timestamp] = odom_poses[0].first;
    index_next_coarse_traj_timestamp++;
  }

  LOG(INFO) << "Added all factors prior to first odom";
  LOG(INFO) << index_next_required_timestamp
            << " required poses before any odom timestamps";
  LOG(INFO) << index_next_coarse_traj_timestamp
            << " coarse traj poses before any odom timestamps";

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
    if (abs(odom_pose_factor_info.measured_pose_deviation_.orientation_
                .angle()) > M_PI_2) {
      LOG(WARNING) << "Excessively high orientation deviation in a factor "
                   << odom_pose_factor_info.measured_pose_deviation_
                          .orientation_.angle();
    }

    odom_pose_factor_info.pose_deviation_cov_ =
        generateOdomCov(odom_pose_factor_info.measured_pose_deviation_,
                        kTranslErrorMultForTranslError,
                        kTranslErrorMultForRotError,
                        kRotErrorMultForTranslError,
                        kRotErrorMultForRotError);

    pose_factor_infos.emplace_back(odom_pose_factor_info);

    while (index_next_required_timestamp < required_timestamps.size()) {
      pose::Timestamp next_required_timestamp =
          required_timestamps[index_next_required_timestamp];
      if (pose::timestamp_sort()(next_required_timestamp,
                                 odom_poses[odom_pose_idx].first)) {
        if ((next_required_timestamp.first ==
             odom_poses[odom_pose_idx].first.first) &&
            (next_required_timestamp.second ==
             odom_poses[odom_pose_idx].first.second)) {
          closest_odom_stamp[next_required_timestamp] =
              odom_poses[odom_pose_idx].first;
          index_next_required_timestamp++;
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
          if (abs(required_pose_factor_1.measured_pose_deviation_.orientation_
                      .angle()) > M_PI_2) {
            LOG(WARNING)
                << "Excessively high orientation deviation in a factor "
                << required_pose_factor_1.measured_pose_deviation_.orientation_
                       .angle();
          }
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
          if (abs(required_pose_factor_2.measured_pose_deviation_.orientation_
                      .angle()) > M_PI_2) {
            LOG(WARNING)
                << "Excessively high orientation deviation in a factor "
                << required_pose_factor_2.measured_pose_deviation_.orientation_
                       .angle();
          }
          required_pose_factor_2.pose_deviation_cov_ =
              generateOdomCov(required_pose_factor_2.measured_pose_deviation_,
                              kTranslErrorMultForTranslError,
                              kTranslErrorMultForRotError,
                              kRotErrorMultForTranslError,
                              kRotErrorMultForRotError);
          pose_factor_infos.emplace_back(required_pose_factor_2);
          closest_odom_stamp[next_required_timestamp] =
              odom_poses[odom_pose_idx].first;
          index_next_required_timestamp++;
        }
      } else {
        break;
      }
    }
    while (index_next_coarse_traj_timestamp < coarse_fixed_poses.size()) {
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
          approx_odom_adjustment = combinePoses(closest_fixed_traj_pose,
                                                poseInverse(curr_odom_pose_3d));
        }

        closest_odom_stamp[next_coarse_traj_timestamp] = curr_timestamp;
        if ((next_coarse_traj_timestamp.first == curr_timestamp.first) &&
            (next_coarse_traj_timestamp.second == curr_timestamp.second)) {
          // Don't need to do anything?
          index_next_coarse_traj_timestamp++;
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
          if (abs(coarse_traj_pose_factor_1.measured_pose_deviation_
                      .orientation_.angle()) > M_PI_2) {
            LOG(WARNING)
                << "Excessively high orientation deviation in a factor "
                << coarse_traj_pose_factor_1.measured_pose_deviation_
                       .orientation_.angle();
          }
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
          if (abs(coarse_traj_pose_factor_2.measured_pose_deviation_
                      .orientation_.angle()) > M_PI_2) {
            LOG(WARNING)
                << "Excessively high orientation deviation in a factor "
                << coarse_traj_pose_factor_2.measured_pose_deviation_
                       .orientation_.angle();
          }
          coarse_traj_pose_factor_2.pose_deviation_cov_ = generateOdomCov(
              coarse_traj_pose_factor_2.measured_pose_deviation_,
              kTranslErrorMultForTranslError,
              kTranslErrorMultForRotError,
              kRotErrorMultForTranslError,
              kRotErrorMultForRotError);
          pose_factor_infos.emplace_back(coarse_traj_pose_factor_2);
          index_next_coarse_traj_timestamp++;
        }
      } else {
        break;
      }
    }
  }

  LOG(INFO) << "Added all factors prior to last odom";
  while (index_next_required_timestamp < required_timestamps.size()) {
    pose::Timestamp next_required_timestamp =
        required_timestamps[index_next_required_timestamp];
    RelativePoseFactorInfo factor_info;
    closest_odom_stamp[next_required_timestamp] = odom_poses.back().first;

    factor_info.before_pose_timestamp_ = next_required_timestamp;
    factor_info.after_pose_timestamp_ = odom_poses.back().first;

    factor_info.measured_pose_deviation_ =
        Pose3D<double>(Position3d<double>::Zero(), Orientation3D<double>());
    Eigen::Matrix<double, 6, 1> std_dev =
        kMinStdDev * Eigen::Matrix<double, 6, 1>::Ones();
    factor_info.pose_deviation_cov_ = createDiagCovFromStdDevs(std_dev);
    pose_factor_infos.emplace_back(factor_info);
    index_next_required_timestamp++;
  }

  while (index_next_coarse_traj_timestamp < coarse_fixed_poses.size()) {
    pose::Timestamp next_coarse_traj_timestamp =
        coarse_fixed_poses[index_next_coarse_traj_timestamp].first;
    RelativePoseFactorInfo factor_info;

    closest_odom_stamp[next_coarse_traj_timestamp] = odom_poses.back().first;

    factor_info.before_pose_timestamp_ = next_coarse_traj_timestamp;
    factor_info.after_pose_timestamp_ = odom_poses.back().first;

    factor_info.measured_pose_deviation_ =
        Pose3D<double>(Position3d<double>::Zero(), Orientation3D<double>());
    Eigen::Matrix<double, 6, 1> std_dev =
        kMinStdDev * Eigen::Matrix<double, 6, 1>::Ones();
    factor_info.pose_deviation_cov_ = createDiagCovFromStdDevs(std_dev);
    pose_factor_infos.emplace_back(factor_info);
    index_next_coarse_traj_timestamp++;
  }

  LOG(INFO) << "Added all factors";
  return pose_factor_infos;
}

void interpolate3dPosesUsingOdom(
    const std::vector<std::pair<pose::Timestamp, pose::Pose2d>> &odom_poses,
    const std::vector<std::pair<pose::Timestamp, Pose3D<double>>>
        &coarse_fixed_poses,
    const std::vector<pose::Timestamp> &required_timestamps,
    const std::function<
        void(const util::BoostHashMap<pose::Timestamp, Pose3D<double>> &,
             const std::vector<RelativePoseFactorInfo> &)> &vis_function,
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

  LOG(INFO) << "Adjusting odom poses to approximately match lidar frame";
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
  util::BoostHashMap<pose::Timestamp, Pose3D<double>> poses_by_timestamp;

  std::vector<pose::Timestamp> all_stamps;
  std::vector<pose::Timestamp> coarse_timestamps;
  LOG(INFO) << "Initializing poses at coarse trajectory timestamps";
  for (const std::pair<pose::Timestamp, Pose3D<double>> &stamp_and_coarse_pose :
       coarse_fixed_poses) {
    poses_by_timestamp[stamp_and_coarse_pose.first] =
        stamp_and_coarse_pose.second;
    coarse_timestamps.emplace_back(stamp_and_coarse_pose.first);
    all_stamps.emplace_back(stamp_and_coarse_pose.first);
  }

  // Initialize poses for all timestamps of concern
  LOG(INFO) << "Initializing poses at required timestamps";
  for (const pose::Timestamp &required_stamp : required_timestamps) {
    if (poses_by_timestamp.find(required_stamp) == poses_by_timestamp.end()) {
      poses_by_timestamp[required_stamp] =
          odom_poses_adjusted_3d.at(closest_odom_stamps.at(required_stamp));
      all_stamps.emplace_back(required_stamp);
    }
  }

  LOG(INFO) << "Initializing poses at odom timestamps";
  for (const auto &odom_stamp_and_time : odom_poses_adjusted_3d) {
    if (poses_by_timestamp.find(odom_stamp_and_time.first) ==
        poses_by_timestamp.end()) {
      poses_by_timestamp[odom_stamp_and_time.first] =
          odom_stamp_and_time.second;
      all_stamps.emplace_back(odom_stamp_and_time.first);
    }
  }
  std::sort(all_stamps.begin(), all_stamps.end(), pose::timestamp_sort());

  // vis_function(poses_by_timestamp, pose_factor_infos);
  runOptimization(poses_by_timestamp,
                  pose_factor_infos,
                  coarse_timestamps,
                  interpolated_poses,
                  all_stamps);
  // vis_function(interpolated_poses, pose_factor_infos);

  LOG(INFO) << "Storing interpolated poses";
}

}  // namespace vslam_types_refactor