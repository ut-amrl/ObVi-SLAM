//
// Created by amanda on 2/14/23.
//

#ifndef UT_VSLAM_TRAJECTORY_INTERPOLATION_UTILS_H
#define UT_VSLAM_TRAJECTORY_INTERPOLATION_UTILS_H

#include <base_lib/basic_utils.h>
#include <base_lib/pose_utils.h>
#include <refactoring/types/vslam_basic_types_refactor.h>
#include <refactoring/types/vslam_math_util.h>

namespace vslam_types_refactor {

const double kTranslErrorMultForTranslError = 0.05;
const double kTranslErrorMultForRotError = 0.05;
const double kRotErrorMultForTranslError = 0.05;
const double kRotErrorMultForRotError = 0.05;

struct RelativePoseFactorInfo {
  Pose3D<double> measured_pose_deviation_;
  Covariance<double, 6> pose_deviation_cov_;
  pose::Timestamp before_pose_timestamp_;
  pose::Timestamp after_pose_timestamp_;
};

Covariance<double, 6> generateOdomCov(
    const Pose3D<double> &relative_pose,
    const double &transl_error_mult_for_transl_error,
    const double &transl_error_mult_for_rot_error,
    const double &rot_error_mult_for_transl_error,
    const double &rot_error_mult_for_rot_error) {
  Eigen::Matrix<double, 6, 1> std_devs;
  std_devs.topRows(3) =
      relative_pose.transl_.cwiseAbs() * transl_error_mult_for_transl_error +
      (abs(relative_pose.orientation_.angle()) *
       rot_error_mult_for_transl_error * Eigen::Vector3d::Ones());
  std_devs.bottomRows(3) =
      (relative_pose.orientation_.axis() * relative_pose.orientation_.angle())
              .cwiseAbs() *
          rot_error_mult_for_rot_error +
      (relative_pose.transl_.norm() * transl_error_mult_for_rot_error *
       Eigen::Vector3d::Ones());

  return createDiagCovFromStdDevs(std_devs);
}

void interpolate3dPosesUsingOdom(
    const std::vector<std::pair<pose::Timestamp, pose::Pose2d>> &odom_poses,
    const std::vector<std::pair<pose::Timestamp, Pose3D<double>>>
        &coarse_fixed_poses,
    const std::vector<pose::Timestamp> &required_timestamps,
    util::BoostHashMap<pose::Timestamp, Pose3D<double>> &interpolated_poses,
    util::BoostHashMap<pose::Timestamp, Pose3D<double>>
        &odom_poses_adjusted_3d);

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_TRAJECTORY_INTERPOLATION_UTILS_H
