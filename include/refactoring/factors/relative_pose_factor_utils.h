//
// Created by amanda on 3/2/23.
//

#ifndef UT_VSLAM_RELATIVE_POSE_FACTOR_UTILS_H
#define UT_VSLAM_RELATIVE_POSE_FACTOR_UTILS_H

#include <refactoring/types/vslam_basic_types_refactor.h>
#include <refactoring/types/vslam_math_util.h>

namespace vslam_types_refactor {

namespace {
const double kMinStdDev = 1e-3;
}

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

  return createDiagCovFromStdDevs(std_devs, kMinStdDev);
}

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_RELATIVE_POSE_FACTOR_UTILS_H
