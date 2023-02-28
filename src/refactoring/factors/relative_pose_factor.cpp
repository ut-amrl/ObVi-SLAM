#include <refactoring/factors/relative_pose_factor.h>

#include <unsupported/Eigen/MatrixFunctions>

namespace vslam_types_refactor {

RelativePoseFactor::RelativePoseFactor(
    const Pose3D<double> &measured_pose_deviation,
    const Covariance<double, 6> &pose_deviation_cov)
    : measured_translation_(measured_pose_deviation.transl_),
      measured_rotation_change_(
          measured_pose_deviation.orientation_.toRotationMatrix()),
      sqrt_inf_mat_rel_pose_(pose_deviation_cov.inverse().sqrt()) {
  if (sqrt_inf_mat_rel_pose_.hasNaN()) {
    LOG(ERROR) << "Relative pose factor had NaN information matrix " << sqrt_inf_mat_rel_pose_;
    exit(1);
  }
}

}  // namespace vslam_types_refactor