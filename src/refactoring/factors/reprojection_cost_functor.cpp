#include <refactoring/factors/reprojection_cost_functor.h>

namespace vslam_types_refactor {

ReprojectionCostFunctor::ReprojectionCostFunctor(
    const vslam_types_refactor::PixelCoord<double> &image_feature,
    const vslam_types_refactor::CameraIntrinsicsMat<double> &intrinsics,
    const vslam_types_refactor::CameraExtrinsics<double> &extrinsics,
    const double &reprojection_error_std_dev)
    : cam_to_robot_tf_inv_(
          (Eigen::Translation3d(extrinsics.transl_) * extrinsics.orientation_)
              .inverse()) {
  rect_feature_x_ = (image_feature.x() - intrinsics(0, 2)) / intrinsics(0, 0);
  rect_feature_y_ = (image_feature.y() - intrinsics(1, 2)) / intrinsics(1, 1);
  rectified_error_multiplier_x_ = intrinsics(0, 0) / reprojection_error_std_dev;
  rectified_error_multiplier_y_ = intrinsics(1, 1) / reprojection_error_std_dev;
}

}  // namespace vslam_types_refactor