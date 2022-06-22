#include <refactoring/factors/reprojection_cost_functor.h>

namespace vslam_types_refactor {

ReprojectionCostFunctor::ReprojectionCostFunctor(
    const vslam_types_refactor::PixelCoord<double> &image_feature,
    const vslam_types_refactor::CameraIntrinsicsMat<double> &intrinsics,
    const vslam_types_refactor::CameraExtrinsics<double> &extrinsics,
    const double &reprojection_error_std_dev)
    : image_feature_(image_feature),
      intrinsics_(intrinsics),
      reprojection_error_std_dev_(reprojection_error_std_dev),
      cam_to_robot_tf_(Eigen::Translation3d(extrinsics.transl_) *
                       extrinsics.orientation_) {}

}  // namespace vslam_solver