#include <reprojection_cost_functor.h>

namespace vslam_solver {

ReprojectionCostFunctor::ReprojectionCostFunctor(
    const Eigen::Vector2f &image_feature,
    const Eigen::Vector3f &point,
    const vslam_types::CameraIntrinsics &intrinsics,
    const vslam_types::CameraExtrinsics &extrinsics,
    const double &reprojection_error_std_dev)
    : image_feature_(image_feature),
      point_(point),
      intrinsics_(intrinsics),
      reprojection_error_std_dev_(reprojection_error_std_dev),
      cam_to_robot_tf_(Eigen::Translation3f(extrinsics.translation) *
                       extrinsics.rotation) {}

}  // namespace vslam_solver