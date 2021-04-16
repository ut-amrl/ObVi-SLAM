#include <pairwise_2d_feature_cost_functor.h>

namespace vslam_solver {
Pairwise2dFeatureCostFunctor::Pairwise2dFeatureCostFunctor(
    const Eigen::Vector2f &image_1_feature,
    const Eigen::Vector2f &image_2_feature,
    const vslam_types::CameraIntrinsics &intrinsics,
    const vslam_types::CameraExtrinsics &extrinsics,
    const double &epipolar_error_std_dev)
    : feature_1_image_coords_(
          ConvertFromPixelToImage(image_1_feature, intrinsics)),
      feature_2_image_coords_(
          ConvertFromPixelToImage(image_2_feature, intrinsics)),
      epipolar_error_std_dev_(epipolar_error_std_dev) {
  // Eigen affine
  cam_to_robot_tf_ =
      Eigen::Translation3f(extrinsics.translation) * extrinsics.rotation;
}

}  // namespace vslam_solver