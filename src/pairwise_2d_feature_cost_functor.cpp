#include <pairwise_2d_feature_cost_functor.h>

namespace vslam_solver {
Pairwise2dFeatureCostFunctor::Pairwise2dFeatureCostFunctor(
    const Eigen::Vector2f &image_1_feature,
    const Eigen::Vector2f &image_2_feature,
    const vslam_types::CameraIntrinsics &intrinsics_feature_1,
    const vslam_types::CameraExtrinsics &extrinsics_feature_1,
    const vslam_types::CameraIntrinsics &intrinsics_feature_2,
    const vslam_types::CameraExtrinsics &extrinsics_feature_2,
    const double &epipolar_error_std_dev)
    : feature_1_image_coords_(
          ConvertFromPixelToImage(image_1_feature, intrinsics_feature_1)),
      feature_2_image_coords_(
          ConvertFromPixelToImage(image_2_feature, intrinsics_feature_2)),
      epipolar_error_std_dev_(epipolar_error_std_dev),
      cam_to_robot_tf_feature_1_(
          Eigen::Translation3f(extrinsics_feature_1.translation) *
          extrinsics_feature_1.rotation),
      cam_to_robot_tf_feature_2_(
          Eigen::Translation3f(extrinsics_feature_2.translation) *
          extrinsics_feature_2.rotation) {}

}  // namespace vslam_solver