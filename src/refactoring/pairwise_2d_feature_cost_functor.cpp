#include <refactoring/factors/pairwise_2d_feature_cost_functor.h>

namespace vslam_types_refactor {
Pairwise2dFeatureCostFunctor::Pairwise2dFeatureCostFunctor(
    const vslam_types_refactor::PixelCoord<double> &image_1_feature,
    const vslam_types_refactor::PixelCoord<double> &image_2_feature,
    const vslam_types_refactor::CameraIntrinsicsMat<double>
        &intrinsics_feature_1,
    const vslam_types_refactor::CameraExtrinsics<double> &extrinsics_feature_1,
    const vslam_types_refactor::CameraIntrinsicsMat<double>
        &intrinsics_feature_2,
    const vslam_types_refactor::CameraExtrinsics<double> &extrinsics_feature_2,
    const double &epipolar_error_std_dev)
    : feature_1_image_coords_(
          ConvertFromPixelToImage(image_1_feature, intrinsics_feature_1)),
      feature_2_image_coords_(
          ConvertFromPixelToImage(image_2_feature, intrinsics_feature_2)),
      epipolar_error_std_dev_(epipolar_error_std_dev),
      cam_to_robot_tf_feature_1_(
          Eigen::Translation3d(extrinsics_feature_1.transl_) *
          extrinsics_feature_1.orientation_),
      cam_to_robot_tf_feature_2_(
          Eigen::Translation3d(extrinsics_feature_2.transl_) *
          extrinsics_feature_2.orientation_) {}

}  // namespace vslam_types_refactor