#include <refactoring/factors/bounding_box_factor.h>

#include <unsupported/Eigen/MatrixFunctions>

namespace vslam_types_refactor {

BoundingBoxFactor::BoundingBoxFactor(
    const vslam_types_refactor::CameraIntrinsicsMat<double> &intrinsics,
    const vslam_types_refactor::CameraExtrinsics<double> &extrinsics,
    const vslam_types_refactor::BbCorners<double> &corner_pixel_locations,
    const vslam_types_refactor::Covariance<double, 4>
        &corner_detections_covariance)
    : corner_detections_(corner_pixel_locations),
      sqrt_inf_mat_bounding_box_corners_(
          corner_detections_covariance.inverse().sqrt()),
      camera_intrinsics_mat_(intrinsics),
      robot_to_cam_tf_(
          (Eigen::Translation3d(extrinsics.transl_) * extrinsics.orientation_)
              .inverse()) {}

}  // namespace vslam_types_refactor