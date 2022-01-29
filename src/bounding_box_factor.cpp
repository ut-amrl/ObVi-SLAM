#include <bounding_box_factor.h>

#include <unsupported/Eigen/MatrixFunctions>

namespace vslam_solver {

BoundingBoxFactor::BoundingBoxFactor(
    const vslam_types::CameraIntrinsics &intrinsics,
    const vslam_types::CameraExtrinsics &extrinsics,
    const Eigen::Vector4f &corner_pixel_locations,
    const Eigen::Matrix4f &corner_detections_covariance)
    : corner_detections_(corner_pixel_locations),
      sqrt_inf_mat_bounding_box_corners_(
          corner_detections_covariance.inverse().sqrt()),
      camera_intrinsics_mat_(intrinsics.camera_mat),
      robot_to_cam_tf_((Eigen::Translation3f(extrinsics.translation) *
                       extrinsics.rotation).inverse()) {}

}  // namespace vslam_solver