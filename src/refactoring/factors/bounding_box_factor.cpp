#include <refactoring/factors/bounding_box_factor.h>

#include <unsupported/Eigen/MatrixFunctions>

namespace vslam_types_refactor {

BoundingBoxFactor::BoundingBoxFactor(
    const double &invalid_ellipse_error,
    const vslam_types_refactor::CameraIntrinsicsMat<double> &intrinsics,
    const vslam_types_refactor::CameraExtrinsics<double> &extrinsics,
    const vslam_types_refactor::BbCorners<double> &corner_pixel_locations,
    const vslam_types_refactor::Covariance<double, 4>
        &corner_detections_covariance,
    const std::optional<ObjectId> &obj_id,
    const std::optional<FrameId> &frame_id,
    const std::optional<CameraId> &camera_id,
    const bool &debug)
    : invalid_ellipse_error_(invalid_ellipse_error),
      corner_detections_(corner_pixel_locations),
      sqrt_inf_mat_bounding_box_corners_(
          corner_detections_covariance.inverse().sqrt()),
      camera_intrinsics_mat_(intrinsics),
      robot_to_cam_tf_(
          (Eigen::Translation3d(extrinsics.transl_) * extrinsics.orientation_)
              .inverse()),
      obj_id_(obj_id),
      frame_id_(frame_id),
      camera_id_(camera_id),
      debug_(debug) {}

}  // namespace vslam_types_refactor