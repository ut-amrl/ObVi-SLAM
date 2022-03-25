//
// Created by amanda on 3/23/22.
//

#include <ellipsoid_utils.h>

namespace vslam_util {
std::pair<Eigen::Vector2f, Eigen::Vector2f> getCornerLocationsPair(
    const vslam_types::EllipsoidEstimate &ellispoid_est,
    const vslam_types::RobotPose &robot_pose,
    const vslam_types::CameraExtrinsics &cam_extrinsics,
    const vslam_types::CameraIntrinsics &cam_intrinsics) {
  Eigen::Vector4d corner_locations_raw;
  vslam_types::EllipsoidEstimateNode ellipsoid_node =
      vslam_util::FromEllipsoidEstimate(ellispoid_est);
  double *ellipsoid_pose_ptr = ellipsoid_node.pose;
  vslam_types::SLAMNode robot_pose_node = vslam_util::FromRobotPose(robot_pose);
  double *robot_pose_ptr = robot_pose_node.pose;

  Eigen::Affine3d robot_to_cam_tf =
      Eigen::Affine3f(Eigen::Translation3f(cam_extrinsics.translation) *
                      cam_extrinsics.rotation)
          .inverse()
          .cast<double>();
  Eigen::Matrix3d intrinsics_mat = cam_intrinsics.camera_mat.cast<double>();

  getCornerLocationsVector(ellipsoid_pose_ptr,
                           robot_pose_ptr,
                           robot_to_cam_tf,
                           intrinsics_mat,
                           corner_locations_raw);
  return cornerLocationsVectorToPair<float>(corner_locations_raw.cast<float>());
}
}  // namespace vslam_util