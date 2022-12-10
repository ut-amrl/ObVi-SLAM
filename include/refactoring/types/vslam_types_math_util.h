//
// Created by amanda on 6/21/22.
//

#ifndef UT_VSLAM_VSLAM_TYPES_MATH_UTIL_H
#define UT_VSLAM_VSLAM_TYPES_MATH_UTIL_H

#include <refactoring/types/vslam_basic_types_refactor.h>
#include <refactoring/types/vslam_math_util.h>

namespace vslam_types_refactor {

template <typename NumType>
Transform6Dof<NumType> convertToAffine(const Pose3D<NumType>& pose_3d) {
  return (Eigen::Translation<NumType, 3>(pose_3d.transl_) *
          pose_3d.orientation_);
}

template <typename NumType>
Pose3D<NumType> convertAffineToPose3D(const Transform6Dof<NumType>& affine) {
  return Pose3D<NumType>(
      affine.translation(),
      Eigen::AngleAxis<NumType>(Eigen::Quaternion<NumType>(affine.linear())));
}

template <typename NumType>
Pose3D<NumType> getPose2RelativeToPose1(const Pose3D<NumType>& pose_1,
                                        const Pose3D<NumType>& pose_2) {
  Transform6Dof<NumType> pose_1_mat = convertToAffine(pose_1);
  Transform6Dof<NumType> pose_2_mat = convertToAffine(pose_2);
  Transform6Dof<NumType> pose_2_rel_to_1_mat =
      pose_1_mat.inverse() * pose_2_mat;
  return convertAffineToPose3D(pose_2_rel_to_1_mat);
}

/**
 * Assuming the pose and position are in the same frame, gets the 3d position of
 * the 'position' variable, relative to the frame of 'pose'. For example, if
 * both pose describes the location of a camera in the world frame and position
 * is the position of a feature in the world frame, this will return the
 * position of the feature relative to the camera.
 * @tparam NumType
 * @param pose
 * @param position
 * @return
 */
template <typename NumType>
Position3d<NumType> getPositionRelativeToPose(
    const Pose3D<NumType>& pose, const Position3d<NumType>& position) {
  Transform6Dof<NumType> pose_1_mat = convertToAffine(pose);
  return pose_1_mat.inverse() * position;
}

template <typename NumType>
Position3d<NumType> combinePoseAndPosition(
    const Pose3D<NumType>& pose, const Position3d<NumType>& position) {
  Transform6Dof<NumType> pose_1_mat = convertToAffine(pose);
  return pose_1_mat * position;
}

template <typename NumType>
Pose3D<NumType> combinePoses(const Pose3D<NumType>& pose_1,
                             const Pose3D<NumType>& pose_2_rel_to_1) {
  Transform6Dof<NumType> pose_1_mat = convertToAffine(pose_1);
  Transform6Dof<NumType> pose_2_rel_to_1_mat = convertToAffine(pose_2_rel_to_1);
  Transform6Dof<NumType> pose_2_mat = pose_1_mat * pose_2_rel_to_1_mat;
  return convertAffineToPose3D(pose_2_mat);
}

template <typename NumType>
Pose3D<NumType> poseInverse(const Pose3D<NumType>& pose_3d) {
  return convertAffineToPose3D(convertToAffine(pose_3d).inverse());
}

template <typename NumType>
Position3d<NumType> pixelToHomogeneous(const PixelCoord<NumType> pixel_coord) {
  return Position3d<NumType>(pixel_coord.x(), pixel_coord.y(), NumType(1));
}

/**
 * Get the position of a feature in the world frame given it's observed depth
 * and pixel coordinate and the extrinsics and robot pose where it was observed.
 *
 * @tparam NumType
 * @param image_feature     Location in the image where the feature was
 *                          observed.
 * @param intrinsics        Intrinsic calibration for the camera that captured
 *                          the feature.
 * @param extrinsics        Extrinsic calibration (where camera is relative to
 *                          robot frame) for the camera that captured the
 * feature.
 * @param robot_pose        Estimated robot pose at which the feature was
 *                          captured.
 * @param depth             Depth of the feature relative to the camera.
 *
 * @return Location of the feature in the world frame (in 3D).
 */
template <typename NumType>
Position3d<NumType> getWorldFramePos(
    const PixelCoord<NumType>& image_feature,
    const CameraIntrinsicsMat<NumType>& intrinsics,
    const CameraExtrinsics<NumType>& extrinsics,
    const Pose3D<NumType>& robot_pose,
    const NumType depth) {
  Position3d<NumType> point_rel_cam =
      depth * intrinsics.inverse() * pixelToHomogeneous(image_feature);
  Pose3D<NumType> cam_pose_in_world = combinePoses(robot_pose, extrinsics);
  return combinePoseAndPosition(cam_pose_in_world, point_rel_cam);
}

template <typename NumType>
PixelCoord<NumType> getProjectedPixelCoord(
    const Position3d<NumType>& feature_pos,
    const Pose3D<NumType>& robot_pose,
    const CameraExtrinsics<NumType>& cam_extrinsics,
    const CameraIntrinsicsMat<NumType>& cam_intrinsics) {
  Position3d<NumType> feature_pos_copy = feature_pos;
  NumType* raw_point = feature_pos_copy.data();
  RawPose3d<NumType> raw_robot_pose = convertPoseToArray(robot_pose);
  NumType* robot_pose_ptr = raw_robot_pose.data();
  PixelCoord<NumType> pixel_coord;
  getProjectedPixelLocation<NumType>(robot_pose_ptr,
                                     raw_point,
                                     convertToAffine(cam_extrinsics),
                                     cam_intrinsics,
                                     pixel_coord);
  return pixel_coord;
}

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_VSLAM_TYPES_MATH_UTIL_H
