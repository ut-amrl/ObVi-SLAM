//
// Created by amanda on 6/21/22.
//

#ifndef UT_VSLAM_VSLAM_TYPES_MATH_UTIL_H
#define UT_VSLAM_VSLAM_TYPES_MATH_UTIL_H

#include <refactoring/types/vslam_basic_types_refactor.h>

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

template <typename NumType>
Eigen::Vector3d getPositionRelativeToPose(const Pose3D<NumType>& pose,
                                          const Position3d<NumType>& position) {
  Transform6Dof<NumType> pose_1_mat = convertToAffine(pose);
  return pose_1_mat.inverse() * position;
}

template <typename NumType>
Pose3D<NumType> combinePoses(const Pose3D<NumType>& pose_1,
                             const Pose3D<NumType>& pose_2_rel_to_1) {
  Transform6Dof<NumType> pose_1_mat = convertToAffine(pose_1);
  Transform6Dof<NumType> pose_2_rel_to_1_mat = convertToAffine(pose_2_rel_to_1);
  Transform6Dof<NumType> pose_2_mat = pose_1_mat * pose_2_rel_to_1_mat;
  return convertAffineToPose3D(pose_2_mat);
}
}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_VSLAM_TYPES_MATH_UTIL_H
