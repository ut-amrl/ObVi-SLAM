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

/**
 * @brief Adopted from base_lib/pose_reps.h
 * need to verify correctness
 */

template <typename NumType>
Pose2D<NumType> createPose2D(const NumType& x, const NumType&y, const double& theta) {
  return Pose2D<NumType>(Eigen::Matrix<NumType, 2, 1>(x, y), theta);
}

template <typename NumType>
Eigen::Transform<NumType, 2, Eigen::Affine> convertPoseToAffine(const Pose2D<NumType> &pose) {
  Eigen::Transform<NumType, 2, Eigen::Affine> mat;
  mat.translation() = pose.transl_;
  mat.linear() = Eigen::Rotation2D<NumType>(pose.orientation_).toRotationMatrix();
  return mat;
}

template <typename NumType>
Pose2D<NumType> convertAffineToPose(const Eigen::Transform<NumType, 2, Eigen::Affine> &mat) {
  Eigen::Rotation2D<NumType> rotation(mat.linear());
  return Pose2D<NumType>(mat.translation,rotation.angle());
}

template <typename NumType>
Pose2D<NumType> getPoseOfObj1RelToObj2(const Pose2D<NumType>& obj1, const Pose2D<NumType>& obj2) {
  Eigen::Transform<NumType, 2, Eigen::Affine> affine1 = convertPoseToAffine(obj1);
  Eigen::Transform<NumType, 2, Eigen::Affine> affine2 = convertPoseToAffine(obj2);
  Eigen::Transform<NumType, 2, Eigen::Affine> combinedAffine = affine2.inverse() * affine1;
  return convertAffineToPose(combinedAffine);
}

template <typename NumType>
bool posesSame(const Pose2D<NumType>& p1, const Pose2D<NumType>& p2) {
  Pose2D<NumType>& rel_pose = getPoseOfObj1RelToObj2(p1, p2);
  return ((rel_pose.first.norm() == 0) && (rel_pose.second == 0));
}

template <typename NumType>
bool posesAlmostSame(const Pose2D<NumType>& p1, const Pose2D<NumType>& p2, const double& transl_tol, const double& angle_tol) {
  Pose2D<NumType>& rel_pose = getPoseOfObj1RelToObj2(p1, p2);

  if (rel_pose.first.norm() > transl_tol) {
    return false;
  }
  return abs(rel_pose.second) <= angle_tol;
}

template <typename NumType>
Eigen::Matrix<NumType, 3, 1> toEulerAngles(const Eigen::Quaternion<NumType> &q) {

  // roll (x-axis rotation)
  NumType sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
  NumType cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
  NumType roll = std::atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  NumType sinp = 2 * (q.w() * q.y() - q.z() * q.x());
  NumType pitch;
  if (std::abs(sinp) >= 1)
      pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  else
      pitch = std::asin(sinp);

  // yaw (z-axis rotation)
  NumType siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
  NumType cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
  NumType yaw = std::atan2(siny_cosp, cosy_cosp);

  if (roll < -M_PI_2) {
      roll -= M_PI;
      yaw -= M_PI;
      pitch = -pitch;
  } else if (roll > M_PI_2) {
      roll += M_PI;
      yaw += M_PI;
      pitch = -pitch;
  }

  return Eigen::Vector3d(roll, pitch, yaw);
}

template <typename NumType>
Pose2D<NumType> toPose2D(const Pose3D<NumType> &pose3d) {
  Eigen::Matrix<NumType, 3, 1> euler_angles = toEulerAngles(Eigen::Quaternion<NumType>(pose3d.orientation_));
  double yaw = euler_angles[2];
  if ((abs(euler_angles[0]) > 0.2) || (abs(euler_angles[1]) > 0.2)) {

    LOG(INFO) << "Roll " << euler_angles[0];
    LOG(INFO) << "Pitch " << euler_angles[1];
    LOG(INFO) << "Yaw " << euler_angles[2];
  }
  Pose2D<NumType>  pose = createPose2D(pose3d.transl_[0], pose3d.transl_[1], yaw);
  return pose;
}

template <typename NumType>
Pose3D<NumType> toPose3D(const Pose2D<NumType>& pose2d, const NumType& z = 0) {
  Pose3D<NumType> pose3d;
  pose3d.transl_ << pose2d.first.x(), pose2d.first.y(), z;
  pose3d.orientation_ =  Eigen::AngleAxis<NumType>(pose2d.second, Eigen::Vector3d::UnitZ());
  return pose3d;
}

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_VSLAM_TYPES_MATH_UTIL_H
