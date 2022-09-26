//
// Created by amanda on 6/21/22.
//

#ifndef UT_VSLAM_VSLAM_TYPES_MATH_UTIL_H
#define UT_VSLAM_VSLAM_TYPES_MATH_UTIL_H

#include <glog/logging.h>
#include <refactoring/types/vslam_basic_types_refactor.h>

namespace vslam_types_refactor {

/**
 * Timestamp related
 */
uint64_t timestampToMillis(const Timestamp &timestamp) {
  return timestamp.first * 1000 + (timestamp.second / 1e6);
}

/**
 * Pose3D related
 */

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
 * Pose2D related
 * Adopted from base_lib/pose_reps.h
 * need to verify correctness
 */

template <typename NumType>
Pose2D<NumType> createPose2D(const NumType& x, const NumType&y, const NumType& theta) {
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
  return Pose2D<NumType>(mat.translation(), rotation.angle());
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
bool posesAlmostSame(const Pose2D<NumType>& p1, const Pose2D<NumType>& p2, const NumType& transl_tol, const NumType& angle_tol) {
  Pose2D<NumType>& rel_pose = getPoseOfObj1RelToObj2(p1, p2);

  if (rel_pose.first.norm() > transl_tol) {
    return false;
  }
  return abs(rel_pose.second) <= angle_tol;
}

template <typename NumType>
Pose2D<NumType> combinePoses(const Pose2D<NumType>& pose1, const Pose2D<NumType>& pose2) {
  Eigen::Transform<NumType, 2, Eigen::Affine> affine1 = convertPoseToAffine(pose1);
  Eigen::Transform<NumType, 2, Eigen::Affine> affine2 = convertPoseToAffine(pose2);
  Eigen::Transform<NumType, 2, Eigen::Affine> combinedAffine = affine1 * affine2;
  return convertAffineToPose(combinedAffine);
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

  return Eigen::Matrix<NumType, 3, 1>(roll, pitch, yaw);
}

template <typename NumType>
Pose2D<NumType> toPose2D(const Pose3D<NumType> &pose3d) {
  Eigen::Matrix<NumType, 3, 1> euler_angles = toEulerAngles(Eigen::Quaternion<NumType>(pose3d.orientation_));
  NumType yaw = euler_angles[2];
  if ((abs(euler_angles[0]) > 0.2) || (abs(euler_angles[1]) > 0.2)) {

    LOG(INFO) << "Roll " << euler_angles[0];
    LOG(INFO) << "Pitch " << euler_angles[1];
    LOG(INFO) << "Yaw " << euler_angles[2];
  }
  Pose2D<NumType>  pose = createPose2D(pose3d.transl_[0], pose3d.transl_[1], yaw);
  return pose;
}

/**
 * @brief Ignore z
 * 
 * @tparam NumType 
 * @param pose1 
 * @param pose2 
 * @param targetTime 
 * @return Pose2D<NumType> 
 */
template <typename NumType>
Pose2D<NumType> interpolatePosesIn2D(
  const std::pair<Timestamp, Pose2D<NumType>>& stampedPose1, 
  const std::pair<Timestamp, Pose2D<NumType>>& stampedPose2, 
  const Timestamp& targetTime) {

  Pose2D<NumType> relPose = getPoseOfObj1RelToObj2(stampedPose2.second, stampedPose1.second);
  uint64_t pose1Millis  = timestampToMillis(stampedPose1.first);
  uint64_t pose2Millis  = timestampToMillis(stampedPose2.first);
  uint64_t targetMillis = timestampToMillis(targetTime);
  NumType fraction = ((NumType) (targetMillis - pose1Millis)) / (pose2Millis - pose1Millis);
  
  Pose2D<NumType> relPoseInterpolated;
  relPoseInterpolated = createPose2D(relPose.transl_[0]*fraction, relPose.transl_[1]*fraction, relPose.orientation_*fraction);
  if (abs(relPose.orientation_) > 1e-10) {
    NumType radius = sqrt(relPose.transl_.squaredNorm() / (2 * (1 - cos(relPose.orientation_))));
    NumType x = radius * sin(abs(relPose.orientation_) * fraction);
    NumType y = radius - (radius * cos(relPose.orientation_ * fraction));
    if (relPose.orientation_ < 0) { y = -y; }
    if (relPose.transl_[0] < 0) {
      y = -y;
      x = -x;
    }
    relPoseInterpolated = createPose2D(x, y, fraction*relPose.orientation_);
  }
  return combinePoses(stampedPose1.second, relPoseInterpolated);
}

template <typename NumType>
Pose2D<NumType> interpolatePosesIn2D(
  const Timestamp& t1, const Pose2D<NumType>& pose1,
  const Timestamp& t2, const Pose2D<NumType>& pose2,
  const Timestamp& targetTime) {

  Pose2D<NumType> relPose = getPoseOfObj1RelToObj2(pose2, pose1);
  uint64_t pose1Millis  = timestampToMillis(t1);
  uint64_t pose2Millis  = timestampToMillis(t2);
  uint64_t targetMillis = timestampToMillis(targetTime);
  NumType fraction = ((NumType) (targetMillis - pose1Millis)) / (pose2Millis - pose1Millis);
  Pose2D<NumType> relPoseInterpolated;
  relPoseInterpolated = createPose2D(relPose.transl_[0]*fraction, relPose.transl_[1]*fraction, relPose.orientation_*fraction);
  if (abs(relPose.orientation_) > 1e-10) {
    NumType radius = sqrt(relPose.transl_.squaredNorm() / (2 * (1 - cos(relPose.orientation_))));
    NumType x = radius * sin(abs(relPose.orientation_) * fraction);
    NumType y = radius - (radius * cos(relPose.orientation_ * fraction));
    if (relPose.orientation_ < 0) { y = -y; }
    if (relPose.transl_[0] < 0) {
      y = -y;
      x = -x;
    }
    relPoseInterpolated = createPose2D(x, y, fraction*relPose.orientation_);
  }
  return combinePoses(pose1, relPoseInterpolated);
}

template <typename NumType>
Pose3D<NumType> toPose3D(const Pose2D<NumType>& pose2d, const NumType& z = 0) {
  Eigen::Matrix<NumType, 3, 1> unitZ;
  unitZ << (NumType)0.0, (NumType)0.0, (NumType)1.0;

  Pose3D<NumType> pose3d;
  pose3d.transl_ << pose2d.transl_.x(), pose2d.transl_.y(), z;
  pose3d.orientation_ = Eigen::AngleAxis<NumType>(pose2d.orientation_, unitZ);
  return pose3d;
}

template <typename NumType>
Pose3D<NumType> interpolatePosesIn2D(
  const std::pair<Timestamp, Pose3D<NumType>>& stampedPose1, 
  const std::pair<Timestamp, Pose3D<NumType>>& stampedPose2, 
  const Timestamp& targetTime) {
  return toPose3D(
    interpolatePosesIn2D(
      std::pair<Timestamp, Pose2D<NumType>>(stampedPose1.first,  toPose2D(stampedPose1.second)), 
      std::pair<Timestamp, Pose2D<NumType>>(stampedPose2.second, toPose2D(stampedPose2.second)), 
      targetTime));
}

template <typename NumType>
Pose3D<NumType> interpolatePosesIn2D(
  const Timestamp& t1, const Pose3D<NumType>& pose1,
  const Timestamp& t2, const Pose3D<NumType>& pose2,
  const Timestamp& targetTime) {

  // Pose2D<NumType> pose2D1, pose2D2,retPose2D;  
  // Pose3D<NumType> retPose;

  // pose2D1 = toPose2D(pose1);
  // pose2D2 = toPose2D(pose2);
  // retPose2D = interpolatePosesIn2D(t1, toPose2D(pose1), t2, toPose2D(pose2), targetTime);
  // retPose = toPose3D(retPose2D);
  // return retPose;
  return toPose3D(interpolatePosesIn2D(t1, toPose2D(pose1), t2, toPose2D(pose2), targetTime));
}

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_VSLAM_TYPES_MATH_UTIL_H
