//
// Created by amanda on 6/9/22.
//

#ifndef UT_VSLAM_VSLAM_TYPES_CONVERSION_H
#define UT_VSLAM_VSLAM_TYPES_CONVERSION_H

#include <refactoring/types/vslam_basic_types_refactor.h>
#include <refactoring/types/vslam_math_util.h>

namespace vslam_types_refactor {

template <typename NumType>
void convertPoseToArray(const Pose3D<NumType> &pose,
                        RawPose3d<NumType> &raw_pose) {
  raw_pose[0] = pose.transl_.x();
  raw_pose[1] = pose.transl_.y();
  raw_pose[2] = pose.transl_.z();
  raw_pose[3] = pose.orientation_.axis().x() * pose.orientation_.angle();
  raw_pose[4] = pose.orientation_.axis().y() * pose.orientation_.angle();
  raw_pose[5] = pose.orientation_.axis().z() * pose.orientation_.angle();
}

template <typename NumType>
RawPose3d<NumType> convertPoseToArray(const Pose3D<NumType> &pose) {
  RawPose3d<NumType> raw_pose;
  convertPoseToArray(pose, raw_pose);
  return raw_pose;
}

template <typename NumType>
void extractPosition(const RawPose3d<NumType> &raw_pose,
                     Position3d<NumType> &position) {
  position(0) = raw_pose[0];
  position(1) = raw_pose[1];
  position(2) = raw_pose[2];
}

template <typename NumType>
Position3d<NumType> extractPosition(const RawPose3d<NumType> &raw_pose) {
  Position3d<NumType> position;
  position(0) = raw_pose[0];
  position(1) = raw_pose[1];
  position(2) = raw_pose[2];
  return position;
}

template <typename NumType>
void extractOrientation(const RawPose3d<NumType> &raw_pose,
                        Orientation3D<NumType> &orientation) {
  Eigen::Matrix<NumType, 3, 1> orientation_vector = raw_pose.bottomRows(3);
  VectorToAxisAngle(orientation_vector, orientation);
}

template <typename NumType>
Orientation3D<NumType> extractOrientation(const RawPose3d<NumType> &raw_pose) {
  Orientation3D<NumType> orientation;
  extractOrientation(raw_pose, orientation);
  return orientation;
}

template <typename NumType>
void convertToPose3D(const RawPose3d<NumType> &raw_pose,
                     Pose3D<NumType> &pose) {
  extractPosition(raw_pose, pose.transl_);
  extractOrientation(raw_pose, pose.orientation_);
}

template <typename NumType>
Pose3D<NumType> convertToPose3D(const RawPose3d<NumType> &raw_pose) {
  Pose3D<NumType> pose;
  convertToPose3D(raw_pose, pose);
  return pose;
}

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_VSLAM_TYPES_CONVERSION_H
