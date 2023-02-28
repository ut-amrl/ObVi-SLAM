//
// Created by amanda on 12/21/20.
//

#ifndef UT_VSLAM_POSE_REPS_H
#define UT_VSLAM_POSE_REPS_H

#include <glog/logging.h>
#include <shared/util/random.h>

#include <eigen3/Eigen/Dense>

namespace pose {
typedef std::pair<Eigen::Vector2d, double> Pose2d;
typedef std::pair<Eigen::Vector3d, Eigen::Quaterniond> Pose3d;

inline std::ostream &operator<<(std::ostream &os, const Pose2d &pose) {
  os << pose.first.x() << ", " << pose.first.y() << ", " << pose.second;
  return os;
}

inline Pose2d createPose2d(const double &x,
                           const double &y,
                           const double &theta) {
  return std::make_pair(Eigen::Vector2d(x, y), theta);
}

inline Eigen::Affine3d convertPoseToAffine(const Pose3d &pose) {
  Eigen::Affine3d mat;
  mat.translation() = pose.first;
  mat.linear() = pose.second.toRotationMatrix();
  return mat;
}

inline Eigen::Affine2d convertPoseToAffine(const Pose2d &pose) {
  Eigen::Affine2d mat;
  mat.translation() = pose.first;
  mat.linear() = Eigen::Rotation2Dd(pose.second).toRotationMatrix();
  return mat;
}

inline Pose3d convertAffineToPose(const Eigen::Affine3d &mat) {
  return std::make_pair(mat.translation(), Eigen::Quaterniond(mat.linear()));
}

inline Pose2d convertAffineToPose(const Eigen::Affine2d &mat) {
  Eigen::Rotation2Dd rotation(mat.linear());
  return std::make_pair(mat.translation(), rotation.angle());
}

inline Pose2d invertPose(const Pose2d &pose) {
  return convertAffineToPose(convertPoseToAffine(pose).inverse());
}

inline Pose3d invertPose(const Pose3d &pose) {
  return convertAffineToPose(convertPoseToAffine(pose).inverse());
}

inline Eigen::Vector3d transformPoint(
    const pose::Pose3d &transform_to_target_frame,
    const Eigen::Vector3d &point_to_transform) {
  return convertPoseToAffine(transform_to_target_frame) * point_to_transform;
}

inline Eigen::Vector2d transformPoint(
    const pose::Pose2d &transform_to_target_frame,
    const Eigen::Vector2d &point_to_transform) {
  return convertPoseToAffine(transform_to_target_frame) * point_to_transform;
}

inline Pose3d getPoseOfObj1RelToObj2(const Pose3d &obj1, const Pose3d &obj2) {
  Eigen::Affine3d affine_1 = convertPoseToAffine(obj1);
  Eigen::Affine3d affine_2 = convertPoseToAffine(obj2);
  Eigen::Affine3d combined_affine = affine_2.inverse() * affine_1;
  return convertAffineToPose(combined_affine);
}

inline Pose2d getPoseOfObj1RelToObj2(const Pose2d &obj1, const Pose2d &obj2) {
  Eigen::Affine2d affine_1 = convertPoseToAffine(obj1);
  Eigen::Affine2d affine_2 = convertPoseToAffine(obj2);
  Eigen::Affine2d combined_affine = affine_2.inverse() * affine_1;
  return convertAffineToPose(combined_affine);
}

inline Pose3d toPose3d(const Pose2d &pose_2d, const double &z = 0) {
  Eigen::Vector3d transl(pose_2d.first.x(), pose_2d.first.y(), z);
  Eigen::Quaterniond rot;
  rot = Eigen::AngleAxisd(pose_2d.second, Eigen::Vector3d::UnitZ());
  return std::make_pair(transl, rot);
}

inline Eigen::Vector3d toEulerAngles(const Eigen::Quaterniond &q) {
  // roll (x-axis rotation)
  double sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
  double cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
  double roll = std::atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  double sinp = 2 * (q.w() * q.y() - q.z() * q.x());
  double pitch;
  if (std::abs(sinp) >= 1)
    pitch = std::copysign(M_PI / 2, sinp);  // use 90 degrees if out of range
  else
    pitch = std::asin(sinp);

  // yaw (z-axis rotation)
  double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
  double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
  double yaw = std::atan2(siny_cosp, cosy_cosp);

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

inline Pose2d toPose2d(const Pose3d &pose_3d) {
  Eigen::Vector3d euler_angles = toEulerAngles(pose_3d.second);
  double yaw = euler_angles[2];
  if ((abs(euler_angles[0]) > 0.2) || (abs(euler_angles[1]) > 0.2)) {
    LOG(INFO) << "Roll " << euler_angles[0];
    LOG(INFO) << "Pitch " << euler_angles[1];
    LOG(INFO) << "Yaw " << euler_angles[2];
  }
  pose::Pose2d pose =
      pose::createPose2d(pose_3d.first.x(), pose_3d.first.y(), yaw);
  return pose;
}

inline Eigen::Vector3d toPoint3d(const Eigen::Vector2d &point2d,
                                 const double &z = 0) {
  return Eigen::Vector3d(point2d.x(), point2d.y(), z);
}

/**
 * Get the pose of object 2 in the frame that pose 1 is relative to.
 *
 * Ex. if pose_1 is in the map frame and pose_2 is relative to pose_1, this
 * returns the position of the coordinate frame for pose 2 in the map frame.
 *
 * @param pose_1
 * @param pose_2
 * @return
 */
inline Pose3d combinePoses(const Pose3d &pose_1, const Pose3d &pose_2) {
  Eigen::Affine3d affine_1 = convertPoseToAffine(pose_1);
  Eigen::Affine3d affine_2 = convertPoseToAffine(pose_2);
  Eigen::Affine3d combined_affine = affine_1 * affine_2;
  return convertAffineToPose(combined_affine);
}

inline Pose2d combinePoses(const Pose2d &pose_1, const Pose2d &pose_2) {
  Eigen::Affine2d affine_1 = convertPoseToAffine(pose_1);
  Eigen::Affine2d affine_2 = convertPoseToAffine(pose_2);
  Eigen::Affine2d combined_affine = affine_1 * affine_2;
  return convertAffineToPose(combined_affine);
}

inline pose::Pose2d addGaussianNoise(const pose::Pose2d &original_pose_2d,
                                     const double &x_std_dev,
                                     const double &y_std_dev,
                                     const double &theta_std_dev,
                                     util_random::Random &rand_gen) {
  return std::make_pair(
      Eigen::Vector2d(rand_gen.Gaussian(original_pose_2d.first.x(), x_std_dev),
                      rand_gen.Gaussian(original_pose_2d.first.y(), y_std_dev)),
      rand_gen.Gaussian(original_pose_2d.second, theta_std_dev));
}

inline pose::Pose2d addGaussianNoise(const pose::Pose2d &original_pose_2d,
                                     const Eigen::Vector3d &variances,
                                     util_random::Random &rand_gen) {
  return std::make_pair(
      Eigen::Vector2d(
          rand_gen.Gaussian(original_pose_2d.first.x(), sqrt(variances(0))),
          rand_gen.Gaussian(original_pose_2d.first.y(), sqrt(variances(1)))),
      rand_gen.Gaussian(original_pose_2d.second, sqrt(variances(2))));
}

inline pose::Pose2d addRelativeGaussianNoise(
    const pose::Pose2d &original_pose_2d,
    const double &x_std_dev,
    const double &y_std_dev,
    const double &theta_std_dev,
    util_random::Random &rand_gen) {
  double scaled_x_std_dev = original_pose_2d.first.x() * x_std_dev;
  double scaled_y_std_dev = original_pose_2d.first.y() * y_std_dev;
  double scaled_yaw_std_dev = original_pose_2d.second * theta_std_dev;

  return addGaussianNoise(original_pose_2d,
                          scaled_x_std_dev,
                          scaled_y_std_dev,
                          scaled_yaw_std_dev,
                          rand_gen);
}

inline std::pair<pose::Pose2d, Eigen::Vector3d>
addPositionRelativeGaussianNoiseAndGetVariance(
    const pose::Pose2d &original_pose_2d,
    const Eigen::Vector3d &variances,
    util_random::Random &rand_gen) {
  double x_std_dev = sqrt(variances(0));
  double y_std_dev = sqrt(variances(1));
  double yaw_std_dev = sqrt(variances(2));
  double scaled_x_std_dev = original_pose_2d.first.x() * x_std_dev;
  double scaled_y_std_dev = original_pose_2d.first.y() * y_std_dev;

  Eigen::Vector3d new_variances(
      pow(scaled_x_std_dev, 2), pow(scaled_y_std_dev, 2), variances(2));

  return std::make_pair(addGaussianNoise(original_pose_2d,
                                         scaled_x_std_dev,
                                         scaled_y_std_dev,
                                         yaw_std_dev,
                                         rand_gen),
                        new_variances);
}

}  // namespace pose
#endif  // UT_VSLAM_POSE_REPS_H
