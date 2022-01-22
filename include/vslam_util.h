#ifndef UT_VSLAM_SLAM_UTIL_H
#define UT_VSLAM_SLAM_UTIL_H

#include <vslam_types.h>

#include <random>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "vslam_types.h"

namespace {
const double kEpsilon = 1e-7;
}  // namespace

namespace vslam_util {

/**
 * Calculate the essential matrix for the camera at two robot poses.
 * @tparam T                Type of the number in the computation. Needed to
 *                          work with ceres optimizatoin.
 * @param node_1_pose_array First robot pose, as an array. First 3 elements are
 *                          position, second 3 are axis angle format for
 *                          rotation.
 * @param node_2_pose_array Second robot pose, as an array. First 3 elements are
 *                          position, second 3 are axis angle format for
 *                          rotation.
 * @param cam_to_robot_tf   Transform that provides the camera position in the
 *                          robot's frame.
 *
 * @return The essential matrix that relates pixels in the image frames.
 */
template <typename T>
Eigen::Matrix<T, 3, 3> CalcEssentialMatrix(
    const T* node_1_pose_array,
    const T* node_2_pose_array,
    Eigen::Transform<T, 3, Eigen::Affine> cam_to_robot_tf_node_1,
    Eigen::Transform<T, 3, Eigen::Affine> cam_to_robot_tf_node_2) {
  // Convert pose init to rotation
  Eigen::Transform<T, 3, Eigen::Affine> first_robot_pose_in_world =
      vslam_types::PoseArrayToAffine(&(node_1_pose_array[3]),
                                     &(node_1_pose_array[0]));

  // Convert pose current to rotation
  Eigen::Transform<T, 3, Eigen::Affine> second_robot_pose_in_world =
      vslam_types::PoseArrayToAffine(&(node_2_pose_array[3]),
                                     &(node_2_pose_array[0]));

  // Want to get rotation and translation of camera frame 1 relative to
  // camera frame 2 (pose of camera 2 in camera 1)
  // if T_r_c is the transformation matrix representing camera extrinsics
  // (camera pose rel robot). We assume this is the same for both poses
  // since the camera doesn't change relative to the robot as the robot moves
  // T_w_r1 is the robot's pose at frame 1 relative to the world
  // T_w_r2 is the robot's pose at frame 2 relative to the world, then camera
  // 2 relative to camera 1 is given by
  // T_c1_c2 =  T_r_c^-1 * T_w_r1^-1 * T_w_r2 * T_r_c
  Eigen::Transform<T, 3, Eigen::Affine> cam_1_to_cam_2_mat =
      cam_to_robot_tf_node_1.inverse() * first_robot_pose_in_world.inverse() *
      second_robot_pose_in_world * cam_to_robot_tf_node_2;

  // Extract Tx and R from cam_1_to_cam_2_mat
  Eigen::Matrix<T, 3, 1> t_vec = cam_1_to_cam_2_mat.translation();
  Eigen::Matrix<T, 3, 3> t_cross;  // skew symmetric
  t_cross << T(0.0), -t_vec(2), t_vec(1), t_vec(2), T(0.0), -t_vec(0),
      -t_vec(1), t_vec(0), T(0.0);
  Eigen::Matrix<T, 3, 3> rotation = cam_1_to_cam_2_mat.linear();
  Eigen::Matrix<T, 3, 3> essential_mat = t_cross * rotation;
  return essential_mat;
}
/**
 * Generate the skew symmetric matrix form of a vector
 *
 * @param w[in]   A 3x1 vector
 *
 * @return  A 3x3 skew symmetric representation of the vector
 */
template <typename T>
Eigen::Matrix<T, 3, 3> SkewSymmetric(const Eigen::Matrix<T, 3, 1>& w) {
  Eigen::Matrix<T, 3, 3> m;
  m << T(0), -w.z(), w.y(), w.z(), T(0), -w.x(), -w.y(), w.x(), T(0);
  return m;
}

/**
 * Produce a 3x1 vector from a matrix - it doesnt check if the matrix was
 * actually skew symmetric
 *
 * @param w[in]   A 3x3 skew symmetric matrix
 *
 * @return  A 3x1 vector representation of the skew symmetric matrix
 */
template <typename T>
Eigen::Matrix<T, 3, 1> FromSkewSymmetric(const Eigen::Matrix<T, 3, 3>& s) {
  Eigen::Matrix<T, 3, 1> w(s(2, 1), s(0, 2), s(1, 0));
  return w;
}

/**
 * Convert SO(3) to so(3)
 *
 * @param R[in]     SO(3) representation of rotation (3x3 matrix)
 * @return          so(3) representation of rotation matrix (3x1 column vector)
 */

template <typename T>
Eigen::Matrix<T, 3, 1> Log(const Eigen::Matrix<T, 3, 3>& R) {
  const T cos_theta = T(0.5) * (R.trace() - T(1.0));
  if (cos_theta > T(1.0 - kEpsilon)) {
    // Small-angle approximation.
    const Eigen::Matrix<T, 3, 3> s = T(0.5) * (R - R.transpose());
    return FromSkewSymmetric(s);
  }
  // Inverse Rodrigues' formula.
  const T theta = acos(cos_theta);
  const Eigen::Matrix<T, 3, 3> s =
      theta / (T(2.0) * sin(theta)) * (R - R.transpose());
  return FromSkewSymmetric(s);
}

/**
 * Convert so(3) to SO(3)
 *
 * @param w[in]     so(3) representation of rotation matrix (3x1 column vector)
 * @return          SO(3) representation of rotation (3x3 matrix)
 */
template <typename T>
Eigen::Matrix<T, 3, 3> Exp(const Eigen::Matrix<T, 3, 1>& w) {
  const T theta = w.norm();
  const Eigen::Matrix<T, 3, 3> s = SkewSymmetric(w);
  if (theta < T(kEpsilon)) {
    // Small-angle approximation.
    return (Eigen::Matrix<T, 3, 3>::Identity() + s);
  }
  // Rodrigues' formula.
  return (Eigen::Matrix<T, 3, 3>::Identity() + sin(theta) / theta * s +
          (T(1.0) - cos(theta)) / (theta * theta) * s * s);
}

/**
 * Calculates Rodrigues Jacobian - J_r(Log(R)) where Log(): SO(3) -> so(3)
 *
 * @param w[in]     so(3) representation of rotation (3x1 column vector)
 * @return          Rodrigues Jacobian (3x3 matrix)
 */

template <typename T>
Eigen::Matrix<T, 3, 3> GetRodriguesJacobian(const Eigen::Matrix<T, 3, 1>& w) {
  Eigen::Matrix<T, 3, 3> e_S = SkewSymmetric(w);
  const T th = w.norm();
  if (th < T(kEpsilon)) {
    return (Eigen::Matrix<T, 3, 3>::Identity() - T(0.5) * e_S);
  } else {
    return (Eigen::Matrix<T, 3, 3>::Identity() -
            (T(1.0) - cos(th)) / (th * th) * e_S +
            (th - sin(th)) / (th * th * th) * e_S * e_S);
  }
}

/**
 * Perturbs a rotation matrix by the noise dw represented in so(3)
 *
 * @param dw[in]        Perturbation in so(3)
 * @param R[in,out]     Perturbed rotation in SO(3)
 */
template <typename T>
void PerturbRotationMatrix(const Eigen::Matrix<T, 3, 1>& dw,
                           Eigen::Matrix<T, 3, 3>& R) {
  // R_perturb = Exp(Log(R) + dw)
  //           = Exp(w + dw)
  //           = Exp(w)*Exp(J_r(w) * dw)
  //           = R * Exp(J_r(Log(R)) * dw)
  Eigen::Matrix<T, 3, 1> J_dw =
      GetRodriguesJacobian(Log(R)) * dw;  // TODO remove this intermediate step
  R = R * Exp(J_dw);
}

/**
 * Add noise to robot poses - Can be used to corrupt groundtruth poses into
 * imperfect odometry measuremets - simulates odometry!
 *
 * @param odom_alphas[in]   Odom alpha 1 and 3 as described :
 * https://wiki.ros.org/amcl
 * @param poses[out]   Ground truth robot poses to add noise to
 */
template <typename T>
void CorruptRobotPoses(const Eigen::Matrix<T, 2, 1>& odom_alphas,
                       std::vector<vslam_types::RobotPose>& poses) {
  std::default_random_engine generator;
  std::normal_distribution<T> distribution(0.0, 1.0);

  std::vector<vslam_types::RobotPose> gt_poses = poses;
  for (int i = 1; i < gt_poses.size(); ++i) {
    // Frame 1
    Eigen::Transform<float, 3, Eigen::Affine> first_robot_pose_in_world =
        Eigen::Translation<float, 3>(gt_poses[i - 1].loc) *
        gt_poses[i - 1].angle;
    // Frame 2
    Eigen::Transform<float, 3, Eigen::Affine> second_robot_pose_in_world =
        Eigen::Translation<float, 3>(gt_poses[i].loc) * gt_poses[i].angle;
    // Relative transform from frame 2 to 1
    Eigen::Transform<float, 3, Eigen::Affine> rel_tf =
        first_robot_pose_in_world.inverse() * second_robot_pose_in_world;

    // Corrupt linear dimension proportional to linear displacement
    Eigen::Matrix<float, 3, 1> dl(fabs(rel_tf.translation().x()) *
                                      odom_alphas(1) * distribution(generator),
                                  fabs(rel_tf.translation().y()) *
                                      odom_alphas(1) * distribution(generator),
                                  fabs(rel_tf.translation().z()) *
                                      odom_alphas(1) * distribution(generator));
    rel_tf.translation() += dl;
    // Corrupt orientation dimension
    Eigen::AngleAxis<T> rel_rotation(rel_tf.linear().cast<T>());
    Eigen::Matrix<T, 3, 1> dw(
        fabs(rel_rotation.angle() * rel_rotation.axis().x()) * odom_alphas(0) *
            distribution(generator),
        fabs(rel_rotation.angle() * rel_rotation.axis().y()) * odom_alphas(0) *
            distribution(generator),
        fabs(rel_rotation.angle() * rel_rotation.axis().z()) * odom_alphas(0) *
            distribution(generator));
    Eigen::Matrix<T, 3, 3> R = rel_tf.linear().cast<T>();
    PerturbRotationMatrix(dw, R);
    rel_tf.linear() = R.template cast<float>();

    // Out most recent noise pose
    Eigen::Transform<float, 3, Eigen::Affine> corrupted_robot_pose_in_world =
        Eigen::Translation<float, 3>(poses[i - 1].loc) * poses[i - 1].angle;

    // Add noisy relative tf to our most recent noisy pose
    Eigen::Transform<float, 3, Eigen::Affine>
        next_corrupted_robot_pose_in_world =
            corrupted_robot_pose_in_world * rel_tf;

    poses[i].loc = next_corrupted_robot_pose_in_world.translation();
    poses[i].angle = next_corrupted_robot_pose_in_world.linear();
  }

  return;
}

/**
 * Corrupt the feature pose by drawing from a gaussian centered around the
 * feature pose.
 *
 * @param sigma_linear          Standard deviations for the gaussian.
 * @param feature_init_pose     Initial feature pose (uncorrupted).
 *
 * @return Corrupted feature pose.
 */
Eigen::Vector3d CorruptFeature(const Eigen::Vector3d& sigma_linear,
                               Eigen::Vector3d& feature_init_pose);

/**
 * Adjust trajectory so that the first pose is at the origin and all
 * other poses are adjusted to maintain the same transform to the first pose.
 *
 * @param original_trajectory[in]   Original trajectory.
 * @param adjusted_trajectory[out]  Trajectory adjusted to have the first pose
 *                                  at the origin.
 */
void AdjustTrajectoryToStartAtZero(
    const std::vector<vslam_types::RobotPose>& original_trajectory,
    std::vector<vslam_types::RobotPose>& adjusted_trajectory);

void SaveKITTIPoses(const std::string& filename,
                    const std::vector<vslam_types::RobotPose>& poses);

/**
 * Get the pose of pose 2 (provided in frame A) relative to pose 1 (also
 * provided in frame A).
 *
 * @param pose_1    Pose 1 -- pose that we want to be the frame for in the
 *                  returned pose.
 * @param pose_2    Pose 2 -- pose that we want to get location of relative to
 * pose 1.
 *
 * @return Relative transform that provides the location of pose 2 relative to
 * pose 1.
 */
vslam_types::RobotPose getPose2RelativeToPose1(
    const vslam_types::RobotPose& pose_1, const vslam_types::RobotPose& pose_2);

/**
 * Get the position of the given position (provided in frame A) relative to
 * pose 1 (also provided in frame A).
 *
 * @param pose_1    Pose 1 -- pose that we want to be the frame for in the
 *                  returned pose.
 * @param position  Position that we want to get relative to pose 1
 *
 * @return Relative position that provides the location of the given position
 * relative to pose 1.
 */
Eigen::Vector3d getPositionRelativeToPose(const vslam_types::RobotPose& pose_1,
                                          const Eigen::Vector3d& position);

/**
 * Combine the given poses. I.e. if pose 1 is in frame A and pose 2 is in the
 * frame of pose 1, then this will return pose 2 relative to frame A.
 *
 * Uses the frame id for pose 2 in constructing the new pose.
 *
 * @param pose_1    Pose 1 -- pose 2 is relative to this.
 * @param pose_2    Pose 2 -- relative to pose 1, but we want to get it in the
 * same frame as pose 1.
 *
 * @return Relative transform that provides the location of pose 2 relative to
 * the frame that pose 1 is in.
 */
vslam_types::RobotPose combinePoses(const vslam_types::RobotPose& pose_1,
                                    const vslam_types::RobotPose& pose_2);

/**
 * Create a diagonal covariance matrix from a vector of standard deviations.
 *
 * @tparam N Number of components in the vector.
 *
 * @param std_devs Standard deviations for each component.
 *
 * @return Covariance matrix created using the standard deviations to form the
 * diagonal.
 */
template <int N>
Eigen::Matrix<float, N, N> createDiagCovFromStdDevs(
    const Eigen::Matrix<float, N, 1>& std_devs) {
  Eigen::Matrix<float, N, 1> variances = std_devs.array().pow(2);

  Eigen::DiagonalMatrix<float, N> cov(variances);
  return cov;
}
}  // namespace vslam_util

#endif  // UT_VSLAM_SLAM_UTIL_H