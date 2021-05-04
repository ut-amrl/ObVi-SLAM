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
    Eigen::Transform<T, 3, Eigen::Affine> cam_to_robot_tf) {
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
      cam_to_robot_tf.inverse() * first_robot_pose_in_world.inverse() *
      second_robot_pose_in_world * cam_to_robot_tf;

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
 *imperfect odometry measuremets
 *
 * @param sigma_linear[in]        Standard deviation of the linear pose
 *                                  dimensions (dx, dy, dz)
 * @param sigma_rotations[in]     Standard deviation of the rotational pose
 *                                  dimensions (dx, dy, dz) in so(3) space
 * @param poses[out]                Ground truth robot poses to add noise to
 */
template <typename T>
void CorruptRobotPoses(const Eigen::Matrix<T, 3, 1>& sigma_linear,
                       const Eigen::Matrix<T, 3, 1>& sigma_rotation,
                       std::vector<vslam_types::RobotPose>& poses) {
  std::default_random_engine generator;
  std::normal_distribution<T> distribution(0.0, 1.0);

  for (auto& pose : poses) {
    // Corrupt linear dimension
    Eigen::Matrix<float, 3, 1> dl(sigma_linear.x() * distribution(generator),
                                  sigma_linear.y() * distribution(generator),
                                  sigma_linear.z() * distribution(generator));
    pose.loc += dl;
    // Corrupt orientation dimension
    Eigen::Matrix<T, 3, 1> dw(sigma_rotation.x() * distribution(generator),
                              sigma_rotation.y() * distribution(generator),
                              sigma_rotation.z() * distribution(generator));
    Eigen::Matrix<T, 3, 3> R = pose.angle.toRotationMatrix().cast<T>();
    PerturbRotationMatrix(dw, R);
    Eigen::Matrix<float, 3, 3> Rf =
        R.template cast<float>();  // TODO this is ugly - we just need to
                                   // template EVERYTHING and make nothing
                                   // explicitly a float
    pose.angle = Eigen::AngleAxisf(Rf);
  }
  return;
}

void SaveKITTIPoses(const std::string& filename,
                    const std::vector<vslam_types::RobotPose>& poses);

}  // namespace vslam_util

#endif  // UT_VSLAM_SLAM_UTIL_H