#ifndef UT_VSLAM_SLAM_MATH_UTIL_H
#define UT_VSLAM_SLAM_MATH_UTIL_H

#include <random>
#include <vector>

#include <eigen3/Eigen/Dense>
#include <glog/logging.h>
#include <unsupported/Eigen/MatrixFunctions>

namespace {
const double kEpsilon = 1e-7;

/**
 * Threshold for a small angle when creating a axis-angle representation.
 */
const double kSmallAngleThreshold = 1e-8;
}  // namespace

namespace vslam_util {

/**
 * Convert from a vector that stores the axis-angle representation (with
 * angle as the magnitude of the vector) to the Eigen AxisAngle
 * representation.
 *
 * @tparam T                Type of each field.
 * @param axis_angle_vec    Vector encoding the axis of rotation (as the
 *                          direction) and the angle as the magnitude of the
 *                          vector.
 *
 * @return Eigen AxisAngle representation for the rotation.
 */
template <typename T>
Eigen::AngleAxis<T> VectorToAxisAngle(
    const Eigen::Matrix<T, 3, 1> axis_angle_vec) {
  const T rotation_angle = axis_angle_vec.norm();
  if (rotation_angle > kSmallAngleThreshold) {
    return Eigen::AngleAxis<T>(rotation_angle, axis_angle_vec / rotation_angle);
  } else {
    return Eigen::AngleAxis<T>(T(0), Eigen::Matrix<T, 3, 1>{T(1), T(0), T(0)});
  }
}

/**
 * Create an Eigen Affine transform from the rotation and translation.
 *
 * @tparam T            Type to use in the matrix.
 * @param rotation      Three entry array containing the axis-angle form of
 * the rotation. Magnitude gives the angle of the rotation and the direction
 * gives the axis of rotation.
 * @param translation   Three entry array containing the translation.
 *
 * @return Eigen Affine transform for the rotation and translation.
 */
template <typename T>
Eigen::Transform<T, 3, Eigen::Affine> PoseArrayToAffine(const T* rotation,
                                                        const T* translation) {
  const Eigen::Matrix<T, 3, 1> rotation_axis(
      rotation[0], rotation[1], rotation[2]);
  const T rotation_angle = rotation_axis.norm();

  Eigen::AngleAxis<T> rotation_aa;
  if (rotation_angle < T(kSmallAngleThreshold)) {
    rotation_aa =
        Eigen::AngleAxis<T>(T(0), Eigen::Matrix<T, 3, 1>(T(0), T(0), T(1)));
  } else {
    rotation_aa = VectorToAxisAngle(rotation_axis);
  }

  const Eigen::Translation<T, 3> translation_tf(
      translation[0], translation[1], translation[2]);
  const Eigen::Transform<T, 3, Eigen::Affine> transform =
      translation_tf * rotation_aa;
  return transform;
}

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
      PoseArrayToAffine(&(node_1_pose_array[3]),
                                     &(node_1_pose_array[0]));

  // Convert pose current to rotation
  Eigen::Transform<T, 3, Eigen::Affine> second_robot_pose_in_world =
      PoseArrayToAffine(&(node_2_pose_array[3]),
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
    const Eigen::Matrix<float, N, 1>& std_devs, const float &min_std_dev = 0) {
  float min_variance = pow(min_std_dev, 2);
  Eigen::Matrix<float, N, 1> variances = std_devs.array().pow(2);
  for (int i = 0; i < N; i++) {
    variances(i) = std::max(min_variance, variances(i));
  }

  Eigen::DiagonalMatrix<float, N> cov(variances);
  return cov;
}
}  // namespace vslam_util

#endif  // UT_VSLAM_SLAM_MATH_UTIL_H