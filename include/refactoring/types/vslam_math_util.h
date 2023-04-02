#ifndef UT_VSLAM_VSLAM_MATH_UTIL_H
#define UT_VSLAM_VSLAM_MATH_UTIL_H

#include <glog/logging.h>

#include <eigen3/Eigen/Dense>
#include <random>
#include <unsupported/Eigen/MatrixFunctions>
#include <vector>

namespace vslam_types_refactor {
namespace {

/**
 * Threshold for a small angle when creating a axis-angle representation.
 */
const double kSmallAngleThreshold = 1e-8;
}  // namespace

/**
 * Convert from a vector that stores the axis-angle representation (with
 * angle as the magnitude of the vector) to the Eigen AxisAngle
 * representation.
 *
 * @tparam T                    Type of each field.
 * @param axis_angle_vec[in]    Vector encoding the axis of rotation (as the
 *                              direction) and the angle as the magnitude of the
 *                              vector.
 * @param angle_axis[out]       Eigen AxisAngle representation for the rotation.
 */
template <typename T>
void VectorToAxisAngle(const Eigen::Matrix<T, 3, 1>& axis_angle_vec,
                       Eigen::AngleAxis<T>& angle_axis) {
  const T rotation_angle = axis_angle_vec.norm();
  if (rotation_angle > kSmallAngleThreshold) {
    angle_axis =
        Eigen::AngleAxis<T>(rotation_angle, axis_angle_vec / rotation_angle);
  } else {
    angle_axis =
        Eigen::AngleAxis<T>(T(0), Eigen::Matrix<T, 3, 1>{T(1), T(0), T(0)});
  }
}

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
  Eigen::AngleAxis<T> angle_axis;
  VectorToAxisAngle(axis_angle_vec, angle_axis);
  return angle_axis;
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
      PoseArrayToAffine(&(node_1_pose_array[3]), &(node_1_pose_array[0]));

  // Convert pose current to rotation
  Eigen::Transform<T, 3, Eigen::Affine> second_robot_pose_in_world =
      PoseArrayToAffine(&(node_2_pose_array[3]), &(node_2_pose_array[0]));

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
 * WARNING: This does not handle some rotations well. Trace can evaluate to
 * number outside of [-1, 1] which cases cos_theta to be NaN. Recommend using
 * Eigen's convert from rotation to AngleAxis and then obtaining the so(3)
 * representation by multiplying the resultant angle with the axis instead.
 *
 *
 * @param R[in]     SO(3) representation of rotation (3x3 matrix)
 * @return          so(3) representation of rotation matrix (3x1 column vector)
 */
template <typename T>
Eigen::Matrix<T, 3, 1> Log(const Eigen::Matrix<T, 3, 3>& R) {
  if (R.trace() == T(-1)) {
    LOG(WARNING) << "Singularity condition";
  }
  const T cos_theta = T(0.5) * (R.trace() - T(1.0));
  if (cos_theta > T(1.0 - kSmallAngleThreshold)) {
    // Small-angle approximation.
    const Eigen::Matrix<T, 3, 3> s = T(0.5) * (R - R.transpose());
    if (s.hasNaN()) {
      LOG(WARNING) << "Small angle Log has nan";
      LOG(WARNING) << cos_theta;
      LOG(WARNING) << R.trace();
      LOG(WARNING) << R;
      LOG(WARNING) << s;
    }
    return FromSkewSymmetric(s);
  }
  // Inverse Rodrigues' formula.
  const T theta = acos(cos_theta);
  const Eigen::Matrix<T, 3, 3> s =
      theta / (T(2.0) * sin(theta)) * (R - R.transpose());
  if (s.hasNaN()) {
    LOG(WARNING) << "Normal angle Log has nan";
    LOG(WARNING) << cos_theta;

    LOG(WARNING) << (cos_theta > 1.0);
    LOG(WARNING) << (cos_theta < -1.0);
    LOG(WARNING) << R.trace();
    LOG(WARNING) << theta;
    LOG(WARNING) << sin(theta);
    LOG(WARNING) << R;
    LOG(WARNING) << s;
  }
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
  if (theta < T(kSmallAngleThreshold)) {
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
  if (th < T(kSmallAngleThreshold)) {
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
template <typename NumType, int N>
Eigen::Matrix<NumType, N, N> createDiagCovFromStdDevs(
    const Eigen::Matrix<NumType, N, 1>& std_devs,
    const NumType& min_std_dev = 0) {
  NumType min_variance = pow(min_std_dev, 2);
  Eigen::Matrix<NumType, N, 1> variances = std_devs.array().pow(2);
  for (int i = 0; i < N; i++) {
    variances(i) = std::max(min_variance, variances(i));
  }

  Eigen::DiagonalMatrix<NumType, N> cov(variances);
  return cov;
}

template <typename T>
void getProjectedPixelLocation(
    const T* pose,
    const T* point,
    const Eigen::Transform<T, 3, Eigen::Affine>& cam_to_robot_tf_,
    const Eigen::Matrix<T, 3, 3>& intrinsics_,
    Eigen::Matrix<T, 2, 1>& pixel_results) {
  // Transform from world to current robot pose
  Eigen::Transform<T, 3, Eigen::Affine> world_to_robot_current =
      vslam_types_refactor::PoseArrayToAffine(&(pose[3]), &(pose[0])).inverse();
  //  LOG(INFO) << "World to robot current " << world_to_robot_current.matrix();

  // Point in world frame
  Eigen::Matrix<T, 3, 1> point_world(point[0], point[1], point[2]);
  //  LOG(INFO) << "Point world " << point_world;

  // Transform the point from global coordinates to frame of current pose.
  Eigen::Matrix<T, 3, 1> point_current =
      cam_to_robot_tf_.inverse() * world_to_robot_current * point_world;
  //  LOG(INFO) << "Point " << point_current;

  // Project the 3  D point into the current image.
  if (point_current.z() < T(0)) {
    //    LOG(WARNING) << "Point projected onto camera was behind the image
    //    plane."
    //                    " This should not happen";
    // TODO should we sleep or exit here to make this error more evident?
  }
  pixel_results.x() =
      intrinsics_(0, 0) * point_current.x() / point_current.z() +
      intrinsics_(0, 2);
  pixel_results.y() =
      intrinsics_(1, 1) * point_current.y() / point_current.z() +
      intrinsics_(1, 2);
}

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_VSLAM_MATH_UTIL_H