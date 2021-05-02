#ifndef UT_VSLAM_SLAM_UTIL_H
#define UT_VSLAM_SLAM_UTIL_H

#include <vslam_types.h>

#include <random>
#include <vector>

#include "eigen3/Eigen/Dense"

namespace {
const double kEpsilon = 1e-7;
}  // namespace

namespace vslam_util {

/**
 * Calculate the essential matrix given a relative transform
 *
 * @param rel_tf[in]    Relative transfrom between two poses
 *
 * @return  Essential matrix corresponding to the realtive transform
 */
template <typename T>
void CalcEssentialMatrix(const Eigen::Transform<T, 3, Eigen::Affine>& rel_tf,
                         const Eigen::Matrix<T, 3, 3>& essential_mat);

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

template <typename T>
Eigen::Matrix<T, 3, 3> Identity() {
  return Eigen::Matrix<T, 3, 3>::Identity();
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
    return (Identity<T>() + s);
  }
  // Rodrigues' formula.
  return (Identity<T>() + sin(theta) / theta * s +
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
    return (Identity<T>() - T(0.5) * e_S);
  } else {
    return (Identity<T>() - (T(1.0) - cos(th)) / (th * th) * e_S +
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

}  // namespace vslam_util

#endif  // UT_VSLAM_SLAM_UTIL_H