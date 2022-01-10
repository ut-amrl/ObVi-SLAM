#ifndef UT_VSLAM_ODOMETRY_COST_FUNCTOR_H
#define UT_VSLAM_ODOMETRY_COST_FUNCTOR_H

#include <ceres/autodiff_cost_function.h>
#include <vslam_types.h>
#include <vslam_util.h>

#include <eigen3/Eigen/Dense>

namespace vslam_solver {

/**
 * Cost functor for odometry constraints.
 */
class OdometryCostFunctor {
 public:
  /**
   * Constructor.
   *
   * @param factor Odometry factor that the cost should be based on.
   */
  OdometryCostFunctor(const vslam_types::OdometryFactor& factor)
      : R_odom(factor.rotation),
        T_odom(factor.translation),
        sqrt_information(factor.sqrt_information) {}

  /**
   * Compute the residual for the odometry constraint.
   *
   * @tparam T                  Type that the cost functor is evaluating.
   * @param pose_i[in]      Robot's pose in the world frame corresponding to
   *                        the first node This is a 6 entry array with the
   *                        first 3 entries corresponding to the translation
   *                        and the second 3 entries containing the axis-angle
   *                        representation (with angle given by the magnitude
   *                        of the vector).
   * @param pose_j[in]      Robot's pose in the world frame corresponding to
   *                        the location of the second feature. This is a 6
   *                        entry array with the first 3 entries corresponding
   *                        to the translation and the second 3 entries
   *                        containing the axis-angle representation (with
   *                        angle given by the magnitude of the vector).
   * @param residual[out]   Residual giving the error. Contains 6 entries
   *                        (1 per pose dimension). First 3 are translation,
   *                        second 3 are rotation.
   *
   * @return True if the residual was computed successfully, false otherwise.
   */
  template <typename T>
  bool operator()(const T* pose_i, const T* pose_j, T* residual) const {
    // Predicted pose_j = pose_i * odometry.
    // Hence, error = pose_j.inverse() * pose_i * odometry;
    typedef Eigen::Matrix<T, 3, 1> Vector3T;
    typedef Eigen::Matrix<T, 3, 3> Matrix3T;

    const Matrix3T Ri =
        vslam_util::Exp(Vector3T(pose_i[3], pose_i[4], pose_i[5]));
    const Matrix3T Rj =
        vslam_util::Exp(Vector3T(pose_j[3], pose_j[4], pose_j[5]));

    const Vector3T Ti(pose_i[0], pose_i[1], pose_i[2]);
    const Vector3T Tj(pose_j[0], pose_j[1], pose_j[2]);

    // Extract the error in translation.
    const Vector3T error_translation =
        Rj.transpose() * (Ri * T_odom.cast<T>() - (Tj - Ti));

    residual[0] = error_translation[0];
    residual[1] = error_translation[1];
    residual[2] = error_translation[2];

    // Extract the error in rotation in angle-axis form.
    const Matrix3T error_rotation_matrix =
        Rj.transpose() * Ri * R_odom.cast<T>();
    const Vector3T error_rotation = vslam_util::Log(error_rotation_matrix);

    residual[3] = error_rotation[0];
    residual[4] = error_rotation[1];
    residual[5] = error_rotation[2];

    Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals(residual);

    residuals.applyOnTheLeft(sqrt_information.template cast<T>());

    return true;
  }

  /**
   * Create a cost function for the given odometry factor.
   *
   * @param factor Odometry factor encoding relative pose and uncertainty
   * between two nodes.
   *
   * @return Cost function that generates a cost for two robot poses determined
   * by how well they align with the odometry data.
   */
  static ceres::AutoDiffCostFunction<OdometryCostFunctor, 6, 6, 6>* create(
      const vslam_types::OdometryFactor& factor) {
    OdometryCostFunctor* residual = new OdometryCostFunctor(factor);
    return new ceres::AutoDiffCostFunction<OdometryCostFunctor, 6, 6, 6>(
        residual);
  }

  /**
   * Rotation measurement.
   */
  const Eigen::Matrix3f R_odom;

  /**
   * Translation measurement.
   */
  const Eigen::Vector3f T_odom;

  /**
   * Sqrt information matrix (sqrt of inv. of covariance).
   */
  const Eigen::Matrix<double, 6, 6> sqrt_information;
};
}  // namespace vslam_solver

#endif  // UT_VSLAM_ODOMETRY_COST_FUNCTOR_H
