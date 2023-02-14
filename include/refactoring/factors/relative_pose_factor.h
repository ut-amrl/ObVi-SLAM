//
// Created by amanda on 2/13/23.
//

#ifndef UT_VSLAM_RELATIVE_POSE_FACTOR_H
#define UT_VSLAM_RELATIVE_POSE_FACTOR_H

#include <ceres/autodiff_cost_function.h>
#include <refactoring/types/vslam_basic_types_refactor.h>
#include <refactoring/types/vslam_math_util.h>

namespace vslam_types_refactor {

class RelativePoseFactor {
 public:
  RelativePoseFactor(const Pose3D<double> &measured_pose_deviation,
                     const Covariance<double, 6> &pose_deviation_cov);

  /**
   * Compute the residual for the bounding box observation.
   *
   * @tparam T                      Type that the cost functor is evaluating.
   * @param robot_pose_before[in]   Robot's estimate pose in the world frame
   *                                before moving.
   * @param robot_pose_after[in]    Robot's estimate pose in the world frame
   *                                after moving.
   * @param residuals_ptr[out]      Residual giving the error. Contains 6
   *                                entries.
   *
   * @return True if the residual was computed successfully, false otherwise.
   */
  template <typename T>
  bool operator()(const T *robot_pose_before,
                  const T *robot_pose_after,
                  T *residuals_ptr) const {
    // Transform from world to current robot pose
    Eigen::Transform<T, 3, Eigen::Affine> world_to_robot_before =
        PoseArrayToAffine(&(robot_pose_before[3]), &(robot_pose_before[0]));
    Eigen::Transform<T, 3, Eigen::Affine> world_to_robot_after =
        PoseArrayToAffine(&(robot_pose_after[3]), &(robot_pose_after[0]));

    Eigen::Transform<T, 3, Eigen::Affine> after_rel_before =
        world_to_robot_before.inverse() * world_to_robot_after;
    Eigen::Matrix<T, 6, 1> unscaled_residuals;

    unscaled_residuals.template block<3, 1>(0, 0) =
        after_rel_before.translation() - measured_translation_.cast<T>();

    Eigen::Matrix<T, 3, 3> rotation_error =
        after_rel_before.linear() * measured_rotation_change_.inverse();

    // TODO is this right?
    unscaled_residuals.template block<3, 1>(3, 0) = Log(rotation_error);

    Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals(residuals_ptr);
    residuals = sqrt_inf_mat_rel_pose_.template cast<T>() * unscaled_residuals;
    return true;
  }

  /**
   * Create the autodiff cost function with this cost functor.
   *
   * @param object_detection            Object detection.
   * @param camera_intrinsics           Camera intrinsics.
   * @param camera_extrinsics           Camera extrinsics (gives pose of camera
   *                                    relative to robot base link).
   * @param bounding_box_covariance     Covariance of bounding box measurements.
   *
   * @return Ceres cost function.
   */
  static ceres::AutoDiffCostFunction<RelativePoseFactor, 6, 6, 6> *
  createRelativePoseFactor(const Pose3D<double> &measured_pose_deviation,
                           const Covariance<double, 6> &pose_deviation_cov) {
    RelativePoseFactor *factor =
        new RelativePoseFactor(measured_pose_deviation, pose_deviation_cov);
    return new ceres::AutoDiffCostFunction<RelativePoseFactor, 6, 6, 6>(factor);
  }

 private:
  Position3d<double> measured_translation_;

  Eigen::Matrix3d measured_rotation_change_;

  /**
   * Square root of the relative pose information matrix (inverse of
   * covariance).
   *
   * Let this value be A. To get the covariance C, C = (A * A)^-1.
   */
  Eigen::Matrix<double, 6, 6> sqrt_inf_mat_rel_pose_;
};

}  // namespace vslam_types_refactor
#endif  // UT_VSLAM_RELATIVE_POSE_FACTOR_H
