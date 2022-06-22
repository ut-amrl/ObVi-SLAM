#ifndef UT_VSLAM_REFACTORING_PAIRWISE_2D_FEATURE_COST_FUNCTOR_H
#define UT_VSLAM_REFACTORING_PAIRWISE_2D_FEATURE_COST_FUNCTOR_H

#include <ceres/autodiff_cost_function.h>
#include <refactoring/types/vslam_basic_types_refactor.h>
#include <refactoring/types/vslam_math_util.h>

#include <eigen3/Eigen/Dense>

namespace vslam_types_refactor {

/**
 * Convert from pixel coords to image coords.
 *
 * @param pixel_coord   Pixel coordinate
 * @param intrinsics    Camera intrinsics.
 *
 * @return  Image coordinates for the point.
 */
static Eigen::Vector3d ConvertFromPixelToImage(
    const Eigen::Vector2d &pixel_coord,
    const vslam_types_refactor::CameraIntrinsicsMat<double> &intrinsics) {
  Eigen::Vector3d pixel_coord_homogeneous(
      pixel_coord.x(), pixel_coord.y(), 1.0);
  return intrinsics.inverse() * pixel_coord_homogeneous;
}

/**
 * Cost functor that adds a residual for Gaussian-distributed epipolar error.
 */
class Pairwise2dFeatureCostFunctor {
 public:
  /**
   * Constructor.
   *
   * @param image_1_feature         Pixel location of the feature in image 1.
   * @param image_2_feature         Pixel location of the feature in image 2.
   * @param intrinsics_feature_1    Camera intrinsics for the first feature.
   * @param extrinsics_feature_1    Camera extrinsics (pose of the camera
   *                                relative to the robot) for the first
   *                                feature.
   * @param intrinsics_feature_2    Camera intrinsics for the second feature.
   * @param extrinsics_feature_2    Camera extrinsics (pose of the camera
   *                                relative to the robot) for the second
   *                                feature.
   * @param epipolar_error_std_dev  Standard deviation of the epipolar error.
   */
  Pairwise2dFeatureCostFunctor(
      const vslam_types_refactor::PixelCoord<double> &image_1_feature,
      const vslam_types_refactor::PixelCoord<double> &image_2_feature,
      const vslam_types_refactor::CameraIntrinsicsMat<double>
          &intrinsics_feature_1,
      const vslam_types_refactor::CameraExtrinsics<double>
          &extrinsics_feature_1,
      const vslam_types_refactor::CameraIntrinsicsMat<double>
          &intrinsics_feature_2,
      const vslam_types_refactor::CameraExtrinsics<double>
          &extrinsics_feature_2,
      const double &epipolar_error_std_dev);

  /**
   * Compute the residual for the epipolar error using the essential matrix.
   *
   * @tparam T                  Type that the cost functor is evaluating.
   * @param pose_init[in]       Robot's pose in the world frame corresponding to
   *                            the location of the first feature. This is a 6
   *                            entry array with the first 3 entries
   *                            corresponding to the translation and the second
   *                            3 entries containing the axis-angle
   *                            representation (with angle given by the
   *                            magnitude of the vector).
   * @param pose_current[in]    Robot's pose in the world frame corresponding to
   *                            the location of the second feature. This is a 6
   *                            entry array with the first 3 entries
   *                            corresponding to the translation and the second
   *                            3 entries containing the axis-angle
   *                            representation (with angle given by the
   *                            magnitude of the vector).
   * @param residual[out]       Residual giving the error. Contains 1 entry.
   *
   * @return True if the residual was computed successfully, false otherwise.
   */
  template <typename T>
  bool operator()(const T *pose_init,
                  const T *pose_current,
                  T *residual) const {
    Eigen::Matrix<T, 3, 3> essential_mat =
        vslam_types_refactor::CalcEssentialMatrix(
            pose_init,
            pose_current,
            cam_to_robot_tf_feature_1_.cast<T>(),
            cam_to_robot_tf_feature_2_.cast<T>());

    Eigen::Matrix<T, 1, 1> epipolar_error_unscaled =
        feature_1_image_coords_.transpose().cast<T>() * essential_mat *
        feature_2_image_coords_.cast<T>();

    Eigen::Matrix<T, 1, 1> essential_mat_scale =
        feature_1_image_coords_.transpose().cast<T>() *
        essential_mat.transpose() * essential_mat *
        feature_1_image_coords_.cast<T>();

    residual[0] =
        epipolar_error_unscaled(0, 0) /
        (T(epipolar_error_std_dev_) * (sqrt(essential_mat_scale(0, 0))));

    return true;
  }

  /**
   * Create the autodiff cost function with this cost functor.
   *
   * @param intrinsics              Camera intrinsics.
   * @param extrinsics              Camera extrinsics (provide camera pose
   *                                relative to robot).
   * @param intrinsics_feature_1    Camera intrinsics for the first feature.
   * @param extrinsics_feature_1    Camera extrinsics (pose of the camera
   *                                relative to the robot) for the first
   *                                feature.
   * @param intrinsics_feature_2    Camera intrinsics for the second feature.
   * @param extrinsics_feature_2    Camera extrinsics (pose of the camera
   *                                relative to the robot) for the second
   *                                feature.
   * @param epipolar_error_std_dev  Standard deviation of the epipolar error
   *                                (assuming this is normally distributed).
   *
   * @return Ceres cost function.
   */
  static ceres::AutoDiffCostFunction<Pairwise2dFeatureCostFunctor, 1, 6, 6> *
  create(const vslam_types_refactor::CameraIntrinsicsMat<double>
             &intrinsics_feature_1,
         const vslam_types_refactor::CameraExtrinsics<double>
             &extrinsics_feature_1,
         const vslam_types_refactor::CameraIntrinsicsMat<double>
             &intrinsics_feature_2,
         const vslam_types_refactor::CameraExtrinsics<double>
             &extrinsics_feature_2,
         const vslam_types_refactor::PixelCoord<double> &frame_1_feature_loc,
         const vslam_types_refactor::PixelCoord<double> &frame_2_feature_loc,
         const double &epipolar_error_std_dev) {
    Pairwise2dFeatureCostFunctor *residual =
        new Pairwise2dFeatureCostFunctor(frame_1_feature_loc,
                                         frame_2_feature_loc,
                                         intrinsics_feature_1,
                                         extrinsics_feature_1,
                                         intrinsics_feature_2,
                                         extrinsics_feature_2,
                                         epipolar_error_std_dev);
    return new ceres::
        AutoDiffCostFunction<Pairwise2dFeatureCostFunctor, 1, 6, 6>(residual);
  }

 private:
  /**
   * Feature 1 in image space (not pixel), have already applied camera
   * intrinsics to this.
   */
  Eigen::Vector3d feature_1_image_coords_;

  /**
   * Feature 2 in image space (not pixel), have already applied camera
   * intrinsics to this.
   */
  Eigen::Vector3d feature_2_image_coords_;

  /**
   * Transform that provides the camera position in the robot's frame for the
   * camera that captured the first feature.
   */
  Eigen::Affine3d cam_to_robot_tf_feature_1_;

  /**
   * Transform that provides the camera position in the robot's frame for the
   * camera that captured the second feature.
   */
  Eigen::Affine3d cam_to_robot_tf_feature_2_;

  /**
   * Epipolar error standard deviation.
   */
  double epipolar_error_std_dev_;
};
}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_REFACTORING_PAIRWISE_2D_FEATURE_COST_FUNCTOR_H
