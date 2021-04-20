#ifndef UT_VSLAM_PAIRWISE_2D_FEATURE_COST_FUNCTOR_H
#define UT_VSLAM_PAIRWISE_2D_FEATURE_COST_FUNCTOR_H

#include <ceres/autodiff_cost_function.h>
#include <vslam_types.h>

#include <eigen3/Eigen/Dense>

namespace vslam_solver {

/**
 * Cost functor that adds a residual for Gaussian-distributed reprojection
 * error.
 */
class ReprojectionCostFunctor {
 public:
  /**
   * Constructor.
   *
   * @param image_feature               Pixel location of the feature in image
   * @param intrinsics                  Camera intrinsics.
   * @param extrinsics                  Camera extrinsics (pose of the camera
   *                                    relative to the robot).
   * @param reprojection_error_std_dev  Standard deviation of the reprojection
   *                                    error
   */
  ReprojectionCostFunctor(const Eigen::Vector2f &image_feature,
                          const Eigen::Vector3f &point,
                          const vslam_types::CameraIntrinsics &intrinsics,
                          const vslam_types::CameraExtrinsics &extrinsics,
                          const double &reprojection_error_std_dev);

  /**
   * Compute the reprojection error
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
   * @param point[in]           3D position of the imaged feature in the world
   *                            frame
   * @param residual[out]       Residual giving the error. Contains 1 entry.
   *
   * @return True if the residual was computed successfully, false otherwise.
   */
  template <typename T>
  bool operator()(const T *pose_init,
                  const T *pose_current,
                  const T *point,
                  T *residual) const {
    // Transform from world to current robot pose
    Eigen::Transform<T, 3, Eigen::Affine> world_to_robot_current =
        vslam_types::PoseArrayToAffine(&(pose_current[3]), &(pose_current[0]))
            .inverse();

    // Point in world frame
    const Eigen::Vector3f point_world(point[0], point[1], point[2]);

    // Transform the point from global coordinates to frame of current pose.
    const Eigen::Vector3f point_current =
        cam_to_robot_tf_.inverse() * world_to_robot_current * point_world;

    // Project the 3D point into the current image.
    T p_x = T(intrinsics_(0, 0)) * point_current.x() / point_current.z() +
            T(intrinsics(0, 2));
    T p_y = T(intrinsics(1, 1)) * point_current.y() / point_current.z() +
            T(intrinsics(1, 2));

    // Compute the residual.
    residual[0] = p_x - T(image_feature.x());
    residual[1] = p_y - T(image_feature.y());

    return true;
  }

  /**
   * Create the autodiff cost function with this cost functor.
   *
   * @param intrinsics              Camera intrinsics.
   * @param extrinsics              Camera extrinsics (provide camera pose
   *                                relative to robot).
   * @param frame_1_feature         Feature from frame 1.
   * @param frame_2_feature         Feature from frame 2.
   * @param epipolar_error_std_dev  Standard deviation of the epipolar error
   *                                (assuming this is normally distributed).
   *
   * @return Ceres cost function.
   */
  static ceres::AutoDiffCostFunction<ReprojectionCostFunctor, 1, 6, 6> *create(
      const vslam_types::CameraIntrinsics &intrinsics,
      const vslam_types::CameraExtrinsics &extrinsics,
      const vslam_types::VisionFeature &frame_1_feature,
      const vslam_types::VisionFeature &frame_2_feature,
      const double &epipolar_error_std_dev) {
    const Eigen::Vector2f &initial_pixel = frame_1_feature.pixel;
    const Eigen::Vector2f &current_pixel = frame_2_feature.pixel;
    ReprojectionCostFunctor *residual =
        new ReprojectionCostFunctor(initial_pixel,
                                    current_pixel,
                                    intrinsics,
                                    extrinsics,
                                    epipolar_error_std_dev);
    return new ceres::AutoDiffCostFunction<ReprojectionCostFunctor, 1, 6, 6>(
        residual);
  }

 private:
  /**
   * Pixel coordinate of the image feature
   */
  Eigen::Vector2f image_feature_;

  /**
   * Coordinate of the feature in the world frame
   */
  Eigen::Vector3f point_;

  /**
   * Camera intrinsic matrix
   */
  vslam_types::CameraIntrinsics intrinsics_;

  /**
   * Transform that provides the camera position in the robot's frame.
   */
  Eigen::Affine3f cam_to_robot_tf_;

  /**
   * Reprojection error standard deviation.
   */
  double reprojection_error_std_dev_;
};
}  // namespace vslam_solver

#endif  // UT_VSLAM_PAIRWISE_2D_FEATURE_COST_FUNCTOR_H
