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
                          const vslam_types::CameraIntrinsics &intrinsics,
                          const vslam_types::CameraExtrinsics &extrinsics,
                          const double &reprojection_error_std_dev);

  /**
   * Compute the reprojection error
   *
   * @tparam T                  Type that the cost functor is evaluating.
   * @param pose[in]       Robot's pose in the world frame corresponding to
   *                            the location of where the feature was imaged.
   *                            This is a 6 entry array with the first 3 entries
   *                            corresponding to the translation and the second
   *                            3 entries containing the axis-angle
   *                            representation (with angle given by the
   *                            magnitude of the vector).
   * @param point[in]           3D position of the imaged feature in the world
   *                            frame
   * @param residual[out]       Residual giving the error. Contains 2 entries.
   *                            The first is the x direction reprojection error
   *                            and the second is the y direction reprojection
   *                            error
   *
   * @return True if the residual was computed successfully, false otherwise.
   */
  template <typename T>
  bool operator()(const T *pose, const T *point, T *residual) const {
    // Transform from world to current robot pose
    Eigen::Transform<T, 3, Eigen::Affine> world_to_robot_current =
        vslam_types::PoseArrayToAffine(&(pose[3]), &(pose[0])).inverse();

    // Point in world frame
    Eigen::Matrix<T, 3, 1> point_world(point[0], point[1], point[2]);

    // Transform the point from global coordinates to frame of current pose.
    Eigen::Matrix<T, 3, 1> point_current =
        cam_to_robot_tf_.inverse().cast<T>() * world_to_robot_current *
        point_world;

    // Project the 3D point into the current image.
    T p_x = T(intrinsics_.camera_mat(0, 0)) * point_current.x() /
                point_current.z() +
            T(intrinsics_.camera_mat(0, 2));
    T p_y = T(intrinsics_.camera_mat(1, 1)) * point_current.y() /
                point_current.z() +
            T(intrinsics_.camera_mat(1, 2));

    // Compute the residual.
    residual[0] = p_x - T(image_feature_.x());
    residual[1] = p_y - T(image_feature_.y());

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
  /*TODO should this be 2,9,9 ?*/
  static ceres::AutoDiffCostFunction<ReprojectionCostFunctor, 2, 6, 3> *create(
      const vslam_types::CameraIntrinsics &intrinsics,
      const vslam_types::CameraExtrinsics &extrinsics,
      const vslam_types::VisionFeature &image_feature,
      const double &reprojection_error_std_dev) {
    const Eigen::Vector2f &feature_pixel = image_feature.pixel;
    ReprojectionCostFunctor *residual = new ReprojectionCostFunctor(
        feature_pixel, intrinsics, extrinsics, reprojection_error_std_dev);
    return new ceres::AutoDiffCostFunction<ReprojectionCostFunctor, 2, 6, 3>(
        residual);
  }

 private:
  /**
   * Pixel coordinate of the image feature
   */
  Eigen::Vector2f image_feature_;

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
