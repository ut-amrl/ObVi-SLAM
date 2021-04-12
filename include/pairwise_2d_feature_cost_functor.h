#ifndef UT_VSLAM_PAIRWISE_2D_FEATURE_COST_FUNCTOR_H
#define UT_VSLAM_PAIRWISE_2D_FEATURE_COST_FUNCTOR_H

#include <ceres/autodiff_cost_function.h>
#include <vslam_types.h>

#include <eigen3/Eigen/Dense>

namespace vslam_solver {

/**
 * Convert from pixel coords to image coords.
 *
 * @param pixel_coord   Pixel coordinate
 * @param intrinsics    Camera intrinsics.
 *
 * @return  Image coordinates for the point.
 */
static Eigen::Vector3f ConvertFromPixelToImage(
    const Eigen::Vector2f &pixel_coord,
    const vslam_types::CameraIntrinsics &intrinsics) {
  Eigen::Matrix3f camera_mat;
  camera_mat << intrinsics.fx, 0, intrinsics.cx, 0, intrinsics.fy,
      intrinsics.cy, 0, 0, 1;

  Eigen::Vector3f pixel_coord_homogeneous(
      pixel_coord.x(), pixel_coord.y(), 1.0);
  return camera_mat.inverse() * pixel_coord_homogeneous;
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
   * @param intrinsics              Camera intrinsics.
   * @param extrinsics              Camera extrinsics (pose of the camera
   *                                relative to the robot).
   * @param epipolar_error_std_dev  Standard deviation of the epipolar error.
   */
  Pairwise2dFeatureCostFunctor(const Eigen::Vector2f &image_1_feature,
                               const Eigen::Vector2f &image_2_feature,
                               const vslam_types::CameraIntrinsics &intrinsics,
                               const vslam_types::CameraExtrinsics &extrinsics,
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
    // Convert pose init to rotation
    Eigen::Transform<T, 3, Eigen::Affine> first_robot_pose_in_world =
        vslam_types::PoseArrayToAffine(&(pose_init[3]), &(pose_init[0]));

    // Convert pose current to rotation
    Eigen::Transform<T, 3, Eigen::Affine> second_robot_pose_in_world =
        vslam_types::PoseArrayToAffine(&(pose_current[3]), &(pose_current[0]));

    // Want to get rotation and translation of camera frame 1 relative to
    // camera frame 2 (pose of camera 2 in camera 1)
    // if T_r_c is the transformation matrix representing camera extrinsics
    // (camera pose rel robot)
    // T_w_r1 is the robot's pose at frame 1 relative to the world
    // T_w_r2 is the robot's pose at frame 2 relative to the world, then camera
    // 2 relative to camera 1 is given by
    // T_c1_c2 =  T_r_c^-1 * T_w_r1^-1 * T_w_r2 * T_r_c
    Eigen::Transform<T, 3, Eigen::Affine> cam_1_to_cam_2_mat =
        cam_to_robot_tf_.inverse().cast<T>() *
        first_robot_pose_in_world.inverse() * second_robot_pose_in_world *
        cam_to_robot_tf_.cast<T>();

    // Extract Tx and R from cam_1_to_cam_2_mat
    Eigen::Matrix<T, 3, 1> t_vec = cam_1_to_cam_2_mat.translation();
    Eigen::Matrix<T, 3, 3> t_cross;
    t_cross << T(0.0), -t_vec(3), t_vec(2), t_vec(3), T(0.0), -t_vec(1),
        -t_vec(2), t_vec(1), T(0.0);
    Eigen::Matrix<T, 3, 3> rotation = cam_1_to_cam_2_mat.linear();

    residual[0] = (feature_2_image_coords_.transpose().cast<T>() * t_cross *
                   rotation * feature_1_image_coords_.cast<T>())(0, 0) /
                  T(epipolar_error_std_dev_);

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
  static ceres::AutoDiffCostFunction<Pairwise2dFeatureCostFunctor, 1, 6, 6>
      *create(const vslam_types::CameraIntrinsics &intrinsics,
              const vslam_types::CameraExtrinsics &extrinsics,
              const vslam_types::VisionFeature &frame_1_feature,
              const vslam_types::VisionFeature &frame_2_feature,
              const double &epipolar_error_std_dev) {
    const Eigen::Vector2f &initial_pixel = frame_1_feature.pixel;
    const Eigen::Vector2f &current_pixel = frame_2_feature.pixel;
    Pairwise2dFeatureCostFunctor *residual =
        new Pairwise2dFeatureCostFunctor(initial_pixel,
                                         current_pixel,
                                         intrinsics,
                                         extrinsics,
                                         epipolar_error_std_dev);
    return new ceres::
        AutoDiffCostFunction<Pairwise2dFeatureCostFunctor, 1, 6, 6>(residual);
  }

 private:
  /**
   * Feature 1 in image space (not pixel), have already applied camera
   * intrinsics to this.
   */
  Eigen::Vector3f feature_1_image_coords_;

  /**
   * Feature 2 in image space (not pixel), have already applied camera
   * intrinsics to this.
   */
  Eigen::Vector3f feature_2_image_coords_;

  /**
   * Transform that provides the camera position in the robot's frame.
   */
  Eigen::Affine3f cam_to_robot_tf_;

  /**
   * Epipolar error standard deviation.
   */
  double epipolar_error_std_dev_;
};
}  // namespace vslam_solver

#endif  // UT_VSLAM_PAIRWISE_2D_FEATURE_COST_FUNCTOR_H
