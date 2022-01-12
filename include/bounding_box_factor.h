#ifndef UT_VSLAM_BOUNDING_BOX_FACTOR_H
#define UT_VSLAM_BOUNDING_BOX_FACTOR_H

#include <ceres/autodiff_cost_function.h>
#include <ellipsoid_utils.h>
#include <glog/logging.h>
#include <vslam_types.h>

#include <eigen3/Eigen/Dense>

namespace vslam_solver {

/**
 * Factor to use in optimization that uses a bounding box observation of an
 * object in an image to constrain the ellipsoid used to approximate the object.
 */
class BoundingBoxFactor {
 public:
  /**
   * Constructor.
   *
   * @param intrinsics                      Camera intrinsics.
   * @param extrinsics                      Camera extrinsics. Provides camera
   *                                        pose relative to robot pose.
   * @param corner_pixel_locations          Pixel locations of the corners of
   *                                        the bounding box for the image
   *                                        detection.
   * @param corner_detections_covariance    Covariance matrix indicating
   *                                        uncertainty in our bounding box
   *                                        measurements.
   */
  BoundingBoxFactor(const vslam_types::CameraIntrinsics &intrinsics,
                    const vslam_types::CameraExtrinsics &extrinsics,
                    const Eigen::Vector4f &corner_pixel_locations,
                    const Eigen::Matrix4f &corner_detections_covariance);

  /**
   * Compute the residual for the bounding box observation.
   *
   * @tparam T                  Type that the cost functor is evaluating.
   * @param ellipsoid[in]       Estimate of the ellipsoid parameters. This is a
   *                            9 entry array with the first 3 entries
   *                            corresponding to the translation, the second 3
   *                            entries containing the axis-angle representation
   *                            (with angle given by the magnitude of the
   *                            vector), and the final 3 entries corresponding
   *                            to the dimensions of the ellipsoid.
   * @param robot_pose[in]      Robot's pose in the world frame corresponding to
   *                            the location of where the feature was imaged.
   *                            This is a 6 entry array with the first 3 entries
   *                            corresponding to the translation and the second
   *                            3 entries containing the axis-angle
   *                            representation (with angle given by the
   *                            magnitude of the vector).
   * @param residuals_ptr[out]  Residual giving the error. Contains 4 entries.
   *
   * @return True if the residual was computed successfully, false otherwise.
   */
  template <typename T>
  bool operator()(const T *ellipsoid,
                  const T *robot_pose,
                  T *residuals_ptr) const {
    Eigen::Transform<T, 3, Eigen::Affine> robot_to_world_current =
        vslam_types::PoseArrayToAffine(&(robot_pose[3]), &(robot_pose[0]));

    // Robot to world defines the robot's pose in the world frame
    // Cam to robot defines the camera pose in the robot's frame
    // We want the camera's pose in the world frame
    Eigen::Transform<T, 3, Eigen::Affine> camera_to_world =
        robot_to_world_current * cam_to_robot_tf_.template cast<T>();
    Eigen::Transform<T, 3, Eigen::AffineCompact> camera_to_world_compact =
        camera_to_world;
    Eigen::Matrix<T, 4, 4> ellipsoid_dual_rep =
        vslam_util::createDualRepresentationForEllipsoid(ellipsoid);

    Eigen::Matrix<T, 3, 3> g_mat =
        camera_intrinsics_mat_.template cast<T>() *
        camera_to_world_compact.matrix() * ellipsoid_dual_rep *
        camera_to_world_compact.matrix().transpose() *
        camera_intrinsics_mat_.transpose().template cast<T>();

    T g1_1 = g_mat(1, 1);
    T g1_3 = g_mat(1, 3);
    T g2_2 = g_mat(2, 2);
    T g2_3 = g_mat(2, 3);
    T g3_3 = g_mat(3, 3);
    T x_sqrt_component = sqrt(pow(g1_3, 2) - (g1_1 * g3_3));
    T y_sqrt_component = sqrt(pow(g2_3, 2) - (g2_2 * g3_3));

    // TODO Verify once we have real data that these are in the right order?
    //  is min and max actually the min and max?
    Eigen::Matrix<T, 4, 1> corner_results(g1_3 - x_sqrt_component,
                                          g1_3 + x_sqrt_component,
                                          g2_3 - y_sqrt_component,
                                          g2_3 + y_sqrt_component);

    Eigen::Matrix<T, 4, 1> deviation =
        corner_results - corner_detections_.template cast<T>();

    Eigen::Map<Eigen::Matrix<T, 4, 1>> residuals(residuals_ptr);
    residuals =
        sqrt_inf_mat_bounding_box_corners_.template cast<T>() * deviation;

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
  static ceres::AutoDiffCostFunction<BoundingBoxFactor, 4, 9, 6>
      *createUsingSemanticClassMapping(
          const vslam_types::ObjectImageBoundingBoxDetection &object_detection,
          const vslam_types::CameraIntrinsics &camera_intrinsics,
          const vslam_types::CameraExtrinsics &camera_extrinsics,
          const Eigen::Matrix4f &bounding_box_covariance) {
    Eigen::Vector4f corner_locations_vector(
        object_detection.pixel_corner_locations.first.x(),
        object_detection.pixel_corner_locations.second.x(),
        object_detection.pixel_corner_locations.first.y(),
        object_detection.pixel_corner_locations.second.y());
    BoundingBoxFactor *factor = new BoundingBoxFactor(camera_intrinsics,
                                                      camera_extrinsics,
                                                      corner_locations_vector,
                                                      bounding_box_covariance);
    return new ceres::AutoDiffCostFunction<BoundingBoxFactor, 4, 9, 6>(factor);
  }

 private:
  /**
   * Camera intrinsics matrix.
   */
  Eigen::Matrix3f camera_intrinsics_mat_;

  /**
   * Transform that provides the camera position in the robot's frame.
   */
  Eigen::Affine3f cam_to_robot_tf_;

  /**
   * Square root of the bounding box information matrix (inverse of covariance).
   *
   * Let this value be A. To get the shape covariance C, C = (A * A)^-1.
   */
  Eigen::Matrix<float, 4, 4> sqrt_inf_mat_bounding_box_corners_;

  /**
   * Corner detections for the bounding box. The first value is the smallest x
   * value, the next is the largest x value, the third is the smallest y value
   * and the fourth is the largest y value. These correspond to the x and y
   * coordinates defining opposite corners of the bounding box.
   */
  Eigen::Vector4f corner_detections_;
};
}  // namespace vslam_solver

#endif  // UT_VSLAM_BOUNDING_BOX_FACTOR_H
