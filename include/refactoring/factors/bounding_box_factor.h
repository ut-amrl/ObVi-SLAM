#ifndef UT_VSLAM_REFACTORING_BOUNDING_BOX_FACTOR_H
#define UT_VSLAM_REFACTORING_BOUNDING_BOX_FACTOR_H

#include <ceres/autodiff_cost_function.h>
#include <glog/logging.h>
#include <refactoring/types/ellipsoid_utils.h>
#include <refactoring/types/vslam_basic_types_refactor.h>
#include <refactoring/types/vslam_obj_opt_types_refactor.h>

#include <eigen3/Eigen/Dense>

namespace vslam_types_refactor {

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
  BoundingBoxFactor(
      const double &invalid_ellipse_error,
      const vslam_types_refactor::CameraIntrinsicsMat<double> &intrinsics,
      const vslam_types_refactor::CameraExtrinsics<double> &extrinsics,
      const vslam_types_refactor::BbCorners<double> &corner_pixel_locations,
      const vslam_types_refactor::Covariance<double, 4>
          &corner_detections_covariance);

  /**
   * Compute the residual for the bounding box observation.
   *
   * @tparam T                  Type that the cost functor is evaluating.
   * @param ellipsoid[in]       Estimate of the ellipsoid parameters. This is a
   *                            9 or 7 entry array with the first 3 entries
   *                            corresponding to the translation, the second 3
   *                            or 1 entries containing the axis-angle
   *                            representation (with angle given by the
   *                            magnitude of the vector) or just yaw, and the
   *                            final 3 entries corresponding to the dimensions
   *                            of the ellipsoid.
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
    // TODO Add mask that zeros out entries if the edge is near the edge of the
    // image (so we don't use it as a constraint). Could also try to do this
    // through the covariance (very wide covariance for sides we don't care
    // about)

    Eigen::Matrix<T, 4, 1> corner_results;
    bool valid_case =
        getCornerLocationsVector<T>(ellipsoid,
                                    robot_pose,
                                    robot_to_cam_tf_.cast<T>(),
                                    camera_intrinsics_mat_.cast<T>(),
                                    corner_results);
    if (!valid_case) {
      residuals_ptr[0] = T(invalid_ellipse_error_);
      residuals_ptr[1] = T(invalid_ellipse_error_);
      residuals_ptr[2] = T(invalid_ellipse_error_);
      residuals_ptr[3] = T(invalid_ellipse_error_);
      return true;
    }

    Eigen::Matrix<T, 4, 1> deviation =
        corner_results - corner_detections_.template cast<T>();
    //        LOG(INFO) << "Detection " << corner_detections_;
    //        LOG(INFO) << "Deviation " << deviation;
    //        LOG(INFO) << "Sqrt inf mat bounding box "
    //                  << sqrt_inf_mat_bounding_box_corners_;

    Eigen::Map<Eigen::Matrix<T, 4, 1>> residuals(residuals_ptr);
    residuals =
        sqrt_inf_mat_bounding_box_corners_.template cast<T>() * deviation;

    //    LOG(INFO) << "Residuals " << residuals;
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
  static ceres::AutoDiffCostFunction<BoundingBoxFactor,
                                     4,
                                     kEllipsoidParamterizationSize,
                                     6> *
  createBoundingBoxFactor(
      const double &invalid_ellipse_error,
      const vslam_types_refactor::BbCorners<double> &object_detection,
      const vslam_types_refactor::CameraIntrinsicsMat<double>
          &camera_intrinsics,
      const vslam_types_refactor::CameraExtrinsics<double> &camera_extrinsics,
      const vslam_types_refactor::Covariance<double, 4>
          &bounding_box_covariance) {
    BoundingBoxFactor *factor = new BoundingBoxFactor(invalid_ellipse_error,
                                                      camera_intrinsics,
                                                      camera_extrinsics,
                                                      object_detection,
                                                      bounding_box_covariance);
    return new ceres::AutoDiffCostFunction<BoundingBoxFactor,
                                           4,
                                           kEllipsoidParamterizationSize,
                                           6>(factor);
  }

 private:
  double invalid_ellipse_error_;

  /**
   * Corner detections for the bounding box. The first value is the smallest x
   * value, the next is the largest x value, the third is the smallest y value
   * and the fourth is the largest y value. These correspond to the x and y
   * coordinates defining opposite corners of the bounding box.
   */
  vslam_types_refactor::BbCorners<double> corner_detections_;

  /**
   * Square root of the bounding box information matrix (inverse of covariance).
   *
   * Let this value be A. To get the shape covariance C, C = (A * A)^-1.
   */
  Eigen::Matrix<double, 4, 4> sqrt_inf_mat_bounding_box_corners_;
  /**
   * Camera intrinsics matrix.
   */
  vslam_types_refactor::CameraIntrinsicsMat<double> camera_intrinsics_mat_;

  /**
   * Transform that provides the robot's position in the camera frame (inverse
   * of extrinsics, which provide the camera's pose in the robot frame).
   */
  Eigen::Affine3d robot_to_cam_tf_;
};
}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_REFACTORING_BOUNDING_BOX_FACTOR_H
