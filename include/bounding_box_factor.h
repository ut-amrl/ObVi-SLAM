#ifndef UT_VSLAM_BOUNDING_BOX_FACTOR_H
#define UT_VSLAM_BOUNDING_BOX_FACTOR_H

#include <ceres/autodiff_cost_function.h>
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
   * Constructor. TODO fix comments
   *
   * @param shape_dim_mean          Mean dimension for the shape.
   * @param shape_dim_covariance    Covariance for the shape's dimensions.
   */
  BoundingBoxFactor(const vslam_types::CameraIntrinsics &intrinsics,
                    const Eigen::Vector4f &corner_pixel_locations);

  /**
   * Compute the residual for the epipolar error using the essential matrix.
   *
   * @tparam T                  Type that the cost functor is evaluating.
   * @param ellipsoid[in]       Estimate of the ellipsoid parameters. This is a
   *                            9 entry array with the first 3 entries
   *                            corresponding to the translation, the second 3
   *                            entries containing the axis-angle representation
   *                            (with angle given by the magnitude of the
   *                            vector), and the final 3 entries corresponding
   *                            to the dimensions of the ellipsoid.
   * @param residuals_ptr[out]  Residual giving the error. Contains 3 entries.
   *
   * @return True if the residual was computed successfully, false otherwise.
   */
  template <typename T>
  bool operator()(const T *ellipsoid, T *residuals_ptr) const {
    Eigen::Matrix<T, 3, 1> dimension_mat(
        ellipsoid[6], ellipsoid[7], ellipsoid[8]);

    Eigen::Matrix<T, 3, 1> deviation_from_mean =
        dimension_mat - shape_dim_mean_.cast<T>();

    Eigen::Map<Eigen::Matrix<T, 3, 1>> residuals(residuals_ptr);
    residuals =
        sqrt_shape_dim_inf_mat_.template cast<T>() * deviation_from_mean;

    return true;
  }

  /**
   * Create the autodiff cost function with this cost functor.
   *
   * @param init_ellipsoid_est              Initial ellipsoid estimate. TODO do
   *                                        we pass the whole initial estimate
   *                                        or just the semantic class, since
   *                                        that's really all that's needed.
   * @param mean_and_cov_by_semantic_class  Mapping of semantic class to the
   *                                        mean and covariance for the shape
   *                                        dimension for the semantic class.
   *
   * @return Ceres cost function.
   */
  static ceres::AutoDiffCostFunction<ShapePriorFactor, 3, 9>
      *createUsingSemanticClassMapping(
          const vslam_types::EllipsoidEstimate &init_ellipsoid_est,
          const std::unordered_map<std::string,
                                   std::pair<Eigen::Vector3f, Eigen::Matrix3f>>
              &mean_and_cov_by_semantic_class) {
    std::string semantic_class = init_ellipsoid_est.semantic_class;
    if (mean_and_cov_by_semantic_class.find(semantic_class) ==
        mean_and_cov_by_semantic_class.end()) {
      LOG(WARNING) << "No mean and covariance on shape dimensions for class "
                   << semantic_class;
      return nullptr;
    }
    std::pair<Eigen::Vector3f, Eigen::Matrix3f> mean_and_cov_for_ellipsoid =
        mean_and_cov_by_semantic_class.at(semantic_class);
    ShapePriorFactor *factor = new ShapePriorFactor(
        mean_and_cov_for_ellipsoid.first, mean_and_cov_for_ellipsoid.second);
    return new ceres::AutoDiffCostFunction<ShapePriorFactor, 3, 9>(factor);
  }

 private:
  /**
   * Mean of the shape
   */
  Eigen::Vector3f shape_dim_mean_;

  /**
   * Square root of the shape dimension information matrix.
   *
   * Let this value be A. To get the shape covariance C, C = (A * A)^-1.
   */
  Eigen::Matrix3f sqrt_shape_dim_inf_mat_;
};
}  // namespace vslam_solver

#endif  // UT_VSLAM_BOUNDING_BOX_FACTOR_H
