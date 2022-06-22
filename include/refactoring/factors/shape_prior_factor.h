#ifndef UT_VSLAM_REFACTORING_SHAPE_PRIOR_FACTOR_H
#define UT_VSLAM_REFACTORING_SHAPE_PRIOR_FACTOR_H

#include <ceres/autodiff_cost_function.h>
#include <glog/logging.h>
#include <refactoring/types/vslam_obj_opt_types_refactor.h>

#include <eigen3/Eigen/Dense>

namespace vslam_types_refactor {

/**
 * Factor to use in optimization that penalizes an ellipsoid based on how far
 * its dimensions deviate from the prior (derived from semantic class, though
 * this happens outside of this class).
 */
class ShapePriorFactor {
 public:
  /**
   * Constructor.
   *
   * @param shape_dim_mean          Mean dimension for the shape.
   * @param shape_dim_covariance    Covariance for the shape's dimensions.
   */
  ShapePriorFactor(
      const vslam_types_refactor::ObjectDim<double> &shape_dim_mean,
      const vslam_types_refactor::Covariance<double, 3> &shape_dim_covariance);

  /**
   * Compute the residual for the deviation from the prior on the ellipsoid's
   * dimensions.
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
      *createShapeDimPrior(
          const vslam_types_refactor::ObjectDim<double> &dimension_prior_mean,
          const vslam_types_refactor::Covariance<double, 3> &dimension_cov) {
    ShapePriorFactor *factor =
        new ShapePriorFactor(dimension_prior_mean, dimension_cov);
    return new ceres::AutoDiffCostFunction<ShapePriorFactor, 3, 9>(factor);
  }

 private:
  /**
   * Mean of the shape
   */
  vslam_types_refactor::ObjectDim<double> shape_dim_mean_;

  /**
   * Square root of the shape dimension information matrix.
   *
   * Let this value be A. To get the shape covariance C, C = (A * A)^-1.
   */
  Eigen::Matrix3d sqrt_shape_dim_inf_mat_;
};
}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_REFACTORING_SHAPE_PRIOR_FACTOR_H
