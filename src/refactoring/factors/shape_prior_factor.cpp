#include <refactoring/factors/shape_prior_factor.h>

// For matrix sqrt
#include <unsupported/Eigen/MatrixFunctions>

namespace vslam_types_refactor {
ShapePriorFactor::ShapePriorFactor(
    const vslam_types_refactor::ObjectDim<double> &shape_dim_mean,
    const vslam_types_refactor::Covariance<double, 3> &shape_dim_covariance)
    : shape_dim_mean_(shape_dim_mean),
      sqrt_shape_dim_inf_mat_(shape_dim_covariance.inverse().sqrt()) {}
}  // namespace vslam_types_refactor