#include <shape_prior_factor.h>

// For matrix sqrt
#include <unsupported/Eigen/MatrixFunctions>

namespace vslam_solver {
ShapePriorFactor::ShapePriorFactor(const Eigen::Vector3f &shape_dim_mean,
                                   const Eigen::Matrix3f &shape_dim_covariance)
    : shape_dim_mean_(shape_dim_mean),
      sqrt_shape_dim_inf_mat_(shape_dim_covariance.inverse().sqrt()) {}
}  // namespace vslam_solver