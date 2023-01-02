#include <refactoring/factors/independent_object_map_factor.h>

#include <unsupported/Eigen/MatrixFunctions>

namespace vslam_types_refactor {

IndependentObjectMapFactor::IndependentObjectMapFactor(
    const EllipsoidState<double> &ellipsoid_mean,
    const Covariance<double, kEllipsoidParamterizationSize> &covariance)
    : ellipsoid_mean_(convertToRawEllipsoid(ellipsoid_mean)),
      sqrt_inf_mat_(covariance.inverse().sqrt()) {}

}  // namespace vslam_types_refactor