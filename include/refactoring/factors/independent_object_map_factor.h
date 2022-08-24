//
// Created by amanda on 8/13/22.
//

#ifndef UT_VSLAM_INDEPENDENT_OBJECT_MAP_FACTOR_H
#define UT_VSLAM_INDEPENDENT_OBJECT_MAP_FACTOR_H

#include <ceres/autodiff_cost_function.h>
#include <glog/logging.h>
#include <refactoring/types/vslam_obj_opt_types_refactor.h>

#include <eigen3/Eigen/Dense>

namespace vslam_types_refactor {

class IndependentObjectMapFactor {
 public:
  IndependentObjectMapFactor(const EllipsoidState<double> &ellipsoid_mean,
                             const Covariance<double, 9> &covariance);
  template <typename T>
  bool operator()(const T *ellipsoid_ptr, T *residuals_ptr) const {
    Eigen::Map<const Eigen::Matrix<T, 9, 1>> ellipsoid(ellipsoid_ptr);

    Eigen::Matrix<T, 9, 1> ellipsoid_deviation = ellipsoid - ellipsoid_mean_.cast<T>();
    Eigen::Map<Eigen::Matrix<T, 9, 1>> residuals(residuals_ptr);

    residuals =
        sqrt_inf_mat_.template cast<T>() * ellipsoid_deviation;
    return true;
  }

  static ceres::AutoDiffCostFunction<IndependentObjectMapFactor, 9, 9> *
  createIndependentObjectMapFactor(const EllipsoidState<double> &ellipsoid_mean,
                                   const Covariance<double, 9> &covariance) {
    IndependentObjectMapFactor *factor =
        new IndependentObjectMapFactor(ellipsoid_mean, covariance);
    return new ceres::AutoDiffCostFunction<IndependentObjectMapFactor, 9, 9>(
        factor);
  }

 private:
  RawEllipsoid<double> ellipsoid_mean_;

  Eigen::Matrix<double, 9, 9> sqrt_inf_mat_;
};

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_PAIRWISE_OBJECT_MAP_FACTOR_H
