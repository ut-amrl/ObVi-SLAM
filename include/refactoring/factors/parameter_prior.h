//
// Created by amanda on 5/10/23.
//

#ifndef UT_VSLAM_PARAMETER_PRIOR_H
#define UT_VSLAM_PARAMETER_PRIOR_H

#include <ceres/autodiff_cost_function.h>
#include <glog/logging.h>
#include <refactoring/types/ellipsoid_utils.h>
#include <refactoring/types/vslam_basic_types_refactor.h>
#include <refactoring/types/vslam_obj_opt_types_refactor.h>

#include <eigen3/Eigen/Dense>

namespace vslam_types_refactor {

class ParameterPrior {
 public:
  ParameterPrior(const size_t &param_idx,
                 const double &param_mean,
                 const double &param_std_dev)
      : param_idx_(param_idx),
        param_mean_(param_mean),
        param_std_dev_(param_std_dev) {}

  template <typename T>
  bool operator()(const T *param_block, T *residuals_ptr) const {
    T residual_val =
        (param_block[param_idx_] - T(param_mean_)) / (param_std_dev_);
    residuals_ptr[0] = residual_val;

    return true;
  }

  template <int ParamBlockSize>
  static ceres::AutoDiffCostFunction<ParameterPrior, 1, ParamBlockSize>
      *createParameterPrior(const size_t &param_idx,
                            const double &param_mean,
                            const double &param_std_dev) {
    ParameterPrior *factor =
        new ParameterPrior(param_idx, param_mean, param_std_dev);
    return new ceres::AutoDiffCostFunction<ParameterPrior, 1, ParamBlockSize>(
        factor);
  }

 private:
  size_t param_idx_;
  double param_mean_;
  double param_std_dev_;
};
}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_PARAMETER_PRIOR_H
