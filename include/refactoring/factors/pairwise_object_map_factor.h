//
// Created by amanda on 8/13/22.
//

#ifndef UT_VSLAM_PAIRWISE_OBJECT_MAP_FACTOR_H
#define UT_VSLAM_PAIRWISE_OBJECT_MAP_FACTOR_H

#include <ceres/autodiff_cost_function.h>
#include <glog/logging.h>
#include <refactoring/types/vslam_obj_opt_types_refactor.h>

#include <eigen3/Eigen/Dense>

namespace vslam_types_refactor {

class PairwiseObjectMapFactor {
 public:
  template <typename T>
  bool operator()(const T *ellipsoid_1,
                  const T *ellipsoid_2,
                  T *residual) const {
    // TOOD
    return true;
  }

  static ceres::AutoDiffCostFunction<PairwiseObjectMapFactor, 9, 9, 9> *
  createPairwiseObjectMapFactor(const EllipsoidState<double> &ellipsoid_1,
                                const EllipsoidState<double> &ellipsoid_2,
                                const Covariance<double, 9> &covariance) {
    PairwiseObjectMapFactor *factor = new PairwiseObjectMapFactor();
    return new ceres::AutoDiffCostFunction<PairwiseObjectMapFactor, 9, 9, 9>(
        factor);
  }
 private:



};

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_PAIRWISE_OBJECT_MAP_FACTOR_H
