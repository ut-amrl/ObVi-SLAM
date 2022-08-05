//
// Created by amanda on 8/3/22.
//

#ifndef UT_VSLAM_LONG_TERM_MAP_H
#define UT_VSLAM_LONG_TERM_MAP_H

#include <base_lib/basic_utils.h>
#include <refactoring/output_problem_data.h>
#include <refactoring/types/vslam_obj_opt_types_refactor.h>

namespace vslam_types_refactor {

class AbsLongTermMap {
 public:
  /**
   * Set the ellipsoid results (contains ellipsoid state estimates).
   *
   * @param ellipsoid_results   Results to store in the long term map.
   */
  virtual void setEllipsoidResults(const EllipsoidResults &ellipsoid_results);

 private:
  // Long term map has
  // -- Set of ellipsoids (ids, semantic class, pose, dimensions)
  // -- Connections between ellipsoids -- this will go in subclass so the
  // representation is flexible

  // Ellipsoid estimates
  EllipsoidResults ellipsoids_;
};

class PairwiseCovarianceLongTermMap : public AbsLongTermMap {
 public:
  /**
   * Set the pairwise ellipsoid covariance result. Pairs should have the object
   * with the smaller id first. Can have all possible pairs or a subset (already
   * sparsified).
   *
   * @param pairwise_ellipsoid_covariances
   */
  void setPairwiseEllipsoidCovariance(
      util::BoostHashMap<std::pair<ObjectId, ObjectId>,
                         Eigen::Matrix<double, 9, 9>>
          &pairwise_ellipsoid_covariances);

 private:
  /**
   * Pairwise ellipsoid covariances. The covariance will be stored with the pair
   * with the smaller of the two object ids first.
   */
  util::BoostHashMap<std::pair<ObjectId, ObjectId>, Eigen::Matrix<double, 9, 9>>
      pairwise_ellipsoid_covariances_;
};

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_LONG_TERM_MAP_H
