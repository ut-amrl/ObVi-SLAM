//
// Created by amanda on 8/3/22.
//

#ifndef UT_VSLAM_LONG_TERM_OBJECT_MAP_H
#define UT_VSLAM_LONG_TERM_OBJECT_MAP_H

#include <base_lib/basic_utils.h>
#include <refactoring/output_problem_data.h>
#include <refactoring/types/vslam_obj_opt_types_refactor.h>

namespace vslam_types_refactor {

// TODO consider splitting this into back-end map data and front end and have a
// structure that contains both
template <typename FrontEndObjMapData>
class AbsLongTermObjectMap {
 public:
  /**
   * Set the ellipsoid results (contains ellipsoid state estimates).
   *
   * @param ellipsoid_results   Results to store in the long term map.
   */
  virtual void setEllipsoidResults(const EllipsoidResults &ellipsoid_results) {
    ellipsoids_ = ellipsoid_results;
    LOG(INFO) << "Ellipsoid results size " << ellipsoids_.ellipsoids_.size();
  }

  virtual void setFrontEndObjMapData(const FrontEndObjMapData &front_end_data) {
    front_end_map_data_ = front_end_data;
  }

  virtual void getEllipsoidResults(EllipsoidResults &ellipsoids) const {
    LOG(INFO) << "Ellipsoids size " << ellipsoids_.ellipsoids_.size();
    ellipsoids = ellipsoids_;
  }

  virtual void getFrontEndObjMapData(FrontEndObjMapData &front_end_data) const {
    front_end_data = front_end_map_data_;
  }

 private:
  // Long term map has
  // -- Set of ellipsoids (ids, semantic class, pose, dimensions)
  // -- Connections between ellipsoids -- this will go in subclass so the
  // representation is flexible

  // Ellipsoid estimates
  EllipsoidResults ellipsoids_;

  FrontEndObjMapData front_end_map_data_;
};

template <typename FrontEndObjMapData>
class IndependentEllipsoidsLongTermObjectMap
    : public AbsLongTermObjectMap<FrontEndObjMapData> {
 public:
  void setEllipsoidCovariances(
      const std::unordered_map<
          ObjectId,
          Covariance<double, kEllipsoidParamterizationSize>>
          &ellipsoid_covariances) {
    ellipsoid_covariances_ = ellipsoid_covariances;
  }

  std::unordered_map<ObjectId,
                     Covariance<double, kEllipsoidParamterizationSize>>
  getEllipsoidCovariances() const {
    return ellipsoid_covariances_;
  }

 private:
  std::unordered_map<ObjectId,
                     Covariance<double, kEllipsoidParamterizationSize>>
      ellipsoid_covariances_;
};

template <typename FrontEndObjMapData>
class PairwiseCovarianceLongTermObjectMap
    : public AbsLongTermObjectMap<FrontEndObjMapData> {
 public:
  PairwiseCovarianceLongTermObjectMap()
      : AbsLongTermObjectMap<FrontEndObjMapData>() {}
  /**
   * Set the pairwise ellipsoid covariance result. Pairs should have the object
   * with the smaller id first. Can have all possible pairs or a subset (already
   * sparsified).
   *
   * @param pairwise_ellipsoid_covariances
   */
  void setPairwiseEllipsoidCovariance(
      util::BoostHashMap<std::pair<ObjectId, ObjectId>,
                         Eigen::Matrix<double,
                                       kEllipsoidParamterizationSize,
                                       kEllipsoidParamterizationSize>>
          &pairwise_ellipsoid_covariances) {
    pairwise_ellipsoid_covariances_ = pairwise_ellipsoid_covariances;
  }

  util::BoostHashMap<std::pair<ObjectId, ObjectId>,
                     Eigen::Matrix<double,
                                   kEllipsoidParamterizationSize,
                                   kEllipsoidParamterizationSize>>
  getPairwiseEllipsoidCovariances() {
    return pairwise_ellipsoid_covariances_;
  }

 private:
  /**
   * Pairwise ellipsoid covariances. The covariance will be stored with the pair
   * with the smaller of the two object ids first.
   */
  util::BoostHashMap<std::pair<ObjectId, ObjectId>,
                     Eigen::Matrix<double,
                                   kEllipsoidParamterizationSize,
                                   kEllipsoidParamterizationSize>>
      pairwise_ellipsoid_covariances_;
};

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_LONG_TERM_OBJECT_MAP_H
