//
// Created by amanda on 8/13/22.
//

#ifndef UT_VSLAM_LONG_TERM_MAP_FACTOR_CREATOR_H
#define UT_VSLAM_LONG_TERM_MAP_FACTOR_CREATOR_H

#include <base_lib/basic_utils.h>
#include <ceres/ceres.h>
#include <refactoring/factors/independent_object_map_factor.h>
#include <refactoring/factors/pairwise_object_map_factor.h>
#include <refactoring/long_term_map/long_term_object_map.h>
#include <refactoring/optimization/object_pose_graph.h>
#include <refactoring/optimization/optimization_solver_params.h>

namespace vslam_types_refactor {

template <typename CachedInfo>
class AbsLongTermMapFactorCreator {
 public:
  virtual bool getFactorsToInclude(
      util::BoostHashSet<std::pair<FactorType, FeatureFactorId>> &ltm_factors)
      const = 0;

  virtual bool createResidual(
      const std::pair<vslam_types_refactor::FactorType,
                      vslam_types_refactor::FeatureFactorId> &factor_info,
      const std::shared_ptr<
          vslam_types_refactor::ObjectAndReprojectionFeaturePoseGraph>
          &pose_graph,
      const pose_graph_optimization::ObjectVisualPoseGraphResidualParams
          &residual_params,
      const std::function<bool(
          const std::pair<vslam_types_refactor::FactorType,
                          vslam_types_refactor::FeatureFactorId> &,
          const std::shared_ptr<
              vslam_types_refactor::ObjectAndReprojectionFeaturePoseGraph> &,
          CachedInfo &)> &cached_info_creator,
      ceres::Problem *problem,
      ceres::ResidualBlockId &residual_id,
      CachedInfo &cached_info) const = 0;
};

struct PairwiseCovarianceLongTermMapFactorData {
  ObjectId obj_1_;
  ObjectId obj_2_;
  EllipsoidState<double> obj_1_map_est_;
  EllipsoidState<double> obj_2_map_est_;
  Eigen::Matrix<double,
                kEllipsoidParamterizationSize,
                kEllipsoidParamterizationSize>
      cross_covariance_;
  // TODO do we need more here?
};

struct IndependentEllipsoidsLongTermMapFactorData {
  ObjectId obj_id_;
  EllipsoidState<double> obj_map_est_;
  Covariance<double, kEllipsoidParamterizationSize> covariance_;
};

template <typename CachedInfo, typename FrontEndMapData>
class PairwiseCovarianceLongTermObjectMapFactorCreator
    : public AbsLongTermMapFactorCreator<CachedInfo> {
 public:
  PairwiseCovarianceLongTermObjectMapFactorCreator(
      const std::shared_ptr<
          PairwiseCovarianceLongTermObjectMap<FrontEndMapData>> &ltm)
      : AbsLongTermMapFactorCreator<CachedInfo>() {
    if (ltm == nullptr) {
      return;
    }
    FeatureFactorId next_feature_factor_id = 0;
    EllipsoidResults map_ellipsoid_ests;
    ltm->getEllipsoidResults(map_ellipsoid_ests);
    for (const auto &ltm_entry : ltm->getPairwiseEllipsoidCovariances()) {
      PairwiseCovarianceLongTermMapFactorData factor_entry;
      factor_entry.obj_1_ = ltm_entry.first.first;
      factor_entry.obj_2_ = ltm_entry.first.second;
      factor_entry.cross_covariance_ = ltm_entry.second;
      factor_entry.obj_1_map_est_ =
          map_ellipsoid_ests.ellipsoids_[factor_entry.obj_1_].second;
      factor_entry.obj_2_map_est_ =
          map_ellipsoid_ests.ellipsoids_[factor_entry.obj_2_].second;
      factor_data_[kLongTermMapFactorTypeId][next_feature_factor_id++] =
          factor_entry;
    }
  }

  virtual bool getFactorsToInclude(
      util::BoostHashSet<std::pair<FactorType, FeatureFactorId>> &ltm_factors)
      const override {
    for (const auto &factor_entry : factor_data_) {
      FactorType factor_type = factor_entry.first;
      for (const auto &feature_entry : factor_entry.second) {
        FeatureFactorId feature_id = feature_entry.first;
        ltm_factors.insert(std::make_pair(factor_type, feature_id));
      }
    }
    return true;
  }

  virtual bool createResidual(
      const std::pair<vslam_types_refactor::FactorType,
                      vslam_types_refactor::FeatureFactorId> &factor_info,
      const std::shared_ptr<
          vslam_types_refactor::ObjectAndReprojectionFeaturePoseGraph>
          &pose_graph,
      const pose_graph_optimization::ObjectVisualPoseGraphResidualParams
          &residual_params,
      const std::function<bool(
          const std::pair<vslam_types_refactor::FactorType,
                          vslam_types_refactor::FeatureFactorId> &,
          const std::shared_ptr<
              vslam_types_refactor::ObjectAndReprojectionFeaturePoseGraph> &,
          CachedInfo &)> &cached_info_creator,
      ceres::Problem *problem,
      ceres::ResidualBlockId &residual_id,
      CachedInfo &cached_info) override {
    if (factor_data_.find(factor_info.first) == factor_data_.end()) {
      LOG(ERROR) << "Could not find factor type " << factor_info.first
                 << " when creating residual.";
      return false;
    }
    std::unordered_map<FeatureFactorId, PairwiseCovarianceLongTermMapFactorData>
        features_for_factor_type = factor_data_.at(factor_info.first);
    if (features_for_factor_type.find(factor_info.second) ==
        features_for_factor_type.end()) {
      LOG(ERROR) << "Could not find feature id " << factor_info.second
                 << " with factor type " << factor_info.first
                 << " when creating residual.";
      return false;
    }
    PairwiseCovarianceLongTermMapFactorData factor_entry =
        features_for_factor_type.at(factor_info.second);

    if (!cached_info_creator(factor_info, pose_graph, cached_info)) {
      LOG(ERROR) << "In using factor with id " << factor_info.second
                 << " could make cached info";
      return false;
    }

    double *ellipsoid_1_param_block;
    if (!pose_graph->getObjectParamPointers(factor_entry.obj_1_,
                                            &ellipsoid_1_param_block)) {
      LOG(ERROR) << "In using factor with id " << factor_info.second
                 << " could not find ellipsoid parameter block for object "
                 << factor_entry.obj_1_ << "; not adding to pose graph";
      return false;
    }

    double *ellipsoid_2_param_block;
    if (!pose_graph->getObjectParamPointers(factor_entry.obj_2_,
                                            &ellipsoid_2_param_block)) {
      LOG(ERROR) << "In using factor with id " << factor_info.second
                 << " could not find ellipsoid parameter block for object "
                 << factor_entry.obj_2_ << "; not adding to pose graph";
      return false;
    }

    residual_id = problem->AddResidualBlock(
        PairwiseObjectMapFactor::createPairwiseObjectMapFactor(
            factor_entry.obj_1_map_est_,
            factor_entry.obj_2_map_est_,
            factor_entry
                .cross_covariance_),  // TODO probably will need to change this
        new ceres::HuberLoss(
            residual_params.long_term_map_params_.pair_huber_loss_param_),
        ellipsoid_1_param_block,
        ellipsoid_2_param_block);
    return true;
  }

 private:
  std::unordered_map<
      FactorType,
      std::unordered_map<FeatureFactorId,
                         PairwiseCovarianceLongTermMapFactorData>>
      factor_data_;
};

template <typename CachedInfo, typename FrontEndMapData>
class IndependentEllipsoidsLongTermObjectMapFactorCreator
    : public AbsLongTermMapFactorCreator<CachedInfo> {
 public:
  IndependentEllipsoidsLongTermObjectMapFactorCreator(
      const std::shared_ptr<
          IndependentEllipsoidsLongTermObjectMap<FrontEndMapData>> &ltm)
      : AbsLongTermMapFactorCreator<CachedInfo>() {
    if (ltm == nullptr) {
      return;
    }
    FeatureFactorId next_feature_factor_id = 0;
    EllipsoidResults map_ellipsoid_ests;
    ltm->getEllipsoidResults(map_ellipsoid_ests);
    for (const auto &ltm_entry : ltm->getEllipsoidCovariances()) {
      IndependentEllipsoidsLongTermMapFactorData factor_entry;
      factor_entry.obj_id_ = ltm_entry.first;
      factor_entry.covariance_ = ltm_entry.second;
      factor_entry.obj_map_est_ =
          map_ellipsoid_ests.ellipsoids_[factor_entry.obj_id_].second;
      factor_data_[kLongTermMapFactorTypeId][next_feature_factor_id++] =
          factor_entry;
    }
  }

  virtual bool getFactorsToInclude(
      util::BoostHashSet<std::pair<FactorType, FeatureFactorId>> &ltm_factors)
      const override {
    for (const auto &factor_entry : factor_data_) {
      FactorType factor_type = factor_entry.first;
      for (const auto &feature_entry : factor_entry.second) {
        FeatureFactorId feature_id = feature_entry.first;
        ltm_factors.insert(std::make_pair(factor_type, feature_id));
      }
    }
    return true;
  }

  virtual bool createResidual(
      const std::pair<vslam_types_refactor::FactorType,
                      vslam_types_refactor::FeatureFactorId> &factor_info,
      const std::shared_ptr<
          vslam_types_refactor::ObjectAndReprojectionFeaturePoseGraph>
          &pose_graph,
      const pose_graph_optimization::ObjectVisualPoseGraphResidualParams
          &residual_params,
      const std::function<bool(
          const std::pair<vslam_types_refactor::FactorType,
                          vslam_types_refactor::FeatureFactorId> &,
          const std::shared_ptr<
              vslam_types_refactor::ObjectAndReprojectionFeaturePoseGraph> &,
          CachedInfo &)> &cached_info_creator,
      ceres::Problem *problem,
      ceres::ResidualBlockId &residual_id,
      CachedInfo &cached_info) const override {
    if (factor_data_.find(factor_info.first) == factor_data_.end()) {
      LOG(ERROR) << "Could not find factor type " << factor_info.first
                 << " when creating residual.";
      return false;
    }
    std::unordered_map<FeatureFactorId,
                       IndependentEllipsoidsLongTermMapFactorData>
        features_for_factor_type = factor_data_.at(factor_info.first);
    if (features_for_factor_type.find(factor_info.second) ==
        features_for_factor_type.end()) {
      LOG(ERROR) << "Could not find feature id " << factor_info.second
                 << " with factor type " << factor_info.first
                 << " when creating residual.";
      return false;
    }
    IndependentEllipsoidsLongTermMapFactorData factor_entry =
        features_for_factor_type.at(factor_info.second);

    if (!cached_info_creator(factor_info, pose_graph, cached_info)) {
      LOG(ERROR) << "In using factor with id " << factor_info.second
                 << " could make cached info";
      return false;
    }

    double *ellipsoid_param_block;
    if (!pose_graph->getObjectParamPointers(factor_entry.obj_id_,
                                            &ellipsoid_param_block)) {
      LOG(ERROR) << "In using factor with id " << factor_info.second
                 << " could not find ellipsoid parameter block for object "
                 << factor_entry.obj_id_ << "; not adding to pose graph";
      return false;
    }

    residual_id = problem->AddResidualBlock(
        IndependentObjectMapFactor::createIndependentObjectMapFactor(
            factor_entry.obj_map_est_, factor_entry.covariance_),
        new ceres::HuberLoss(
            residual_params.long_term_map_params_.pair_huber_loss_param_),
        ellipsoid_param_block);
    return true;
  }

 private:
  std::unordered_map<
      FactorType,
      std::unordered_map<FeatureFactorId,
                         IndependentEllipsoidsLongTermMapFactorData>>
      factor_data_;
};

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_LONG_TERM_MAP_FACTOR_CREATOR_H
