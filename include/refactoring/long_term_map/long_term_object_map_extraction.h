//
// Created by amanda on 8/3/22.
//

#ifndef UT_VSLAM_LONG_TERM_OBJECT_MAP_EXTRACTION_H
#define UT_VSLAM_LONG_TERM_OBJECT_MAP_EXTRACTION_H

#include <ceres/ceres.h>
#include <ceres/problem.h>
#include <refactoring/long_term_map/long_term_object_map.h>
#include <refactoring/optimization/jacobian_extraction.h>
#include <refactoring/optimization/object_pose_graph.h>
#include <refactoring/optimization/object_pose_graph_optimizer.h>
#include <refactoring/output_problem_data_extraction.h>
#include <refactoring/types/vslam_types_math_util.h>

namespace vslam_types_refactor {

static const double kFarFeatureThreshold = 75;

bool runOptimizationForLtmExtraction(
    const std::function<bool(
        const std::pair<vslam_types_refactor::FactorType,
                        vslam_types_refactor::FeatureFactorId> &,
        const pose_graph_optimization::ObjectVisualPoseGraphResidualParams &,
        const std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph> &,
        ceres::Problem *,
        ceres::ResidualBlockId &,
        util::EmptyStruct &)> &residual_creator,
    const pose_graph_optimization::ObjectVisualPoseGraphResidualParams
        &residual_params,
    const pose_graph_optimization::OptimizationSolverParams &solver_params,
    const pose_graph_optimizer::OptimizationFactorsEnabledParams
        &optimization_factor_configuration,
    std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph> &pose_graph,
    ceres::Problem *problem,
    std::unordered_map<vslam_types_refactor::FactorType,
                       std::unordered_map<vslam_types_refactor::FeatureFactorId,
                                          ceres::ResidualBlockId>>
        &residual_info) {
  std::pair<FrameId, FrameId> min_and_max_frame_id =
      pose_graph->getMinMaxFrameId();
  std::function<bool(
      const std::pair<vslam_types_refactor::FactorType,
                      vslam_types_refactor::FeatureFactorId> &,
      const std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph> &,
      const util::EmptyStruct &)>
      refresh_residual_checker =
          [](const std::pair<vslam_types_refactor::FactorType,
                             vslam_types_refactor::FeatureFactorId> &,
             const std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph> &,
             const util::EmptyStruct &) { return true; };

  pose_graph_optimizer::OptimizationScopeParams ltm_optimization_scope_params;
  ltm_optimization_scope_params.fix_poses_ =
      optimization_factor_configuration.fix_poses_;
  ltm_optimization_scope_params.fix_objects_ =
      optimization_factor_configuration.fix_objects_;
  ltm_optimization_scope_params.fix_visual_features_ =
      optimization_factor_configuration.fix_visual_features_;
  ltm_optimization_scope_params.fix_ltm_objects_ =
      optimization_factor_configuration.fix_ltm_objects_;
  ltm_optimization_scope_params.include_visual_factors_ =
      optimization_factor_configuration.include_visual_factors_;
  ltm_optimization_scope_params.include_object_factors_ =
      optimization_factor_configuration.include_object_factors_;
  ltm_optimization_scope_params.use_pom_ =
      optimization_factor_configuration.use_pom_;
  ltm_optimization_scope_params.factor_types_to_exclude = {
      kShapeDimPriorFactorTypeId};
  ltm_optimization_scope_params.min_frame_id_ = min_and_max_frame_id.first;
  ltm_optimization_scope_params.max_frame_id_ = min_and_max_frame_id.second;

  pose_graph_optimizer::ObjectPoseGraphOptimizer<
      ReprojectionErrorFactor,
      util::EmptyStruct,
      ObjectAndReprojectionFeaturePoseGraph>
      optimizer(refresh_residual_checker, residual_creator);

  optimizer.buildPoseGraphOptimization(
      ltm_optimization_scope_params, residual_params, pose_graph, problem);

  bool opt_success = optimizer.solveOptimization(problem, solver_params, {});
  if (!opt_success) {
    LOG(ERROR) << "Optimization failed during LTM extraction";
    return false;
  }

  // Run the optimization again and effectively just remove the bad features
  // TODO How does this affect covariance?
  pose_graph_optimization::OptimizationSolverParams solver_params_copy =
      solver_params;
  solver_params_copy.max_num_iterations_ = 0;

  util::BoostHashSet<std::pair<FactorType, FeatureFactorId>>
      factors_for_bad_feats;
  std::unordered_map<FeatureId, Position3d<double>> feature_estimates;
  pose_graph->getVisualFeatureEstimates(feature_estimates);

  std::unordered_map<FrameId, RawPose3d<double>> before_filter_estimates_raw;
  pose_graph->getRobotPoseEstimates(before_filter_estimates_raw);
  std::unordered_map<FrameId, Pose3D<double>> before_filter_estimates;
  for (const auto &raw_est : before_filter_estimates_raw) {
    before_filter_estimates[raw_est.first] = convertToPose3D(raw_est.second);
  }

  for (const auto &feat_est : feature_estimates) {
    double min_dist_from_viewing_frame = std::numeric_limits<double>::max();

    util::BoostHashSet<std::pair<FactorType, FeatureFactorId>> factors_for_feat;
    pose_graph->getFactorsForFeature(feat_est.first, factors_for_feat);
    for (const std::pair<FactorType, FeatureFactorId> &factor :
         factors_for_feat) {
      if (factor.first != kReprojectionErrorFactorTypeId) {
        LOG(WARNING) << "Unexpected factor type for visual feature "
                     << factor.first;
        continue;
      }
      ReprojectionErrorFactor reprojection_factor;
      if (!pose_graph->getVisualFactor(factor.second, reprojection_factor)) {
        LOG(ERROR) << "Could not find visual factor with id " << factor.second;
        continue;
      }

      if (before_filter_estimates.find(reprojection_factor.frame_id_) ==
          before_filter_estimates.end()) {
        LOG(ERROR) << "Could not find pose estimate for frame "
                   << reprojection_factor.frame_id_;
        continue;
      }

      Pose3D<double> relative_pose =
          before_filter_estimates.at(reprojection_factor.frame_id_);
      Pose3D<double> extrinsics;
      if (!pose_graph->getExtrinsicsForCamera(reprojection_factor.camera_id_,
                                              extrinsics)) {
        LOG(WARNING) << "Could not find extrinsics for camera "
                     << reprojection_factor.camera_id_
                     << "; falling back to robot pose";
      } else {
        relative_pose = combinePoses(relative_pose, extrinsics);
      }

      min_dist_from_viewing_frame =
          std::min((relative_pose.transl_ - feat_est.second).norm(),
                   min_dist_from_viewing_frame);
    }
    if (min_dist_from_viewing_frame > kFarFeatureThreshold) {
      factors_for_bad_feats.insert(factors_for_feat.begin(),
                                   factors_for_feat.end());
    }
  }

  residual_info =
      optimizer.buildPoseGraphOptimization(ltm_optimization_scope_params,
                                           residual_params,
                                           pose_graph,
                                           problem,
                                           factors_for_bad_feats);
  opt_success = optimizer.solveOptimization(problem, solver_params_copy, {});

  if (!opt_success) {
    LOG(ERROR) << "Second round optimization failed during LTM extraction";
    return false;
  }

  return true;
}

/**
 * Parameters used in pairwise covariance extraction process.
 */
class CovarianceExtractorParams {
 public:
  /**
   * See Ceres covariance documentation.
   */
  int num_threads_ = 1;

  /**
   * See Ceres covariance documentation.
   */
  ceres::CovarianceAlgorithmType covariance_estimation_algorithm_type_ =
      ceres::CovarianceAlgorithmType::SPARSE_QR;
};

/**
 * Class for extracting the pairwise covariance long-term map from the
 * optimization problem/pose graph.
 */
template <typename FrontEndObjMapData>
class PairwiseCovarianceLongTermObjectMapExtractor {
 public:
  /**
   * Create the long term map extractor.
   *
   * @param covariance_extractor_params Parameters to be used during
   * covariance extraction.
   */
  PairwiseCovarianceLongTermObjectMapExtractor(
      const CovarianceExtractorParams &covariance_extractor_params,
      const std::function<bool(
          const std::pair<vslam_types_refactor::FactorType,
                          vslam_types_refactor::FeatureFactorId> &,
          const pose_graph_optimization::ObjectVisualPoseGraphResidualParams &,
          const std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph> &,
          ceres::Problem *,
          ceres::ResidualBlockId &,
          util::EmptyStruct &)> &residual_creator,
      const std::function<bool(const FactorType &,
                               const FeatureFactorId &,
                               ObjectId &)> &long_term_map_obj_retriever,
      const pose_graph_optimization::ObjectVisualPoseGraphResidualParams
          &ltm_residual_params,
      const pose_graph_optimization::OptimizationSolverParams
          &ltm_solver_params)
      : covariance_extractor_params_(covariance_extractor_params),
        residual_creator_(residual_creator),
        long_term_map_obj_retriever_(long_term_map_obj_retriever),
        ltm_residual_params_(ltm_residual_params),
        ltm_solver_params_(ltm_solver_params) {}

  ~PairwiseCovarianceLongTermObjectMapExtractor() = default;

  /**
   * Extract the long term map.
   *
   * @param pose_graph[in]      Pose graph from which to extract information
   *                            needed for long-term map.
   * @param problem[in]         The ceres problem from which to extract the
   *                            covariance. Assumes that the problem was left
   *                            in a complete state (not missing variables
   *                            that should be included in the long-term map or
   *                            factored into the covariance extraction.
   * @param long_term_map[out]  Long term map to update with information.
   *
   * @return True if the long term map extraction was successful. False if
   * something failed. If returns false, may or may not have updated the
   * long-term map object.
   */
  bool extractLongTermObjectMap(
      const std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph> &pose_graph,
      const pose_graph_optimizer::OptimizationFactorsEnabledParams
          &optimization_factor_configuration,
      const std::function<bool(FrontEndObjMapData &)>
          front_end_map_data_extractor,
      const std::string &jacobian_output_dir,
      IndependentEllipsoidsLongTermObjectMap<FrontEndObjMapData>
          &long_term_obj_map) {
    std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph> pose_graph_copy =
        pose_graph->makeDeepCopy();
    ceres::Problem problem_for_ltm;
    std::unordered_map<vslam_types_refactor::FactorType,
                       std::unordered_map<vslam_types_refactor::FeatureFactorId,
                                          ceres::ResidualBlockId>>
        residual_info;
    runOptimizationForLtmExtraction(residual_creator_,
                                    ltm_residual_params_,
                                    ltm_solver_params_,
                                    optimization_factor_configuration,
                                    pose_graph_copy,
                                    &problem_for_ltm,
                                    residual_info);

    if (!jacobian_output_dir.empty()) {
      outputJacobianInfo(jacobian_output_dir,
                         residual_info,
                         pose_graph_copy,
                         long_term_map_obj_retriever_,
                         problem_for_ltm);
    }

    EllipsoidResults ellipsoid_results;
    extractEllipsoidEstimates(pose_graph_copy, ellipsoid_results);
    long_term_obj_map.setEllipsoidResults(ellipsoid_results);

    ceres::Covariance::Options covariance_options;
    covariance_options.num_threads = covariance_extractor_params_.num_threads_;
    covariance_options.algorithm_type =
        covariance_extractor_params_.covariance_estimation_algorithm_type_;

    ceres::Covariance covariance_extractor(covariance_options);
    std::vector<ObjectId> object_ids;
    for (const auto &object_id_est_pair : ellipsoid_results.ellipsoids_) {
      object_ids.emplace_back(object_id_est_pair.first);
    }

    // Sort object ids
    std::sort(object_ids.begin(), object_ids.end());

    std::vector<std::pair<const double *, const double *>> covariance_blocks;
    for (size_t obj_1_idx = 0; obj_1_idx < object_ids.size(); obj_1_idx++) {
      ObjectId obj_1 = object_ids[obj_1_idx];
      double *obj_1_ptr;
      pose_graph_copy->getObjectParamPointers(obj_1, &obj_1_ptr);
      for (size_t obj_2_idx = obj_1_idx + 1; obj_2_idx < object_ids.size();
           obj_2_idx++) {
        ObjectId obj_2 = object_ids[obj_2_idx];
        double *obj_2_ptr;
        pose_graph_copy->getObjectParamPointers(obj_2, &obj_2_ptr);
        covariance_blocks.emplace_back(std::make_pair(obj_1_ptr, obj_2_ptr));
      }
    }

    bool covariance_compute_result =
        covariance_extractor.Compute(covariance_blocks, &problem_for_ltm);

    if (!covariance_compute_result) {
      LOG(ERROR) << "Covariance computation failed";
      return false;
    }

    util::BoostHashMap<std::pair<ObjectId, ObjectId>,
                       Eigen::Matrix<double,
                                     kEllipsoidParamterizationSize,
                                     kEllipsoidParamterizationSize>>
        pairwise_ellipsoid_covariances;
    for (size_t obj_1_idx = 0; obj_1_idx < object_ids.size(); obj_1_idx++) {
      ObjectId obj_1 = object_ids[obj_1_idx];
      double *obj_1_ptr;
      pose_graph_copy->getObjectParamPointers(obj_1, &obj_1_ptr);
      for (size_t obj_2_idx = obj_1_idx + 1; obj_2_idx < object_ids.size();
           obj_2_idx++) {
        ObjectId obj_2 = object_ids[obj_2_idx];
        double *obj_2_ptr;
        pose_graph_copy->getObjectParamPointers(obj_2, &obj_2_ptr);
        Eigen::Matrix<double,
                      kEllipsoidParamterizationSize,
                      kEllipsoidParamterizationSize>
            cov_result;
        bool success = covariance_extractor.GetCovarianceBlock(
            obj_1_ptr, obj_2_ptr, cov_result.data());
        if (!success) {
          LOG(ERROR) << "Failed to get the covariance block for objects "
                     << obj_1 << " and " << obj_2;
          return false;
        }
        std::pair<ObjectId, ObjectId> object_pair =
            std::make_pair(obj_1, obj_2);
        pairwise_ellipsoid_covariances[object_pair] = cov_result;
      }
    }

    FrontEndObjMapData front_end_map_data;
    if (!front_end_map_data_extractor(front_end_map_data)) {
      LOG(ERROR) << "Could not extract the front end data required for the "
                    "long-term map";
      return false;
    }
    long_term_obj_map.setFrontEndObjMapData(front_end_map_data);

    long_term_obj_map.setPairwiseEllipsoidCovariance(
        pairwise_ellipsoid_covariances);
    return true;
  }

 private:
  /**
   * Covariance extractor params.
   */
  CovarianceExtractorParams covariance_extractor_params_;

  std::function<bool(
      const std::pair<vslam_types_refactor::FactorType,
                      vslam_types_refactor::FeatureFactorId> &,
      const pose_graph_optimization::ObjectVisualPoseGraphResidualParams &,
      const std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph> &,
      ceres::Problem *,
      ceres::ResidualBlockId &,
      util::EmptyStruct &)>
      residual_creator_;
  std::function<bool(const FactorType &, const FeatureFactorId &, ObjectId &)>
      long_term_map_obj_retriever_;
  pose_graph_optimization::ObjectVisualPoseGraphResidualParams
      ltm_residual_params_;
  pose_graph_optimization::OptimizationSolverParams ltm_solver_params_;
};

template <typename FrontEndObjMapData>
class IndependentEllipsoidsLongTermObjectMapExtractor {
 public:
  /**
   * Create the long term map extractor.
   *
   * @param covariance_extractor_params Parameters to be used during
   * covariance extraction.
   */
  IndependentEllipsoidsLongTermObjectMapExtractor(
      const CovarianceExtractorParams &covariance_extractor_params,
      const std::function<bool(
          const std::pair<vslam_types_refactor::FactorType,
                          vslam_types_refactor::FeatureFactorId> &,
          const pose_graph_optimization::ObjectVisualPoseGraphResidualParams &,
          const std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph> &,
          ceres::Problem *,
          ceres::ResidualBlockId &,
          util::EmptyStruct &)> &residual_creator,
      const std::function<bool(const FactorType &,
                               const FeatureFactorId &,
                               ObjectId &)> &long_term_map_obj_retriever,
      const pose_graph_optimization::ObjectVisualPoseGraphResidualParams
          &ltm_residual_params,
      const pose_graph_optimization::OptimizationSolverParams
          &ltm_solver_params)
      : covariance_extractor_params_(covariance_extractor_params),
        residual_creator_(residual_creator),
        long_term_map_obj_retriever_(long_term_map_obj_retriever),
        ltm_residual_params_(ltm_residual_params),
        ltm_solver_params_(ltm_solver_params) {}

  ~IndependentEllipsoidsLongTermObjectMapExtractor() = default;

  /**
   * Extract the long term map.
   *
   * @param pose_graph[in]      Pose graph from which to extract information
   *                            needed for long-term map.
   * @param problem[in]         The ceres problem from which to extract the
   *                            covariance. Assumes that the problem was left
   *                            in a complete state (not missing variables
   *                            that should be included in the long-term map or
   *                            factored into the covariance extraction.
   * @param long_term_map[out]  Long term map to update with information.
   *
   * @return True if the long term map extraction was successful. False if
   * something failed. If returns false, may or may not have updated the
   * long-term map object.
   */
  bool extractLongTermObjectMap(
      const std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph> &pose_graph,
      const pose_graph_optimizer::OptimizationFactorsEnabledParams
          &optimization_factor_configuration,
      const std::function<bool(FrontEndObjMapData &)>
          front_end_map_data_extractor,
      const std::string &jacobian_output_dir,
      IndependentEllipsoidsLongTermObjectMap<FrontEndObjMapData>
          &long_term_obj_map) {
    std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph> pose_graph_copy =
        pose_graph->makeDeepCopy();
    ceres::Problem problem_for_ltm;
    std::unordered_map<vslam_types_refactor::FactorType,
                       std::unordered_map<vslam_types_refactor::FeatureFactorId,
                                          ceres::ResidualBlockId>>
        residual_info;
    runOptimizationForLtmExtraction(residual_creator_,
                                    ltm_residual_params_,
                                    ltm_solver_params_,
                                    optimization_factor_configuration,
                                    pose_graph_copy,
                                    &problem_for_ltm,
                                    residual_info);

    if (!jacobian_output_dir.empty()) {
      outputJacobianInfo(jacobian_output_dir,
                         residual_info,
                         pose_graph_copy,
                         long_term_map_obj_retriever_,
                         problem_for_ltm);
    }

    EllipsoidResults ellipsoid_results;
    extractEllipsoidEstimates(pose_graph_copy, ellipsoid_results);
    long_term_obj_map.setEllipsoidResults(ellipsoid_results);

    ceres::Covariance::Options covariance_options;

    covariance_options.num_threads = covariance_extractor_params_.num_threads_;
    covariance_options.algorithm_type =
        covariance_extractor_params_.covariance_estimation_algorithm_type_;

    ceres::Covariance covariance_extractor(covariance_options);
    std::vector<ObjectId> object_ids;
    for (const auto &object_id_est_pair : ellipsoid_results.ellipsoids_) {
      object_ids.emplace_back(object_id_est_pair.first);
    }

    std::vector<std::pair<const double *, const double *>> covariance_blocks;
    for (const ObjectId &obj_id : object_ids) {
      double *obj_ptr;
      pose_graph_copy->getObjectParamPointers(obj_id, &obj_ptr);
      covariance_blocks.emplace_back(std::make_pair(obj_ptr, obj_ptr));
    }
    double *frame_1_block;
    pose_graph_copy->getPosePointers(1, &frame_1_block);
    covariance_blocks.emplace_back(
        std::make_pair(frame_1_block, frame_1_block));

    bool covariance_compute_result =
        covariance_extractor.Compute(covariance_blocks, &problem_for_ltm);

    if (!covariance_compute_result) {
      LOG(ERROR) << "Covariance computation failed";
      return false;
    }

    std::unordered_map<ObjectId,
                       Covariance<double, kEllipsoidParamterizationSize>>
        ellipsoid_covariances;
    for (size_t obj_idx = 0; obj_idx < object_ids.size(); obj_idx++) {
      ObjectId obj_id = object_ids[obj_idx];
      double *obj_ptr;
      pose_graph_copy->getObjectParamPointers(obj_id, &obj_ptr);

      Covariance<double, kEllipsoidParamterizationSize> cov_result;
      bool success = covariance_extractor.GetCovarianceBlock(
          obj_ptr, obj_ptr, cov_result.data());
      if (!success) {
        LOG(ERROR) << "Failed to get the covariance block for objects "
                   << obj_id;
        return false;
      }
      ellipsoid_covariances[obj_id] = cov_result;
    }

    LOG(INFO) << "Extracting front end map data";
    FrontEndObjMapData front_end_map_data;
    if (!front_end_map_data_extractor(front_end_map_data)) {
      LOG(ERROR) << "Could not extract the front end data required for the "
                    "long-term map";
      return false;
    }
    long_term_obj_map.setFrontEndObjMapData(front_end_map_data);

    long_term_obj_map.setEllipsoidCovariances(ellipsoid_covariances);
    return true;
  }

 private:
  /**
   * Covariance extractor params.
   */
  CovarianceExtractorParams covariance_extractor_params_;

  std::function<bool(
      const std::pair<vslam_types_refactor::FactorType,
                      vslam_types_refactor::FeatureFactorId> &,
      const pose_graph_optimization::ObjectVisualPoseGraphResidualParams &,
      const std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph> &,
      ceres::Problem *,
      ceres::ResidualBlockId &,
      util::EmptyStruct &)>
      residual_creator_;
  std::function<bool(const FactorType &, const FeatureFactorId &, ObjectId &)>
      long_term_map_obj_retriever_;
  pose_graph_optimization::ObjectVisualPoseGraphResidualParams
      ltm_residual_params_;
  pose_graph_optimization::OptimizationSolverParams ltm_solver_params_;
};

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_LONG_TERM_OBJECT_MAP_EXTRACTION_H
