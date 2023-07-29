//
// Created by amanda on 8/3/22.
//

#ifndef UT_VSLAM_LONG_TERM_OBJECT_MAP_EXTRACTION_H
#define UT_VSLAM_LONG_TERM_OBJECT_MAP_EXTRACTION_H

#include <ceres/ceres.h>
#include <ceres/problem.h>
#include <refactoring/long_term_map/long_term_map_extraction_tunable_params.h>
#include <refactoring/long_term_map/long_term_object_map.h>
#include <refactoring/optimization/jacobian_extraction.h>
#include <refactoring/optimization/object_pose_graph.h>
#include <refactoring/optimization/object_pose_graph_optimizer.h>
#include <refactoring/output_problem_data_extraction.h>
#include <refactoring/types/vslam_types_math_util.h>

namespace vslam_types_refactor {

const static int kMaxJacobianExtractionRetries = 5;
const static int kRankDeficiencyColsBuffer = 20;

/**
 * Parameters used in pairwise covariance extraction process.
 */
struct CovarianceExtractorParams {
  /**
   * See Ceres covariance documentation.
   */
  int num_threads_ = 5;

  /**
   * See Ceres covariance documentation.
   */
  ceres::CovarianceAlgorithmType covariance_estimation_algorithm_type_ =
      ceres::CovarianceAlgorithmType::SPARSE_QR;
};

struct InsufficientRankInfo {
  std::unordered_map<ObjectId, util::BoostHashSet<std::pair<size_t, double>>>
      objects_with_rank_deficient_entries;
  std::unordered_map<FrameId, util::BoostHashSet<std::pair<size_t, double>>>
      frames_with_rank_deficient_entries;
  std::unordered_map<FeatureId, util::BoostHashSet<std::pair<size_t, double>>>
      features_with_rank_deficient_entries;

  double min_non_prob_col_norm_;
};

bool runOptimizationForLtmExtraction(
    const std::function<bool(
        const std::pair<vslam_types_refactor::FactorType,
                        vslam_types_refactor::FeatureFactorId> &,
        const pose_graph_optimization::ObjectVisualPoseGraphResidualParams &,
        const std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph> &,
        const bool &,
        ceres::Problem *,
        ceres::ResidualBlockId &,
        util::EmptyStruct &)> &residual_creator,
    const pose_graph_optimization::ObjectVisualPoseGraphResidualParams
        &residual_params,
    const pose_graph_optimization::OptimizationSolverParams &solver_params,
    const pose_graph_optimizer::OptimizationFactorsEnabledParams
        &optimization_factor_configuration,
    const double &far_feature_threshold,
    const std::optional<std::pair<FrameId, FrameId>> &override_min_max_frame_id,
    std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph> &pose_graph,
    ceres::Problem *problem,
    std::unordered_map<ceres::ResidualBlockId,
                       std::pair<vslam_types_refactor::FactorType,
                                 vslam_types_refactor::FeatureFactorId>>
        &residual_info,
    std::unordered_map<ceres::ResidualBlockId, double>
        &block_ids_and_residuals);

std::pair<bool, std::shared_ptr<ceres::Covariance>> extractCovariance(
    const std::function<bool(
        const std::pair<vslam_types_refactor::FactorType,
                        vslam_types_refactor::FeatureFactorId> &,
        const pose_graph_optimization::ObjectVisualPoseGraphResidualParams &,
        const std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph> &,
        const bool &,
        ceres::Problem *,
        ceres::ResidualBlockId &,
        util::EmptyStruct &)> &residual_creator,
    const pose_graph_optimization::ObjectVisualPoseGraphResidualParams
        &ltm_residual_params,
    const pose_graph_optimization::OptimizationSolverParams &ltm_solver_params,
    const pose_graph_optimizer::OptimizationFactorsEnabledParams
        &optimization_factor_configuration,
    const LongTermMapExtractionTunableParams &long_term_map_tunable_params,
    const std::optional<std::pair<FrameId, FrameId>> &override_min_max_frame_id,
    const std::string &jacobian_output_dir,
    const std::function<bool(const FactorType &,
                             const FeatureFactorId &,
                             ObjectId &)> &long_term_map_obj_retriever,
    const CovarianceExtractorParams &covariance_extractor_params,
    const std::function<
        std::vector<std::pair<const double *, const double *>>()>
        &parameter_block_cov_retriever,
    const std::vector<std::pair<ceres::ResidualBlockId, ParamPriorFactor>>
        &added_factors,
    std::unordered_map<ceres::ResidualBlockId,
                       std::pair<vslam_types_refactor::FactorType,
                                 vslam_types_refactor::FeatureFactorId>>
        &residual_info,
    std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph> &pose_graph_copy,
    ceres::Problem &problem_for_ltm,
    const int &attempt_num = 0);

void getFramesFeaturesAndObjectsForFactor(
    const std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph> &pose_graph,
    const FactorType &factor_type,
    const FeatureFactorId &feature_factor_id,
    const std::function<bool(const FactorType &,
                             const FeatureFactorId &,
                             ObjectId &)> &long_term_map_obj_retriever,
    std::unordered_set<FrameId> &added_frames,
    std::unordered_set<ObjectId> &added_objects,
    std::unordered_set<FeatureId> &added_features);

bool getRankDeficiency(const ceres::Covariance::Options &cov_options,
                       ceres::Problem &problem,
                       int &rank_deficiency);

InsufficientRankInfo findRankDeficiencies(
    const CovarianceExtractorParams &covariance_extractor_params,
    const std::vector<std::pair<ceres::ResidualBlockId, ParamPriorFactor>>
        &added_factors,
    const std::unordered_map<ceres::ResidualBlockId,
                             std::pair<vslam_types_refactor::FactorType,
                                       vslam_types_refactor::FeatureFactorId>>
        &residual_info,
    const std::function<bool(const FactorType &,
                             const FeatureFactorId &,
                             ObjectId &)> &long_term_map_obj_retriever,
    const double &min_col_norm,
    std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph> &pose_graph_copy,
    ceres::Problem &problem_for_ltm,
    double &min_non_prob_norm);

void addPriorToProblemParams(
    const InsufficientRankInfo &insufficient_rank_info,
    const LongTermMapExtractionTunableParams &long_term_map_tunable_params,
    const double &min_non_prob_col_norm,
    std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph> &pose_graph_copy,
    ceres::Problem &problem_for_ltm,
    std::vector<std::pair<ceres::ResidualBlockId, ParamPriorFactor>>
        &added_factors);

std::pair<bool, std::shared_ptr<ceres::Covariance>>
extractCovarianceWithRankDeficiencyHandling(
    const std::function<bool(
        const std::pair<vslam_types_refactor::FactorType,
                        vslam_types_refactor::FeatureFactorId> &,
        const pose_graph_optimization::ObjectVisualPoseGraphResidualParams &,
        const std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph> &,
        const bool &,
        ceres::Problem *,
        ceres::ResidualBlockId &,
        util::EmptyStruct &)> &residual_creator,
    const pose_graph_optimization::ObjectVisualPoseGraphResidualParams
        &ltm_residual_params,
    const pose_graph_optimization::OptimizationSolverParams &ltm_solver_params,
    const pose_graph_optimizer::OptimizationFactorsEnabledParams
        &optimization_factor_configuration,
    const LongTermMapExtractionTunableParams &long_term_map_tunable_params,
    const std::optional<std::pair<FrameId, FrameId>> &override_min_max_frame_id,
    const std::string &jacobian_output_dir,
    const std::function<bool(const FactorType &,
                             const FeatureFactorId &,
                             ObjectId &)> &long_term_map_obj_retriever,
    const CovarianceExtractorParams &covariance_extractor_params,
    const std::function<
        std::vector<std::pair<const double *, const double *>>()>
        &parameter_block_cov_retriever,
    std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph> &pose_graph_copy,
    ceres::Problem &problem_for_ltm);

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
          const bool &,
          ceres::Problem *,
          ceres::ResidualBlockId &,
          util::EmptyStruct &)> &residual_creator,
      const std::function<bool(const FactorType &,
                               const FeatureFactorId &,
                               ObjectId &)> &long_term_map_obj_retriever,
      const LongTermMapExtractionTunableParams &long_term_map_tunable_params,
      const pose_graph_optimization::ObjectVisualPoseGraphResidualParams
          &ltm_residual_params,
      const pose_graph_optimization::OptimizationSolverParams
          &ltm_solver_params)
      : covariance_extractor_params_(covariance_extractor_params),
        residual_creator_(residual_creator),
        long_term_map_obj_retriever_(long_term_map_obj_retriever),
        long_term_map_tunable_params_(long_term_map_tunable_params),
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
      const std::function<bool(std::unordered_map<ObjectId, FrontEndObjMapData>
                                   &)> front_end_map_data_extractor,
      const std::string &jacobian_output_dir,
      const std::optional<std::pair<FrameId, FrameId>>
          &override_min_max_frame_id,
      IndependentEllipsoidsLongTermObjectMap<FrontEndObjMapData>
          &long_term_obj_map) {
    EllipsoidResults prev_run_ellipsoid_results;
    extractEllipsoidEstimates(pose_graph, prev_run_ellipsoid_results);

    std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph> pose_graph_copy =
        pose_graph->makeDeepCopy();
    ceres::Problem problem_for_ltm;

    std::vector<ObjectId> object_ids;
    for (const auto &object_id_est_pair :
         prev_run_ellipsoid_results.ellipsoids_) {
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

    std::function<std::vector<std::pair<const double *, const double *>>()>
        parameter_block_cov_retriever = [&]() { return covariance_blocks; };

    std::pair<bool, std::shared_ptr<ceres::Covariance>> covariance_result =
        extractCovarianceWithRankDeficiencyHandling(
            residual_creator_,
            ltm_residual_params_,
            ltm_solver_params_,
            optimization_factor_configuration,
            long_term_map_tunable_params_,
            override_min_max_frame_id,
            jacobian_output_dir,
            long_term_map_obj_retriever_,
            covariance_extractor_params_,
            parameter_block_cov_retriever,
            pose_graph_copy,
            problem_for_ltm);

    if (!covariance_result.first) {
      LOG(ERROR) << "Covariance extraction failed on the second try with "
                    "additional priors";
      return false;
    }

    EllipsoidResults ellipsoid_results;
    extractEllipsoidEstimates(pose_graph_copy, ellipsoid_results);
    long_term_obj_map.setLtmEllipsoidResults(ellipsoid_results);
    long_term_obj_map.setEllipsoidResults(prev_run_ellipsoid_results);

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
        bool success = covariance_result.second->GetCovarianceBlock(
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

    std::unordered_map<ObjectId, FrontEndObjMapData> front_end_map_data;
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
      const bool &,
      ceres::Problem *,
      ceres::ResidualBlockId &,
      util::EmptyStruct &)>
      residual_creator_;
  std::function<bool(const FactorType &, const FeatureFactorId &, ObjectId &)>
      long_term_map_obj_retriever_;
  LongTermMapExtractionTunableParams long_term_map_tunable_params_;
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
          const bool &,
          ceres::Problem *,
          ceres::ResidualBlockId &,
          util::EmptyStruct &)> &residual_creator,
      const std::function<bool(const FactorType &,
                               const FeatureFactorId &,
                               ObjectId &)> &long_term_map_obj_retriever,
      const LongTermMapExtractionTunableParams &long_term_map_tunable_params,
      const pose_graph_optimization::ObjectVisualPoseGraphResidualParams
          &ltm_residual_params,
      const pose_graph_optimization::OptimizationSolverParams
          &ltm_solver_params)
      : covariance_extractor_params_(covariance_extractor_params),
        residual_creator_(residual_creator),
        long_term_map_obj_retriever_(long_term_map_obj_retriever),
        long_term_map_tunable_params_(long_term_map_tunable_params),
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
      const std::function<bool(std::unordered_map<ObjectId, FrontEndObjMapData>
                                   &)> front_end_map_data_extractor,
      const std::string &jacobian_output_dir,
      const std::optional<std::pair<FrameId, FrameId>>
          &override_min_max_frame_id,
      IndependentEllipsoidsLongTermObjectMap<FrontEndObjMapData>
          &long_term_obj_map) {
    EllipsoidResults prev_run_ellipsoid_results;
    extractEllipsoidEstimates(pose_graph, prev_run_ellipsoid_results);
    std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph> pose_graph_copy =
        pose_graph->makeDeepCopy();
    ceres::Problem problem_for_ltm;
    std::vector<ObjectId> object_ids;
    for (const auto &object_id_est_pair :
         prev_run_ellipsoid_results.ellipsoids_) {
      object_ids.emplace_back(object_id_est_pair.first);
    }
    std::vector<std::pair<const double *, const double *>> covariance_blocks;
    for (const ObjectId &obj_id : object_ids) {
      double *obj_ptr;
      pose_graph_copy->getObjectParamPointers(obj_id, &obj_ptr);
      covariance_blocks.emplace_back(std::make_pair(obj_ptr, obj_ptr));
    }

    std::function<std::vector<std::pair<const double *, const double *>>()>
        parameter_block_cov_retriever = [&]() { return covariance_blocks; };

    std::pair<bool, std::shared_ptr<ceres::Covariance>> covariance_result =
        extractCovarianceWithRankDeficiencyHandling(
            residual_creator_,
            ltm_residual_params_,
            ltm_solver_params_,
            optimization_factor_configuration,
            long_term_map_tunable_params_,
            override_min_max_frame_id,
            jacobian_output_dir,
            long_term_map_obj_retriever_,
            covariance_extractor_params_,
            parameter_block_cov_retriever,
            pose_graph_copy,
            problem_for_ltm);

    if (!covariance_result.first) {
      LOG(ERROR) << "Covariance extraction failed on the second try with "
                    "additional priors";
      return false;
    }

    EllipsoidResults ellipsoid_results;
    extractEllipsoidEstimates(pose_graph_copy, ellipsoid_results);
    long_term_obj_map.setLtmEllipsoidResults(ellipsoid_results);
    long_term_obj_map.setEllipsoidResults(prev_run_ellipsoid_results);

    // TODO consider weakening covariances for
    std::unordered_map<ObjectId,
                       Covariance<double, kEllipsoidParamterizationSize>>
        ellipsoid_covariances;
    for (size_t obj_idx = 0; obj_idx < object_ids.size(); obj_idx++) {
      ObjectId obj_id = object_ids[obj_idx];
      double *obj_ptr;
      pose_graph_copy->getObjectParamPointers(obj_id, &obj_ptr);

      Covariance<double, kEllipsoidParamterizationSize> cov_result;
      bool success = covariance_result.second->GetCovarianceBlock(
          obj_ptr, obj_ptr, cov_result.data());
      if (!success) {
        LOG(ERROR) << "Failed to get the covariance block for objects "
                   << obj_id;
        return false;
      }
      ellipsoid_covariances[obj_id] = cov_result;
    }

    LOG(INFO) << "Extracting front end map data";
    std::unordered_map<ObjectId, FrontEndObjMapData> front_end_map_data;
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
      const bool &,
      ceres::Problem *,
      ceres::ResidualBlockId &,
      util::EmptyStruct &)>
      residual_creator_;
  std::function<bool(const FactorType &, const FeatureFactorId &, ObjectId &)>
      long_term_map_obj_retriever_;
  LongTermMapExtractionTunableParams long_term_map_tunable_params_;
  pose_graph_optimization::ObjectVisualPoseGraphResidualParams
      ltm_residual_params_;
  pose_graph_optimization::OptimizationSolverParams ltm_solver_params_;
};

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_LONG_TERM_OBJECT_MAP_EXTRACTION_H
