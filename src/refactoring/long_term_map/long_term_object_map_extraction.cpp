//
// Created by amanda on 5/17/23.
//

#include <ceres/crs_matrix.h>
#include <ceres/problem.h>
#include <refactoring/factors/parameter_prior.h>
#include <refactoring/long_term_map/long_term_object_map_extraction.h>

#include <eigen3/Eigen/SparseCore>
#include <eigen3/Eigen/SparseQR>
#include <suitesparse/SuiteSparseQR.hpp>

namespace vslam_types_refactor {
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
        &block_ids_and_residuals) {
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
  if (override_min_max_frame_id.has_value()) {
    ltm_optimization_scope_params.min_frame_id_ = std::max(
        override_min_max_frame_id.value().first, min_and_max_frame_id.first);
    ltm_optimization_scope_params.max_frame_id_ = std::min(
        override_min_max_frame_id.value().second, min_and_max_frame_id.second);
  } else {
    ltm_optimization_scope_params.min_frame_id_ = min_and_max_frame_id.first;
    ltm_optimization_scope_params.max_frame_id_ = min_and_max_frame_id.second;
  }
  ltm_optimization_scope_params.force_include_ltm_objs_ = true;

  std::function<bool(
      const std::pair<vslam_types_refactor::FactorType,
                      vslam_types_refactor::FeatureFactorId> &,
      const pose_graph_optimization::ObjectVisualPoseGraphResidualParams &,
      const std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph> &,
      ceres::Problem *,
      ceres::ResidualBlockId &,
      util::EmptyStruct &)>
      debug_residual_creator =
          [&](const std::pair<vslam_types_refactor::FactorType,
                              vslam_types_refactor::FeatureFactorId> &factor_id,
              const pose_graph_optimization::ObjectVisualPoseGraphResidualParams
                  &solver_residual_params,
              const std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph>
                  &pose_graph,
              ceres::Problem *problem,
              ceres::ResidualBlockId &residual_id,
              util::EmptyStruct &cached_info) {
            return residual_creator(factor_id,
                                    solver_residual_params,
                                    pose_graph,
                                    true,
                                    problem,
                                    residual_id,
                                    cached_info);
          };

  pose_graph_optimizer::ObjectPoseGraphOptimizer<
      ReprojectionErrorFactor,
      util::EmptyStruct,
      ObjectAndReprojectionFeaturePoseGraph>
      optimizer(refresh_residual_checker, debug_residual_creator);

  std::optional<OptimizationLogger> opt_log;

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
    if (min_dist_from_viewing_frame > far_feature_threshold) {
      LOG(INFO) << "Minimum distance from viewing frame for feature "
                << feat_est.first << " was " << min_dist_from_viewing_frame
                << " (more than threshold " << far_feature_threshold
                << "). Excluding";
      factors_for_bad_feats.insert(factors_for_feat.begin(),
                                   factors_for_feat.end());
    }
  }

  residual_info =
      optimizer.buildPoseGraphOptimization(ltm_optimization_scope_params,
                                           residual_params,
                                           pose_graph,
                                           problem,
                                           opt_log,
                                           factors_for_bad_feats);

  std::shared_ptr<std::unordered_map<ceres::ResidualBlockId, double>>
      block_ids_and_residuals_ptr = std::make_shared<
          std::unordered_map<ceres::ResidualBlockId, double>>();
  bool opt_success = optimizer.solveOptimization(
      problem, solver_params_copy, {}, opt_log, block_ids_and_residuals_ptr);

  block_ids_and_residuals = *block_ids_and_residuals_ptr;

  if (!opt_success) {
    LOG(ERROR) << "Second round optimization failed during LTM extraction";
    return false;
  }

  return true;
}

bool getRankDeficiency(const ceres::Covariance::Options &cov_options,
                       ceres::Problem &problem,
                       int &rank_deficiency) {
  using EigenSparseMatrix = Eigen::SparseMatrix<double, Eigen::ColMajor>;

  ceres::Problem::EvaluateOptions eval_options;
  eval_options.apply_loss_function = cov_options.apply_loss_function;
  eval_options.num_threads = cov_options.num_threads;

  // Code based on
  // https://github.com/ceres-solver/ceres-solver/blob/master/internal/ceres/covariance_impl.cc
  // Determine an ordering for the parameter block, by sorting the
  // parameter blocks by their pointers.
  std::vector<double *> all_parameter_blocks;
  problem.GetParameterBlocks(&all_parameter_blocks);
  std::unordered_set<double *> parameter_blocks_in_use;
  std::vector<ceres::ResidualBlockId> residual_blocks;
  problem.GetResidualBlocks(&residual_blocks);

  for (int i = 0; i < residual_blocks.size(); ++i) {
    ceres::ResidualBlockId residual_block = residual_blocks[i];
    std::vector<double *> parameter_blocks_for_res;
    problem.GetParameterBlocksForResidualBlock(residual_block,
                                               &parameter_blocks_for_res);
    parameter_blocks_in_use.insert(parameter_blocks_for_res.begin(),
                                   parameter_blocks_for_res.end());
  }

  std::vector<double *> &active_parameter_blocks =
      eval_options.parameter_blocks;
  active_parameter_blocks.clear();
  for (int i = 0; i < all_parameter_blocks.size(); ++i) {
    double *parameter_block = all_parameter_blocks[i];

    if (!problem.IsParameterBlockConstant(parameter_block) &&
        (parameter_blocks_in_use.count(parameter_block) > 0)) {
      active_parameter_blocks.push_back(parameter_block);
    }
  }

  std::sort(active_parameter_blocks.begin(), active_parameter_blocks.end());

  ceres::CRSMatrix jacobian;
  problem.Evaluate(eval_options, NULL, NULL, NULL, &jacobian);
  if (cov_options.sparse_linear_algebra_library_type == ceres::SUITE_SPARSE) {
    LOG(INFO) << "Rank detection using suite sparse";
    // Construct a compressed column form of the Jacobian.
    const int num_rows = jacobian.num_rows;
    const int num_cols = jacobian.num_cols;
    const int num_nonzeros = jacobian.values.size();

    std::vector<SuiteSparse_long> transpose_rows(num_cols + 1, 0);
    std::vector<SuiteSparse_long> transpose_cols(num_nonzeros, 0);
    std::vector<double> transpose_values(num_nonzeros, 0);

    for (int idx = 0; idx < num_nonzeros; ++idx) {
      transpose_rows[jacobian.cols[idx] + 1] += 1;
    }

    for (int i = 1; i < transpose_rows.size(); ++i) {
      transpose_rows[i] += transpose_rows[i - 1];
    }

    for (int r = 0; r < num_rows; ++r) {
      for (int idx = jacobian.rows[r]; idx < jacobian.rows[r + 1]; ++idx) {
        const int c = jacobian.cols[idx];
        const int transpose_idx = transpose_rows[c];
        transpose_cols[transpose_idx] = r;
        transpose_values[transpose_idx] = jacobian.values[idx];
        ++transpose_rows[c];
      }
    }

    for (int i = transpose_rows.size() - 1; i > 0; --i) {
      transpose_rows[i] = transpose_rows[i - 1];
    }
    transpose_rows[0] = 0;

    cholmod_sparse cholmod_jacobian;
    cholmod_jacobian.nrow = num_rows;
    cholmod_jacobian.ncol = num_cols;
    cholmod_jacobian.nzmax = num_nonzeros;
    cholmod_jacobian.nz = NULL;
    cholmod_jacobian.p = reinterpret_cast<void *>(&transpose_rows[0]);
    cholmod_jacobian.i = reinterpret_cast<void *>(&transpose_cols[0]);
    cholmod_jacobian.x = reinterpret_cast<void *>(&transpose_values[0]);
    cholmod_jacobian.z = NULL;
    cholmod_jacobian.stype = 0;  // Matrix is not symmetric.
    cholmod_jacobian.itype = CHOLMOD_LONG;
    cholmod_jacobian.xtype = CHOLMOD_REAL;
    cholmod_jacobian.dtype = CHOLMOD_DOUBLE;
    cholmod_jacobian.sorted = 1;
    cholmod_jacobian.packed = 1;

    cholmod_common cc;
    cholmod_l_start(&cc);

    cholmod_sparse *R = NULL;
    SuiteSparse_long *permutation = NULL;

    // Compute a Q-less QR factorization of the Jacobian. Since we are
    // only interested in inverting J'J = R'R, we do not need Q. This
    // saves memory and gives us R as a permuted compressed column
    // sparse matrix.
    //
    // TODO(sameeragarwal): Currently the symbolic factorization and the
    // numeric factorization is done at the same time, and this does not
    // explicitly account for the block column and row structure in the
    // matrix. When using AMD, we have observed in the past that
    // computing the ordering with the block matrix is significantly
    // more efficient, both in runtime as well as the quality of
    // ordering computed. So, it maybe worth doing that analysis
    // separately.
    const SuiteSparse_long rank = SuiteSparseQR<double>(SPQR_ORDERING_BESTAMD,
                                                        SPQR_DEFAULT_TOL,
                                                        cholmod_jacobian.ncol,
                                                        &cholmod_jacobian,
                                                        &R,
                                                        &permutation,
                                                        &cc);
    CHECK_NOTNULL(permutation);
    CHECK_NOTNULL(R);

    rank_deficiency = cholmod_jacobian.ncol - rank;

    free(permutation);
    cholmod_l_free_sparse(&R, &cc);
    cholmod_l_finish(&cc);

  } else {
    LOG(INFO) << "Rank detection using eigen sparse";

    typedef Eigen::SparseMatrix<double, Eigen::ColMajor> EigenSparseMatrix;

    // Convert the matrix to column major order as required by SparseQR.
    EigenSparseMatrix sparse_jacobian =
        Eigen::MappedSparseMatrix<double, Eigen::RowMajor>(
            jacobian.num_rows,
            jacobian.num_cols,
            static_cast<int>(jacobian.values.size()),
            jacobian.rows.data(),
            jacobian.cols.data(),
            jacobian.values.data());

    Eigen::SparseQR<EigenSparseMatrix, Eigen::COLAMDOrdering<int>> qr_solver(
        sparse_jacobian);

    rank_deficiency = jacobian.num_cols - qr_solver.rank();
  }
  LOG(INFO) << "Rank deficiency: " << rank_deficiency;
  return true;
}

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
    const int &attempt_num) {
  std::unordered_map<ceres::ResidualBlockId, double> block_ids_and_residuals;
  runOptimizationForLtmExtraction(
      residual_creator,
      ltm_residual_params,
      ltm_solver_params,
      optimization_factor_configuration,
      long_term_map_tunable_params.far_feature_threshold_,
      override_min_max_frame_id,
      pose_graph_copy,
      &problem_for_ltm,
      residual_info,
      block_ids_and_residuals);

  if (!jacobian_output_dir.empty()) {
    outputJacobianInfo(jacobian_output_dir,
                       residual_info,
                       block_ids_and_residuals,
                       added_factors,
                       pose_graph_copy,
                       long_term_map_obj_retriever,
                       problem_for_ltm,
                       attempt_num);
  }

  ceres::Covariance::Options covariance_options;

  covariance_options.num_threads = covariance_extractor_params.num_threads_;
  covariance_options.algorithm_type =
      covariance_extractor_params.covariance_estimation_algorithm_type_;

  std::shared_ptr<ceres::Covariance> covariance_extractor =
      std::make_shared<ceres::Covariance>(covariance_options);

  std::vector<std::pair<const double *, const double *>> covariance_blocks =
      parameter_block_cov_retriever();

  bool covariance_compute_result =
      covariance_extractor->Compute(covariance_blocks, &problem_for_ltm);

  if (!covariance_compute_result) {
    LOG(WARNING) << "Covariance computation failed";
  }
  return std::make_pair(covariance_compute_result, covariance_extractor);
}

void getFramesFeaturesAndObjectsForFactor(
    const std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph> &pose_graph,
    const FactorType &factor_type,
    const FeatureFactorId &feature_factor_id,
    const std::function<bool(const FactorType &,
                             const FeatureFactorId &,
                             ObjectId &)> &long_term_map_obj_retriever,
    std::unordered_set<FrameId> &added_frames,
    std::unordered_set<ObjectId> &added_objects,
    std::unordered_set<FeatureId> &added_features) {
  if (factor_type == kObjectObservationFactorTypeId) {
    ObjectObservationFactor factor;
    if (!pose_graph->getObjectObservationFactor(feature_factor_id, factor)) {
      LOG(ERROR) << "Could not find object observation factor with id "
                 << feature_factor_id << "; not adding to pose graph";
      return;
    }
    added_objects.insert(factor.object_id_);
    added_frames.insert(factor.frame_id_);
  } else if (factor_type == kReprojectionErrorFactorTypeId) {
    ReprojectionErrorFactor factor;
    if (!pose_graph->getVisualFactor(feature_factor_id, factor)) {
      LOG(ERROR) << "Could not find visual feature factor with id "
                 << feature_factor_id << "; not adding to pose graph";
      return;
    }
    added_frames.insert(factor.frame_id_);
    added_features.insert(factor.feature_id_);

  } else if (factor_type == kShapeDimPriorFactorTypeId) {
    ShapeDimPriorFactor factor;
    if (!pose_graph->getShapeDimPriorFactor(feature_factor_id, factor)) {
      LOG(ERROR) << "Could not find shape dim prior factor with id "
                 << feature_factor_id << "; not adding to pose graph";
      return;
    }
    added_objects.insert(factor.object_id_);
  } else if (factor_type == kLongTermMapFactorTypeId) {
    ObjectId ltm_obj_id;
    if (!long_term_map_obj_retriever(
            factor_type, feature_factor_id, ltm_obj_id)) {
      LOG(ERROR) << "Could not find object id for long term map factor with id "
                 << feature_factor_id;
      return;
    }
    added_objects.insert(ltm_obj_id);
  } else if (factor_type == kPairwiseRobotPoseFactorTypeId) {
    RelPoseFactor factor;
    if (!pose_graph->getPoseFactor(feature_factor_id, factor)) {
      LOG(ERROR) << "Could not find relative pose factor with id "
                 << feature_factor_id << "; not adding to pose graph";
      return;
    }
    added_frames.insert(factor.frame_id_1_);
    added_frames.insert(factor.frame_id_2_);
  } else if (factor_type == kPairwiseErrorFactorTypeId) {
    LOG(ERROR) << "Pairwise error observation type not supported with a "
                  "reprojection error factor graph";
    return;
  } else {
    LOG(ERROR) << "Unrecognized factor type " << factor_type
               << "; not adding residual";
    return;
  }
}

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
    ceres::Problem &problem_for_ltm) {
  std::vector<ceres::ResidualBlockId> residual_block_ids;

  std::unordered_set<FrameId> added_frames;
  std::unordered_set<ObjectId> added_objects;
  std::unordered_set<FeatureId> added_features;

  std::vector<double *> ordered_parameter_blocks;
  std::vector<ParameterBlockInfo> ordered_parameter_block_infos;

  for (const auto &residual_info_entry : residual_info) {
    residual_block_ids.emplace_back(residual_info_entry.first);
    getFramesFeaturesAndObjectsForFactor(pose_graph_copy,
                                         residual_info_entry.second.first,
                                         residual_info_entry.second.second,
                                         long_term_map_obj_retriever,
                                         added_frames,
                                         added_objects,
                                         added_features);
  }
  for (const std::pair<ceres::ResidualBlockId, ParamPriorFactor> &added_factor :
       added_factors) {
    residual_block_ids.emplace_back(added_factor.first);
    if (added_factor.second.frame_id_.has_value()) {
      added_frames.insert(added_factor.second.frame_id_.value());
    } else if (added_factor.second.feat_id_.has_value()) {
      added_features.insert(added_factor.second.feat_id_.value());
    } else if (added_factor.second.obj_id_.has_value()) {
      added_objects.insert(added_factor.second.obj_id_.value());
    } else {
      LOG(ERROR) << "Param prior for unknown block type";
    }
  }

  std::vector<FeatureId> added_features_vec;
  added_features_vec.insert(
      added_features_vec.end(), added_features.begin(), added_features.end());
  std::sort(added_features_vec.begin(), added_features_vec.end());
  std::vector<FrameId> added_frames_vec;
  added_frames_vec.insert(
      added_frames_vec.end(), added_frames.begin(), added_frames.end());
  std::sort(added_frames_vec.begin(), added_frames_vec.end());
  std::vector<ObjectId> added_objects_vec;
  added_objects_vec.insert(
      added_objects_vec.end(), added_objects.begin(), added_objects.end());
  std::sort(added_objects_vec.begin(), added_objects_vec.end());

  for (const FeatureId &feat_id : added_features_vec) {
    ParameterBlockInfo param_block_info;
    param_block_info.feature_id_ = feat_id;
    double *feature_ptr;
    pose_graph_copy->getFeaturePointers(feat_id, &feature_ptr);
    ordered_parameter_blocks.emplace_back(feature_ptr);
    ordered_parameter_block_infos.emplace_back(param_block_info);
  }
  for (const FrameId &frame_id : added_frames_vec) {
    ParameterBlockInfo param_block_info;
    param_block_info.frame_id_ = frame_id;
    double *robot_pose_ptr;
    pose_graph_copy->getPosePointers(frame_id, &robot_pose_ptr);
    ordered_parameter_blocks.emplace_back(robot_pose_ptr);
    ordered_parameter_block_infos.emplace_back(param_block_info);
  }
  for (const ObjectId &obj_id : added_objects_vec) {
    ParameterBlockInfo param_block_info;
    param_block_info.obj_id_ = obj_id;
    double *obj_ptr;
    pose_graph_copy->getObjectParamPointers(obj_id, &obj_ptr);
    ordered_parameter_blocks.emplace_back(obj_ptr);
    ordered_parameter_block_infos.emplace_back(param_block_info);
  }

  ceres::Problem::EvaluateOptions options;
  options.apply_loss_function = true;
  options.residual_blocks = residual_block_ids;
  options.parameter_blocks = ordered_parameter_blocks;
  ceres::CRSMatrix sparse_jacobian_ordered;
  problem_for_ltm.Evaluate(
      options, nullptr, nullptr, nullptr, &sparse_jacobian_ordered);

  std::unordered_map<size_t, double> norm_for_cols;
  for (size_t val_num = 0; val_num < sparse_jacobian_ordered.values.size();
       val_num++) {
    size_t col_num = sparse_jacobian_ordered.cols[val_num];
    if (norm_for_cols.find(col_num) == norm_for_cols.end()) {
      norm_for_cols[col_num] = 0.0;
    }
    norm_for_cols[col_num] += pow(sparse_jacobian_ordered.values[val_num], 2);
  }

  ceres::Covariance::Options covariance_options;

  covariance_options.num_threads = covariance_extractor_params.num_threads_;
  covariance_options.algorithm_type =
      covariance_extractor_params.covariance_estimation_algorithm_type_;

  int rank_deficiency;
  getRankDeficiency(covariance_options, problem_for_ltm, rank_deficiency);

  // TODO consider adding this buffer to the config
  // In the past, there are columns close to those with the minimum rank
  // adding some buffer allows us to add a prior for those as well to hopefully
  // fix the rank issue with less retries
  size_t rank_deficiency_compensation_count =
      std::min((size_t)rank_deficiency + kRankDeficiencyColsBuffer + 1, norm_for_cols.size());
  std::vector<std::pair<size_t, double>> smallest_n(
      rank_deficiency_compensation_count);
  std::partial_sort_copy(norm_for_cols.begin(),
                         norm_for_cols.end(),
                         smallest_n.begin(),
                         smallest_n.end(),
                         [](std::pair<const size_t, double> const &l,
                            std::pair<const size_t, double> const &r) {
                           return l.second < r.second;
                         });

  std::vector<size_t> rank_deficient_cols;
  //  for (const auto &col_and_norm : norm_for_cols) {
  //    if (col_and_norm.second < min_col_norm) {
  //      rank_deficient_cols.emplace_back(col_and_norm.first);
  //    }
  //  }

  for (size_t col_size_idx = 0; col_size_idx < smallest_n.size() - 1;
       col_size_idx++) {
    rank_deficient_cols.emplace_back(smallest_n.at(col_size_idx).first);
    LOG(INFO) << "Col num " << smallest_n.at(col_size_idx).first;
    LOG(INFO) << "Norm: " << smallest_n.at(col_size_idx).second;
  }
  std::sort(rank_deficient_cols.begin(), rank_deficient_cols.end());

  InsufficientRankInfo insufficient_rank_info;

  insufficient_rank_info.min_non_prob_col_norm_ = smallest_n.back().second;

  LOG(INFO) << "Minimum non-problem column norm "
            << insufficient_rank_info.min_non_prob_col_norm_;
  if (rank_deficient_cols.empty()) {
    LOG(WARNING) << "No rank deficient columns identified ";
    return insufficient_rank_info;
  }

  size_t current_param_block = 0;
  size_t current_param_in_block_idx = 0;
  int col_num = 0;
  size_t next_small_norm_column = 0;
  while (col_num <= rank_deficient_cols.back()) {
    ParameterBlockInfo param_block =
        ordered_parameter_block_infos[current_param_block];
    if (param_block.frame_id_.has_value()) {
      if (rank_deficient_cols[next_small_norm_column] == col_num) {
        FrameId frame = param_block.frame_id_.value();
        if (insufficient_rank_info.frames_with_rank_deficient_entries.find(
                frame) ==
            insufficient_rank_info.frames_with_rank_deficient_entries.end()) {
          insufficient_rank_info.frames_with_rank_deficient_entries[frame] = {};
        }
        std::pair<size_t, double> deficiency_info = std::make_pair(
            current_param_in_block_idx,
            norm_for_cols[rank_deficient_cols[next_small_norm_column]]);
        insufficient_rank_info.frames_with_rank_deficient_entries[frame].insert(
            deficiency_info);
        next_small_norm_column++;
      }

      if (current_param_in_block_idx < 5) {
        current_param_in_block_idx++;
      } else if (current_param_in_block_idx == 5) {
        current_param_in_block_idx = 0;
        current_param_block++;
      } else {
        LOG(ERROR) << "Current param in block index was supposed to be between "
                      "0 and 5 inclusive but was "
                   << current_param_in_block_idx;
        exit(1);
      }

    } else if (param_block.obj_id_.has_value()) {
      if (rank_deficient_cols[next_small_norm_column] == col_num) {
        ObjectId obj = param_block.obj_id_.value();
        if (insufficient_rank_info.objects_with_rank_deficient_entries.find(
                obj) ==
            insufficient_rank_info.objects_with_rank_deficient_entries.end()) {
          insufficient_rank_info.objects_with_rank_deficient_entries[obj] = {};
        }
        std::pair<size_t, double> deficiency_info = std::make_pair(
            current_param_in_block_idx,
            norm_for_cols[rank_deficient_cols[next_small_norm_column]]);
        insufficient_rank_info.objects_with_rank_deficient_entries[obj].insert(
            deficiency_info);
        next_small_norm_column++;
      }
      if (current_param_in_block_idx < (kEllipsoidParamterizationSize - 1)) {
        current_param_in_block_idx++;
      } else if (current_param_in_block_idx ==
                 (kEllipsoidParamterizationSize - 1)) {
        current_param_in_block_idx = 0;
        current_param_block++;
      } else {
        LOG(ERROR) << "Current param in block index was supposed to be between "
                      "0 and "
                   << (kEllipsoidParamterizationSize - 1)
                   << " inclusive but was " << current_param_in_block_idx;
        exit(1);
      }
    } else if (param_block.feature_id_.has_value()) {
      if (rank_deficient_cols[next_small_norm_column] == col_num) {
        FeatureId feat = param_block.feature_id_.value();
        if (insufficient_rank_info.features_with_rank_deficient_entries.find(
                feat) ==
            insufficient_rank_info.features_with_rank_deficient_entries.end()) {
          insufficient_rank_info
              .features_with_rank_deficient_entries[feat] = {};
        }
        std::pair<size_t, double> deficiency_info = std::make_pair(
            current_param_in_block_idx,
            norm_for_cols[rank_deficient_cols[next_small_norm_column]]);
        insufficient_rank_info.features_with_rank_deficient_entries[feat]
            .insert(deficiency_info);
        next_small_norm_column++;
      }
      if (current_param_in_block_idx < 2) {
        current_param_in_block_idx++;
      } else if (current_param_in_block_idx == 2) {
        current_param_in_block_idx = 0;
        current_param_block++;
      } else {
        LOG(ERROR) << "Current param in block index was supposed to be between "
                      "0 and 5 inclusive but was "
                   << current_param_in_block_idx;
        exit(1);
      }
    } else {
      LOG(ERROR) << "Param block had no id that wasn't empty. Error.";
      exit(1);
    }
    col_num++;
  }
  return insufficient_rank_info;
}

void addPriorToProblemParams(
    const InsufficientRankInfo &insufficient_rank_info,
    const LongTermMapExtractionTunableParams &long_term_map_tunable_params,
    std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph> &pose_graph_copy,
    ceres::Problem &problem_for_ltm,
    std::vector<std::pair<ceres::ResidualBlockId, ParamPriorFactor>>
        &added_factors) {
  double min_col_norm = insufficient_rank_info.min_non_prob_col_norm_;
  //  double min_col_norm = long_term_map_tunable_params.min_col_norm_;
  if (!insufficient_rank_info.features_with_rank_deficient_entries.empty()) {
    std::unordered_map<FeatureId, Position3d<double>> visual_feature_estimates;
    pose_graph_copy->getVisualFeatureEstimates(visual_feature_estimates);

    // Find the features for which there are rank deficient entries (can be
    // multiple entries for a single feature)
    for (const auto &problem_feat_info :
         insufficient_rank_info.features_with_rank_deficient_entries) {
      double *feature_position_block;
      if (!pose_graph_copy->getFeaturePointers(problem_feat_info.first,
                                               &feature_position_block)) {
        LOG(ERROR)
            << "Could not find visual feature parameter block for feature "
            << problem_feat_info.first << "; Not adding prior";
        continue;
      }
      if (visual_feature_estimates.find(problem_feat_info.first) ==
          visual_feature_estimates.end()) {
        LOG(ERROR) << "Could not find estimate for feature "
                   << problem_feat_info.first << "; Not adding prior";
        continue;
      }
      Position3d<double> feat_pos =
          visual_feature_estimates.at(problem_feat_info.first);
      // Iterate over the entries that are rank deficient
      for (const std::pair<size_t, double> &problem_param_info :
           problem_feat_info.second) {
        double mean_val;
        if (problem_param_info.first == 0) {
          mean_val = feat_pos.x();
        } else if (problem_param_info.first == 1) {
          mean_val = feat_pos.y();
        } else if (problem_param_info.first == 2) {
          mean_val = feat_pos.z();
        } else {
          LOG(ERROR) << "Had parameter idx in param block not in 0-2 range for "
                        "visual feature. Skipping";
          continue;
        }
        ParamPriorFactor param_prior_factor(
            std::nullopt,
            problem_feat_info.first,
            std::nullopt,
            problem_param_info.first,
            mean_val,
            1 / sqrt(min_col_norm - problem_param_info.second));
        ceres::ResidualBlockId res_block_id = problem_for_ltm.AddResidualBlock(
            ParameterPrior::createParameterPrior<3>(
                problem_param_info.first,
                param_prior_factor.param_mean_,
                param_prior_factor.param_std_dev_),
            nullptr,
            feature_position_block);
        added_factors.emplace_back(
            std::make_pair(res_block_id, param_prior_factor));
      }
    }
  }

  if (!insufficient_rank_info.objects_with_rank_deficient_entries.empty()) {
    std::unordered_map<ObjectId, std::pair<std::string, RawEllipsoid<double>>>
        object_estimates;
    pose_graph_copy->getObjectEstimates(object_estimates);
    for (const auto &problem_obj_info :
         insufficient_rank_info.objects_with_rank_deficient_entries) {
      double *obj_pose_block;
      if (!pose_graph_copy->getObjectParamPointers(problem_obj_info.first,
                                                   &obj_pose_block)) {
        LOG(ERROR) << "Could not find object parameter block for object "
                   << problem_obj_info.first << "; Not adding prior";
        continue;
      }
      if (object_estimates.find(problem_obj_info.first) ==
          object_estimates.end()) {
        LOG(ERROR) << "Could not find estimate for obj "
                   << problem_obj_info.first << "; Not adding prior";
        continue;
      }
      RawEllipsoid<double> obj_est =
          object_estimates.at(problem_obj_info.first).second;
      for (const std::pair<size_t, double> &problem_param_info :
           problem_obj_info.second) {
        if ((problem_param_info.first < 0) ||
            (problem_param_info.first >= kEllipsoidParamterizationSize)) {
          LOG(ERROR) << "Had parameter idx in param block not in 0-"
                     << (kEllipsoidParamterizationSize - 1)
                     << " range for object. Skipping";
          continue;
        }
        double mean_val = obj_est(problem_param_info.first);

        ParamPriorFactor param_prior_factor(
            std::nullopt,
            std::nullopt,
            problem_obj_info.first,
            problem_param_info.first,
            mean_val,
            1 / sqrt(min_col_norm - problem_param_info.second));
        ceres::ResidualBlockId res_block_id = problem_for_ltm.AddResidualBlock(
            ParameterPrior::createParameterPrior<kEllipsoidParamterizationSize>(
                problem_param_info.first,
                param_prior_factor.param_mean_,
                param_prior_factor.param_std_dev_),
            nullptr,
            obj_pose_block);
        added_factors.emplace_back(
            std::make_pair(res_block_id, param_prior_factor));
      }
    }
  }

  if (!insufficient_rank_info.frames_with_rank_deficient_entries.empty()) {
    for (const auto &problem_frame_info :
         insufficient_rank_info.frames_with_rank_deficient_entries) {
      double *frame_block;
      if (!pose_graph_copy->getPosePointers(problem_frame_info.first,
                                            &frame_block)) {
        LOG(ERROR) << "Could not find frame parameter block for frame "
                   << problem_frame_info.first << "; Not adding prior";
        continue;
      }
      std::optional<RawPose3d<double>> opt_raw_pose =
          pose_graph_copy->getRobotPose(problem_frame_info.first);
      if (!opt_raw_pose.has_value()) {
        LOG(ERROR) << "Could not find estimate for pose "
                   << problem_frame_info.first << "; Not adding prior";
        continue;
      }
      RawPose3d<double> raw_pose = opt_raw_pose.value();
      for (const std::pair<size_t, double> &problem_param_info :
           problem_frame_info.second) {
        if ((problem_param_info.first < 0) || (problem_param_info.first >= 6)) {
          LOG(ERROR) << "Had parameter idx in param block not in 0-5 range for "
                        "frame. Skipping";
          continue;
        }
        double mean_val = raw_pose(problem_param_info.first);

        ParamPriorFactor param_prior_factor(
            problem_frame_info.first,
            std::nullopt,
            std::nullopt,
            problem_param_info.first,
            mean_val,
            1 / sqrt(min_col_norm - problem_param_info.second));
        ceres::ResidualBlockId res_block_id = problem_for_ltm.AddResidualBlock(
            ParameterPrior::createParameterPrior<6>(
                problem_param_info.first,
                param_prior_factor.param_mean_,
                param_prior_factor.param_std_dev_),
            nullptr,
            frame_block);
        added_factors.emplace_back(
            std::make_pair(res_block_id, param_prior_factor));
      }
    }
  }
}

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
    ceres::Problem &problem_for_ltm) {
  std::unordered_map<ceres::ResidualBlockId,
                     std::pair<vslam_types_refactor::FactorType,
                               vslam_types_refactor::FeatureFactorId>>
      residual_info;
  std::vector<std::pair<ceres::ResidualBlockId, ParamPriorFactor>>
      added_factors;
  std::pair<bool, std::shared_ptr<ceres::Covariance>> covariance_result =
      extractCovariance(residual_creator,
                        ltm_residual_params,
                        ltm_solver_params,
                        optimization_factor_configuration,
                        long_term_map_tunable_params,
                        override_min_max_frame_id,
                        jacobian_output_dir,
                        long_term_map_obj_retriever,
                        covariance_extractor_params,
                        parameter_block_cov_retriever,
                        added_factors,
                        residual_info,
                        pose_graph_copy,
                        problem_for_ltm);

  int retry_count = 0;
  // TODO consider adding retry maximum to
  while ((!covariance_result.first) && (retry_count < kMaxJacobianExtractionRetries)) {
    retry_count++;
    LOG(INFO) << "Retrying rank deficient jacobian, retry num " << retry_count;
    // Identify bad parameter blocks + values
    InsufficientRankInfo insufficient_rank_info =
        findRankDeficiencies(covariance_extractor_params,
                             added_factors,
                             residual_info,
                             long_term_map_obj_retriever,
                             long_term_map_tunable_params.min_col_norm_,
                             pose_graph_copy,
                             problem_for_ltm);

    if (!insufficient_rank_info.features_with_rank_deficient_entries.empty()) {
      LOG(INFO) << "Features with problems ";
      for (const auto &insufficient_feat_info :
           insufficient_rank_info.features_with_rank_deficient_entries) {
        for (const auto &indiv_param_info : insufficient_feat_info.second) {
          LOG(INFO) << "Feat: " << insufficient_feat_info.first
                    << ", param idx: " << indiv_param_info.first << ";, norm"
                    << indiv_param_info.second;
        }
      }
    }

    if (!insufficient_rank_info.objects_with_rank_deficient_entries.empty()) {
      LOG(INFO) << "Objects with problems ";
      for (const auto &insufficient_obj_info :
           insufficient_rank_info.objects_with_rank_deficient_entries) {
        for (const auto &indiv_param_info : insufficient_obj_info.second) {
          LOG(INFO) << "Obj: " << insufficient_obj_info.first
                    << ", param idx: " << indiv_param_info.first << ";, norm "
                    << indiv_param_info.second;
        }
      }
    }

    if (!insufficient_rank_info.frames_with_rank_deficient_entries.empty()) {
      LOG(INFO) << "Frames with problems ";
      for (const auto &insufficient_frame_info :
           insufficient_rank_info.frames_with_rank_deficient_entries) {
        for (const auto &indiv_param_info : insufficient_frame_info.second) {
          LOG(INFO) << "Frame: " << insufficient_frame_info.first
                    << ", param idx: " << indiv_param_info.first << ";, norm"
                    << indiv_param_info.second;
        }
      }
    }

    // Add residuals for each
    addPriorToProblemParams(insufficient_rank_info,
                            long_term_map_tunable_params,
                            pose_graph_copy,
                            problem_for_ltm,
                            added_factors);

    // Rerun covariance extraction
    covariance_result = extractCovariance(residual_creator,
                                          ltm_residual_params,
                                          ltm_solver_params,
                                          optimization_factor_configuration,
                                          long_term_map_tunable_params,
                                          override_min_max_frame_id,
                                          jacobian_output_dir,
                                          long_term_map_obj_retriever,
                                          covariance_extractor_params,
                                          parameter_block_cov_retriever,
                                          added_factors,
                                          residual_info,
                                          pose_graph_copy,
                                          problem_for_ltm,
                                          retry_count);

    if (!covariance_result.first) {
      LOG(ERROR) << "Covariance extraction failed on retry " << retry_count
                 << " with additional priors; consider revising buffer "
                 << kRankDeficiencyColsBuffer;
    }
  }
  return covariance_result;
}

}  // namespace vslam_types_refactor