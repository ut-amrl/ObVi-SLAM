//
// Created by amanda on 8/3/22.
//

#ifndef UT_VSLAM_LONG_TERM_OBJECT_MAP_EXTRACTION_H
#define UT_VSLAM_LONG_TERM_OBJECT_MAP_EXTRACTION_H

#include <ceres/ceres.h>
#include <ceres/problem.h>
#include <file_io/cv_file_storage/generic_factor_info_file_storage_io.h>
#include <file_io/file_access_utils.h>
#include <refactoring/factors/generic_factor_info.h>
#include <refactoring/long_term_map/long_term_object_map.h>
#include <refactoring/optimization/object_pose_graph.h>
#include <refactoring/optimization/object_pose_graph_optimizer.h>
#include <refactoring/output_problem_data_extraction.h>

namespace vslam_types_refactor {

const std::string kSparseJacobianOutBaseFileName = "sparse_jacobian.csv";
const std::string kSparseJacobianMatlabOutBaseFileName =
    "sparse_jacobian_matlab.csv";
const std::string kResidualInfoForJacobianFile = "jacobian_residual_info.json";

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

  residual_info = optimizer.buildPoseGraphOptimization(
      ltm_optimization_scope_params, residual_params, pose_graph, problem);

  bool opt_success = optimizer.solveOptimization(problem, solver_params, {});

  if (!opt_success) {
    LOG(ERROR) << "Optimization failed during LTM extraction";
    return false;
  }
  return true;
}

void writeJacobianToFile(const ceres::CRSMatrix &crs_matrix,
                         const std::string &jacobian_file) {
  std::ofstream csv_file(jacobian_file, std::ios::trunc);

  // Write num_rows, num_cols
  file_io::writeCommaSeparatedStringsLineToFile(
      {std::to_string(crs_matrix.num_rows),
       std::to_string(crs_matrix.num_cols)},
      csv_file);

  // Write contents of rows (see
  // https://github.com/ceres-solver/ceres-solver/blob/e269b64f55f30f5e9802632b79479aa24d9d07de/include/ceres/crs_matrix.h#L49
  // for how to interpret CRS matrix contents)
  std::vector<std::string> rows_strings;
  for (const int &row : crs_matrix.rows) {
    rows_strings.emplace_back(std::to_string(row));
  }
  file_io::writeCommaSeparatedStringsLineToFile(rows_strings, csv_file);

  // Write contents of cols vector
  std::vector<std::string> cols_strings;
  for (const int &col : crs_matrix.cols) {
    cols_strings.emplace_back(std::to_string(col));
  }
  file_io::writeCommaSeparatedStringsLineToFile(cols_strings, csv_file);

  // Write contents of values vector
  std::vector<std::string> values_strings;
  for (const double &value : crs_matrix.values) {
    values_strings.emplace_back(std::to_string(value));
  }
  file_io::writeCommaSeparatedStringsLineToFile(values_strings, csv_file);
}

void writeJacobianMatlabFormatToFile(const ceres::CRSMatrix &crs_matrix,
                                     const std::string &jacobian_file) {
  std::vector<int> row_indices;
  std::vector<int> col_indices;
  std::vector<double> matlab_vals;

  for (int row_num = 0; row_num < crs_matrix.num_rows; row_num++) {
    size_t matlab_row_num = row_num + 1;
    int row_val = crs_matrix.rows[row_num];
    int next_row_val = crs_matrix.rows[row_num + 1];
    for (size_t val_idx = row_val; val_idx < next_row_val; val_idx++) {
      int col_num = crs_matrix.cols[val_idx];
      int matlab_col_num = col_num + 1;
      double value = crs_matrix.values[val_idx];
      row_indices.emplace_back(matlab_row_num);
      col_indices.emplace_back(matlab_col_num);
      matlab_vals.emplace_back(value);
    }
  }

  std::ofstream csv_file(jacobian_file, std::ios::trunc);

  // Write num_rows, num_cols
  file_io::writeCommaSeparatedStringsLineToFile(
      {std::to_string(crs_matrix.num_rows),
       std::to_string(crs_matrix.num_cols)},
      csv_file);

  // Write rows of non-zero entries
  // Data format:
  // https://www.mathworks.com/help/matlab/ref/sparse.html#d124e1404622
  std::vector<std::string> rows_strings;
  for (const int &row : row_indices) {
    rows_strings.emplace_back(std::to_string(row));
  }
  file_io::writeCommaSeparatedStringsLineToFile(rows_strings, csv_file);

  // Write contents of cols vector
  std::vector<std::string> cols_strings;
  for (const int &col : col_indices) {
    cols_strings.emplace_back(std::to_string(col));
  }
  file_io::writeCommaSeparatedStringsLineToFile(cols_strings, csv_file);

  // Write contents of values vector
  std::vector<std::string> values_strings;
  for (const double &value : matlab_vals) {
    values_strings.emplace_back(std::to_string(value));
  }
  file_io::writeCommaSeparatedStringsLineToFile(values_strings, csv_file);
}

void generateGenericFactorInfoForFactor(
    const std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph> &pose_graph,
    const FactorType &factor_type,
    const FeatureFactorId &feature_factor_id,
    const std::function<bool(const FactorType &,
                             const FeatureFactorId &,
                             ObjectId &)> &long_term_map_obj_retriever,
    const std::unordered_set<FrameId> &added_frames,
    const std::unordered_set<ObjectId> &added_objects,
    const std::unordered_set<FeatureId> &added_features,
    GenericFactorInfo &generic_factor_info,
    std::vector<ParameterBlockInfo> &new_param_block_info) {
  generic_factor_info.factor_type_ = factor_type;
  if (factor_type == kObjectObservationFactorTypeId) {
    ObjectObservationFactor factor;
    if (!pose_graph->getObjectObservationFactor(feature_factor_id, factor)) {
      LOG(ERROR) << "Could not find object observation factor with id "
                 << feature_factor_id << "; not adding to pose graph";
      return;
    }
    generic_factor_info.frame_id_ = factor.frame_id_;
    generic_factor_info.camera_id_ = factor.camera_id_;
    generic_factor_info.obj_id_ = factor.object_id_;

    if (added_objects.find(factor.object_id_) == added_objects.end()) {
      ParameterBlockInfo param_block;
      param_block.obj_id_ = factor.object_id_;
      new_param_block_info.emplace_back(param_block);
    }
    if (added_frames.find(factor.frame_id_) == added_frames.end()) {
      ParameterBlockInfo param_block;
      param_block.frame_id_ = factor.frame_id_;
      new_param_block_info.emplace_back(param_block);
    }
  } else if (factor_type == kReprojectionErrorFactorTypeId) {
    ReprojectionErrorFactor factor;
    if (!pose_graph->getVisualFactor(feature_factor_id, factor)) {
      LOG(ERROR) << "Could not find visual feature factor with id "
                 << feature_factor_id << "; not adding to pose graph";
      return;
    }
    generic_factor_info.frame_id_ = factor.frame_id_;
    generic_factor_info.camera_id_ = factor.camera_id_;
    generic_factor_info.feature_id_ = factor.feature_id_;

    if (added_features.find(factor.feature_id_) == added_features.end()) {
      ParameterBlockInfo param_block;
      param_block.feature_id_ = factor.feature_id_;
      new_param_block_info.emplace_back(param_block);
    }
    if (added_frames.find(factor.frame_id_) == added_frames.end()) {
      ParameterBlockInfo param_block;
      param_block.frame_id_ = factor.frame_id_;
      new_param_block_info.emplace_back(param_block);
    }
  } else if (factor_type == kShapeDimPriorFactorTypeId) {
    ShapeDimPriorFactor factor;
    if (!pose_graph->getShapeDimPriorFactor(feature_factor_id, factor)) {
      LOG(ERROR) << "Could not find shape dim prior factor with id "
                 << feature_factor_id << "; not adding to pose graph";
      return;
    }
    generic_factor_info.obj_id_ = factor.object_id_;
    if (added_objects.find(factor.object_id_) == added_objects.end()) {
      ParameterBlockInfo param_block;
      param_block.obj_id_ = factor.object_id_;
      new_param_block_info.emplace_back(param_block);
    }
  } else if (factor_type == kLongTermMapFactorTypeId) {
    ObjectId ltm_obj_id;
    if (!long_term_map_obj_retriever(
            factor_type, feature_factor_id, ltm_obj_id)) {
      LOG(ERROR) << "Could not find object id for long term map factor with id "
                 << feature_factor_id;
      return;
    }
    generic_factor_info.obj_id_ = ltm_obj_id;
    if (added_objects.find(ltm_obj_id) == added_objects.end()) {
      ParameterBlockInfo param_block;
      param_block.obj_id_ = ltm_obj_id;
      new_param_block_info.emplace_back(param_block);
    }
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

void outputJacobianInfo(
    const std::string &jacobian_output_dir,
    const std::unordered_map<
        vslam_types_refactor::FactorType,
        std::unordered_map<vslam_types_refactor::FeatureFactorId,
                           ceres::ResidualBlockId>> &residual_info,
    const std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph> &pose_graph,
    const std::function<bool(const FactorType &,
                             const FeatureFactorId &,
                             ObjectId &)> &long_term_map_obj_retriever,
    ceres::Problem &problem_for_ltm) {
  ceres::Problem::EvaluateOptions options;
  options.apply_loss_function = true;
  std::vector<ceres::ResidualBlockId> residual_block_ids;
  std::vector<GenericFactorInfo> generic_factor_infos;
  std::vector<ParameterBlockInfo> parameter_block_infos;
  std::vector<double *> parameter_blocks;
  std::unordered_set<FrameId> added_frames;
  std::unordered_set<ObjectId> added_objects;
  std::unordered_set<FeatureId> added_features;
  for (const auto &factor_type_and_factors : residual_info) {
    for (const auto &factor_id_and_residual_block :
         factor_type_and_factors.second) {
      residual_block_ids.emplace_back(factor_id_and_residual_block.second);
      GenericFactorInfo generic_factor_info;
      std::vector<ParameterBlockInfo> new_parameter_blocks;
      generateGenericFactorInfoForFactor(pose_graph,
                                         factor_type_and_factors.first,
                                         factor_id_and_residual_block.first,
                                         long_term_map_obj_retriever,
                                         added_frames,
                                         added_objects,
                                         added_features,
                                         generic_factor_info,
                                         new_parameter_blocks);
      generic_factor_infos.emplace_back(generic_factor_info);
      for (const ParameterBlockInfo &param_block : new_parameter_blocks) {
        parameter_block_infos.emplace_back(param_block);
        if (param_block.frame_id_.has_value()) {
          added_frames.insert(param_block.frame_id_.value());
          double *robot_pose_ptr;
          pose_graph->getPosePointers(param_block.frame_id_.value(),
                                      &robot_pose_ptr);
          parameter_blocks.emplace_back(robot_pose_ptr);
        }
        if (param_block.feature_id_.has_value()) {
          added_features.insert(param_block.feature_id_.value());
          double *feature_ptr;
          pose_graph->getFeaturePointers(param_block.feature_id_.value(),
                                         &feature_ptr);
          parameter_blocks.emplace_back(feature_ptr);
        }
        if (param_block.obj_id_.has_value()) {
          added_objects.insert(param_block.obj_id_.value());
          double *obj_ptr;
          pose_graph->getObjectParamPointers(param_block.obj_id_.value(),
                                             &obj_ptr);
          parameter_blocks.emplace_back(obj_ptr);
        }
      }
    }
  }

  options.parameter_blocks = parameter_blocks;
  options.residual_blocks = residual_block_ids;
  ceres::CRSMatrix sparse_jacobian;
  problem_for_ltm.Evaluate(
      options, nullptr, nullptr, nullptr, &sparse_jacobian);

  writeJacobianToFile(
      sparse_jacobian,
      file_io::ensureDirectoryPathEndsWithSlash(jacobian_output_dir) +
          kSparseJacobianOutBaseFileName);
  writeJacobianMatlabFormatToFile(
      sparse_jacobian,
      file_io::ensureDirectoryPathEndsWithSlash(jacobian_output_dir) +
          kSparseJacobianMatlabOutBaseFileName);

  std::string jacobian_residual_file_name =
      file_io::ensureDirectoryPathEndsWithSlash(jacobian_output_dir) +
      kResidualInfoForJacobianFile;
  cv::FileStorage jacobian_residual_info_out(jacobian_residual_file_name,
                                             cv::FileStorage::WRITE);
  jacobian_residual_info_out
      << "jacobian_param_block_info"
      << SerializableVector<ParameterBlockInfo, SerializableParameterBlockInfo>(
          parameter_block_infos);
  jacobian_residual_info_out
      << "jacobian_residual_info"
      << SerializableVector<GenericFactorInfo, SerializableGenericFactorInfo>(
             generic_factor_infos);
  jacobian_residual_info_out.release();
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
    covariance_blocks.emplace_back(std::make_pair(frame_1_block, frame_1_block));

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
