//
// Created by amanda on 1/27/23.
//

#ifndef UT_VSLAM_JACOBIAN_EXTRACTION_H
#define UT_VSLAM_JACOBIAN_EXTRACTION_H

#include <ceres/ceres.h>
#include <ceres/problem.h>
#include <file_io/cv_file_storage/generic_factor_info_file_storage_io.h>
#include <file_io/file_access_utils.h>
#include <refactoring/factors/generic_factor_info.h>

namespace vslam_types_refactor {
const std::string kSparseJacobianOutBaseFileName = "sparse_jacobian.csv";
const std::string kSparseJacobianMatlabOutBaseFileName =
    "sparse_jacobian_matlab.csv";
const std::string kSparseJacobianOrderedMatlabOutBaseFileName =
    "ordered_sparse_jacobian_matlab.csv";
const std::string kResidualInfoForJacobianFile = "jacobian_residual_info.json";
const std::string kResidualInfoOrderedForJacobianFile =
    "ordered_jacobian_residual_info.json";
const std::string kSparseJacobianOrderedOutBaseFileName =
    "ordered_sparse_jacobian.csv";

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
  std::vector<ParameterBlockInfo> unordered_parameter_block_infos;
  std::vector<double *> unordered_parameter_blocks;
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
        unordered_parameter_block_infos.emplace_back(param_block);
        if (param_block.frame_id_.has_value()) {
          added_frames.insert(param_block.frame_id_.value());
          double *robot_pose_ptr;
          pose_graph->getPosePointers(param_block.frame_id_.value(),
                                      &robot_pose_ptr);
          unordered_parameter_blocks.emplace_back(robot_pose_ptr);
        }
        if (param_block.feature_id_.has_value()) {
          added_features.insert(param_block.feature_id_.value());
          double *feature_ptr;
          pose_graph->getFeaturePointers(param_block.feature_id_.value(),
                                         &feature_ptr);
          unordered_parameter_blocks.emplace_back(feature_ptr);
        }
        if (param_block.obj_id_.has_value()) {
          added_objects.insert(param_block.obj_id_.value());
          double *obj_ptr;
          pose_graph->getObjectParamPointers(param_block.obj_id_.value(),
                                             &obj_ptr);
          unordered_parameter_blocks.emplace_back(obj_ptr);
        }
      }
    }
  }

  options.parameter_blocks = unordered_parameter_blocks;
  options.residual_blocks = residual_block_ids;
  ceres::CRSMatrix sparse_jacobian_unordered;
  problem_for_ltm.Evaluate(
      options, nullptr, nullptr, nullptr, &sparse_jacobian_unordered);

  std::vector<ParameterBlockInfo> ordered_parameter_block_infos;
  std::vector<double *> ordered_parameter_blocks;

  if (ordered_parameter_block_infos.size() != ordered_parameter_blocks.size()) {
    LOG(ERROR) << "Dimension mismatch between ordered parameter blocks and "
                  "ordered param block infos";
  }

  for (const FeatureId &feat_id : added_features) {
    ParameterBlockInfo param_block;
    param_block.feature_id_ = feat_id;
    double *feature_ptr;
    pose_graph->getFeaturePointers(param_block.feature_id_.value(),
                                   &feature_ptr);
    ordered_parameter_blocks.emplace_back(feature_ptr);
    ordered_parameter_block_infos.emplace_back(param_block);
  }
  for (const FrameId &frame_id : added_frames) {
    ParameterBlockInfo param_block;
    param_block.frame_id_ = frame_id;
    double *robot_pose_ptr;
    pose_graph->getPosePointers(param_block.frame_id_.value(), &robot_pose_ptr);
    ordered_parameter_blocks.emplace_back(robot_pose_ptr);
    ordered_parameter_block_infos.emplace_back(param_block);
  }
  for (const ObjectId &obj_id : added_objects) {
    ParameterBlockInfo param_block;
    param_block.obj_id_ = obj_id;
    double *obj_ptr;
    pose_graph->getObjectParamPointers(param_block.obj_id_.value(), &obj_ptr);
    ordered_parameter_blocks.emplace_back(obj_ptr);
    ordered_parameter_block_infos.emplace_back(param_block);
  }

  options.parameter_blocks = ordered_parameter_blocks;
  ceres::CRSMatrix sparse_jacobian_ordered;
  problem_for_ltm.Evaluate(
      options, nullptr, nullptr, nullptr, &sparse_jacobian_ordered);

  writeJacobianToFile(
      sparse_jacobian_unordered,
      file_io::ensureDirectoryPathEndsWithSlash(jacobian_output_dir) +
          kSparseJacobianOutBaseFileName);
  writeJacobianToFile(
      sparse_jacobian_ordered,
      file_io::ensureDirectoryPathEndsWithSlash(jacobian_output_dir) +
          kSparseJacobianOrderedOutBaseFileName);
  writeJacobianMatlabFormatToFile(
      sparse_jacobian_unordered,
      file_io::ensureDirectoryPathEndsWithSlash(jacobian_output_dir) +
          kSparseJacobianMatlabOutBaseFileName);
  writeJacobianMatlabFormatToFile(
      sparse_jacobian_ordered,
      file_io::ensureDirectoryPathEndsWithSlash(jacobian_output_dir) +
          kSparseJacobianOrderedMatlabOutBaseFileName);

  std::string jacobian_residual_file_name =
      file_io::ensureDirectoryPathEndsWithSlash(jacobian_output_dir) +
      kResidualInfoForJacobianFile;
  cv::FileStorage jacobian_residual_info_out(jacobian_residual_file_name,
                                             cv::FileStorage::WRITE);
  jacobian_residual_info_out
      << "jacobian_param_block_info"
      << SerializableVector<ParameterBlockInfo, SerializableParameterBlockInfo>(
             unordered_parameter_block_infos);
  jacobian_residual_info_out
      << "jacobian_residual_info"
      << SerializableVector<GenericFactorInfo, SerializableGenericFactorInfo>(
             generic_factor_infos);
  jacobian_residual_info_out.release();

  std::string ordered_jacobian_residual_file_name =
      file_io::ensureDirectoryPathEndsWithSlash(jacobian_output_dir) +
      kResidualInfoOrderedForJacobianFile;
  cv::FileStorage ordered_jacobian_residual_info_out(
      ordered_jacobian_residual_file_name, cv::FileStorage::WRITE);
  ordered_jacobian_residual_info_out
      << "jacobian_param_block_info"
      << SerializableVector<ParameterBlockInfo, SerializableParameterBlockInfo>(
             ordered_parameter_block_infos);
  ordered_jacobian_residual_info_out
      << "jacobian_residual_info"
      << SerializableVector<GenericFactorInfo, SerializableGenericFactorInfo>(
             generic_factor_infos);
  ordered_jacobian_residual_info_out.release();
}
}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_JACOBIAN_EXTRACTION_H
