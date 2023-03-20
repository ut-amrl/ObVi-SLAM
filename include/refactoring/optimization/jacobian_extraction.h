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
  std::unordered_map<int, std::unordered_set<size_t>> indices_for_cols;
  for (size_t col_idx = 0; col_idx < crs_matrix.cols.size(); col_idx++) {
    int col = crs_matrix.cols[col_idx];
    if (indices_for_cols.find(col) == indices_for_cols.end()) {
      indices_for_cols[col] = {};
    }
    indices_for_cols[col].insert(col_idx);
    cols_strings.emplace_back(std::to_string(col));
  }
  file_io::writeCommaSeparatedStringsLineToFile(cols_strings, csv_file);

  // Write contents of values vector
  std::vector<std::string> values_strings;
  for (const double &value : crs_matrix.values) {
    std::stringstream str_stream;
    str_stream << value;
    values_strings.emplace_back(str_stream.str());
  }
  std::unordered_set<int> cols_with_0_val;
  for (size_t val_num = 0; val_num < crs_matrix.values.size(); val_num++) {
    if (crs_matrix.values.at(val_num) == 0) {
      LOG(INFO) << "Value at index " << val_num << " was exactly 0";
      cols_with_0_val.insert(crs_matrix.cols[val_num]);
    }
  }
  file_io::writeCommaSeparatedStringsLineToFile(values_strings, csv_file);

  std::vector<int> cols_with_only_zeros;
  for (const int &col_with_zero_val : cols_with_0_val) {
    std::unordered_set<size_t> indices_for_col =
        indices_for_cols.at(col_with_zero_val);
    bool has_non_zero_entry = false;
    for (const size_t &index_for_col : indices_for_col) {
      if (crs_matrix.values[index_for_col] != 0) {
        has_non_zero_entry = true;
        break;
      }
    }
    LOG(INFO) << "Column " << col_with_zero_val
              << " does/does not have non-zero entry: " << has_non_zero_entry;
    if (!has_non_zero_entry) {
      cols_with_only_zeros.emplace_back(col_with_zero_val);
    }
  }
  std::sort(cols_with_only_zeros.begin(), cols_with_only_zeros.end());
  LOG(INFO) << "Cols with only zeros, total size: "
            << cols_with_only_zeros.size();
  for (const auto &col_with_only_zeros : cols_with_only_zeros) {
    LOG(INFO) << col_with_only_zeros;
  }
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
    std::stringstream str_stream;
    str_stream << value;
    values_strings.emplace_back(str_stream.str());
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
    const std::unordered_map<ceres::ResidualBlockId,
                             std::pair<vslam_types_refactor::FactorType,
                                       vslam_types_refactor::FeatureFactorId>>
        &residual_info,
    const std::unordered_map<ceres::ResidualBlockId, double>
        &block_ids_and_residuals,
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

  // Keep track of the added frames, objects, and features, so that they can be
  // added in order
  std::unordered_set<FrameId> added_frames;
  std::unordered_set<ObjectId> added_objects;
  std::unordered_set<FeatureId> added_features;

  for (const auto &residual_info_entry : residual_info) {
    residual_block_ids.emplace_back(residual_info_entry.first);
    GenericFactorInfo generic_factor_info;
    std::vector<ParameterBlockInfo> new_parameter_blocks;
    generateGenericFactorInfoForFactor(pose_graph,
                                       residual_info_entry.second.first,
                                       residual_info_entry.second.second,
                                       long_term_map_obj_retriever,
                                       added_frames,
                                       added_objects,
                                       added_features,
                                       generic_factor_info,
                                       new_parameter_blocks);
    if (block_ids_and_residuals.find(residual_info_entry.first) !=
        block_ids_and_residuals.end()) {
      generic_factor_info.final_residual_val_ =
          block_ids_and_residuals.at(residual_info_entry.first);
    }
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

  // Sort the entries to put them in order
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
    ParameterBlockInfo param_block;
    param_block.feature_id_ = feat_id;
    double *feature_ptr;
    pose_graph->getFeaturePointers(param_block.feature_id_.value(),
                                   &feature_ptr);
    ordered_parameter_blocks.emplace_back(feature_ptr);
    ordered_parameter_block_infos.emplace_back(param_block);
  }
  for (const FrameId &frame_id : added_frames_vec) {
    ParameterBlockInfo param_block;
    param_block.frame_id_ = frame_id;
    double *robot_pose_ptr;
    pose_graph->getPosePointers(param_block.frame_id_.value(), &robot_pose_ptr);
    ordered_parameter_blocks.emplace_back(robot_pose_ptr);
    ordered_parameter_block_infos.emplace_back(param_block);
  }
  for (const ObjectId &obj_id : added_objects_vec) {
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
