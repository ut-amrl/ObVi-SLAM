//
// Created by amanda on 1/27/23.
//

#include <ceres/ceres.h>
#include <ceres/problem.h>
#include <file_io/cv_file_storage/generic_factor_info_file_storage_io.h>
#include <file_io/file_access_utils.h>
#include <file_io/file_io_utils.h>
#include <refactoring/factors/generic_factor_info.h>
#include <refactoring/optimization/jacobian_extraction.h>

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

std::pair<std::pair<std::vector<int>, std::vector<int>>, std::vector<int>>
writeJacobianToFile(const ceres::CRSMatrix &crs_matrix,
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
  file_io::writeCommaSeparatedStringsLineToFile(values_strings, csv_file);

  std::unordered_set<int> cols_with_some_tiny_values;
  std::vector<int> cols_with_tiny_values;
  std::unordered_set<int> cols_with_0_val;
  for (size_t val_num = 0; val_num < crs_matrix.values.size(); val_num++) {
    if (crs_matrix.values.at(val_num) == 0) {
      LOG(INFO) << "Value at index " << val_num << " was exactly 0";
      cols_with_0_val.insert(crs_matrix.cols[val_num]);
    } else if (abs(crs_matrix.values.at(val_num)) < 1e-5) {
      cols_with_some_tiny_values.insert(crs_matrix.cols[val_num]);
    }
  }
  std::vector<int> cols_with_0_val_vec;
  std::vector<int> cols_with_only_zeros;
  for (const int &col_with_zero_val : cols_with_0_val) {
    std::unordered_set<size_t> indices_for_col =
        indices_for_cols.at(col_with_zero_val);
    cols_with_0_val_vec.emplace_back(col_with_zero_val);
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

  for (const int &col_with_tiny_val : cols_with_some_tiny_values) {
    std::unordered_set<size_t> indices_for_col =
        indices_for_cols.at(col_with_tiny_val);
    bool has_non_zero_entry = false;
    for (const size_t &index_for_col : indices_for_col) {
      if (abs(crs_matrix.values[index_for_col]) >= 1e-5) {
        has_non_zero_entry = true;
        break;
      }
    }
    if (!has_non_zero_entry) {
      cols_with_tiny_values.emplace_back(col_with_tiny_val);
      std::string col_vals;
      for (const size_t &index_for_col : indices_for_col) {
        col_vals += std::to_string(crs_matrix.values[index_for_col]);
        col_vals += ", ";
      }

      LOG(INFO) << "Column " << col_with_tiny_val
                << " has only tiny vals: " + col_vals;
    }
  }

  std::sort(cols_with_0_val_vec.begin(), cols_with_0_val_vec.end());
  std::sort(cols_with_only_zeros.begin(), cols_with_only_zeros.end());
  std::sort(cols_with_tiny_values.begin(), cols_with_tiny_values.end());
  LOG(INFO) << "Cols with only zeros, total size: "
            << cols_with_only_zeros.size();
  for (const auto &col_with_only_zeros : cols_with_only_zeros) {
    LOG(INFO) << col_with_only_zeros;
  }

  return std::make_pair(
      std::make_pair(cols_with_0_val_vec, cols_with_only_zeros),
      cols_with_tiny_values);
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
    if (factor.object_id_ == 45) {
      LOG(INFO) << "Adding observation factor for object " << factor.object_id_;
    }

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

void displayInfoForSmallJacobian(const ceres::CRSMatrix &jacobian_mat) {
  int num_nonzero = 0;
  for (const double &value : jacobian_mat.values) {
    if (value != 0) {
      num_nonzero++;
    }
  }
  LOG(INFO) << "Nonzero entries count: " << num_nonzero;
  LOG(INFO) << "Matrix size: (" << jacobian_mat.num_rows << ", "
            << jacobian_mat.num_cols << ")";
  std::stringstream rows_stream;
  std::stringstream cols_stream;
  std::stringstream vals_stream;

  for (const int &row : jacobian_mat.rows) {
    rows_stream << row << ", ";
  }
  for (const int &col : jacobian_mat.cols) {
    cols_stream << col << ", ";
  }
  for (const double &val : jacobian_mat.values) {
    vals_stream << val << ", ";
  }

  LOG(INFO) << "Rows: " << rows_stream.str();
  LOG(INFO) << "Cols: " << cols_stream.str();
  LOG(INFO) << "Vals: " << vals_stream.str();
}

void displayInfoForObj(
    const ObjectId &obj,
    const std::vector<GenericFactorInfo> &generic_factor_infos,
    const std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph> &pose_graph,
    const std::vector<ceres::ResidualBlockId> &residual_block_ids,
    const std::unordered_map<ObjectId, std::unordered_set<size_t>>
        &associated_factor_infos_for_obj,
    ceres::Problem &problem_for_ltm) {
  LOG(INFO) << "Object: " << obj;
  std::unordered_set<size_t> factor_info_nums =
      associated_factor_infos_for_obj.at(obj);
  FrameId min_frame_id = UINT64_MAX;
  FrameId max_frame_id = 0;
  for (const size_t &factor_info_num : factor_info_nums) {
    GenericFactorInfo factor = generic_factor_infos[factor_info_num];
    if (factor.frame_id_.has_value()) {
      min_frame_id = std::min(factor.frame_id_.value(), min_frame_id);
      max_frame_id = std::max(factor.frame_id_.value(), max_frame_id);
    }
  }
  LOG(INFO) << "Min frame for obj " << min_frame_id;
  LOG(INFO) << "Max frame for obj " << max_frame_id;
  for (const size_t &factor_info_num : factor_info_nums) {
    GenericFactorInfo factor = generic_factor_infos[factor_info_num];
    if (factor.factor_type_ == kLongTermMapFactorTypeId) {
      LOG(INFO) << "Long term map factor";
    } else if (factor.factor_type_ == kShapeDimPriorFactorTypeId) {
      LOG(INFO) << "Shape dim prior";
    } else if (factor.factor_type_ == kObjectObservationFactorTypeId) {
      LOG(INFO) << "Bounding box observation at " << factor.frame_id_.value()
                << " with camera " << factor.camera_id_.value() << " for obj "
                << obj;
    }
    ceres::Problem::EvaluateOptions options;
    options.apply_loss_function = true;
    double *obj_ptr;
    pose_graph->getObjectParamPointers(obj, &obj_ptr);
    options.parameter_blocks = {obj_ptr};
    options.residual_blocks = {residual_block_ids[factor_info_num]};
    ceres::CRSMatrix jacobian_for_obj_factor;
    problem_for_ltm.Evaluate(
        options, nullptr, nullptr, nullptr, &jacobian_for_obj_factor);
    displayInfoForSmallJacobian(jacobian_for_obj_factor);
  }
}

void validateZeroColumnEntries(
    const std::pair<std::pair<std::vector<int>, std::vector<int>>,
                    std::vector<int>> &problem_cols,
    const std::vector<ParameterBlockInfo> &ordered_parameter_block_infos,

    const std::vector<ceres::ResidualBlockId> &residual_block_ids,
    const std::vector<GenericFactorInfo> &generic_factor_infos,
    const std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph> &pose_graph,
    ceres::Problem &problem_for_ltm) {
  std::unordered_map<ObjectId, std::unordered_set<int>>
      problem_object_ids_with_cols;
  std::unordered_map<FrameId, std::unordered_set<int>> problem_frames;
  std::unordered_map<FeatureId, std::unordered_set<int>> problem_feats;

  std::unordered_map<ObjectId, std::unordered_set<int>>
      problem_object_ids_with_cols_some_zeros;
  std::unordered_map<FrameId, std::unordered_set<int>>
      problem_frames_some_zeros;
  std::unordered_map<FeatureId, std::unordered_set<int>>
      problem_feats_some_zeros;

  std::unordered_map<ObjectId, std::unordered_set<int>>
      problem_object_ids_with_cols_all_tiny;
  std::unordered_map<FrameId, std::unordered_set<int>> problem_frames_all_tiny;
  std::unordered_map<FeatureId, std::unordered_set<int>> problem_feats_all_tiny;
  std::unordered_map<int, int> param_block_num_for_problem_col;

  size_t current_param_block = 0;
  size_t current_param_in_block_idx = 0;
  int param_num = 0;
  size_t next_all_zero_column = 0;
  size_t next_some_zero_column = 0;
  size_t next_tiny_column = 0;
  std::vector<int> some_zero_columns = problem_cols.first.first;
  std::vector<int> all_zero_columns = problem_cols.first.second;
  std::vector<int> all_tiny_columns = problem_cols.second;
  if (!all_zero_columns.empty() || !some_zero_columns.empty() ||
      !all_tiny_columns.empty()) {
    int max_col;
    if (!some_zero_columns.empty() && !all_tiny_columns.empty()) {
      max_col = std::max(some_zero_columns.back(), all_tiny_columns.back());
    } else if (some_zero_columns.empty()) {
      max_col = all_tiny_columns.back();
    } else {
      max_col = some_zero_columns.back();
    }
    while (param_num <= max_col) {
      if (current_param_block >= ordered_parameter_block_infos.size()) {
        LOG(ERROR) << "Parameter block num greater than total number of "
                      "parameter blocks "
                   << ordered_parameter_block_infos.size();
        LOG(ERROR) << "Occurred with param num " << param_num;
        exit(1);
      }
      ParameterBlockInfo param_block =
          ordered_parameter_block_infos[current_param_block];
      if (param_block.frame_id_.has_value()) {
        if (next_all_zero_column < all_zero_columns.size()) {
          if (all_zero_columns[next_all_zero_column] == param_num) {
            if (problem_frames.find(param_block.frame_id_.value()) ==
                problem_frames.end()) {
              problem_frames[param_block.frame_id_.value()] = {};
            }
            problem_frames[param_block.frame_id_.value()].insert(
                all_zero_columns[next_all_zero_column]);
            next_all_zero_column++;
            param_block_num_for_problem_col[param_num] =
                current_param_in_block_idx;
          }
        }
        if (next_all_zero_column < some_zero_columns.size()) {
          if (some_zero_columns[next_some_zero_column] == param_num) {
            if (problem_frames_some_zeros.find(param_block.frame_id_.value()) ==
                problem_frames_some_zeros.end()) {
              problem_frames_some_zeros[param_block.frame_id_.value()] = {};
            }
            problem_frames_some_zeros[param_block.frame_id_.value()].insert(
                some_zero_columns[next_some_zero_column]);
            next_some_zero_column++;
            param_block_num_for_problem_col[param_num] =
                current_param_in_block_idx;
          }
        }
        if (next_tiny_column < all_tiny_columns.size()) {
          if (all_tiny_columns[next_tiny_column] == param_num) {
            if (problem_frames_all_tiny.find(param_block.frame_id_.value()) ==
                problem_frames_all_tiny.end()) {
              problem_frames_all_tiny[param_block.frame_id_.value()] = {};
            }
            problem_frames_all_tiny[param_block.frame_id_.value()].insert(
                all_tiny_columns[next_tiny_column]);
            next_tiny_column++;
            param_block_num_for_problem_col[param_num] =
                current_param_in_block_idx;
          }
        }
        if (current_param_in_block_idx < 5) {
          current_param_in_block_idx++;
        } else if (current_param_in_block_idx == 5) {
          current_param_in_block_idx = 0;
          current_param_block++;
        } else {
          LOG(ERROR)
              << "Current param in block index was supposed to be between "
                 "0 and 5 inclusive but was "
              << current_param_in_block_idx;
          exit(1);
        }
      } else if (param_block.obj_id_.has_value()) {
        if (next_all_zero_column < all_zero_columns.size()) {
          if (all_zero_columns[next_all_zero_column] == param_num) {
            if (problem_object_ids_with_cols.find(
                    param_block.obj_id_.value()) ==
                problem_object_ids_with_cols.end()) {
              problem_object_ids_with_cols[param_block.obj_id_.value()] = {};
            }
            problem_object_ids_with_cols[param_block.obj_id_.value()].insert(
                all_zero_columns[next_all_zero_column]);
            next_all_zero_column++;
            param_block_num_for_problem_col[param_num] =
                current_param_in_block_idx;
          }
        }
        if (next_all_zero_column < some_zero_columns.size()) {
          if (some_zero_columns[next_some_zero_column] == param_num) {
            if (problem_object_ids_with_cols_some_zeros.find(
                    param_block.obj_id_.value()) ==
                problem_object_ids_with_cols_some_zeros.end()) {
              problem_object_ids_with_cols_some_zeros[param_block.obj_id_
                                                          .value()] = {};
            }
            problem_object_ids_with_cols_some_zeros[param_block.obj_id_.value()]
                .insert(some_zero_columns[next_some_zero_column]);
            next_some_zero_column++;
            param_block_num_for_problem_col[param_num] =
                current_param_in_block_idx;
          }
        }
        if (next_tiny_column < all_tiny_columns.size()) {
          if (all_tiny_columns[next_tiny_column] == param_num) {
            if (problem_object_ids_with_cols_all_tiny.find(
                    param_block.obj_id_.value()) ==
                problem_object_ids_with_cols_all_tiny.end()) {
              problem_object_ids_with_cols_all_tiny[param_block.obj_id_
                                                        .value()] = {};
            }
            problem_object_ids_with_cols_all_tiny[param_block.obj_id_.value()]
                .insert(all_tiny_columns[next_tiny_column]);
            next_tiny_column++;
            param_block_num_for_problem_col[param_num] =
                current_param_in_block_idx;
          }
        }
        if (current_param_in_block_idx < (kEllipsoidParamterizationSize - 1)) {
          current_param_in_block_idx++;
        } else if (current_param_in_block_idx ==
                   (kEllipsoidParamterizationSize - 1)) {
          current_param_in_block_idx = 0;
          current_param_block++;
        } else {
          LOG(ERROR)
              << "Current param in block index was supposed to be between "
                 "0 and "
              << (kEllipsoidParamterizationSize - 1) << " inclusive but was "
              << current_param_in_block_idx;
          exit(1);
        }
      } else if (param_block.feature_id_.has_value()) {
        if (next_all_zero_column < all_zero_columns.size()) {
          if (all_zero_columns[next_all_zero_column] == param_num) {
            if (problem_feats.find(param_block.feature_id_.value()) ==
                problem_feats.end()) {
              problem_feats[param_block.feature_id_.value()] = {};
            }
            problem_feats[param_block.feature_id_.value()].insert(
                all_zero_columns[next_all_zero_column]);
            next_all_zero_column++;
            param_block_num_for_problem_col[param_num] =
                current_param_in_block_idx;
          }
        }
        if (next_all_zero_column < some_zero_columns.size()) {
          if (some_zero_columns[next_some_zero_column] == param_num) {
            if (problem_feats_some_zeros.find(
                    param_block.feature_id_.value()) ==
                problem_feats_some_zeros.end()) {
              problem_feats_some_zeros[param_block.feature_id_.value()] = {};
            }
            problem_feats_some_zeros[param_block.feature_id_.value()].insert(
                some_zero_columns[next_some_zero_column]);
            next_some_zero_column++;
            param_block_num_for_problem_col[param_num] =
                current_param_in_block_idx;
          }
        }
        if (next_tiny_column < all_tiny_columns.size()) {
          if (all_tiny_columns[next_tiny_column] == param_num) {
            if (problem_feats_all_tiny.find(param_block.feature_id_.value()) ==
                problem_feats_all_tiny.end()) {
              problem_feats_all_tiny[param_block.feature_id_.value()] = {};
            }
            problem_feats_all_tiny[param_block.feature_id_.value()].insert(
                all_tiny_columns[next_tiny_column]);
            next_tiny_column++;
            param_block_num_for_problem_col[param_num] =
                current_param_in_block_idx;
          }
        }
        if (current_param_in_block_idx < 2) {
          current_param_in_block_idx++;
        } else if (current_param_in_block_idx == 2) {
          current_param_in_block_idx = 0;
          current_param_block++;
        } else {
          LOG(ERROR)
              << "Current param in block index was supposed to be between "
                 "0 and 5 inclusive but was "
              << current_param_in_block_idx;
          exit(1);
        }
      } else {
        LOG(ERROR) << "Param block had no id that wasn't empty. Error.";
        exit(1);
      }
      param_num++;
    }
  }

  std::unordered_map<ObjectId, std::unordered_set<size_t>>
      associated_factor_infos_for_obj;
  std::unordered_map<ObjectId, std::unordered_set<size_t>>
      associated_factor_infos_for_feat;
  std::unordered_map<ObjectId, std::unordered_set<size_t>>
      associated_factor_infos_for_frame;
  for (size_t factor_info_num = 0;
       factor_info_num < generic_factor_infos.size();
       factor_info_num++) {
    GenericFactorInfo factor_info = generic_factor_infos[factor_info_num];
    if (factor_info.feature_id_.has_value()) {
      if (associated_factor_infos_for_feat.find(
              factor_info.feature_id_.value()) ==
          associated_factor_infos_for_feat.end()) {
        associated_factor_infos_for_feat[factor_info.feature_id_.value()] = {};
      }
      associated_factor_infos_for_feat[factor_info.feature_id_.value()].insert(
          factor_info_num);
    }
    if (factor_info.frame_id_.has_value()) {
      if (associated_factor_infos_for_frame.find(
              factor_info.frame_id_.value()) ==
          associated_factor_infos_for_frame.end()) {
        associated_factor_infos_for_frame[factor_info.frame_id_.value()] = {};
      }
      associated_factor_infos_for_frame[factor_info.frame_id_.value()].insert(
          factor_info_num);
    }
    if (factor_info.obj_id_.has_value()) {
      if (associated_factor_infos_for_obj.find(factor_info.obj_id_.value()) ==
          associated_factor_infos_for_obj.end()) {
        associated_factor_infos_for_obj[factor_info.obj_id_.value()] = {};
      }
      associated_factor_infos_for_obj[factor_info.obj_id_.value()].insert(
          factor_info_num);
    }
  }

  LOG(INFO) << "Getting jacobian info for problematic objects, count "
            << problem_object_ids_with_cols.size();
  ;
  for (const auto &objs_and_problem_cols : problem_object_ids_with_cols) {
    ObjectId obj = objs_and_problem_cols.first;
    for (const int &problem_col : objs_and_problem_cols.second) {
      LOG(INFO) << "Object param block entries "
                << param_block_num_for_problem_col[problem_col];
    }
    displayInfoForObj(obj,
                      generic_factor_infos,
                      pose_graph,
                      residual_block_ids,
                      associated_factor_infos_for_obj,
                      problem_for_ltm);
  }
  LOG(INFO) << "Objects with some zeros, count "
            << problem_object_ids_with_cols_some_zeros.size();
  for (const auto &objs_and_problem_cols :
       problem_object_ids_with_cols_some_zeros) {
    ObjectId obj = objs_and_problem_cols.first;
    for (const int &problem_col : objs_and_problem_cols.second) {
      LOG(INFO) << "Object param block entries "
                << param_block_num_for_problem_col[problem_col];
    }
    if (problem_object_ids_with_cols.find(obj) ==
        problem_object_ids_with_cols.end()) {
      displayInfoForObj(obj,
                        generic_factor_infos,
                        pose_graph,
                        residual_block_ids,
                        associated_factor_infos_for_obj,
                        problem_for_ltm);
    }
  }

  LOG(INFO) << "Objects with tiny cols (all tiny), count "
            << problem_object_ids_with_cols_all_tiny.size();
  for (const auto &objs_and_problem_cols :
       problem_object_ids_with_cols_all_tiny) {
    ObjectId obj = objs_and_problem_cols.first;
    for (const int &problem_col : objs_and_problem_cols.second) {
      LOG(INFO) << "Object param block entries, col: " << problem_col
                << ", entry: " << param_block_num_for_problem_col[problem_col];
    }
    if (problem_object_ids_with_cols.find(obj) ==
        problem_object_ids_with_cols.end()) {
      displayInfoForObj(obj,
                        generic_factor_infos,
                        pose_graph,
                        residual_block_ids,
                        associated_factor_infos_for_obj,
                        problem_for_ltm);
    }
  }

  LOG(INFO) << "Okay object examples";
  if (associated_factor_infos_for_obj.find(40) !=
      associated_factor_infos_for_obj.end()) {
    displayInfoForObj(40,
                      generic_factor_infos,
                      pose_graph,
                      residual_block_ids,
                      associated_factor_infos_for_obj,
                      problem_for_ltm);
  }
  if (associated_factor_infos_for_obj.find(29) !=
      associated_factor_infos_for_obj.end()) {
    displayInfoForObj(29,
                      generic_factor_infos,
                      pose_graph,
                      residual_block_ids,
                      associated_factor_infos_for_obj,
                      problem_for_ltm);
  }

  LOG(INFO) << "Getting jacobian info for problematic frames";
  for (const auto &frames_and_problem_cols : problem_frames) {
    FrameId frame = frames_and_problem_cols.first;
    for (const int &problem_col : frames_and_problem_cols.second) {
      LOG(INFO) << "Frame param block entries "
                << param_block_num_for_problem_col[problem_col];
    }
    LOG(INFO) << "Frame: " << frame;
    std::unordered_set<size_t> factor_info_nums =
        associated_factor_infos_for_frame[frame];
    for (const size_t &factor_info_num : factor_info_nums) {
      GenericFactorInfo factor = generic_factor_infos[factor_info_num];
      if (factor.factor_type_ == kReprojectionErrorFactorTypeId) {
        LOG(INFO) << "Reprojection error of feature "
                  << factor.feature_id_.value() << " with camera "
                  << factor.camera_id_.value();
      } else if (factor.factor_type_ == kObjectObservationFactorTypeId) {
        LOG(INFO) << "Bounding box observation of obj "
                  << factor.obj_id_.value() << " with camera "
                  << factor.camera_id_.value();
      }
      ceres::Problem::EvaluateOptions options;
      options.apply_loss_function = true;
      double *frame_ptr;
      pose_graph->getPosePointers(frame, &frame_ptr);
      options.parameter_blocks = {frame_ptr};
      options.residual_blocks = {residual_block_ids[factor_info_num]};
      ceres::CRSMatrix jacobian_for_frame_factor;
      problem_for_ltm.Evaluate(
          options, nullptr, nullptr, nullptr, &jacobian_for_frame_factor);
      displayInfoForSmallJacobian(jacobian_for_frame_factor);

      options.apply_loss_function = false;
      ceres::CRSMatrix jacobian_for_frame_factor_no_loss;
      problem_for_ltm.Evaluate(options,
                               nullptr,
                               nullptr,
                               nullptr,
                               &jacobian_for_frame_factor_no_loss);
      displayInfoForSmallJacobian(jacobian_for_frame_factor_no_loss);
    }
  }

  LOG(INFO) << "Getting jacobian info for problematic features";
  for (const auto &feat_and_problem_cols : problem_feats) {
    FeatureId feat = feat_and_problem_cols.first;
    for (const int &problem_col : feat_and_problem_cols.second) {
      LOG(INFO) << "Feat param block entries "
                << param_block_num_for_problem_col[problem_col];
    }
    LOG(INFO) << "Feat: " << feat;
    std::unordered_set<size_t> factor_info_nums =
        associated_factor_infos_for_feat[feat];
    for (const size_t &factor_info_num : factor_info_nums) {
      GenericFactorInfo factor = generic_factor_infos[factor_info_num];
      if (factor.factor_type_ == kReprojectionErrorFactorTypeId) {
        LOG(INFO) << "Reprojection error of feature at frame "
                  << factor.frame_id_.value() << " with camera "
                  << factor.camera_id_.value();
      }
      ceres::Problem::EvaluateOptions options;
      options.apply_loss_function = true;
      double *feat_ptr;
      pose_graph->getFeaturePointers(feat, &feat_ptr);
      options.parameter_blocks = {feat_ptr};
      options.residual_blocks = {residual_block_ids[factor_info_num]};
      ceres::CRSMatrix jacobian_for_feat_factor;
      problem_for_ltm.Evaluate(
          options, nullptr, nullptr, nullptr, &jacobian_for_feat_factor);
      displayInfoForSmallJacobian(jacobian_for_feat_factor);
    }
  }

  LOG(INFO) << "Getting jacobian info for problematic frames (all tiny)";
  for (const auto &frames_and_problem_cols : problem_frames_all_tiny) {
    FrameId frame = frames_and_problem_cols.first;
    for (const int &problem_col : frames_and_problem_cols.second) {
      LOG(INFO) << "Frame param block entries "
                << param_block_num_for_problem_col[problem_col];
    }
    LOG(INFO) << "Frame: " << frame;
    std::unordered_set<size_t> factor_info_nums =
        associated_factor_infos_for_frame[frame];
    for (const size_t &factor_info_num : factor_info_nums) {
      GenericFactorInfo factor = generic_factor_infos[factor_info_num];
      if (factor.factor_type_ == kReprojectionErrorFactorTypeId) {
        LOG(INFO) << "Reprojection error of feature "
                  << factor.feature_id_.value() << " with camera "
                  << factor.camera_id_.value();
      } else if (factor.factor_type_ == kObjectObservationFactorTypeId) {
        LOG(INFO) << "Bounding box observation of obj "
                  << factor.obj_id_.value() << " with camera "
                  << factor.camera_id_.value();
      }
      ceres::Problem::EvaluateOptions options;
      options.apply_loss_function = true;
      double *frame_ptr;
      pose_graph->getPosePointers(frame, &frame_ptr);
      options.parameter_blocks = {frame_ptr};
      options.residual_blocks = {residual_block_ids[factor_info_num]};
      ceres::CRSMatrix jacobian_for_frame_factor;
      problem_for_ltm.Evaluate(
          options, nullptr, nullptr, nullptr, &jacobian_for_frame_factor);
      displayInfoForSmallJacobian(jacobian_for_frame_factor);

      options.apply_loss_function = false;
      ceres::CRSMatrix jacobian_for_frame_factor_no_loss;
      problem_for_ltm.Evaluate(options,
                               nullptr,
                               nullptr,
                               nullptr,
                               &jacobian_for_frame_factor_no_loss);
      displayInfoForSmallJacobian(jacobian_for_frame_factor_no_loss);
    }
  }

  LOG(INFO) << "Getting jacobian info for problematic features (tiny)";
  for (const auto &feat_and_problem_cols : problem_feats_all_tiny) {
    FeatureId feat = feat_and_problem_cols.first;
    for (const int &problem_col : feat_and_problem_cols.second) {
      LOG(INFO) << "Feat param block entries "
                << param_block_num_for_problem_col[problem_col];
    }
    LOG(INFO) << "Feat: " << feat;
    std::unordered_set<size_t> factor_info_nums =
        associated_factor_infos_for_feat[feat];
    for (const size_t &factor_info_num : factor_info_nums) {
      GenericFactorInfo factor = generic_factor_infos[factor_info_num];
      if (factor.factor_type_ == kReprojectionErrorFactorTypeId) {
        LOG(INFO) << "Reprojection error of feature at frame "
                  << factor.frame_id_.value() << " with camera "
                  << factor.camera_id_.value();
      }
      ceres::Problem::EvaluateOptions options;
      options.apply_loss_function = true;
      double *feat_ptr;
      pose_graph->getFeaturePointers(feat, &feat_ptr);
      options.parameter_blocks = {feat_ptr};
      options.residual_blocks = {residual_block_ids[factor_info_num]};
      ceres::CRSMatrix jacobian_for_feat_factor;
      problem_for_ltm.Evaluate(
          options, nullptr, nullptr, nullptr, &jacobian_for_feat_factor);
      displayInfoForSmallJacobian(jacobian_for_feat_factor);
    }
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
  std::pair<std::pair<std::vector<int>, std::vector<int>>, std::vector<int>>
      problem_cols = writeJacobianToFile(
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

  //  if (!all_zero_columns.empty()) {
  // Get jacobian for param blocks corresponding to zero entries one at a time
  validateZeroColumnEntries(problem_cols,
                            ordered_parameter_block_infos,
                            residual_block_ids,
                            generic_factor_infos,
                            pose_graph,
                            problem_for_ltm);
  //  }

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
