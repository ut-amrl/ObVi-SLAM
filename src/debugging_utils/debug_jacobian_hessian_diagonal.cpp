//
// Created by amanda on 1/27/23.
//

#include <base_lib/basic_utils.h>
#include <file_io/camera_extrinsics_with_id_io.h>
#include <file_io/cv_file_storage/output_problem_data_file_storage_io.h>
#include <file_io/cv_file_storage/vslam_basic_types_file_storage_io.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <refactoring/image_processing/debugging_image_utils.h>
#include <refactoring/image_processing/image_processing_utils.h>
#include <refactoring/long_term_map/long_term_object_map_extraction.h>
#include <refactoring/offline/offline_problem_data.h>
#include <refactoring/output_problem_data.h>
#include <refactoring/visual_feature_processing/orb_output_low_level_feature_reader.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

using namespace vslam_types_refactor;

DEFINE_string(jacobian_residual_info_file,
              "",
              "File containing jacobian parameter and residual info");
DEFINE_string(hessian_diag_entries_file,
              "",
              "File that contains the entries of the hessian diagonal");
DEFINE_string(data_association_file,
              "",
              "File that contains the data associations for objects");

enum SlamParameterType {
  TRANSL_X,
  TRANSL_Y,
  TRANSL_Z,
  ORIENT_X,
  ORIENT_Y,
  ORIENT_Z,
  DIM_X,
  DIM_Y,
  DIM_Z
};

template <typename First, typename Second>
bool sortPairByFirstEntry(std::pair<First, Second> &pair1,
                          std::pair<First, Second> &pair2) {
  return pair1.first < pair2.first;
}

struct SlamParameterInfo {
  ParameterBlockInfo parameter_block_info_;
  SlamParameterType param_type_;
};

std::string getNameOfSlamParameterType(const SlamParameterType &param_type) {
  switch (param_type) {
    case TRANSL_X:
      return "TRANSL_X";
    case TRANSL_Y:
      return "TRANSL_Y";
    case TRANSL_Z:
      return "TRANSL_Z";
    case DIM_X:
      return "DIM_X";
    case DIM_Y:
      return "DIM_Y";
    case DIM_Z:
      return "DIM_Z";
    case ORIENT_X:
      return "ORIENT_X";
    case ORIENT_Y:
      return "ORIENT_Y";
    case ORIENT_Z:
      return "ORIENT_Z";
    default:
      return "";
  }
}

int main(int argc, char **argv) {
  google::ParseCommandLineFlags(&argc, &argv, false);

  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = true;
  FLAGS_colorlogtostderr = true;
  if (FLAGS_jacobian_residual_info_file.empty()) {
    LOG(INFO) << "Need jacobian info file";
    exit(1);
  }
  if (FLAGS_hessian_diag_entries_file.empty()) {
    LOG(INFO) << "Need hessian diagonal entries file";
    exit(1);
  }

  std::vector<GenericFactorInfo> generic_factor_infos;
  std::vector<ParameterBlockInfo> parameter_block_infos;

  SerializableVector<ParameterBlockInfo, SerializableParameterBlockInfo>
      serializable_param_info;
  SerializableVector<GenericFactorInfo, SerializableGenericFactorInfo>
      serializable_factors;

  LOG(INFO) << "Reading jacobian param/residual info from "
            << FLAGS_jacobian_residual_info_file;
  cv::FileStorage jacobian_info_fs(FLAGS_jacobian_residual_info_file,
                                   cv::FileStorage::READ);
  jacobian_info_fs["jacobian_param_block_info"] >> serializable_param_info;
  jacobian_info_fs["jacobian_residual_info"] >> serializable_factors;
  jacobian_info_fs.release();

  SerializableObjectDataAssociationResults ser_data_assoc_results;
  ObjectDataAssociationResults data_assoc_results;
  cv::FileStorage data_assoc_results_in(FLAGS_data_association_file,
                                        cv::FileStorage::READ);
  data_assoc_results_in["bounding_box_associations"] >> ser_data_assoc_results;
  data_assoc_results_in.release();
  data_assoc_results = ser_data_assoc_results.getEntry();

  generic_factor_infos = serializable_factors.getEntry();
  parameter_block_infos = serializable_param_info.getEntry();

  std::unordered_map<ObjectId, util::BoostHashSet<std::pair<FrameId, CameraId>>>
      frames_for_each_obj;
  for (const auto &data_assoc_frame_entry :
       data_assoc_results.associated_bounding_boxes_) {
    for (const auto &data_assoc_cam_entry : data_assoc_frame_entry.second) {
      for (const auto &data_assoc_obj_entry : data_assoc_cam_entry.second) {
        ObjectId obj_id = data_assoc_obj_entry.first;
        if (frames_for_each_obj.find(obj_id) == frames_for_each_obj.end()) {
          frames_for_each_obj[obj_id] = {};
        }
        frames_for_each_obj[obj_id].insert(std::make_pair(
            data_assoc_frame_entry.first, data_assoc_cam_entry.first));
      }
    }
  }

  LOG(INFO) << "Reading hessian entries from "
            << FLAGS_hessian_diag_entries_file;
  std::vector<double> hessian_diagonal_values;
  std::ifstream hessian_file_obj(FLAGS_hessian_diag_entries_file);
  std::string hessian_line;
  while (std::getline(hessian_file_obj, hessian_line)) {
    double hessian_entry = std::stod(hessian_line);
    hessian_diagonal_values.emplace_back(hessian_entry);
  }
  LOG(INFO) << "Hessian entries size " << hessian_diagonal_values.size();

  size_t current_param_block = 0;
  size_t current_param_in_block_idx = 0;
  std::vector<std::pair<double, std::pair<size_t, SlamParameterInfo>>>
      param_infos_with_param_num_and_hessian_vals;
  for (size_t param_num = 0; param_num < hessian_diagonal_values.size();
       param_num++) {
    if (current_param_block >= parameter_block_infos.size()) {
      LOG(ERROR) << "Parameter block num greater than total number of "
                    "parameter blocks "
                 << parameter_block_infos.size();
      LOG(ERROR) << "Occurred with param num " << param_num;
      exit(1);
    }
    ParameterBlockInfo param_block = parameter_block_infos[current_param_block];
    SlamParameterInfo parameter_info;
    parameter_info.parameter_block_info_ = param_block;
    if (param_block.frame_id_.has_value()) {
      if (current_param_in_block_idx < 5) {
        if (current_param_in_block_idx == 0) {
          parameter_info.param_type_ = TRANSL_X;
        } else if (current_param_in_block_idx == 1) {
          parameter_info.param_type_ = TRANSL_Y;
        } else if (current_param_in_block_idx == 2) {
          parameter_info.param_type_ = TRANSL_Z;
        } else if (current_param_in_block_idx == 3) {
          parameter_info.param_type_ = ORIENT_X;
        } else if (current_param_in_block_idx == 4) {
          parameter_info.param_type_ = ORIENT_Y;
        } else {
          LOG(ERROR) << "current param in block index was negative ";
          exit(1);
        }
        current_param_in_block_idx++;
      } else if (current_param_in_block_idx == 5) {
        parameter_info.param_type_ = ORIENT_Z;
        current_param_in_block_idx = 0;
        current_param_block++;
      } else {
        LOG(ERROR) << "Current param in block index was supposed to be between "
                      "0 and 5 inclusive but was "
                   << current_param_in_block_idx;
        exit(1);
      }
    } else if (param_block.obj_id_.has_value()) {
      if (current_param_in_block_idx < (kEllipsoidParamterizationSize - 1)) {
        if (current_param_in_block_idx == 0) {
          parameter_info.param_type_ = TRANSL_X;
        } else if (current_param_in_block_idx == 1) {
          parameter_info.param_type_ = TRANSL_Y;
        } else if (current_param_in_block_idx == 2) {
          parameter_info.param_type_ = TRANSL_Z;
#ifdef CONSTRAIN_ELLIPSOID_ORIENTATION
        } else if (current_param_in_block_idx == 3) {
          parameter_info.param_type_ = ORIENT_Z;
        } else if (current_param_in_block_idx == 4) {
          parameter_info.param_type_ = DIM_X;
        } else if (current_param_in_block_idx == 5) {
          parameter_info.param_type_ = DIM_Y;
#else
        } else if (current_param_in_block_idx == 3) {
          parameter_info.param_type_ = ORIENT_X;
        } else if (current_param_in_block_idx == 4) {
          parameter_info.param_type_ = ORIENT_Y;
        } else if (current_param_in_block_idx == 5) {
          parameter_info.param_type_ = ORIENT_Z;
        } else if (current_param_in_block_idx == 6) {
          parameter_info.param_type_ = DIM_X;
        } else if (current_param_in_block_idx == 7) {
          parameter_info.param_type_ = DIM_Y;
#endif
        } else {
          LOG(ERROR) << "current param in block index was negative ";
          exit(1);
        }
        current_param_in_block_idx++;
      } else if (current_param_in_block_idx ==
                 (kEllipsoidParamterizationSize - 1)) {
        parameter_info.param_type_ = DIM_Z;
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
      if (current_param_in_block_idx < 2) {
        if (current_param_in_block_idx == 0) {
          parameter_info.param_type_ = TRANSL_X;
        } else if (current_param_in_block_idx == 1) {
          parameter_info.param_type_ = TRANSL_Y;
        } else {
          LOG(ERROR) << "current param in block index was negative ";
          exit(1);
        }
        current_param_in_block_idx++;
      } else if (current_param_in_block_idx == 2) {
        parameter_info.param_type_ = TRANSL_Z;
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
    param_infos_with_param_num_and_hessian_vals.emplace_back(
        std::make_pair(hessian_diagonal_values.at(param_num),
                       std::make_pair(param_num, parameter_info)));
  }

  std::sort(param_infos_with_param_num_and_hessian_vals.begin(),
            param_infos_with_param_num_and_hessian_vals.end(),
            sortPairByFirstEntry<double, std::pair<size_t, SlamParameterInfo>>);

  int num_smallest_hessian_diag_values = 100;
  int num_largest_hessian_diag_values = 30;

  std::vector<std::pair<double, std::pair<size_t, SlamParameterInfo>>>
      smallest_elements(
          param_infos_with_param_num_and_hessian_vals.begin(),
          param_infos_with_param_num_and_hessian_vals.begin() +
              std::min((size_t)num_smallest_hessian_diag_values,
                       param_infos_with_param_num_and_hessian_vals.size()));

  std::vector<std::pair<double, std::pair<size_t, SlamParameterInfo>>>
      largest_elements(
          param_infos_with_param_num_and_hessian_vals.end() -
              std::min((size_t)num_largest_hessian_diag_values,
                       param_infos_with_param_num_and_hessian_vals.size()),
          param_infos_with_param_num_and_hessian_vals.end());

  LOG(INFO) << "Smallest " << num_smallest_hessian_diag_values << " elements";
  std::unordered_map<ObjectId, std::vector<std::string>> zero_objs;
  for (size_t param_block_num = 0; param_block_num < smallest_elements.size();
       param_block_num++) {
    std::pair<double, std::pair<size_t, SlamParameterInfo>> block_info =
        smallest_elements.at(param_block_num);
    LOG(INFO) << param_block_num << "th smallest";
    LOG(INFO) << "Hessian entry: " << block_info.first;
    uint64_t block_id;
    std::string block_type;
    if (block_info.second.second.parameter_block_info_.obj_id_.has_value()) {
      block_type = "object";
      block_id = block_info.second.second.parameter_block_info_.obj_id_.value();
      if (block_info.first == 0) {
        if (zero_objs.find(block_id) == zero_objs.end()) {
          zero_objs[block_id] = {};
        }
        zero_objs[block_id].emplace_back(
            getNameOfSlamParameterType(block_info.second.second.param_type_));
      }
    } else if (block_info.second.second.parameter_block_info_.frame_id_
                   .has_value()) {
      block_type = "frame";
      block_id =
          block_info.second.second.parameter_block_info_.frame_id_.value();
    } else if (block_info.second.second.parameter_block_info_.feature_id_
                   .has_value()) {
      block_type = "feature";
      block_id =
          block_info.second.second.parameter_block_info_.feature_id_.value();
    }
    LOG(INFO) << "Param block type and id: " << block_type << " " << block_id;
    LOG(INFO) << "Param entry type: "
              << getNameOfSlamParameterType(
                     block_info.second.second.param_type_);
    LOG(INFO) << "Param num: " << block_info.second.first;
  }

  LOG(INFO) << "Largest " << num_largest_hessian_diag_values << " elements";
  for (size_t param_block_num = 0; param_block_num < largest_elements.size();
       param_block_num++) {
    std::pair<double, std::pair<size_t, SlamParameterInfo>> block_info =
        largest_elements.at(param_block_num);
    LOG(INFO) << (largest_elements.size() - param_block_num) << "th largest";
    LOG(INFO) << "Hessian entry: " << block_info.first;
    uint64_t block_id;
    std::string block_type;
    if (block_info.second.second.parameter_block_info_.obj_id_.has_value()) {
      block_type = "object";
      block_id = block_info.second.second.parameter_block_info_.obj_id_.value();
    } else if (block_info.second.second.parameter_block_info_.frame_id_
                   .has_value()) {
      block_type = "frame";
      block_id =
          block_info.second.second.parameter_block_info_.frame_id_.value();
    } else if (block_info.second.second.parameter_block_info_.feature_id_
                   .has_value()) {
      block_type = "feature";
      block_id =
          block_info.second.second.parameter_block_info_.feature_id_.value();
    }
    LOG(INFO) << "Param block type and id: " << block_type << " " << block_id;
    LOG(INFO) << "Param entry type: "
              << getNameOfSlamParameterType(
                     block_info.second.second.param_type_);
    LOG(INFO) << "Param num: " << block_info.second.first;
  }

  std::unordered_map<ObjectId,
                     std::vector<std::pair<FactorType, std::optional<double>>>>
      objs_with_factors;
  std::unordered_map<
      ObjectId,
      std::unordered_map<FrameId, std::unordered_map<CameraId, double>>>
      objs_with_residuals;
  for (const GenericFactorInfo &factor : generic_factor_infos) {
    if (factor.obj_id_.has_value() && (factor.factor_type_ != 4)) {
      ObjectId obj_id = factor.obj_id_.value();
      if (objs_with_factors.find(obj_id) == objs_with_factors.end()) {
        objs_with_factors[obj_id] = {};
      }
      if (factor.final_residual_val_.has_value() &&
          factor.frame_ids_.has_value() && factor.camera_id_.has_value()) {
        FrameId frame_id = *(factor.frame_ids_.value().begin());
        objs_with_residuals[obj_id][frame_id][factor.camera_id_.value()] =
            factor.final_residual_val_.value();
      }
      objs_with_factors[obj_id].emplace_back(
          std::make_pair(factor.factor_type_, factor.final_residual_val_));
    }
  }

  LOG(INFO) << "Problem ids";
  for (const auto &obj_id_and_param_type : zero_objs) {
    ObjectId obj_id = obj_id_and_param_type.first;
    LOG(INFO) << obj_id;
    //    LOG(INFO) << "in factor set " << (objs_with_factors.find(obj_id) !=
    //        objs_with_factors.end());
    if (objs_with_factors.find(obj_id) != objs_with_factors.end()) {
      std::vector<std::pair<FactorType, std::optional<double>>> factors =
          objs_with_factors.at(obj_id);
      LOG(INFO) << "Factors for problem vec " << factors.size();
      //      for (const std::pair<FactorType, std::optional<double>>
      //      &factor_and_res : factors) {
      //        if (factor_and_res.second.has_value()) {
      //          LOG(INFO) << "res: " << factor_and_res.second.value();
      //        }
      //      }
      LOG(INFO) << "Viewed in frames";
      std::vector<std::pair<FrameId, CameraId>> sorted_frames(
          frames_for_each_obj[obj_id].begin(),
          frames_for_each_obj[obj_id].end());

      std::sort(sorted_frames.begin(),
                sorted_frames.end(),
                sortPairByFirstEntry<FrameId, CameraId>);

      for (const std::pair<FrameId, CameraId> &viewing_info : sorted_frames) {
        LOG(INFO) << "Frame " << viewing_info.first << ", cam "
                  << viewing_info.second;
        if (objs_with_residuals.find(obj_id) != objs_with_residuals.end()) {
          if (objs_with_residuals[obj_id].find(viewing_info.first) !=
              objs_with_residuals[obj_id].end()) {
            if (objs_with_residuals[obj_id][viewing_info.first].find(
                    viewing_info.second) !=
                objs_with_residuals[obj_id][viewing_info.first].end()) {
              LOG(INFO) << "Corresponding residual: "
                        << objs_with_residuals[obj_id][viewing_info.first]
                                              [viewing_info.second];
            }
          }
        }
      }
    }
    LOG(INFO) << "Param types, size " << obj_id_and_param_type.second.size();
    //    for (const std::string &param_type : obj_id_and_param_type.second) {
    //      LOG(INFO) << param_type;
    //    }
  }
}