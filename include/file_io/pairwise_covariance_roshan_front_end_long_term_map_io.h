//
// Created by amanda on 8/9/22.
//

#ifndef UT_VSLAM_PAIRWISE_COVARIANCE_ROSHAN_FRONT_END_LONG_TERM_MAP_IO_H
#define UT_VSLAM_PAIRWISE_COVARIANCE_ROSHAN_FRONT_END_LONG_TERM_MAP_IO_H

#include <file_io/file_io_utils.h>
#include <glog/logging.h>
#include <refactoring/bounding_box_frontend/roshan_bounding_box_front_end.h>
#include <refactoring/long_term_map/long_term_object_map.h>

namespace file_io {

void writeRoshanFrontEndEntryToFile(
    const std::pair<vslam_types_refactor::ObjectId,
                    vslam_types_refactor::RoshanAggregateBbInfo>
        &roshan_front_end_entry,
    std::ofstream &file_stream) {
  vslam_types_refactor::RoshanAggregateBbInfo vis_info =
      roshan_front_end_entry.second;
  writeCommaSeparatedStringsLineToFile(
      {std::to_string(roshan_front_end_entry.first)}, file_stream);
  writeCommaSeparatedStringsLineToFile(
      {std::to_string(
          roshan_front_end_entry.second.infos_for_observed_bbs_.size())},
      file_stream);
  for (const vslam_types_refactor::RoshanBbInfo &indiv_entry :
       roshan_front_end_entry.second.infos_for_observed_bbs_) {
    vslam_types_refactor::Pose3D<double> ellipsoid_pose =
        indiv_entry.single_bb_init_est_.pose_;
    vslam_types_refactor::ObjectDim<double> ellipsoid_dim =
        indiv_entry.single_bb_init_est_.dimensions_;

    writeCommaSeparatedStringsLineToFile(
        {std::to_string(indiv_entry.est_generated_ ? 1 : 0),
         std::to_string(ellipsoid_pose.transl_.x()),
         std::to_string(ellipsoid_pose.transl_.y()),
         std::to_string(ellipsoid_pose.transl_.z()),
         std::to_string(ellipsoid_pose.orientation_.angle()),
         std::to_string(ellipsoid_pose.orientation_.axis().x()),
         std::to_string(ellipsoid_pose.orientation_.axis().y()),
         std::to_string(ellipsoid_pose.orientation_.axis().z()),
         std::to_string(ellipsoid_dim.x()),
         std::to_string(ellipsoid_dim.y()),
         std::to_string(ellipsoid_dim.z())},
        file_stream);

    file_stream << indiv_entry.hue_sat_histogram_;
    file_stream << "\n";
  }
}

void writeRoshanFrontEndDataToFile(
    const std::unordered_map<vslam_types_refactor::ObjectId,
                             vslam_types_refactor::RoshanAggregateBbInfo>
        &roshan_front_end_map,
    std::ofstream &file_stream) {
  // Write pairwise ellipsoid covariance
  std::vector<std::pair<vslam_types_refactor::ObjectId,
                        vslam_types_refactor::RoshanAggregateBbInfo>>
      roshan_entries;

  for (const auto &roshan_entry : roshan_front_end_map) {
    roshan_entries.emplace_back(
        std::make_pair(roshan_entry.first, roshan_entry.second));
  }
  writeOneObjectPerLinePrecededByNumObjects<
      std::pair<vslam_types_refactor::ObjectId,
                vslam_types_refactor::RoshanAggregateBbInfo>>(
      roshan_entries, writeRoshanFrontEndEntryToFile, file_stream);
}

bool readRoshanFrontEndDataFromFile(
    std::ifstream &file_stream,
    const std::unordered_map<vslam_types_refactor::ObjectId,
                             vslam_types_refactor::RoshanAggregateBbInfo>
        &roshan_front_end_map) {
  size_t num_ellipsoids;

  std::string line;
  if (!std::getline(file_stream, line)) {
    LOG(ERROR) << "File ended unexpectedly";
    return false;
  }

  std::istringstream stream_num_ellipsoids(line);
  stream_num_ellipsoids >> num_ellipsoids;

  for (size_t i = 0; i < num_ellipsoids; i++) {
    // Read object id

    if (!std::getline(file_stream, line)) {
      LOG(ERROR) << "File ended unexpectedly";
      return false;
    }

    vslam_types_refactor::ObjectId obj_id;
    std::istringstream stream_obj_id(line);
    stream_obj_id >> obj_id;

    if (!std::getline(file_stream, line)) {
      LOG(ERROR) << "File ended unexpectedly";
      return false;
    }

    size_t num_entries_per_ellipsoid;
    std::istringstream stream_num_entries_per_ellipsoid(line);
    stream_num_entries_per_ellipsoid >> num_entries_per_ellipsoid;

    for (size_t ellipsoid_entry_num = 0;
         ellipsoid_entry_num < num_entries_per_ellipsoid;
         ellipsoid_entry_num++) {
      if (!std::getline(file_stream, line)) {
        LOG(ERROR) << "File ended unexpectedly";
        return false;
      }

      std::vector<std::string> comma_separated_strings =
          parseCommaSeparatedStrings(line);
      vslam_types_refactor::RoshanBbInfo bb_info;
      size_t list_idx = 0;
      bb_info.est_generated_ = comma_separated_strings[list_idx++] == "1";

      double transl_x = std::stod(comma_separated_strings[list_idx++]);
      double transl_y = std::stod(comma_separated_strings[list_idx++]);
      double transl_z = std::stod(comma_separated_strings[list_idx++]);

      double orientation_angle = std::stod(comma_separated_strings[list_idx++]);
      double orientation_axis_x =
          std::stod(comma_separated_strings[list_idx++]);
      double orientation_axis_y =
          std::stod(comma_separated_strings[list_idx++]);
      double orientation_axis_z =
          std::stod(comma_separated_strings[list_idx++]);

      double dim_x = std::stod(comma_separated_strings[list_idx++]);
      double dim_y = std::stod(comma_separated_strings[list_idx++]);
      double dim_z = std::stod(comma_separated_strings[list_idx++]);

      bb_info.single_bb_init_est_ =
          vslam_types_refactor::EllipsoidState<double>(
              vslam_types_refactor::Pose3D<double>(
                  vslam_types_refactor::Position3d<double>(
                      transl_x, transl_y, transl_z),
                  Eigen::AngleAxis<double>(
                      orientation_angle,
                      Eigen::Vector3d(orientation_axis_x,
                                      orientation_axis_y,
                                      orientation_axis_z))),
              vslam_types_refactor::ObjectDim<double>(dim_x, dim_y, dim_z));

      //      file_stream >> bb_info.hue_sat_histogram_;
    }
  }
}

void writeEllipsoidResultEntryToFile(
    const std::pair<
        vslam_types_refactor::ObjectId,
        std::pair<std::string, vslam_types_refactor::EllipsoidState<double>>>
        &ellipsoid_entry,
    std::ofstream &file_stream) {
  vslam_types_refactor::Pose3D<double> ellipsoid_pose =
      ellipsoid_entry.second.second.pose_;
  vslam_types_refactor::ObjectDim<double> ellipsoid_dim =
      ellipsoid_entry.second.second.dimensions_;
  std::vector<std::string> entry_as_string_list = {
      std::to_string(ellipsoid_entry.first),
      ellipsoid_entry.second.first,
      std::to_string(ellipsoid_pose.transl_.x()),
      std::to_string(ellipsoid_pose.transl_.y()),
      std::to_string(ellipsoid_pose.transl_.z()),
      std::to_string(ellipsoid_pose.orientation_.angle()),
      std::to_string(ellipsoid_pose.orientation_.axis().x()),
      std::to_string(ellipsoid_pose.orientation_.axis().y()),
      std::to_string(ellipsoid_pose.orientation_.axis().z()),
      std::to_string(ellipsoid_dim.x()),
      std::to_string(ellipsoid_dim.y()),
      std::to_string(ellipsoid_dim.z())};
  writeCommaSeparatedStringsLineToFile(entry_as_string_list, file_stream);
}
bool readEllipsoidResultEntryFromFile(
    const std::string &line,
    std::pair<
        vslam_types_refactor::ObjectId,
        std::pair<std::string, vslam_types_refactor::EllipsoidState<double>>>
        &ellipsoid_entry) {
  std::vector<std::string> comma_separated_strings =
      parseCommaSeparatedStrings(line);
  size_t list_idx = 0;

  std::istringstream obj_id_stream(comma_separated_strings[list_idx++]);
  vslam_types_refactor::ObjectId obj_id;
  obj_id_stream >> obj_id;

  std::string semantic_class = comma_separated_strings[list_idx++];

  double transl_x = std::stod(comma_separated_strings[list_idx++]);
  double transl_y = std::stod(comma_separated_strings[list_idx++]);
  double transl_z = std::stod(comma_separated_strings[list_idx++]);

  double orientation_angle = std::stod(comma_separated_strings[list_idx++]);
  double orientation_axis_x = std::stod(comma_separated_strings[list_idx++]);
  double orientation_axis_y = std::stod(comma_separated_strings[list_idx++]);
  double orientation_axis_z = std::stod(comma_separated_strings[list_idx++]);

  double dim_x = std::stod(comma_separated_strings[list_idx++]);
  double dim_y = std::stod(comma_separated_strings[list_idx++]);
  double dim_z = std::stod(comma_separated_strings[list_idx++]);

  ellipsoid_entry = std::make_pair(
      obj_id,
      std::make_pair(
          semantic_class,
          vslam_types_refactor::EllipsoidState<double>(
              vslam_types_refactor::Pose3D<double>(
                  vslam_types_refactor::Position3d<double>(
                      transl_x, transl_y, transl_z),
                  Eigen::AngleAxis<double>(
                      orientation_angle,
                      Eigen::Vector3d(orientation_axis_x,
                                      orientation_axis_y,
                                      orientation_axis_z))),
              vslam_types_refactor::ObjectDim<double>(dim_x, dim_y, dim_z))));

  return true;
}

void writePairwiseEllipsoidCovarianceEntryToFile(
    const std::pair<std::pair<vslam_types_refactor::ObjectId,
                              vslam_types_refactor::ObjectId>,
                    Eigen::Matrix<double, 9, 9>> &ellipsoid_covariance_entry,
    std::ofstream &file_stream) {
  std::vector<std::string> entry_as_string_list = {
      std::to_string(ellipsoid_covariance_entry.first.first),
      std::to_string(ellipsoid_covariance_entry.first.second),
      std::to_string(ellipsoid_covariance_entry.second(0, 0)),
      std::to_string(ellipsoid_covariance_entry.second(0, 1)),
      std::to_string(ellipsoid_covariance_entry.second(0, 2)),
      std::to_string(ellipsoid_covariance_entry.second(0, 3)),
      std::to_string(ellipsoid_covariance_entry.second(0, 4)),
      std::to_string(ellipsoid_covariance_entry.second(0, 5)),
      std::to_string(ellipsoid_covariance_entry.second(0, 6)),
      std::to_string(ellipsoid_covariance_entry.second(0, 7)),
      std::to_string(ellipsoid_covariance_entry.second(0, 8)),

      std::to_string(ellipsoid_covariance_entry.second(1, 0)),
      std::to_string(ellipsoid_covariance_entry.second(1, 1)),
      std::to_string(ellipsoid_covariance_entry.second(1, 2)),
      std::to_string(ellipsoid_covariance_entry.second(1, 3)),
      std::to_string(ellipsoid_covariance_entry.second(1, 4)),
      std::to_string(ellipsoid_covariance_entry.second(1, 5)),
      std::to_string(ellipsoid_covariance_entry.second(1, 6)),
      std::to_string(ellipsoid_covariance_entry.second(1, 7)),
      std::to_string(ellipsoid_covariance_entry.second(1, 8)),

      std::to_string(ellipsoid_covariance_entry.second(2, 0)),
      std::to_string(ellipsoid_covariance_entry.second(2, 1)),
      std::to_string(ellipsoid_covariance_entry.second(2, 2)),
      std::to_string(ellipsoid_covariance_entry.second(2, 3)),
      std::to_string(ellipsoid_covariance_entry.second(2, 4)),
      std::to_string(ellipsoid_covariance_entry.second(2, 5)),
      std::to_string(ellipsoid_covariance_entry.second(2, 6)),
      std::to_string(ellipsoid_covariance_entry.second(2, 7)),
      std::to_string(ellipsoid_covariance_entry.second(2, 8)),

      std::to_string(ellipsoid_covariance_entry.second(3, 0)),
      std::to_string(ellipsoid_covariance_entry.second(3, 1)),
      std::to_string(ellipsoid_covariance_entry.second(3, 2)),
      std::to_string(ellipsoid_covariance_entry.second(3, 3)),
      std::to_string(ellipsoid_covariance_entry.second(3, 4)),
      std::to_string(ellipsoid_covariance_entry.second(3, 5)),
      std::to_string(ellipsoid_covariance_entry.second(3, 6)),
      std::to_string(ellipsoid_covariance_entry.second(3, 7)),
      std::to_string(ellipsoid_covariance_entry.second(3, 8)),

      std::to_string(ellipsoid_covariance_entry.second(4, 0)),
      std::to_string(ellipsoid_covariance_entry.second(4, 1)),
      std::to_string(ellipsoid_covariance_entry.second(4, 2)),
      std::to_string(ellipsoid_covariance_entry.second(4, 3)),
      std::to_string(ellipsoid_covariance_entry.second(4, 4)),
      std::to_string(ellipsoid_covariance_entry.second(4, 5)),
      std::to_string(ellipsoid_covariance_entry.second(4, 6)),
      std::to_string(ellipsoid_covariance_entry.second(4, 7)),
      std::to_string(ellipsoid_covariance_entry.second(4, 8)),

      std::to_string(ellipsoid_covariance_entry.second(5, 0)),
      std::to_string(ellipsoid_covariance_entry.second(5, 1)),
      std::to_string(ellipsoid_covariance_entry.second(5, 2)),
      std::to_string(ellipsoid_covariance_entry.second(5, 3)),
      std::to_string(ellipsoid_covariance_entry.second(5, 4)),
      std::to_string(ellipsoid_covariance_entry.second(5, 5)),
      std::to_string(ellipsoid_covariance_entry.second(5, 6)),
      std::to_string(ellipsoid_covariance_entry.second(5, 7)),
      std::to_string(ellipsoid_covariance_entry.second(5, 8)),

      std::to_string(ellipsoid_covariance_entry.second(6, 0)),
      std::to_string(ellipsoid_covariance_entry.second(6, 1)),
      std::to_string(ellipsoid_covariance_entry.second(6, 2)),
      std::to_string(ellipsoid_covariance_entry.second(6, 3)),
      std::to_string(ellipsoid_covariance_entry.second(6, 4)),
      std::to_string(ellipsoid_covariance_entry.second(6, 5)),
      std::to_string(ellipsoid_covariance_entry.second(6, 6)),
      std::to_string(ellipsoid_covariance_entry.second(6, 7)),
      std::to_string(ellipsoid_covariance_entry.second(6, 8)),

      std::to_string(ellipsoid_covariance_entry.second(7, 0)),
      std::to_string(ellipsoid_covariance_entry.second(7, 1)),
      std::to_string(ellipsoid_covariance_entry.second(7, 2)),
      std::to_string(ellipsoid_covariance_entry.second(7, 3)),
      std::to_string(ellipsoid_covariance_entry.second(7, 4)),
      std::to_string(ellipsoid_covariance_entry.second(7, 5)),
      std::to_string(ellipsoid_covariance_entry.second(7, 6)),
      std::to_string(ellipsoid_covariance_entry.second(7, 7)),
      std::to_string(ellipsoid_covariance_entry.second(7, 8)),

      std::to_string(ellipsoid_covariance_entry.second(8, 0)),
      std::to_string(ellipsoid_covariance_entry.second(8, 1)),
      std::to_string(ellipsoid_covariance_entry.second(8, 2)),
      std::to_string(ellipsoid_covariance_entry.second(8, 3)),
      std::to_string(ellipsoid_covariance_entry.second(8, 4)),
      std::to_string(ellipsoid_covariance_entry.second(8, 5)),
      std::to_string(ellipsoid_covariance_entry.second(8, 6)),
      std::to_string(ellipsoid_covariance_entry.second(8, 7)),
      std::to_string(ellipsoid_covariance_entry.second(8, 8))};
  writeCommaSeparatedStringsLineToFile(entry_as_string_list, file_stream);
}

bool readPairwiseEllipsoidCovarianceEntryFromFile(
    const std::string &line,
    std::pair<std::pair<vslam_types_refactor::ObjectId,
                        vslam_types_refactor::ObjectId>,
              Eigen::Matrix<double, 9, 9>> &ellipsoid_covariance_entry) {
  std::vector<std::string> com_sep_str = parseCommaSeparatedStrings(line);
  size_t list_idx = 0;

  std::istringstream obj_id_1_stream(com_sep_str[list_idx++]);
  vslam_types_refactor::ObjectId obj_id_1;
  obj_id_1_stream >> obj_id_1;

  std::istringstream obj_id_2_stream(com_sep_str[list_idx++]);
  vslam_types_refactor::ObjectId obj_id_2;
  obj_id_2_stream >> obj_id_2;

  double cov_00 = std::stod(com_sep_str[list_idx++]);
  double cov_01 = std::stod(com_sep_str[list_idx++]);
  double cov_02 = std::stod(com_sep_str[list_idx++]);
  double cov_03 = std::stod(com_sep_str[list_idx++]);
  double cov_04 = std::stod(com_sep_str[list_idx++]);
  double cov_05 = std::stod(com_sep_str[list_idx++]);
  double cov_06 = std::stod(com_sep_str[list_idx++]);
  double cov_07 = std::stod(com_sep_str[list_idx++]);
  double cov_08 = std::stod(com_sep_str[list_idx++]);

  double cov_10 = std::stod(com_sep_str[list_idx++]);
  double cov_11 = std::stod(com_sep_str[list_idx++]);
  double cov_12 = std::stod(com_sep_str[list_idx++]);
  double cov_13 = std::stod(com_sep_str[list_idx++]);
  double cov_14 = std::stod(com_sep_str[list_idx++]);
  double cov_15 = std::stod(com_sep_str[list_idx++]);
  double cov_16 = std::stod(com_sep_str[list_idx++]);
  double cov_17 = std::stod(com_sep_str[list_idx++]);
  double cov_18 = std::stod(com_sep_str[list_idx++]);

  double cov_20 = std::stod(com_sep_str[list_idx++]);
  double cov_21 = std::stod(com_sep_str[list_idx++]);
  double cov_22 = std::stod(com_sep_str[list_idx++]);
  double cov_23 = std::stod(com_sep_str[list_idx++]);
  double cov_24 = std::stod(com_sep_str[list_idx++]);
  double cov_25 = std::stod(com_sep_str[list_idx++]);
  double cov_26 = std::stod(com_sep_str[list_idx++]);
  double cov_27 = std::stod(com_sep_str[list_idx++]);
  double cov_28 = std::stod(com_sep_str[list_idx++]);

  double cov_30 = std::stod(com_sep_str[list_idx++]);
  double cov_31 = std::stod(com_sep_str[list_idx++]);
  double cov_32 = std::stod(com_sep_str[list_idx++]);
  double cov_33 = std::stod(com_sep_str[list_idx++]);
  double cov_34 = std::stod(com_sep_str[list_idx++]);
  double cov_35 = std::stod(com_sep_str[list_idx++]);
  double cov_36 = std::stod(com_sep_str[list_idx++]);
  double cov_37 = std::stod(com_sep_str[list_idx++]);
  double cov_38 = std::stod(com_sep_str[list_idx++]);

  double cov_40 = std::stod(com_sep_str[list_idx++]);
  double cov_41 = std::stod(com_sep_str[list_idx++]);
  double cov_42 = std::stod(com_sep_str[list_idx++]);
  double cov_43 = std::stod(com_sep_str[list_idx++]);
  double cov_44 = std::stod(com_sep_str[list_idx++]);
  double cov_45 = std::stod(com_sep_str[list_idx++]);
  double cov_46 = std::stod(com_sep_str[list_idx++]);
  double cov_47 = std::stod(com_sep_str[list_idx++]);
  double cov_48 = std::stod(com_sep_str[list_idx++]);

  double cov_50 = std::stod(com_sep_str[list_idx++]);
  double cov_51 = std::stod(com_sep_str[list_idx++]);
  double cov_52 = std::stod(com_sep_str[list_idx++]);
  double cov_53 = std::stod(com_sep_str[list_idx++]);
  double cov_54 = std::stod(com_sep_str[list_idx++]);
  double cov_55 = std::stod(com_sep_str[list_idx++]);
  double cov_56 = std::stod(com_sep_str[list_idx++]);
  double cov_57 = std::stod(com_sep_str[list_idx++]);
  double cov_58 = std::stod(com_sep_str[list_idx++]);

  double cov_60 = std::stod(com_sep_str[list_idx++]);
  double cov_61 = std::stod(com_sep_str[list_idx++]);
  double cov_62 = std::stod(com_sep_str[list_idx++]);
  double cov_63 = std::stod(com_sep_str[list_idx++]);
  double cov_64 = std::stod(com_sep_str[list_idx++]);
  double cov_65 = std::stod(com_sep_str[list_idx++]);
  double cov_66 = std::stod(com_sep_str[list_idx++]);
  double cov_67 = std::stod(com_sep_str[list_idx++]);
  double cov_68 = std::stod(com_sep_str[list_idx++]);

  double cov_70 = std::stod(com_sep_str[list_idx++]);
  double cov_71 = std::stod(com_sep_str[list_idx++]);
  double cov_72 = std::stod(com_sep_str[list_idx++]);
  double cov_73 = std::stod(com_sep_str[list_idx++]);
  double cov_74 = std::stod(com_sep_str[list_idx++]);
  double cov_75 = std::stod(com_sep_str[list_idx++]);
  double cov_76 = std::stod(com_sep_str[list_idx++]);
  double cov_77 = std::stod(com_sep_str[list_idx++]);
  double cov_78 = std::stod(com_sep_str[list_idx++]);

  double cov_80 = std::stod(com_sep_str[list_idx++]);
  double cov_81 = std::stod(com_sep_str[list_idx++]);
  double cov_82 = std::stod(com_sep_str[list_idx++]);
  double cov_83 = std::stod(com_sep_str[list_idx++]);
  double cov_84 = std::stod(com_sep_str[list_idx++]);
  double cov_85 = std::stod(com_sep_str[list_idx++]);
  double cov_86 = std::stod(com_sep_str[list_idx++]);
  double cov_87 = std::stod(com_sep_str[list_idx++]);
  double cov_88 = std::stod(com_sep_str[list_idx++]);

  Eigen::Matrix<double, 9, 9> cov;
  cov << cov_00, cov_01, cov_02, cov_03, cov_04, cov_05, cov_06, cov_07, cov_08,
      cov_10, cov_11, cov_12, cov_13, cov_14, cov_15, cov_16, cov_17, cov_18,
      cov_20, cov_21, cov_22, cov_23, cov_24, cov_25, cov_26, cov_27, cov_28,
      cov_30, cov_31, cov_32, cov_33, cov_34, cov_35, cov_36, cov_37, cov_38,
      cov_40, cov_41, cov_42, cov_43, cov_44, cov_45, cov_46, cov_47, cov_48,
      cov_50, cov_51, cov_52, cov_53, cov_54, cov_55, cov_56, cov_57, cov_58,
      cov_60, cov_61, cov_62, cov_63, cov_64, cov_65, cov_66, cov_67, cov_68,
      cov_70, cov_71, cov_72, cov_73, cov_74, cov_75, cov_76, cov_77, cov_78,
      cov_80, cov_81, cov_82, cov_83, cov_84, cov_85, cov_86, cov_87, cov_88;

  ellipsoid_covariance_entry =
      std::make_pair(std::make_pair(obj_id_1, obj_id_2), cov);
  return true;
}

template <typename FrontEndObjMapData>
bool readPairwiseCovarianceMapFromFile(
    const std::string &file_name,
    const std::function<bool(std::ifstream &, FrontEndObjMapData &)>
        &front_end_map_data_reader,
    std::shared_ptr<vslam_types_refactor::PairwiseCovarianceLongTermObjectMap<
        FrontEndObjMapData>> &map_data) {
  std::ifstream file_obj(file_name);
  map_data = std::make_shared<
      vslam_types_refactor::PairwiseCovarianceLongTermObjectMap<
          FrontEndObjMapData>>();

  // Read ellipsoid results
  std::vector<std::pair<
      vslam_types_refactor::ObjectId,
      std::pair<std::string, vslam_types_refactor::EllipsoidState<double>>>>
      ellipsoid_states;
  if (!readNumObjectsIndicatedByFirstLine<
          std::pair<vslam_types_refactor::ObjectId,
                    std::pair<std::string,
                              vslam_types_refactor::EllipsoidState<double>>>>(
          file_obj, readEllipsoidResultEntryFromFile, ellipsoid_states)) {
    LOG(ERROR) << "Could not read ellipsoid states from file";
    return false;
  }

  vslam_types_refactor::EllipsoidResults ellipsoid_results;
  for (const std::pair<
           vslam_types_refactor::ObjectId,
           std::pair<std::string, vslam_types_refactor::EllipsoidState<double>>>
           &ellipsoid_entry : ellipsoid_states) {
    ellipsoid_results.ellipsoids_[ellipsoid_entry.first] =
        ellipsoid_entry.second;
  }
  map_data->setEllipsoidResults(ellipsoid_results);

  // Read pairwise ellipsoid covariance
  std::vector<std::pair<
      std::pair<vslam_types_refactor::ObjectId, vslam_types_refactor::ObjectId>,
      Eigen::Matrix<double, 9, 9>>>
      pairwise_covariance_entries;
  if (!readNumObjectsIndicatedByFirstLine<
          std::pair<std::pair<vslam_types_refactor::ObjectId,
                              vslam_types_refactor::ObjectId>,
                    Eigen::Matrix<double, 9, 9>>>(
          file_obj,
          readPairwiseEllipsoidCovarianceEntryFromFile,
          pairwise_covariance_entries)) {
    LOG(ERROR) << "Could not read covariance entries from file";
    return false;
  }

  util::BoostHashMap<
      std::pair<vslam_types_refactor::ObjectId, vslam_types_refactor::ObjectId>,
      Eigen::Matrix<double, 9, 9>>
      cov_entries_map;
  for (const std::pair<std::pair<vslam_types_refactor::ObjectId,
                                 vslam_types_refactor::ObjectId>,
                       Eigen::Matrix<double, 9, 9>> &cov_entry :
       pairwise_covariance_entries) {
    cov_entries_map[cov_entry.first] = cov_entry.second;
  }
  map_data->setPairwiseEllipsoidCovariance(cov_entries_map);

  // Read front end map data
  FrontEndObjMapData front_end_data;
  if (!front_end_map_data_reader(file_obj, front_end_data)) {
    LOG(ERROR) << "Error while reading the front end data from the map";
    return false;
  }
  map_data->setFrontEndObjMapData(front_end_data);
  return true;
}

template <typename FrontEndObjMapData>
void writePairwiseCovarianceMapToFile(
    const std::shared_ptr<
        vslam_types_refactor::PairwiseCovarianceLongTermObjectMap<
            FrontEndObjMapData>> &map_data,
    const std::string &file_name,
    const std::function<void(const FrontEndObjMapData &, std::ofstream &)>
        &front_end_map_data_writer) {
  std::ofstream file_obj(file_name, std::ios::trunc);

  vslam_types_refactor::EllipsoidResults ellipsoid_results;
  map_data->getEllipsoidResults(ellipsoid_results);

  // Write ellipsoid results
  std::vector<std::pair<
      vslam_types_refactor::ObjectId,
      std::pair<std::string, vslam_types_refactor::EllipsoidState<double>>>>
      ellipsoid_states;
  for (const auto &ellipsoid_entry : ellipsoid_results.ellipsoids_) {
    ellipsoid_states.push_back(
        std::make_pair(ellipsoid_entry.first, ellipsoid_entry.second));
  }
  writeOneObjectPerLinePrecededByNumObjects<std::pair<
      vslam_types_refactor::ObjectId,
      std::pair<std::string, vslam_types_refactor::EllipsoidState<double>>>>(
      ellipsoid_states, writeEllipsoidResultEntryToFile, file_obj);

  // Write pairwise ellipsoid covariance
  util::BoostHashMap<
      std::pair<vslam_types_refactor::ObjectId, vslam_types_refactor::ObjectId>,
      Eigen::Matrix<double, 9, 9>>
      cov_entries_map = map_data->getPairwiseEllipsoidCovariances();
  std::vector<std::pair<
      std::pair<vslam_types_refactor::ObjectId, vslam_types_refactor::ObjectId>,
      Eigen::Matrix<double, 9, 9>>>
      pairwise_covariance_entries;
  for (const auto &cov_entry : cov_entries_map) {
    pairwise_covariance_entries.emplace_back(
        std::make_pair(cov_entry.first, cov_entry.second));
  }
  writeOneObjectPerLinePrecededByNumObjects<std::pair<
      std::pair<vslam_types_refactor::ObjectId, vslam_types_refactor::ObjectId>,
      Eigen::Matrix<double, 9, 9>>>(pairwise_covariance_entries,
                                    writePairwiseEllipsoidCovarianceEntryToFile,
                                    file_obj);

  // Write front end map data
  FrontEndObjMapData front_end_data;
  map_data->getFrontEndObjMapData(front_end_data);
  front_end_map_data_writer(front_end_data, file_obj);
}

}  // namespace file_io

#endif  // UT_VSLAM_PAIRWISE_COVARIANCE_ROSHAN_FRONT_END_LONG_TERM_MAP_IO_H
