//
// Created by amanda on 2/5/22.
//

#ifndef UT_VSLAM_GLOBAL_OBJECT_ESTIMATES_IO_H
#define UT_VSLAM_GLOBAL_OBJECT_ESTIMATES_IO_H

#include <file_io/file_io_utils.h>
#include <refactoring/types/vslam_obj_opt_types_refactor.h>

#include <fstream>

namespace file_io {

namespace {
static const size_t kIdEntryIdx = 11;
}

struct ObjectEst {
  std::string semantic_class_;

  double transl_x_;
  double transl_y_;
  double transl_z_;
  double quat_x_;
  double quat_y_;
  double quat_z_;
  double quat_w_;
  double d_x_;
  double d_y_;
  double d_z_;
};

void readObjectEstLine(const std::vector<std::string> &entries_in_file_line,
                       ObjectEst &object_est) {
  size_t list_idx = 0;
  object_est.semantic_class_ = entries_in_file_line[list_idx++];

  object_est.transl_x_ = std::stod(entries_in_file_line[list_idx++]);
  object_est.transl_y_ = std::stod(entries_in_file_line[list_idx++]);
  object_est.transl_z_ = std::stod(entries_in_file_line[list_idx++]);
  object_est.quat_x_ = std::stod(entries_in_file_line[list_idx++]);
  object_est.quat_y_ = std::stod(entries_in_file_line[list_idx++]);
  object_est.quat_z_ = std::stod(entries_in_file_line[list_idx++]);
  object_est.quat_w_ = std::stod(entries_in_file_line[list_idx++]);
  object_est.d_x_ = std::stod(entries_in_file_line[list_idx++]);
  object_est.d_y_ = std::stod(entries_in_file_line[list_idx++]);
  object_est.d_z_ = std::stod(entries_in_file_line[list_idx++]);
}

void readObjectEstsFromFile(const std::string &file_name,
                            std::vector<ObjectEst> &object_ests) {
  std::function<void(const std::vector<std::string> &, ObjectEst &)>
      object_from_line_reader = readObjectEstLine;
  file_io::readObjectListFromFileWithHeader(
      file_name, object_from_line_reader, object_ests);
}

std::vector<std::string> convertObjectEstToStringList(
    const ObjectEst &object_est) {
  return {object_est.semantic_class_,
          std::to_string(object_est.transl_x_),
          std::to_string(object_est.transl_y_),
          std::to_string(object_est.transl_z_),
          std::to_string(object_est.quat_x_),
          std::to_string(object_est.quat_y_),
          std::to_string(object_est.quat_z_),
          std::to_string(object_est.quat_w_),
          std::to_string(object_est.d_x_),
          std::to_string(object_est.d_y_),
          std::to_string(object_est.d_z_)};
}

void writeObjectEstsToFile(const std::string &file_name,
                           const std::vector<ObjectEst> &object_ests) {
  std::function<std::vector<std::string>(const ObjectEst &)>
      object_to_str_list_converter = convertObjectEstToStringList;
  file_io::writeObjectsToFile(file_name,
                              {"semantic_class",
                               "transl_x",
                               "transl_y",
                               "transl_z",
                               "quat_x",
                               "quat_y",
                               "quat_z",
                               "quat_w",
                               "d_x",
                               "d_y",
                               "d_z"},
                              object_ests,
                              object_to_str_list_converter);
}

}  // namespace file_io

#endif  // UT_VSLAM_GLOBAL_OBJECT_ESTIMATES_IO_H
