//
// Created by amanda on 8/17/23.
//

#ifndef UT_VSLAM_OBJ_YAML_READER_H
#define UT_VSLAM_OBJ_YAML_READER_H

#include <base_lib/pose_reps.h>
#include <file_io/global_object_estimates_io.h>
#include <yaml-cpp/yaml.h>

namespace file_io {
std::vector<ObjectEst> readObjectEstimatesFromYaml(std::string &file) {
  YAML::Node node;
  try {
    node = YAML::LoadFile(file);
  } catch (YAML::Exception const &e) {
    LOG(WARNING) << "Failed to open " << file << ": " << e.msg;
    return {};
  }

  YAML::Node tracks = node["tracks"];
  std::vector<ObjectEst> obj_estimates;
  for (size_t i = 0; i < tracks.size(); ++i) {
    YAML::Node annotation = tracks[i];
    YAML::Node t = annotation["track"];
    for (size_t j = 0; j < t.size(); ++j) {
      ObjectEst obj_est;
      YAML::Node inst = t[j];
      obj_est.semantic_class_ = inst["label"].as<std::string>();

      YAML::Node origin = inst["translation"];
      obj_est.transl_x_ = origin["x"].as<double>();
      obj_est.transl_y_ = origin["y"].as<double>();
      obj_est.transl_z_ = origin["z"].as<double>();

      YAML::Node rotation = inst["rotation"];
      obj_est.quat_w_ = rotation["w"].as<double>();
      obj_est.quat_x_ = rotation["x"].as<double>();
      obj_est.quat_y_ = rotation["y"].as<double>();
      obj_est.quat_z_ = rotation["z"].as<double>();

      YAML::Node dimension = inst["box"];
      // TODO are are dx and dy right or should they be switched
      obj_est.d_x_ = dimension["length"].as<double>();
      obj_est.d_y_ = dimension["width"].as<double>();
      obj_est.d_z_ = dimension["height"].as<double>();

      obj_estimates.emplace_back(obj_est);
    }
  }
  return obj_estimates;
}
}  // namespace file_io

#endif  // UT_VSLAM_OBJ_YAML_READER_H
