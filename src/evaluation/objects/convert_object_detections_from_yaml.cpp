#include <file_io/global_object_estimates_io.h>
#include <file_io/obj_yaml_reader.h>
#include <glog/logging.h>
#include <ros/ros.h>

#include <iostream>

DEFINE_string(param_prefix, "", "param_prefix");

DEFINE_string(convert_object_detections_from_yaml, "", "yaml input file");
DEFINE_string(obj_output_file_name, "", "csv output file");

int main(int argc, char **argv) {
  google::ParseCommandLineFlags(&argc, &argv, false);

  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = true;

  std::string param_prefix = FLAGS_param_prefix;
  std::string node_prefix = FLAGS_param_prefix;
  if (!param_prefix.empty()) {
    param_prefix = "/" + param_prefix + "/";
    node_prefix += "_";
  }
  LOG(INFO) << "Prefix: " << param_prefix;

  ros::init(argc, argv, node_prefix + "obj_detection_converter");
  ros::NodeHandle n;

  std::string object_yaml_file = FLAGS_convert_object_detections_from_yaml;
  std::string obj_output_file = FLAGS_obj_output_file_name;

  std::vector<file_io::ObjectEst> obj_ests =
      file_io::readObjectEstimatesFromYaml(object_yaml_file);

  file_io::writeObjectEstsToFile(obj_output_file, obj_ests);

  return 0;
}