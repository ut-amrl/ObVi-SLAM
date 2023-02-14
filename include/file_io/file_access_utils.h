//
// Created by amanda on 11/30/22.
//

#ifndef UT_VSLAM_FILE_ACCESS_UTILS_H
#define UT_VSLAM_FILE_ACCESS_UTILS_H

#include <glog/logging.h>

#include <boost/filesystem/operations.hpp>
#include <string>

namespace file_io {

const static std::string kJsonExtension = ".json";

std::string ensureDirectoryPathEndsWithSlash(
    const std::string &unvalidated_dir_path) {
  CHECK(!unvalidated_dir_path.empty()) << "Directory path cannot be empty";

  std::stringstream stringstream1;
  stringstream1 << unvalidated_dir_path.back();
  std::string lastChar = stringstream1.str();
  if (lastChar != "/") {
    return unvalidated_dir_path + "/";
  }
  return unvalidated_dir_path;
}

void makeDirectoryIfDoesNotExist(const std::string &dir_name) {
  boost::filesystem::create_directories(dir_name);
}
}  // namespace file_io

#endif  // UT_VSLAM_FILE_ACCESS_UTILS_H
