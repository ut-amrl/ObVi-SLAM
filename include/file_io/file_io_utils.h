//
// Created by amanda on 6/10/21.
//

#ifndef UT_VSLAM_FILE_IO_UTILS_H
#define UT_VSLAM_FILE_IO_UTILS_H

#include <glog/logging.h>

#include <fstream>
#include <functional>
#include <sstream>
#include <string>
#include <vector>

namespace file_io {
void writeCommaSeparatedStringsLineToFile(
    const std::vector<std::string> &strings, std::ofstream &file_stream) {
  for (size_t i = 0; i < strings.size(); i++) {
    file_stream << strings[i];
    if (i == (strings.size() - 1)) {
      file_stream << "\n";
    } else {
      file_stream << ", ";
    }
  }
}

std::vector<std::string> parseCommaSeparatedStrings(const std::string &str) {
  std::vector<std::string> strs;
  std::stringstream ss(str);

  std::string substr;
  while (getline(ss, substr, ',')) {
    strs.emplace_back(substr);
  }
  return strs;
}

template <typename T>
void readObjectListFromFileWithHeader(
    const std::string &file_name,
    std::function<void(const std::vector<std::string> &, T &)> object_from_line_reader,
    std::vector<T> &objects) {
  std::ifstream file_obj(file_name);
  std::string line;
  bool first_line = true;
  while (std::getline(file_obj, line)) {
    if (first_line) {
      first_line = false;
      continue;
    }
    T object;
    object_from_line_reader(parseCommaSeparatedStrings(line), object);
    objects.emplace_back(object);
  }
  if (first_line) {
    LOG(ERROR) << "The file was completely empty (and likely doesn't exist)";
    exit(1);
  }
}

template <typename T>
void writeObjectsToFile(
    const std::string &file_name,
    const std::vector<std::string> &entry_names,
    const std::vector<T> &objects,
    std::function<std::vector<std::string>(const T &)> object_to_str_list_converter) {
  std::ofstream csv_file(file_name, std::ios::trunc);
  writeCommaSeparatedStringsLineToFile(entry_names, csv_file);
  for (const T &obj :objects) {
    writeCommaSeparatedStringsLineToFile(object_to_str_list_converter(obj), csv_file);
  }
  csv_file.close();
}
}  // namespace file_io

#endif  // UT_VSLAM_FILE_IO_UTILS_H
