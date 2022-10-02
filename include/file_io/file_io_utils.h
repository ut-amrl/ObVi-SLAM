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
    std::function<void(const std::vector<std::string> &, T &)>
        object_from_line_reader,
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
    LOG(ERROR) << "The file was completely empty (and likely doesn't exist). File " << file_name;
    exit(1);
  }
}

template <typename T>
void writeObjectsToFile(const std::string &file_name,
                        const std::vector<std::string> &entry_names,
                        const std::vector<T> &objects,
                        std::function<std::vector<std::string>(const T &)>
                            object_to_str_list_converter) {
  std::ofstream csv_file(file_name, std::ios::trunc);
  writeCommaSeparatedStringsLineToFile(entry_names, csv_file);
  for (const T &obj : objects) {
    writeCommaSeparatedStringsLineToFile(object_to_str_list_converter(obj),
                                         csv_file);
  }
  csv_file.close();
}

template <typename LineEntry>
bool readNumObjectsIndicatedByFirstLine(
    std::ifstream &file_stream,
    const std::function<bool(const std::string &, LineEntry &)> &entry_reader,
    std::vector<LineEntry> &line_contents) {
  std::string line;
  if (!std::getline(file_stream, line)) {
    LOG(ERROR) << "File ended unexpectedly";
    return false;
  }
  // Read the number
  int num_entries = std::stoi(line);
  for (int i = 0; i < num_entries; i++) {
    if (!std::getline(file_stream, line)) {
      LOG(ERROR) << "File ended unexpectedly";
      return false;
    }
    LineEntry line_entry;
    if (!entry_reader(line, line_contents)) {
      LOG(ERROR)
          << "Could not convert string to appropriate entry type; string: "
          << line;
      return false;
    }
    line_contents.emplace_back(line_entry);
  }
  return true;
}

/**
 * This assumes entry writer adds the new line.
 *
 * @tparam LineEntry
 * @param data
 * @param entry_writer
 * @param file_stream
 */
template <typename LineEntry>
void writeOneObjectPerLinePrecededByNumObjects(
    const std::vector<LineEntry> &data,
    const std::function<void(const LineEntry &, std::ofstream &)> &entry_writer,
    std::ofstream &file_stream) {
  writeCommaSeparatedStringsLineToFile({std::to_string(data.size())},
                                       file_stream);
  for (const LineEntry &entry : data) {
    entry_writer(entry, file_stream);
  }
}

}  // namespace file_io

#endif  // UT_VSLAM_FILE_IO_UTILS_H
