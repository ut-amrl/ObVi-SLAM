//
// Created by amanda on 8/23/22.
//

#ifndef UT_VSLAM_FILE_STORAGE_IO_UTILS_H
#define UT_VSLAM_FILE_STORAGE_IO_UTILS_H

#include <opencv2/core.hpp>

namespace vslam_types_refactor {

template <typename DataType>
class FileStorageSerializable {
 public:
  FileStorageSerializable() = default;
  FileStorageSerializable(const DataType &data) : data_(data) {}

  virtual void write(cv::FileStorage &fs) const = 0;

  virtual void read(const cv::FileNode &node) = 0;

  DataType getEntry() { return data_; }

 protected:
  DataType data_;
};
}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_FILE_STORAGE_IO_UTILS_H
