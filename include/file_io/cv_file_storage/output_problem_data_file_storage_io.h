//
// Created by amanda on 8/23/22.
//

#ifndef UT_VSLAM_OUTPUT_PROBLEM_DATA_FILE_STORAGE_IO_H
#define UT_VSLAM_OUTPUT_PROBLEM_DATA_FILE_STORAGE_IO_H

#include <file_io/cv_file_storage/file_storage_io_utils.h>
#include <file_io/cv_file_storage/vslam_obj_types_file_storage_io.h>
#include <glog/logging.h>
#include <refactoring/output_problem_data.h>

namespace vslam_types_refactor {
class SerializableEllipsoidResults
    : public FileStorageSerializable<EllipsoidResults> {
 public:
  SerializableEllipsoidResults()
      : FileStorageSerializable<EllipsoidResults>() {}
  SerializableEllipsoidResults(const EllipsoidResults &data)
      : FileStorageSerializable<EllipsoidResults>(data) {}

  virtual void write(cv::FileStorage &fs) const override {
    fs << "{" << kResultsMapLabel << "[";
    for (const auto &ellipsoid_entry : data_.ellipsoids_) {
      SerializableEllipsoidState<double> entry_state(
          ellipsoid_entry.second.second);
      fs << "{";
      fs << kObjectIdLabel << SerializableObjectId(ellipsoid_entry.first);
      fs << kSemanticClassLabel << ellipsoid_entry.second.first;
      fs << kEllipsoidStateLabel << entry_state;
      fs << "}";
    }
    fs << "]";
    fs << "}";
  }

  virtual void read(const cv::FileNode &node) override {
    cv::FileNode results_map_data = node[kResultsMapLabel];
    for (cv::FileNodeIterator it = results_map_data.begin();
         it != results_map_data.end();
         it++) {
      cv::FileNode entry = *it;
      SerializableObjectId obj_id;
      entry[kObjectIdLabel] >> obj_id;

      std::string semantic_class;
      entry[kSemanticClassLabel] >> semantic_class;

      SerializableEllipsoidState<double> ellipsoid_state;
      entry[kEllipsoidStateLabel] >> ellipsoid_state;

      data_.ellipsoids_[obj_id.getEntry()] =
          std::make_pair(semantic_class, ellipsoid_state.getEntry());
    }
  }

 protected:
  using FileStorageSerializable<EllipsoidResults>::data_;

 private:
  inline static const std::string kResultsMapLabel = "ellipsoid_results_map";
  inline static const std::string kObjectIdLabel = "object_id";
  inline static const std::string kSemanticClassLabel = "class";
  inline static const std::string kEllipsoidStateLabel = "state";
};

static void write(cv::FileStorage &fs,
                  const std::string &,
                  const SerializableEllipsoidResults &data) {
  data.write(fs);
}

static void read(const cv::FileNode &node,
                 SerializableEllipsoidResults &data,
                 const SerializableEllipsoidResults &default_data =
                     SerializableEllipsoidResults()) {
  if (node.empty()) {
    data = default_data;
  } else {
    data.read(node);
  }
}

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_OUTPUT_PROBLEM_DATA_FILE_STORAGE_IO_H
