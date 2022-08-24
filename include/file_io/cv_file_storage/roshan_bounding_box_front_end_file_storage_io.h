//
// Created by amanda on 8/23/22.
//

#ifndef UT_VSLAM_ROSHAN_BOUNDING_BOX_FRONT_END_FILE_STORAGE_IO_H
#define UT_VSLAM_ROSHAN_BOUNDING_BOX_FRONT_END_FILE_STORAGE_IO_H

#include <file_io/cv_file_storage/file_storage_io_utils.h>
#include <file_io/cv_file_storage/vslam_obj_types_file_storage_io.h>
#include <glog/logging.h>
#include <refactoring/bounding_box_frontend/roshan_bounding_box_front_end.h>

namespace vslam_types_refactor {
class SerializableRoshanBbInfo : public FileStorageSerializable<RoshanBbInfo> {
 public:
  SerializableRoshanBbInfo() : FileStorageSerializable<RoshanBbInfo>() {}
  SerializableRoshanBbInfo(const RoshanBbInfo &data)
      : FileStorageSerializable<RoshanBbInfo>(data) {}

  virtual void write(cv::FileStorage &fs) const override {
    fs << "{";
    fs << kHistoLabel << data_.hue_sat_histogram_;
    fs << kSingleBbInitEstLabel
       << SerializableEllipsoidState<double>(data_.single_bb_init_est_);
    int est_gen = data_.est_generated_ ? 1 : 0;
    fs << kEstGeneratedLabel << est_gen;
    fs << "}";
  }

  virtual void read(const cv::FileNode &node) override {
    int est_gen;
    SerializableEllipsoidState<double> ellipsoid_state;
    node[kHistoLabel] >> data_.hue_sat_histogram_;
    node[kSingleBbInitEstLabel] >> ellipsoid_state;
    node[kEstGeneratedLabel] >> est_gen;
    data_.est_generated_ = est_gen != 0;
    data_.single_bb_init_est_ = ellipsoid_state.getEntry();
  }

 protected:
  using FileStorageSerializable<RoshanBbInfo>::data_;

 private:
  inline static const std::string kHistoLabel = "hue_sat_histo";
  inline static const std::string kSingleBbInitEstLabel = "single_bb_init_est";
  inline static const std::string kEstGeneratedLabel = "est_generated";
};

static void write(cv::FileStorage &fs,
                  const std::string &,
                  const SerializableRoshanBbInfo &data) {
  data.write(fs);
}

static void read(
    const cv::FileNode &node,
    SerializableRoshanBbInfo &data,
    const SerializableRoshanBbInfo &default_data = SerializableRoshanBbInfo()) {
  if (node.empty()) {
    data = default_data;
  } else {
    data.read(node);
  }
}

class SerializableRoshanAggregateBbInfo
    : public FileStorageSerializable<RoshanAggregateBbInfo> {
 public:
  SerializableRoshanAggregateBbInfo()
      : FileStorageSerializable<RoshanAggregateBbInfo>() {}
  SerializableRoshanAggregateBbInfo(const RoshanAggregateBbInfo &data)
      : FileStorageSerializable<RoshanAggregateBbInfo>(data) {}

  virtual void write(cv::FileStorage &fs) const override {
    fs << "[";
    for (const RoshanBbInfo &bb_info : data_.infos_for_observed_bbs_) {
      fs << SerializableRoshanBbInfo(bb_info);
    }
    fs << "]";
  }

  virtual void read(const cv::FileNode &node) override {
    for (cv::FileNodeIterator it = node.begin(); it != node.end(); it++) {
      cv::FileNode entry = *it;
      SerializableRoshanBbInfo serializable_bb_entry;
      entry >> serializable_bb_entry;
      data_.infos_for_observed_bbs_.emplace_back(
          serializable_bb_entry.getEntry());
    }
  }

 protected:
  using FileStorageSerializable<RoshanAggregateBbInfo>::data_;
};

static void write(cv::FileStorage &fs,
                  const std::string &,
                  const SerializableRoshanAggregateBbInfo &data) {
  data.write(fs);
}

static void read(const cv::FileNode &node,
                 SerializableRoshanAggregateBbInfo &data,
                 const SerializableRoshanAggregateBbInfo &default_data =
                     SerializableRoshanAggregateBbInfo()) {
  if (node.empty()) {
    data = default_data;
  } else {
    data.read(node);
  }
}

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_ROSHAN_BOUNDING_BOX_FRONT_END_FILE_STORAGE_IO_H
