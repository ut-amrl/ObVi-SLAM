//
// Created by amanda on 2/19/23.
//

#ifndef UT_VSLAM_SEQUENCE_FILE_STORAGE_IO_H
#define UT_VSLAM_SEQUENCE_FILE_STORAGE_IO_H

#include <file_io/cv_file_storage/file_storage_io_utils.h>
#include <types/sequence_utils.h>

namespace vslam_types_refactor {
class SerializableSequenceInfo : public FileStorageSerializable<SequenceInfo> {
 public:
  SerializableSequenceInfo() : FileStorageSerializable<SequenceInfo>() {}
  SerializableSequenceInfo(const SequenceInfo &data)
      : FileStorageSerializable<SequenceInfo>(data) {}

  virtual void write(cv::FileStorage &fs) const override {
    fs << kSequenceIdLabel << data_.sequence_id_;
    fs << kSequenceLabel << "[";
    for (const auto &bag_name : data_.bag_base_names_) {
      fs << bag_name;
    }
    fs << "]";
  }

  virtual void read(const cv::FileNode &node) override {
    node[kSequenceIdLabel] >> data_.sequence_id_;
    cv::FileNode bag_names_node = node[kSequenceLabel];
    for (cv::FileNodeIterator it = bag_names_node.begin();
         it != bag_names_node.end();
         it++) {
      cv::FileNode entry = *it;
      std::string bag_name;
      entry >> bag_name;
      data_.bag_base_names_.emplace_back(bag_name);
    }
  }

 protected:
  using FileStorageSerializable<SequenceInfo>::data_;

 private:
  inline static const std::string kSequenceIdLabel = "seq_id";
  inline static const std::string kSequenceLabel = "sequence";
};

static void write(cv::FileStorage &fs,
                  const std::string &,
                  const SerializableSequenceInfo &data) {
  data.write(fs);
}

static void read(
    const cv::FileNode &node,
    SerializableSequenceInfo &data,
    const SerializableSequenceInfo &default_data = SerializableSequenceInfo()) {
  if (node.empty()) {
    data = default_data;
  } else {
    data.read(node);
  }
}

void writeSequenceInfo(const std::string &sequence_file,
                       const SequenceInfo &sequence_info) {
  cv::FileStorage sequence_out(sequence_file, cv::FileStorage::WRITE);
  sequence_out << SerializableSequenceInfo(sequence_info);
  sequence_out.release();
}

void readSequenceInfo(const std::string &sequence_file,
                      SequenceInfo &sequence_info) {
  cv::FileStorage sequence_in(sequence_file, cv::FileStorage::READ);
  SerializableSequenceInfo serializable_sequence_info;
  sequence_in.root() >> serializable_sequence_info;
  sequence_in.release();
  sequence_info = serializable_sequence_info.getEntry();
}
}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_SEQUENCE_FILE_STORAGE_IO_H
