//
// Created by amanda on 2/19/23.
//

#ifndef UT_VSLAM_SEQUENCE_FILE_STORAGE_IO_H
#define UT_VSLAM_SEQUENCE_FILE_STORAGE_IO_H

#include <file_io/cv_file_storage/file_storage_io_utils.h>
#include <types/sequence_utils.h>

namespace vslam_types_refactor {

const std::string kSequenceInfoKey = "sequence_info";

class SerializableBagBaseNameAndWaypointFile
    : public FileStorageSerializable<BagBaseNameAndWaypointFile> {
 public:
  SerializableBagBaseNameAndWaypointFile()
      : FileStorageSerializable<BagBaseNameAndWaypointFile>() {}
  SerializableBagBaseNameAndWaypointFile(const BagBaseNameAndWaypointFile &data)
      : FileStorageSerializable<BagBaseNameAndWaypointFile>(data) {}

  virtual void write(cv::FileStorage &fs) const override {
    fs << "{";
    fs << kBagBaseNameLabel << data_.bag_base_name_;
    fs << kOptionalWaypointFileBaseNameLabel
       << SerializableOptional<std::string, SerializableString>(
              data_.optional_waypoint_file_base_name_);
    fs << "}";
  }

  virtual void read(const cv::FileNode &node) override {
    node[kBagBaseNameLabel] >> data_.bag_base_name_;
    SerializableOptional<std::string, SerializableString>
        ser_optional_waypoint_file_base_name;
    node[kOptionalWaypointFileBaseNameLabel] >>
        ser_optional_waypoint_file_base_name;
    data_.optional_waypoint_file_base_name_ =
        ser_optional_waypoint_file_base_name.getEntry();
  }

 protected:
  using FileStorageSerializable<BagBaseNameAndWaypointFile>::data_;

 private:
  inline static const std::string kBagBaseNameLabel = "bag_base_name";
  inline static const std::string kOptionalWaypointFileBaseNameLabel =
      "waypoint_file_base_name";
};

static void write(cv::FileStorage &fs,
                  const std::string &,
                  const SerializableBagBaseNameAndWaypointFile &data) {
  data.write(fs);
}

static void read(const cv::FileNode &node,
                 SerializableBagBaseNameAndWaypointFile &data,
                 const SerializableBagBaseNameAndWaypointFile &default_data =
                     SerializableBagBaseNameAndWaypointFile()) {
  if (node.empty()) {
    data = default_data;
  } else {
    data.read(node);
  }
}

class SerializableSequenceInfo : public FileStorageSerializable<SequenceInfo> {
 public:
  SerializableSequenceInfo() : FileStorageSerializable<SequenceInfo>() {}
  SerializableSequenceInfo(const SequenceInfo &data)
      : FileStorageSerializable<SequenceInfo>(data) {}

  virtual void write(cv::FileStorage &fs) const override {
    fs << "{";
    fs << kSequenceIdLabel << data_.sequence_id_;
    fs << kSequenceLabel << "[";
    for (const auto &bag_name_and_waypoint_info :
         data_.bag_base_names_and_waypoint_files) {
      fs << SerializableBagBaseNameAndWaypointFile(bag_name_and_waypoint_info);
    }
    fs << "]";
    fs << "}";
  }

  virtual void read(const cv::FileNode &node) override {
    node[kSequenceIdLabel] >> data_.sequence_id_;
    cv::FileNode bag_names_node = node[kSequenceLabel];
    for (cv::FileNodeIterator it = bag_names_node.begin();
         it != bag_names_node.end();
         it++) {
      cv::FileNode entry = *it;
      std::string bag_name;
      SerializableBagBaseNameAndWaypointFile ser_bag_name_and_waypoint_info;
      entry >> ser_bag_name_and_waypoint_info;
      data_.bag_base_names_and_waypoint_files.emplace_back(
          ser_bag_name_and_waypoint_info.getEntry());
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
  sequence_out << kSequenceInfoKey << SerializableSequenceInfo(sequence_info);
  sequence_out.release();
}

void readSequenceInfo(const std::string &sequence_file,
                      SequenceInfo &sequence_info) {
  cv::FileStorage sequence_in(sequence_file, cv::FileStorage::READ);
  SerializableSequenceInfo serializable_sequence_info;
  sequence_in[kSequenceInfoKey] >> serializable_sequence_info;
  sequence_in.release();
  sequence_info = serializable_sequence_info.getEntry();
}
}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_SEQUENCE_FILE_STORAGE_IO_H
