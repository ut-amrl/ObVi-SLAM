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

template <typename KeyType,
          typename SerializableKeyType,
          typename ValueType,
          typename SerializableValueType>
class SerializableMap
    : public FileStorageSerializable<std::unordered_map<KeyType, ValueType>> {
 public:
  SerializableMap()
      : FileStorageSerializable<std::unordered_map<KeyType, ValueType>>() {}
  SerializableMap(const std::unordered_map<KeyType, ValueType> &data)
      : FileStorageSerializable<std::unordered_map<KeyType, ValueType>>(data) {}

  virtual void write(cv::FileStorage &fs) const override {
    fs << "[";
    for (const auto &map_entry : data_) {
      fs << "{";
      fs << kKeyLabel << SerializableKeyType(map_entry.first);
      fs << kValueLabel << SerializableValueType(map_entry.second);
      fs << "}";
    }
    fs << "]";
  }

  virtual void read(const cv::FileNode &node) override {
    for (cv::FileNodeIterator it = node.begin(); it != node.end(); it++) {
      cv::FileNode entry = *it;
      SerializableKeyType serializable_key;
      SerializableValueType serializable_val;
      entry[kKeyLabel] >> serializable_key;
      entry[kValueLabel] >> serializable_val;
      data_[serializable_key.getEntry()] = serializable_val.getEntry();
    }
  }

 protected:
  using FileStorageSerializable<std::unordered_map<KeyType, ValueType>>::data_;

 private:
  inline static const std::string kKeyLabel = "key";
  inline static const std::string kValueLabel = "value";
};

template <typename KeyType,
          typename SerializableKeyType,
          typename ValueType,
          typename SerializableValueType>
static void write(cv::FileStorage &fs,
                  const std::string &,
                  const SerializableMap<KeyType,
                                        SerializableKeyType,
                                        ValueType,
                                        SerializableValueType> &data) {
  data.write(fs);
}
template <typename KeyType,
          typename SerializableKeyType,
          typename ValueType,
          typename SerializableValueType>
static void read(const cv::FileNode &node,
                 SerializableMap<KeyType,
                                 SerializableKeyType,
                                 ValueType,
                                 SerializableValueType> &data,
                 const SerializableMap<KeyType,
                                       SerializableKeyType,
                                       ValueType,
                                       SerializableValueType> &default_data =
                     SerializableMap<KeyType,
                                     SerializableKeyType,
                                     ValueType,
                                     SerializableValueType>()) {
  if (node.empty()) {
    data = default_data;
  } else {
    data.read(node);
  }
}

template <typename FirstEntryType,
          typename SerializableFirstEntryType,
          typename SecondEntryType,
          typename SerializableSecondEntryType>
class SerializablePair : public FileStorageSerializable<
                             std::pair<FirstEntryType, SecondEntryType>> {
 public:
  SerializablePair()
      : FileStorageSerializable<std::pair<FirstEntryType, SecondEntryType>>() {}
  SerializablePair(const std::pair<FirstEntryType, SecondEntryType> &data)
      : FileStorageSerializable<std::pair<FirstEntryType, SecondEntryType>>(
            data) {}

  virtual void write(cv::FileStorage &fs) const override {
    fs << "{";
    fs << kFirstEntryLabel << SerializableFirstEntryType(data_.first);
    fs << kSecondEntryLabel << SerializableSecondEntryType(data_.second);
    fs << "}";
  }

  virtual void read(const cv::FileNode &node) override {
    SerializableFirstEntryType serializable_first;
    SerializableSecondEntryType serializable_second;
    node[kFirstEntryLabel] >> serializable_first;
    node[kSecondEntryLabel] >> serializable_second;
    data_ = std::make_pair(serializable_first.getEntry(), serializable_second.getEntry());
  }

 protected:
  using FileStorageSerializable<std::pair<FirstEntryType, SecondEntryType>>::data_;

 private:
  inline static const std::string kFirstEntryLabel = "first";
  inline static const std::string kSecondEntryLabel = "second";
};

template <typename FirstEntryType,
          typename SerializableFirstEntryType,
          typename SecondEntryType,
          typename SerializableSecondEntryType>
static void write(cv::FileStorage &fs,
                  const std::string &,
                  const SerializablePair<FirstEntryType,
                                         SerializableFirstEntryType,
                                         SecondEntryType,
                                         SerializableSecondEntryType> &data) {
  data.write(fs);
}
template <typename FirstEntryType,
          typename SerializableFirstEntryType,
          typename SecondEntryType,
          typename SerializableSecondEntryType>
static void read(
    const cv::FileNode &node,
    SerializablePair<FirstEntryType,
                     SerializableFirstEntryType,
                     SecondEntryType,
                     SerializableSecondEntryType> &data,
    const SerializablePair<FirstEntryType,
                           SerializableFirstEntryType,
                           SecondEntryType,
                           SerializableSecondEntryType> &default_data =
        SerializablePair<FirstEntryType,
                         SerializableFirstEntryType,
                         SecondEntryType,
                         SerializableSecondEntryType>()) {
  if (node.empty()) {
    data = default_data;
  } else {
    data.read(node);
  }
}

template <typename EntryType, typename SerializableEntryType>
class SerializableVector
    : public FileStorageSerializable<std::vector<EntryType>> {
 public:
  SerializableVector() : FileStorageSerializable<std::vector<EntryType>>() {}
  SerializableVector(const std::vector<EntryType> &data)
      : FileStorageSerializable<std::vector<EntryType>>(data) {}

  virtual void write(cv::FileStorage &fs) const override {
    fs << "[";
    for (const EntryType &vec_entry : data_) {
      fs << SerializableEntryType(vec_entry);
    }
    fs << "]";
  }

  virtual void read(const cv::FileNode &node) override {
    for (cv::FileNodeIterator it = node.begin(); it != node.end(); it++) {
      cv::FileNode entry = *it;
      SerializableEntryType serializable_entry;
      entry >> serializable_entry;
      data_.emplace_back(serializable_entry.getEntry());
    }
  }

 protected:
  using FileStorageSerializable<std::vector<EntryType>>::data_;
};

template <typename EntryType, typename SerializableEntryType>
static void write(
    cv::FileStorage &fs,
    const std::string &,
    const SerializableVector<EntryType, SerializableEntryType> &data) {
  data.write(fs);
}
template <typename EntryType, typename SerializableEntryType>
static void read(
    const cv::FileNode &node,
    SerializableVector<EntryType, SerializableEntryType> &data,
    const SerializableVector<EntryType, SerializableEntryType> &default_data =
        SerializableVector<EntryType, SerializableEntryType>()) {
  if (node.empty()) {
    data = default_data;
  } else {
    data.read(node);
  }
}

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_FILE_STORAGE_IO_UTILS_H
