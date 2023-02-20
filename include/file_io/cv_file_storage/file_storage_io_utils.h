//
// Created by amanda on 8/23/22.
//

#ifndef UT_VSLAM_FILE_STORAGE_IO_UTILS_H
#define UT_VSLAM_FILE_STORAGE_IO_UTILS_H

#include <base_lib/basic_utils.h>
#include <glog/logging.h>

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
  inline static const std::string kKeyLabel = "k";
  inline static const std::string kValueLabel = "v";
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
    data_ = std::make_pair(serializable_first.getEntry(),
                           serializable_second.getEntry());
  }

 protected:
  using FileStorageSerializable<
      std::pair<FirstEntryType, SecondEntryType>>::data_;

 private:
  inline static const std::string kFirstEntryLabel = "f";
  inline static const std::string kSecondEntryLabel = "s";
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
    for (size_t i = 0; i < data_.size(); i++) {
      EntryType vec_entry = data_[i];
      fs << "{";
      fs << kIndexLabel << (int)i;
      fs << kValueLabel << SerializableEntryType(vec_entry);
      fs << "}";
    }
    fs << "]";
  }

  virtual void read(const cv::FileNode &node) override {
    for (cv::FileNodeIterator it = node.begin(); it != node.end(); it++) {
      cv::FileNode entry = *it;
      SerializableEntryType serializable_entry;
      entry[kValueLabel] >> serializable_entry;
      data_.emplace_back(serializable_entry.getEntry());
    }
  }

 protected:
  using FileStorageSerializable<std::vector<EntryType>>::data_;

 private:
  inline static const std::string kIndexLabel = "i";
  inline static const std::string kValueLabel = "v";
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

class SerializableEmptyStruct
    : public FileStorageSerializable<util::EmptyStruct> {
 public:
  SerializableEmptyStruct() : FileStorageSerializable<util::EmptyStruct>() {}
  SerializableEmptyStruct(const util::EmptyStruct &data)
      : FileStorageSerializable<util::EmptyStruct>(data) {}

  virtual void write(cv::FileStorage &fs) const override {
    fs << "{";
    fs << "}";
  }

  virtual void read(const cv::FileNode &node) override {
    // TODO verify that read inherently does take care of the {} left on write
  }
};

static void write(cv::FileStorage &fs,
                  const std::string &,
                  const SerializableEmptyStruct &data) {
  data.write(fs);
}

static void read(
    const cv::FileNode &node,
    SerializableEmptyStruct &data,
    const SerializableEmptyStruct &default_data = SerializableEmptyStruct()) {
  if (node.empty()) {
    data = default_data;
  } else {
    data.read(node);
  }
}

template <typename EntryType, typename SerializableEntryType>
class SerializableOptional
    : public FileStorageSerializable<std::optional<EntryType>> {
 public:
  SerializableOptional()
      : FileStorageSerializable<std::optional<EntryType>>() {}
  SerializableOptional(const std::optional<EntryType> &data)
      : FileStorageSerializable<std::optional<EntryType>>(data) {}

  virtual void write(cv::FileStorage &fs) const override {
    fs << "{";
    if (data_.has_value()) {
      fs << kHasValueLabel << 1;
      fs << kValueLabel << SerializableEntryType(data_.value());
    } else {
      fs << kHasValueLabel << 0;
    }
    fs << "}";
  }

  virtual void read(const cv::FileNode &node) override {
    int hasValue = node[kHasValueLabel];
    if (hasValue == 1) {
      SerializableEntryType serializable_entry;
      node[kValueLabel] >> serializable_entry;
      data_ = serializable_entry.getEntry();
    } else if (hasValue == 0) {
      data_ = std::nullopt;
    } else {
      LOG(ERROR) << "Entry for " << kHasValueLabel
                 << " should have either been 0 or 1 but was " << hasValue
                 << ". Exiting";
      exit(1);
    }
  }

 protected:
  using FileStorageSerializable<std::optional<EntryType>>::data_;

 private:
  inline static const std::string kHasValueLabel = "has_v";
  inline static const std::string kValueLabel = "v";
};

template <typename EntryType, typename SerializableEntryType>
static void write(
    cv::FileStorage &fs,
    const std::string &,
    const SerializableOptional<EntryType, SerializableEntryType> &data) {
  data.write(fs);
}
template <typename EntryType, typename SerializableEntryType>
static void read(
    const cv::FileNode &node,
    SerializableOptional<EntryType, SerializableEntryType> &data,
    const SerializableOptional<EntryType, SerializableEntryType> &default_data =
        SerializableOptional<EntryType, SerializableEntryType>()) {
  if (node.empty()) {
    data = default_data;
  } else {
    data.read(node);
  }
}

class SerializableUint64 : public FileStorageSerializable<uint64_t> {
 public:
  SerializableUint64() : FileStorageSerializable<uint64_t>() {}
  SerializableUint64(const uint64_t &data)
      : FileStorageSerializable<uint64_t>(data) {}

  virtual void write(cv::FileStorage &fs) const override {
    std::string obj_id_str = std::to_string(data_);
    fs << obj_id_str;
  }

  virtual void read(const cv::FileNode &node) override {
    std::string obj_id_str;
    node >> obj_id_str;
    std::istringstream obj_id_stream(obj_id_str);
    obj_id_stream >> data_;
  }

 protected:
  using FileStorageSerializable<uint64_t>::data_;
};

static void write(cv::FileStorage &fs,
                  const std::string &,
                  const SerializableUint64 &data) {
  data.write(fs);
}

static void read(
    const cv::FileNode &node,
    SerializableUint64 &data,
    const SerializableUint64 &default_data = SerializableUint64()) {
  if (node.empty()) {
    data = default_data;
  } else {
    data.read(node);
  }
}

class SerializableDouble : public FileStorageSerializable<double> {
 public:
  SerializableDouble() : FileStorageSerializable<double>() {}
  SerializableDouble(const double &data)
      : FileStorageSerializable<double>(data) {}

  virtual void write(cv::FileStorage &fs) const override { fs << data_; }

  virtual void read(const cv::FileNode &node) override { node >> data_; }

 protected:
  using FileStorageSerializable<double>::data_;
};

static void write(cv::FileStorage &fs,
                  const std::string &,
                  const SerializableDouble &data) {
  data.write(fs);
}

static void read(
    const cv::FileNode &node,
    SerializableDouble &data,
    const SerializableDouble &default_data = SerializableDouble()) {
  if (node.empty()) {
    data = default_data;
  } else {
    data.read(node);
  }
}

class SerializableInt : public FileStorageSerializable<int> {
 public:
  SerializableInt() : FileStorageSerializable<int>() {}
  SerializableInt(const int &data)
      : FileStorageSerializable<int>(data) {}

  virtual void write(cv::FileStorage &fs) const override { fs << data_; }

  virtual void read(const cv::FileNode &node) override { node >> data_; }

 protected:
  using FileStorageSerializable<int>::data_;
};

static void write(cv::FileStorage &fs,
                  const std::string &,
                  const SerializableInt &data) {
  data.write(fs);
}

static void read(
    const cv::FileNode &node,
    SerializableInt &data,
    const SerializableInt &default_data = SerializableInt()) {
  if (node.empty()) {
    data = default_data;
  } else {
    data.read(node);
  }
}

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_FILE_STORAGE_IO_UTILS_H
