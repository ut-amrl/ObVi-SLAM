//
// Created by amanda on 1/18/23.
//

#ifndef UT_VSLAM_GENERIC_FACTOR_INFO_FILE_STORAGE_IO_H
#define UT_VSLAM_GENERIC_FACTOR_INFO_FILE_STORAGE_IO_H

#include <refactoring/factors/generic_factor_info.h>
namespace vslam_types_refactor {
class SerializableGenericFactorInfo
    : public FileStorageSerializable<GenericFactorInfo> {
 public:
  SerializableGenericFactorInfo()
      : FileStorageSerializable<GenericFactorInfo>() {}
  SerializableGenericFactorInfo(const GenericFactorInfo &data)
      : FileStorageSerializable<GenericFactorInfo>(data) {}

  virtual void write(cv::FileStorage &fs) const override {
    fs << "{";
    fs << kFactorTypeLabel << data_.factor_type_;
    fs << kFrameIdLabel
       << SerializableOptional<FrameId, SerializableFrameId>(data_.frame_id_);
    fs << kCameraIdLabel
       << SerializableOptional<CameraId, SerializableCameraId>(
              data_.camera_id_);
    fs << kObjIdLabel
       << SerializableOptional<ObjectId, SerializableObjectId>(data_.obj_id_);
    fs << kFeatureIdLabel
       << SerializableOptional<FeatureId, SerializableFeatureId>(
              data_.feature_id_);
    fs << "}";
  }

  virtual void read(const cv::FileNode &node) override {
    node[kFactorTypeLabel] >> data_.factor_type_;

    SerializableOptional<FrameId, SerializableFrameId> serializable_frame_id;
    SerializableOptional<CameraId, SerializableCameraId> serializable_camera_id;
    SerializableOptional<ObjectId, SerializableObjectId> serializable_obj_id;
    SerializableOptional<FeatureId, SerializableFeatureId>
        serializable_feature_id;

    node[kFrameIdLabel] >> serializable_frame_id;
    node[kCameraIdLabel] >> serializable_camera_id;
    node[kObjIdLabel] >> serializable_obj_id;
    node[kFeatureIdLabel] >> serializable_feature_id;

    data_.frame_id_ = serializable_frame_id.getEntry();
    data_.camera_id_ = serializable_camera_id.getEntry();
    data_.obj_id_ = serializable_obj_id.getEntry();
    data_.feature_id_ = serializable_feature_id.getEntry();
  }

 protected:
  using FileStorageSerializable<GenericFactorInfo>::data_;

 private:
  inline static const std::string kFactorTypeLabel = "type";
  inline static const std::string kFrameIdLabel = "frame";
  inline static const std::string kCameraIdLabel = "cam";
  inline static const std::string kObjIdLabel = "obj";
  inline static const std::string kFeatureIdLabel = "feat";
};

static void write(cv::FileStorage &fs,
                  const std::string &,
                  const SerializableGenericFactorInfo &data) {
  data.write(fs);
}

static void read(const cv::FileNode &node,
                 SerializableGenericFactorInfo &data,
                 const SerializableGenericFactorInfo &default_data =
                     SerializableGenericFactorInfo()) {
  if (node.empty()) {
    data = default_data;
  } else {
    data.read(node);
  }
}

class SerializableParameterBlockInfo
    : public FileStorageSerializable<ParameterBlockInfo> {
 public:
  SerializableParameterBlockInfo()
      : FileStorageSerializable<ParameterBlockInfo>() {}
  SerializableParameterBlockInfo(const ParameterBlockInfo &data)
      : FileStorageSerializable<ParameterBlockInfo>(data) {}

  virtual void write(cv::FileStorage &fs) const override {
    fs << "{";
    fs << kFrameIdLabel
       << SerializableOptional<FrameId, SerializableFrameId>(data_.frame_id_);
    fs << kObjIdLabel
       << SerializableOptional<ObjectId, SerializableObjectId>(data_.obj_id_);
    fs << kFeatureIdLabel
       << SerializableOptional<FeatureId, SerializableFeatureId>(
              data_.feature_id_);
    fs << "}";
  }

  virtual void read(const cv::FileNode &node) override {
    SerializableOptional<FrameId, SerializableFrameId> serializable_frame_id;
    SerializableOptional<ObjectId, SerializableObjectId> serializable_obj_id;
    SerializableOptional<FeatureId, SerializableFeatureId>
        serializable_feature_id;

    node[kFrameIdLabel] >> serializable_frame_id;
    node[kObjIdLabel] >> serializable_obj_id;
    node[kFeatureIdLabel] >> serializable_feature_id;

    data_.frame_id_ = serializable_frame_id.getEntry();
    data_.obj_id_ = serializable_obj_id.getEntry();
    data_.feature_id_ = serializable_feature_id.getEntry();
  }

 protected:
  using FileStorageSerializable<ParameterBlockInfo>::data_;

 private:
  inline static const std::string kFrameIdLabel = "frame";
  inline static const std::string kObjIdLabel = "obj";
  inline static const std::string kFeatureIdLabel = "feat";
};

static void write(cv::FileStorage &fs,
                  const std::string &,
                  const SerializableParameterBlockInfo &data) {
  data.write(fs);
}

static void read(const cv::FileNode &node,
                 SerializableParameterBlockInfo &data,
                 const SerializableParameterBlockInfo &default_data =
                     SerializableParameterBlockInfo()) {
  if (node.empty()) {
    data = default_data;
  } else {
    data.read(node);
  }
}
}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_GENERIC_FACTOR_INFO_FILE_STORAGE_IO_H
