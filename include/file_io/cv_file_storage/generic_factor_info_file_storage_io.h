//
// Created by amanda on 1/18/23.
//

#ifndef UT_VSLAM_GENERIC_FACTOR_INFO_FILE_STORAGE_IO_H
#define UT_VSLAM_GENERIC_FACTOR_INFO_FILE_STORAGE_IO_H

#include <file_io/cv_file_storage/file_storage_io_utils.h>
#include <file_io/cv_file_storage/vslam_basic_types_file_storage_io.h>
#include <file_io/cv_file_storage/vslam_obj_types_file_storage_io.h>
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
       << SerializableOptional<
              std::unordered_set<FrameId>,
              SerializableUnorderedSet<FrameId, SerializableFrameId>>(
              data_.frame_ids_);
    fs << kCameraIdLabel
       << SerializableOptional<CameraId, SerializableCameraId>(
              data_.camera_id_);
    fs << kObjIdLabel
       << SerializableOptional<ObjectId, SerializableObjectId>(data_.obj_id_);
    fs << kFeatureIdLabel
       << SerializableOptional<FeatureId, SerializableFeatureId>(
              data_.feature_id_);
    fs << kFinalResidualValLabel
       << SerializableOptional<double, SerializableDouble>(
              data_.final_residual_val_);
    fs << "}";
  }

  virtual void read(const cv::FileNode &node) override {
    node[kFactorTypeLabel] >> data_.factor_type_;

    SerializableOptional<CameraId, SerializableCameraId> serializable_camera_id;
    SerializableOptional<ObjectId, SerializableObjectId> serializable_obj_id;
    SerializableOptional<FeatureId, SerializableFeatureId>
        serializable_feature_id;

    node[kCameraIdLabel] >> serializable_camera_id;
    node[kObjIdLabel] >> serializable_obj_id;
    node[kFeatureIdLabel] >> serializable_feature_id;

    data_.camera_id_ = serializable_camera_id.getEntry();
    data_.obj_id_ = serializable_obj_id.getEntry();
    data_.feature_id_ = serializable_feature_id.getEntry();

    for (const std::string &key : node.keys()) {
      if (key == kFrameIdLabel) {
        SerializableOptional<FrameId, SerializableFrameId>
            serializable_frame_id;
        node[kFrameIdLabel] >> serializable_frame_id;
        std::optional<FrameId> frame = serializable_frame_id.getEntry();
        if (frame.has_value()) {
          data_.frame_ids_ = {frame.value()};
        }
      } else if (key == kFrameIdsLabel) {
        SerializableOptional<
            std::unordered_set<FrameId>,
            SerializableUnorderedSet<FrameId, SerializableFrameId>>
            serializable_frame_ids;
        node[kFrameIdsLabel] >> serializable_frame_ids;
        data_.frame_ids_ = serializable_frame_ids.getEntry();
      } else if (key == kFinalResidualValLabel) {
        SerializableOptional<double, SerializableDouble> ser_final_residual_val;
        node[kFinalResidualValLabel] >> ser_final_residual_val;
        data_.final_residual_val_ = ser_final_residual_val.getEntry();
        break;
      }
    }
  }

 protected:
  using FileStorageSerializable<GenericFactorInfo>::data_;

 private:
  inline static const std::string kFactorTypeLabel = "type";
  inline static const std::string kFrameIdLabel = "frame";
  inline static const std::string kFrameIdsLabel = "frames";
  inline static const std::string kCameraIdLabel = "cam";
  inline static const std::string kObjIdLabel = "obj";
  inline static const std::string kFeatureIdLabel = "feat";
  inline static const std::string kFinalResidualValLabel = "final_residual_val";
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
