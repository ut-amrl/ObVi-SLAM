//
// Created by amanda on 8/23/22.
//

#ifndef UT_VSLAM_VSLAM_OBJ_TYPES_FILE_STORAGE_IO_H
#define UT_VSLAM_VSLAM_OBJ_TYPES_FILE_STORAGE_IO_H

#include <file_io/cv_file_storage/file_storage_io_utils.h>
#include <file_io/cv_file_storage/vslam_basic_types_file_storage_io.h>
#include <glog/logging.h>
#include <refactoring/types/vslam_obj_opt_types_refactor.h>

namespace vslam_types_refactor {

using SerializableObjectId = SerializableUint64;

using SerializableBbCornerPair = SerializablePair<PixelCoord<double>,
                                                  SerializablePixelCoord,
                                                  PixelCoord<double>,
                                                  SerializablePixelCoord>;

template <typename NumType>
class SerializableEllipsoidState
    : public FileStorageSerializable<EllipsoidState<NumType>> {
 public:
  SerializableEllipsoidState()
      : FileStorageSerializable<EllipsoidState<NumType>>() {}
  SerializableEllipsoidState(const EllipsoidState<NumType> &data)
      : FileStorageSerializable<EllipsoidState<NumType>>(data) {}

  virtual void write(cv::FileStorage &fs) const override {
#ifdef CONSTRAIN_ELLIPSOID_ORIENTATION
    SerializablePose3DYawOnly<NumType> pose(data_.pose_);
#else
    SerializablePose3D<NumType> pose(data_.pose_);
#endif
    SerializableEigenMat<NumType, 3, 1> dimensions(data_.dimensions_);

    fs << "{";
    fs << kPoseLabel << pose;
    fs << kDimensionsLabel << dimensions;
    fs << "}";
  }

  virtual void read(const cv::FileNode &node) override {
#ifdef CONSTRAIN_ELLIPSOID_ORIENTATION
    SerializablePose3DYawOnly<NumType> pose;
#else
    SerializablePose3D<NumType> pose;
#endif
    SerializableEigenMat<NumType, 3, 1> dimensions;
    node[kPoseLabel] >> pose;
    node[kDimensionsLabel] >> dimensions;
    data_ = EllipsoidState<NumType>(pose.getEntry(), dimensions.getEntry());
  }

 protected:
  using FileStorageSerializable<EllipsoidState<NumType>>::data_;

 private:
  inline static const std::string kPoseLabel = "pose";
  inline static const std::string kDimensionsLabel = "dim";
};

template <typename NumType>
static void write(cv::FileStorage &fs,
                  const std::string &,
                  const SerializableEllipsoidState<NumType> &data) {
  data.write(fs);
}

template <typename NumType>
static void read(const cv::FileNode &node,
                 SerializableEllipsoidState<NumType> &data,
                 const SerializableEllipsoidState<NumType> &default_data =
                     SerializableEllipsoidState<NumType>()) {
  if (node.empty()) {
    data = default_data;
  } else {
    data.read(node);
  }
}

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_VSLAM_OBJ_TYPES_FILE_STORAGE_IO_H
