//
// Created by amanda on 8/23/22.
//

#ifndef UT_VSLAM_VSLAM_BASIC_TYPES_FILE_STORAGE_IO_H
#define UT_VSLAM_VSLAM_BASIC_TYPES_FILE_STORAGE_IO_H

#include <file_io/cv_file_storage/file_storage_io_utils.h>
#include <glog/logging.h>
#include <refactoring/types/vslam_basic_types_refactor.h>

namespace vslam_types_refactor {

template <typename NumType, int Rows, int Cols>
class SerializableEigenMat
    : public FileStorageSerializable<Eigen::Matrix<NumType, Rows, Cols>> {
 public:
  SerializableEigenMat()
      : FileStorageSerializable<Eigen::Matrix<NumType, Rows, Cols>>() {}
  SerializableEigenMat(const Eigen::Matrix<NumType, Rows, Cols> &mat_entry)
      : FileStorageSerializable<Eigen::Matrix<NumType, Rows, Cols>>(mat_entry) {
  }

  virtual void write(cv::FileStorage &fs) const override {
    fs << "{" << kRowsLabel << Rows << kColsLabel << Cols << kDataLabel << "[";
    //    fs << "{" << "rows" << Rows << "cols" << Cols << "data" << "[";
    for (size_t row = 0; row < Rows; row++) {
      for (size_t col = 0; col < Cols; col++) {
        fs << data_(row, col);
      }
    }
    fs << "]"
       << "}";
  }

  virtual void read(const cv::FileNode &node) override {
    int num_rows = node[kRowsLabel];
    int num_cols = node[kColsLabel];
    //    int num_rows = node["rows"];
    //    int num_cols = node["cols"];
    CHECK_EQ(num_rows, Rows);
    CHECK_EQ(num_cols, Cols);
    cv::FileNode mat_data = node[kDataLabel];
    //    cv::FileNode mat_data = node["data"];
    CHECK_EQ(mat_data.type(), cv::FileNode::SEQ);

    cv::FileNodeIterator mat_data_iter = mat_data.begin();
    cv::FileNodeIterator mat_data_iter_end = mat_data.end();
    int row = 0;
    int col = 0;
    for (; mat_data_iter != mat_data_iter_end; mat_data_iter++) {
      data_(row, col) = *mat_data_iter;
      col++;
      if (col == num_cols) {
        row++;
        col = 0;
      }
    }
    CHECK_EQ(row, Rows);
    CHECK_EQ(col, 0);
  }

 protected:
  using FileStorageSerializable<Eigen::Matrix<NumType, Rows, Cols>>::data_;

 private:
  inline static const std::string kRowsLabel = "Rows";
  inline static const std::string kColsLabel = "Cols";
  inline static const std::string kDataLabel = "Data";
};

template <typename NumType, int Rows, int Cols>
static void write(cv::FileStorage &fs,
                  const std::string &,
                  const SerializableEigenMat<NumType, Rows, Cols> &data) {
  data.write(fs);
}

template <typename NumType, int Rows, int Cols>
static void read(const cv::FileNode &node,
                 SerializableEigenMat<NumType, Rows, Cols> &data,
                 const SerializableEigenMat<NumType, Rows, Cols> &default_data =
                     SerializableEigenMat<NumType, Rows, Cols>()) {
  if (node.empty()) {
    data = default_data;
  } else {
    data.read(node);
  }
}

template <typename NumType>
class SerializableEigenAngleAxis
    : public FileStorageSerializable<Eigen::AngleAxis<NumType>> {
 public:
  SerializableEigenAngleAxis()
      : FileStorageSerializable<Eigen::AngleAxis<NumType>>() {}
  SerializableEigenAngleAxis(const Eigen::AngleAxis<NumType> &angle_axis)
      : FileStorageSerializable<Eigen::AngleAxis<NumType>>(angle_axis) {}

  virtual void write(cv::FileStorage &fs) const override {
    fs << "{" << kAngleMagnitudeLabel << data_.angle();
    SerializableEigenMat<NumType, 3, 1> axis(data_.axis());
    fs << kAxisLabel << axis;
    fs << "}";
  }

  virtual void read(const cv::FileNode &node) override {
    NumType angle = node[kAngleMagnitudeLabel];
    SerializableEigenMat<NumType, 3, 1> readable_axis;
    node[kAxisLabel] >> readable_axis;
    data_ = Eigen::AngleAxis<NumType>(angle, readable_axis.getEntry());
  }

 protected:
  using FileStorageSerializable<Eigen::AngleAxis<NumType>>::data_;

 private:
  inline static const std::string kAngleMagnitudeLabel = "angle";
  inline static const std::string kAxisLabel = "axis";
};

template <typename NumType>
static void write(cv::FileStorage &fs,
                  const std::string &,
                  const SerializableEigenAngleAxis<NumType> &data) {
  data.write(fs);
}

template <typename NumType>
static void read(const cv::FileNode &node,
                 SerializableEigenAngleAxis<NumType> &data,
                 const SerializableEigenAngleAxis<NumType> &default_data =
                     SerializableEigenAngleAxis<NumType>()) {
  if (node.empty()) {
    data = default_data;
  } else {
    data.read(node);
  }
}

template <typename NumType>
class SerializablePose3D : public FileStorageSerializable<Pose3D<NumType>> {
 public:
  SerializablePose3D() : FileStorageSerializable<Pose3D<NumType>>() {}
  SerializablePose3D(const Pose3D<NumType> &data)
      : FileStorageSerializable<Pose3D<NumType>>(data) {}

  virtual void write(cv::FileStorage &fs) const override {
    SerializableEigenMat<NumType, 3, 1> transl(data_.transl_);
    SerializableEigenAngleAxis<NumType> rot(data_.orientation_);

    fs << "{";
    fs << kTranslationLabel << transl;
    fs << kOrientationLabel << rot;
    fs << "}";
  }

  virtual void read(const cv::FileNode &node) override {
    SerializableEigenMat<NumType, 3, 1> transl;
    SerializableEigenAngleAxis<NumType> rot;
    node[kTranslationLabel] >> transl;
    node[kOrientationLabel] >> rot;
    data_ = Pose3D(transl.getEntry(), rot.getEntry());
  }

 protected:
  using FileStorageSerializable<Pose3D<NumType>>::data_;

 private:
  inline static const std::string kTranslationLabel = "transl";
  inline static const std::string kOrientationLabel = "rot";
};

template <typename NumType>
static void write(cv::FileStorage &fs,
                  const std::string &,
                  const SerializablePose3D<NumType> &data) {
  data.write(fs);
}

template <typename NumType>
static void read(const cv::FileNode &node,
                 SerializablePose3D<NumType> &data,
                 const SerializablePose3D<NumType> &default_data =
                     SerializablePose3D<NumType>()) {
  if (node.empty()) {
    data = default_data;
  } else {
    data.read(node);
  }
}

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_VSLAM_BASIC_TYPES_FILE_STORAGE_IO_H
