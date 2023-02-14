//
// Created by amanda on 8/23/22.
//

#ifndef UT_VSLAM_LONG_TERM_OBJECT_MAP_FILE_STORAGE_IO_H
#define UT_VSLAM_LONG_TERM_OBJECT_MAP_FILE_STORAGE_IO_H

#include <file_io/cv_file_storage/file_storage_io_utils.h>
#include <file_io/cv_file_storage/output_problem_data_file_storage_io.h>
#include <file_io/cv_file_storage/roshan_bounding_box_front_end_file_storage_io.h>
#include <glog/logging.h>
#include <refactoring/long_term_map/long_term_object_map.h>

namespace vslam_types_refactor {

template <typename FrontEndObjMapData, typename SerializableFrontEndObjMapData>
class SerializableIndependentEllipsoidsLongTermObjectMap
    : public FileStorageSerializable<
          IndependentEllipsoidsLongTermObjectMap<FrontEndObjMapData>> {
 public:
  SerializableIndependentEllipsoidsLongTermObjectMap()
      : FileStorageSerializable<
            IndependentEllipsoidsLongTermObjectMap<FrontEndObjMapData>>() {}
  SerializableIndependentEllipsoidsLongTermObjectMap(
      const IndependentEllipsoidsLongTermObjectMap<FrontEndObjMapData> &data)
      : FileStorageSerializable<
            IndependentEllipsoidsLongTermObjectMap<FrontEndObjMapData>>(data) {}

  virtual void write(cv::FileStorage &fs) const override {
    fs << "{";
    fs << kEllipsoidParameterizationLabel << kEllipsoidParameterizationType;
    EllipsoidResults ellipsoid_results;
    data_.getEllipsoidResults(ellipsoid_results);
    fs << kEllipsoidResultsLabel
       << SerializableEllipsoidResults(ellipsoid_results);
    fs << kEllipsoidCovariancesLabel
       << SerializableMap<ObjectId,
                          SerializableObjectId,
                          Covariance<double, kEllipsoidParamterizationSize>,
                          SerializableEigenMat<double,
                                               kEllipsoidParamterizationSize,
                                               kEllipsoidParamterizationSize>>(
              data_.getEllipsoidCovariances());
    std::unordered_map<ObjectId, FrontEndObjMapData> front_end_data;
    data_.getFrontEndObjMapData(front_end_data);
    fs << kFrontEndMapDataLabel
       << SerializableMap<ObjectId,
                          SerializableObjectId,
                          FrontEndObjMapData,
                          SerializableFrontEndObjMapData>(front_end_data);
    fs << "}";
  }

  virtual void read(const cv::FileNode &node) override {
    std::string ellipsoid_parameterization_type;
    node[kEllipsoidParameterizationLabel] >> ellipsoid_parameterization_type;
    if (ellipsoid_parameterization_type != kEllipsoidParameterizationType) {
      LOG(ERROR) << "Ellipsoid parameterization was "
                 << ellipsoid_parameterization_type
                 << ", but code was compiled to expect "
                 << kEllipsoidParameterizationType << ". Exiting";
      exit(1);
    }
    SerializableEllipsoidResults ser_ellipsoid_results;
    node[kEllipsoidResultsLabel] >> ser_ellipsoid_results;

    data_.setEllipsoidResults(ser_ellipsoid_results.getEntry());

    cv::FileNode results_map_data = node[kEllipsoidCovariancesLabel];
    SerializableMap<ObjectId,
                    SerializableObjectId,
                    Covariance<double, kEllipsoidParamterizationSize>,
                    SerializableEigenMat<double,
                                         kEllipsoidParamterizationSize,
                                         kEllipsoidParamterizationSize>>
        serializable_cov_map;
    results_map_data >> serializable_cov_map;
    data_.setEllipsoidCovariances(serializable_cov_map.getEntry());

    SerializableMap<ObjectId,
                    SerializableObjectId,
                    FrontEndObjMapData,
                    SerializableFrontEndObjMapData>
        ser_front_end_data;
    node[kFrontEndMapDataLabel] >> ser_front_end_data;
    data_.setFrontEndObjMapData(ser_front_end_data.getEntry());
  }

 protected:
  using FileStorageSerializable<
      IndependentEllipsoidsLongTermObjectMap<FrontEndObjMapData>>::data_;

 private:
  inline static const std::string kEllipsoidParameterizationLabel =
      "ellipsoid_parameterization";
#ifdef CONSTRAIN_ELLIPSOID_ORIENTATION
  inline static const std::string kEllipsoidParameterizationType = "yaw_only";
#else
  inline static const std::string kEllipsoidParameterizationType = "full_dof";
#endif

  inline static const std::string kEllipsoidResultsLabel = "ellipsoid_results";
  inline static const std::string kEllipsoidCovariancesLabel =
      "obj_id_covariance_map";
  inline static const std::string kFrontEndMapDataLabel = "front_end_map_data";
};

template <typename FrontEndObjMapData, typename SerializableFrontEndObjMapData>
static void write(cv::FileStorage &fs,
                  const std::string &,
                  const SerializableIndependentEllipsoidsLongTermObjectMap<
                      FrontEndObjMapData,
                      SerializableFrontEndObjMapData> &data) {
  data.write(fs);
}

template <typename FrontEndObjMapData, typename SerializableFrontEndObjMapData>
static void read(const cv::FileNode &node,
                 SerializableIndependentEllipsoidsLongTermObjectMap<
                     FrontEndObjMapData,
                     SerializableFrontEndObjMapData> &data,
                 const SerializableIndependentEllipsoidsLongTermObjectMap<
                     FrontEndObjMapData,
                     SerializableFrontEndObjMapData> &default_data =
                     SerializableIndependentEllipsoidsLongTermObjectMap<
                         FrontEndObjMapData,
                         SerializableFrontEndObjMapData>()) {
  if (node.empty()) {
    data = default_data;
  } else {
    data.read(node);
  }
}

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_LONG_TERM_OBJECT_MAP_FILE_STORAGE_IO_H
