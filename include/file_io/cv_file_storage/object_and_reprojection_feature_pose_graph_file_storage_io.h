//
// Created by amanda on 3/28/23.
//

#ifndef UT_VSLAM_OBJECT_AND_REPROJECTION_FEATURE_POSE_GRAPH_FILE_STORAGE_IO_H
#define UT_VSLAM_OBJECT_AND_REPROJECTION_FEATURE_POSE_GRAPH_FILE_STORAGE_IO_H

#include <file_io/cv_file_storage/file_storage_io_utils.h>
#include <file_io/cv_file_storage/vslam_basic_types_file_storage_io.h>
#include <file_io/cv_file_storage/vslam_obj_types_file_storage_io.h>
#include <refactoring/optimization/object_pose_graph.h>

#include <filesystem>

namespace vslam_types_refactor {

const std::string kLtmCheckpointOutputFileBaseName = "long_term_map_checkpoint";
const std::string kPreOptimizationCheckpointOutputFileBaseName = "pose_graph_state_checkpoint_pre_optimization_";
const std::string kPoseGraphStateKey = "pose_graph";

using SerializableFeatureFactorId = SerializableUint64;

class SerializableFactorType : public FileStorageSerializable<FactorType> {
 public:
  SerializableFactorType() : FileStorageSerializable<FactorType>() {}
  SerializableFactorType(const FactorType &data)
      : FileStorageSerializable<FactorType>(data) {}

  virtual void write(cv::FileStorage &fs) const override { fs << (int)data_; }

  virtual void read(const cv::FileNode &node) override {
    int factor_type_int;
    node >> factor_type_int;
    data_ = (int8_t)factor_type_int;
  }

 protected:
  using FileStorageSerializable<FactorType>::data_;
};

static void write(cv::FileStorage &fs,
                  const std::string &,
                  const SerializableFactorType &data) {
  data.write(fs);
}

static void read(
    const cv::FileNode &node,
    SerializableFactorType &data,
    const SerializableFactorType &default_data = SerializableFactorType()) {
  if (node.empty()) {
    data = default_data;
  } else {
    data.read(node);
  }
}

class SerializableRelPoseFactor
    : public FileStorageSerializable<RelPoseFactor> {
 public:
  SerializableRelPoseFactor() : FileStorageSerializable<RelPoseFactor>() {}
  SerializableRelPoseFactor(const RelPoseFactor &data)
      : FileStorageSerializable<RelPoseFactor>(data) {}

  virtual void write(cv::FileStorage &fs) const override {
    fs << "{";
    fs << kFrameId1Label << SerializableFrameId(data_.frame_id_1_);
    fs << kFrameId2Label << SerializableFrameId(data_.frame_id_2_);
    fs << kMeasuredPoseDeviationLabel
       << SerializablePose3D<double>(data_.measured_pose_deviation_);
    fs << kPoseDeviationCovLabel
       << SerializableCovariance<double, 6>(data_.pose_deviation_cov_);
    fs << "}";
  }

  virtual void read(const cv::FileNode &node) override {
    SerializableFrameId ser_frame_id_1;
    node[kFrameId1Label] >> ser_frame_id_1;
    data_.frame_id_1_ = ser_frame_id_1.getEntry();
    SerializableFrameId ser_frame_id_2;
    node[kFrameId2Label] >> ser_frame_id_2;
    data_.frame_id_2_ = ser_frame_id_2.getEntry();
    SerializablePose3D<double> ser_measured_pose_deviation;
    node[kMeasuredPoseDeviationLabel] >> ser_measured_pose_deviation;
    data_.measured_pose_deviation_ = ser_measured_pose_deviation.getEntry();
    SerializableCovariance<double, 6> ser_pose_deviation_cov;
    node[kPoseDeviationCovLabel] >> ser_pose_deviation_cov;
    data_.pose_deviation_cov_ = ser_pose_deviation_cov.getEntry();
  }

 protected:
  using FileStorageSerializable<RelPoseFactor>::data_;

 private:
  inline static const std::string kFrameId1Label = "frame_id_1";
  inline static const std::string kFrameId2Label = "frame_id_2";
  inline static const std::string kMeasuredPoseDeviationLabel =
      "measured_pose_deviation";
  inline static const std::string kPoseDeviationCovLabel = "pose_deviation_cov";
};

static void write(cv::FileStorage &fs,
                  const std::string &,
                  const SerializableRelPoseFactor &data) {
  data.write(fs);
}

static void read(const cv::FileNode &node,
                 SerializableRelPoseFactor &data,
                 const SerializableRelPoseFactor &default_data =
                     SerializableRelPoseFactor()) {
  if (node.empty()) {
    data = default_data;
  } else {
    data.read(node);
  }
}

class SerializableShapeDimPriorFactor
    : public FileStorageSerializable<ShapeDimPriorFactor> {
 public:
  SerializableShapeDimPriorFactor()
      : FileStorageSerializable<ShapeDimPriorFactor>() {}
  SerializableShapeDimPriorFactor(const ShapeDimPriorFactor &data)
      : FileStorageSerializable<ShapeDimPriorFactor>(data) {}

  virtual void write(cv::FileStorage &fs) const override {
    fs << "{";
    fs << kObjectIdLabel << SerializableObjectId(data_.object_id_);
    fs << kMeanShapeDimLabel
       << SerializableObjectDim<double>(data_.mean_shape_dim_);
    fs << kShapeDimCovLabel
       << SerializableCovariance<double, 3>(data_.shape_dim_cov_);
    fs << "}";
  }

  virtual void read(const cv::FileNode &node) override {
    SerializableObjectId ser_object_id;
    node[kObjectIdLabel] >> ser_object_id;
    data_.object_id_ = ser_object_id.getEntry();
    SerializableObjectDim<double> ser_mean_shape_dim;
    node[kMeanShapeDimLabel] >> ser_mean_shape_dim;
    data_.mean_shape_dim_ = ser_mean_shape_dim.getEntry();
    SerializableCovariance<double, 3> ser_shape_dim_cov;
    node[kShapeDimCovLabel] >> ser_shape_dim_cov;
    data_.shape_dim_cov_ = ser_shape_dim_cov.getEntry();
  }

 protected:
  using FileStorageSerializable<ShapeDimPriorFactor>::data_;

 private:
  inline static const std::string kObjectIdLabel = "object_id";
  inline static const std::string kMeanShapeDimLabel = "mean_shape_dim";
  inline static const std::string kShapeDimCovLabel = "shape_dim_cov";
};

static void write(cv::FileStorage &fs,
                  const std::string &,
                  const SerializableShapeDimPriorFactor &data) {
  data.write(fs);
}

static void read(const cv::FileNode &node,
                 SerializableShapeDimPriorFactor &data,
                 const SerializableShapeDimPriorFactor &default_data =
                     SerializableShapeDimPriorFactor()) {
  if (node.empty()) {
    data = default_data;
  } else {
    data.read(node);
  }
}

class SerializableObjectObservationFactor
    : public FileStorageSerializable<ObjectObservationFactor> {
 public:
  SerializableObjectObservationFactor()
      : FileStorageSerializable<ObjectObservationFactor>() {}
  SerializableObjectObservationFactor(const ObjectObservationFactor &data)
      : FileStorageSerializable<ObjectObservationFactor>(data) {}

  virtual void write(cv::FileStorage &fs) const override {
    fs << "{";
    fs << kFrameIdLabel << SerializableFrameId(data_.frame_id_);
    fs << kCameraIdLabel << SerializableCameraId(data_.camera_id_);
    fs << kObjectIdLabel << SerializableObjectId(data_.object_id_);
    fs << kBoundingBoxCornersLabel
       << SerializableBbCorners<double>(data_.bounding_box_corners_);
    fs << kBoundingBoxCornersCovarianceLabel
       << SerializableCovariance<double, 4>(
              data_.bounding_box_corners_covariance_);
    fs << kDetectionConfidenceLabel << data_.detection_confidence_;
    fs << "}";
  }

  virtual void read(const cv::FileNode &node) override {
    SerializableFrameId ser_frame_id;
    node[kFrameIdLabel] >> ser_frame_id;
    data_.frame_id_ = ser_frame_id.getEntry();
    SerializableCameraId ser_camera_id;
    node[kCameraIdLabel] >> ser_camera_id;
    data_.camera_id_ = ser_camera_id.getEntry();
    SerializableObjectId ser_object_id;
    node[kObjectIdLabel] >> ser_object_id;
    data_.object_id_ = ser_object_id.getEntry();
    SerializableBbCorners<double> ser_bounding_box_corners;
    node[kBoundingBoxCornersLabel] >> ser_bounding_box_corners;
    data_.bounding_box_corners_ = ser_bounding_box_corners.getEntry();
    SerializableCovariance<double, 4> ser_bounding_box_corners_covariance;
    node[kBoundingBoxCornersCovarianceLabel] >>
        ser_bounding_box_corners_covariance;
    data_.bounding_box_corners_covariance_ =
        ser_bounding_box_corners_covariance.getEntry();
    data_.detection_confidence_ = node[kDetectionConfidenceLabel];
  }

 protected:
  using FileStorageSerializable<ObjectObservationFactor>::data_;

 private:
  inline static const std::string kFrameIdLabel = "frame_id";
  inline static const std::string kCameraIdLabel = "camera_id";
  inline static const std::string kObjectIdLabel = "object_id";
  inline static const std::string kBoundingBoxCornersLabel =
      "bounding_box_corners";
  inline static const std::string kBoundingBoxCornersCovarianceLabel =
      "bounding_box_corners_covariance";
  inline static const std::string kDetectionConfidenceLabel =
      "detection_confidence";
};

static void write(cv::FileStorage &fs,
                  const std::string &,
                  const SerializableObjectObservationFactor &data) {
  data.write(fs);
}

static void read(const cv::FileNode &node,
                 SerializableObjectObservationFactor &data,
                 const SerializableObjectObservationFactor &default_data =
                     SerializableObjectObservationFactor()) {
  if (node.empty()) {
    data = default_data;
  } else {
    data.read(node);
  }
}

template <typename VisualFeatureFactorType,
          typename SerializableVisualFeatureFactorType>
class SerializableLowLevelFeaturePoseGraphState
    : public FileStorageSerializable<
          LowLevelFeaturePoseGraphState<VisualFeatureFactorType>> {
 public:
  SerializableLowLevelFeaturePoseGraphState()
      : FileStorageSerializable<
            LowLevelFeaturePoseGraphState<VisualFeatureFactorType>>() {}
  SerializableLowLevelFeaturePoseGraphState(
      const LowLevelFeaturePoseGraphState<VisualFeatureFactorType> &data)
      : FileStorageSerializable<
            LowLevelFeaturePoseGraphState<VisualFeatureFactorType>>(data) {}

  virtual void write(cv::FileStorage &fs) const override {
    fs << "{";
    fs << kCameraExtrinsicsByCameraLabel
       << SerializableMap<CameraId,
                          SerializableCameraId,
                          CameraExtrinsics<double>,
                          SerializableCameraExtrinsics<double>>(
              data_.camera_extrinsics_by_camera_);
    fs << kCameraIntrinsicsByCameraLabel
       << SerializableMap<CameraId,
                          SerializableCameraId,
                          CameraIntrinsicsMat<double>,
                          SerializableCameraIntrinsicsMat<double>>(
              data_.camera_intrinsics_by_camera_);
    fs << kVisualFactorTypeLabel
       << SerializableFactorType(data_.visual_factor_type_);
    fs << kMinFrameIdLabel << SerializableFrameId(data_.min_frame_id_);
    fs << kMaxFrameIdLabel << SerializableFrameId(data_.max_frame_id_);
    fs << kMaxFeatureFactorIdLabel
       << SerializableFeatureFactorId(data_.max_feature_factor_id_);
    fs << kMaxPoseFactorIdLabel
       << SerializableFeatureFactorId(data_.max_pose_factor_id_);
    fs << kRobotPosesLabel
       << SerializableMap<FrameId,
                          SerializableFrameId,
                          RawPose3d<double>,
                          SerializableRawPose3d<double>>(data_.robot_poses_);
    fs << kPoseFactorsByFrameLabel
       << SerializableMap<
              FrameId,
              SerializableFrameId,
              util::BoostHashSet<std::pair<FactorType, FeatureFactorId>>,
              SerializableBoostHashSet<
                  std::pair<FactorType, FeatureFactorId>,
                  SerializablePair<FactorType,
                                   SerializableFactorType,
                                   FeatureFactorId,
                                   SerializableFeatureFactorId>>>(
              data_.pose_factors_by_frame_);
    fs << kVisualFeatureFactorsByFrameLabel
       << SerializableMap<FrameId,
                          SerializableFrameId,
                          std::vector<std::pair<FactorType, FeatureFactorId>>,
                          SerializableVector<
                              std::pair<FactorType, FeatureFactorId>,
                              SerializablePair<FactorType,
                                               SerializableFactorType,
                                               FeatureFactorId,
                                               SerializableFeatureFactorId>>>(
              data_.visual_feature_factors_by_frame_);
    fs << kVisualFactorsByFeatureLabel
       << SerializableMap<
              FeatureId,
              SerializableFeatureId,
              util::BoostHashSet<std::pair<FactorType, FeatureFactorId>>,
              SerializableBoostHashSet<
                  std::pair<FactorType, FeatureFactorId>,
                  SerializablePair<FactorType,
                                   SerializableFactorType,
                                   FeatureFactorId,
                                   SerializableFeatureFactorId>>>(
              data_.visual_factors_by_feature_);
    fs << kPoseFactorsLabel
       << SerializableMap<FeatureFactorId,
                          SerializableFeatureFactorId,
                          RelPoseFactor,
                          SerializableRelPoseFactor>(data_.pose_factors_);
    fs << kFactorsLabel
       << SerializableMap<FeatureFactorId,
                          SerializableFeatureFactorId,
                          VisualFeatureFactorType,
                          SerializableVisualFeatureFactorType>(data_.factors_);
    fs << kLastObservedFrameByFeatureLabel
       << SerializableMap<FeatureId,
                          SerializableFeatureId,
                          FrameId,
                          SerializableFrameId>(
              data_.last_observed_frame_by_feature_);
    fs << kFirstObservedFrameByFeatureLabel
       << SerializableMap<FeatureId,
                          SerializableFeatureId,
                          FrameId,
                          SerializableFrameId>(
              data_.first_observed_frame_by_feature_);
    fs << "}";
  }

  virtual void read(const cv::FileNode &node) override {
    SerializableMap<CameraId,
                    SerializableCameraId,
                    CameraExtrinsics<double>,
                    SerializableCameraExtrinsics<double>>
        ser_camera_extrinsics_by_camera;
    node[kCameraExtrinsicsByCameraLabel] >> ser_camera_extrinsics_by_camera;
    data_.camera_extrinsics_by_camera_ =
        ser_camera_extrinsics_by_camera.getEntry();
    SerializableMap<CameraId,
                    SerializableCameraId,
                    CameraIntrinsicsMat<double>,
                    SerializableCameraIntrinsicsMat<double>>
        ser_camera_intrinsics_by_camera;
    node[kCameraIntrinsicsByCameraLabel] >> ser_camera_intrinsics_by_camera;
    data_.camera_intrinsics_by_camera_ =
        ser_camera_intrinsics_by_camera.getEntry();
    SerializableFactorType ser_visual_factor_type;
    node[kVisualFactorTypeLabel] >> ser_visual_factor_type;
    data_.visual_factor_type_ = ser_visual_factor_type.getEntry();
    SerializableFrameId ser_min_frame_id;
    node[kMinFrameIdLabel] >> ser_min_frame_id;
    data_.min_frame_id_ = ser_min_frame_id.getEntry();
    SerializableFrameId ser_max_frame_id;
    node[kMaxFrameIdLabel] >> ser_max_frame_id;
    data_.max_frame_id_ = ser_max_frame_id.getEntry();
    SerializableFeatureFactorId ser_max_feature_factor_id;
    node[kMaxFeatureFactorIdLabel] >> ser_max_feature_factor_id;
    data_.max_feature_factor_id_ = ser_max_feature_factor_id.getEntry();
    SerializableFeatureFactorId ser_max_pose_factor_id;
    node[kMaxPoseFactorIdLabel] >> ser_max_pose_factor_id;
    data_.max_pose_factor_id_ = ser_max_pose_factor_id.getEntry();
    SerializableMap<FrameId,
                    SerializableFrameId,
                    RawPose3d<double>,
                    SerializableRawPose3d<double>>
        ser_robot_poses;
    node[kRobotPosesLabel] >> ser_robot_poses;
    data_.robot_poses_ = ser_robot_poses.getEntry();

    SerializableMap<
        FrameId,
        SerializableFrameId,
        util::BoostHashSet<std::pair<FactorType, FeatureFactorId>>,
        SerializableBoostHashSet<std::pair<FactorType, FeatureFactorId>,
                                 SerializablePair<FactorType,
                                                  SerializableFactorType,
                                                  FeatureFactorId,
                                                  SerializableFeatureFactorId>>>
        ser_pose_factors_by_frame;
    node[kPoseFactorsByFrameLabel] >> ser_pose_factors_by_frame;
    data_.pose_factors_by_frame_ = ser_pose_factors_by_frame.getEntry();
    SerializableMap<
        FrameId,
        SerializableFrameId,
        std::vector<std::pair<FactorType, FeatureFactorId>>,
        SerializableVector<std::pair<FactorType, FeatureFactorId>,
                           SerializablePair<FactorType,
                                            SerializableFactorType,
                                            FeatureFactorId,
                                            SerializableFeatureFactorId>>>
        ser_visual_feature_factors_by_frame;
    node[kVisualFeatureFactorsByFrameLabel] >>
        ser_visual_feature_factors_by_frame;
    data_.visual_feature_factors_by_frame_ =
        ser_visual_feature_factors_by_frame.getEntry();

    SerializableMap<
        FeatureId,
        SerializableFeatureId,
        util::BoostHashSet<std::pair<FactorType, FeatureFactorId>>,
        SerializableBoostHashSet<std::pair<FactorType, FeatureFactorId>,
                                 SerializablePair<FactorType,
                                                  SerializableFactorType,
                                                  FeatureFactorId,
                                                  SerializableFeatureFactorId>>>
        ser_visual_factors_by_feature;
    node[kVisualFactorsByFeatureLabel] >> ser_visual_factors_by_feature;
    data_.visual_factors_by_feature_ = ser_visual_factors_by_feature.getEntry();
    SerializableMap<FeatureFactorId,
                    SerializableFeatureFactorId,
                    RelPoseFactor,
                    SerializableRelPoseFactor>
        ser_pose_factors;
    node[kPoseFactorsLabel] >> ser_pose_factors;
    data_.pose_factors_ = ser_pose_factors.getEntry();
    SerializableMap<FeatureFactorId,
                    SerializableFeatureFactorId,
                    VisualFeatureFactorType,
                    SerializableVisualFeatureFactorType>
        ser_factors;
    node[kFactorsLabel] >> ser_factors;
    data_.factors_ = ser_factors.getEntry();
    SerializableMap<FeatureId,
                    SerializableFeatureId,
                    FrameId,
                    SerializableFrameId>
        ser_last_observed_frame_by_feature;
    node[kLastObservedFrameByFeatureLabel] >>
        ser_last_observed_frame_by_feature;
    data_.last_observed_frame_by_feature_ =
        ser_last_observed_frame_by_feature.getEntry();
    SerializableMap<FeatureId,
                    SerializableFeatureId,
                    FrameId,
                    SerializableFrameId>
        ser_first_observed_frame_by_feature;
    node[kFirstObservedFrameByFeatureLabel] >>
        ser_first_observed_frame_by_feature;
    data_.first_observed_frame_by_feature_ =
        ser_first_observed_frame_by_feature.getEntry();
  }

 protected:
  using FileStorageSerializable<
      LowLevelFeaturePoseGraphState<VisualFeatureFactorType>>::data_;

 private:
  inline static const std::string kCameraExtrinsicsByCameraLabel =
      "camera_extrinsics_by_camera";
  inline static const std::string kCameraIntrinsicsByCameraLabel =
      "camera_intrinsics_by_camera";
  inline static const std::string kVisualFactorTypeLabel = "visual_factor_type";
  inline static const std::string kMinFrameIdLabel = "min_frame_id";
  inline static const std::string kMaxFrameIdLabel = "max_frame_id";
  inline static const std::string kMaxFeatureFactorIdLabel =
      "max_feature_factor_id";
  inline static const std::string kMaxPoseFactorIdLabel = "max_pose_factor_id";
  inline static const std::string kRobotPosesLabel = "robot_poses";
  inline static const std::string kPoseFactorsByFrameLabel =
      "pose_factors_by_frame";
  inline static const std::string kVisualFeatureFactorsByFrameLabel =
      "visual_feature_factors_by_frame";
  inline static const std::string kVisualFactorsByFeatureLabel =
      "visual_factors_by_feature";
  inline static const std::string kPoseFactorsLabel = "pose_factors";
  inline static const std::string kFactorsLabel = "factors";
  inline static const std::string kLastObservedFrameByFeatureLabel =
      "last_observed_frame_by_feature";
  inline static const std::string kFirstObservedFrameByFeatureLabel =
      "first_observed_frame_by_feature";
};

template <typename VisualFeatureFactorType,
          typename SerializableVisualFeatureFactorType>
static void write(cv::FileStorage &fs,
                  const std::string &,
                  const SerializableLowLevelFeaturePoseGraphState<
                      VisualFeatureFactorType,
                      SerializableVisualFeatureFactorType> &data) {
  data.write(fs);
}

template <typename VisualFeatureFactorType,
          typename SerializableVisualFeatureFactorType>
static void read(const cv::FileNode &node,
                 SerializableLowLevelFeaturePoseGraphState<
                     VisualFeatureFactorType,
                     SerializableVisualFeatureFactorType> &data,
                 const SerializableLowLevelFeaturePoseGraphState<
                     VisualFeatureFactorType,
                     SerializableVisualFeatureFactorType> &default_data =
                     SerializableLowLevelFeaturePoseGraphState<
                         VisualFeatureFactorType,
                         SerializableVisualFeatureFactorType>()) {
  if (node.empty()) {
    data = default_data;
  } else {
    data.read(node);
  }
}

class SerializableReprojectionErrorFactor
    : public FileStorageSerializable<ReprojectionErrorFactor> {
 public:
  SerializableReprojectionErrorFactor()
      : FileStorageSerializable<ReprojectionErrorFactor>() {}
  SerializableReprojectionErrorFactor(const ReprojectionErrorFactor &data)
      : FileStorageSerializable<ReprojectionErrorFactor>(data) {}

  virtual void write(cv::FileStorage &fs) const override {
    fs << "{";
    fs << kFrameIdLabel << SerializableFrameId(data_.frame_id_);
    fs << kFeatureIdLabel << SerializableFeatureId(data_.feature_id_);
    fs << kCameraIdLabel << SerializableCameraId(data_.camera_id_);
    fs << kFeaturePosLabel << SerializablePixelCoord(data_.feature_pos_);
    fs << kReprojectionErrorStdDevLabel << data_.reprojection_error_std_dev_;
    fs << "}";
  }

  virtual void read(const cv::FileNode &node) override {
    SerializableFrameId ser_frame_id;
    node[kFrameIdLabel] >> ser_frame_id;
    data_.frame_id_ = ser_frame_id.getEntry();
    SerializableFeatureId ser_feature_id;
    node[kFeatureIdLabel] >> ser_feature_id;
    data_.feature_id_ = ser_feature_id.getEntry();
    SerializableCameraId ser_camera_id;
    node[kCameraIdLabel] >> ser_camera_id;
    data_.camera_id_ = ser_camera_id.getEntry();
    PixelCoord<double> feature_pos_;
    SerializablePixelCoord ser_pixel_coord;
    node[kFeaturePosLabel] >> ser_pixel_coord;
    data_.feature_pos_ = ser_pixel_coord.getEntry();
    data_.reprojection_error_std_dev_ = node[kReprojectionErrorStdDevLabel];
  }

 protected:
  using FileStorageSerializable<ReprojectionErrorFactor>::data_;

 private:
  inline static const std::string kFrameIdLabel = "frame_id";
  inline static const std::string kFeatureIdLabel = "feature_id";
  inline static const std::string kCameraIdLabel = "camera_id";
  inline static const std::string kFeaturePosLabel = "feature_pos";
  inline static const std::string kReprojectionErrorStdDevLabel =
      "reprojection_error_std_dev";
};

static void write(cv::FileStorage &fs,
                  const std::string &,
                  const SerializableReprojectionErrorFactor &data) {
  data.write(fs);
}

static void read(const cv::FileNode &node,
                 SerializableReprojectionErrorFactor &data,
                 const SerializableReprojectionErrorFactor &default_data =
                     SerializableReprojectionErrorFactor()) {
  if (node.empty()) {
    data = default_data;
  } else {
    data.read(node);
  }
}

class SerializableReprojectionLowLevelFeaturePoseGraphState
    : public FileStorageSerializable<
          ReprojectionLowLevelFeaturePoseGraphState> {
 public:
  SerializableReprojectionLowLevelFeaturePoseGraphState()
      : FileStorageSerializable<ReprojectionLowLevelFeaturePoseGraphState>() {}
  SerializableReprojectionLowLevelFeaturePoseGraphState(
      const ReprojectionLowLevelFeaturePoseGraphState &data)
      : FileStorageSerializable<ReprojectionLowLevelFeaturePoseGraphState>(
            data) {}

  virtual void write(cv::FileStorage &fs) const override {
    fs << "{";
    fs << kLowLevelPgStateLabel
       << SerializableLowLevelFeaturePoseGraphState<
              ReprojectionErrorFactor,
              SerializableReprojectionErrorFactor>(data_.low_level_pg_state_);
    fs << kMinFeatureIdLabel << SerializableFeatureId(data_.min_feature_id_);
    fs << kMaxFeatureIdLabel << SerializableFeatureId(data_.max_feature_id_);
    fs << kFeaturePositionsLabel
       << SerializableMap<FeatureId,
                          SerializableFeatureId,
                          Position3d<double>,
                          SerializablePosition3d<double>>(
              data_.feature_positions_);
    fs << "}";
  }

  virtual void read(const cv::FileNode &node) override {
    SerializableLowLevelFeaturePoseGraphState<
        ReprojectionErrorFactor,
        SerializableReprojectionErrorFactor>
        ser_low_level_pg_state;
    node[kLowLevelPgStateLabel] >> ser_low_level_pg_state;
    data_.low_level_pg_state_ = ser_low_level_pg_state.getEntry();
    SerializableFeatureId ser_min_feature_id;
    node[kMinFeatureIdLabel] >> ser_min_feature_id;
    data_.min_feature_id_ = ser_min_feature_id.getEntry();
    SerializableFeatureId ser_max_feature_id;
    node[kMaxFeatureIdLabel] >> ser_max_feature_id;
    data_.max_feature_id_ = ser_max_feature_id.getEntry();
    SerializableMap<FeatureId,
                    SerializableFeatureId,
                    Position3d<double>,
                    SerializablePosition3d<double>>
        ser_feature_positions;
    node[kFeaturePositionsLabel] >> ser_feature_positions;
    data_.feature_positions_ = ser_feature_positions.getEntry();
  }

 protected:
  using FileStorageSerializable<
      ReprojectionLowLevelFeaturePoseGraphState>::data_;

 private:
  inline static const std::string kLowLevelPgStateLabel = "low_level_pg_state";
  inline static const std::string kMinFeatureIdLabel = "min_feature_id";
  inline static const std::string kMaxFeatureIdLabel = "max_feature_id";
  inline static const std::string kFeaturePositionsLabel = "feature_positions";
};

static void write(
    cv::FileStorage &fs,
    const std::string &,
    const SerializableReprojectionLowLevelFeaturePoseGraphState &data) {
  data.write(fs);
}

static void read(
    const cv::FileNode &node,
    SerializableReprojectionLowLevelFeaturePoseGraphState &data,
    const SerializableReprojectionLowLevelFeaturePoseGraphState &default_data =
        SerializableReprojectionLowLevelFeaturePoseGraphState()) {
  if (node.empty()) {
    data = default_data;
  } else {
    data.read(node);
  }
}

class SerializableObjOnlyPoseGraphState
    : public FileStorageSerializable<ObjOnlyPoseGraphState> {
 public:
  SerializableObjOnlyPoseGraphState()
      : FileStorageSerializable<ObjOnlyPoseGraphState>() {}
  SerializableObjOnlyPoseGraphState(const ObjOnlyPoseGraphState &data)
      : FileStorageSerializable<ObjOnlyPoseGraphState>(data) {}

  virtual void write(cv::FileStorage &fs) const override {
    fs << "{";
    fs << kMeanAndCovBySemanticClassLabel
       << SerializableMap<std::string,
                          SerializableString,
                          std::pair<ObjectDim<double>, Covariance<double, 3>>,
                          SerializablePair<ObjectDim<double>,
                                           SerializableObjectDim<double>,
                                           Covariance<double, 3>,
                                           SerializableCovariance<double, 3>>>(
              data_.mean_and_cov_by_semantic_class_);
    fs << kMinObjectIdLabel << SerializableObjectId(data_.min_object_id_);
    fs << kMaxObjectIdLabel << SerializableObjectId(data_.max_object_id_);
    fs << kEllipsoidEstimatesLabel
       << SerializableMap<ObjectId,
                          SerializableObjectId,
                          RawEllipsoid<double>,
                          SerializableRawEllipsoid<double>>(
              data_.ellipsoid_estimates_);
    fs << kSemanticClassForObjectLabel
       << SerializableMap<ObjectId,
                          SerializableObjectId,
                          std::string,
                          SerializableString>(data_.semantic_class_for_object_);
    fs << kLastObservedFrameByObjectLabel
       << SerializableMap<ObjectId,
                          SerializableObjectId,
                          FrameId,
                          SerializableFrameId>(
              data_.last_observed_frame_by_object_);
    fs << kFirstObservedFrameByObjectLabel
       << SerializableMap<ObjectId,
                          SerializableObjectId,
                          FrameId,
                          SerializableFrameId>(
              data_.first_observed_frame_by_object_);
    fs << kMinObjectObservationFactorLabel
       << SerializableFeatureFactorId(data_.min_object_observation_factor_);
    fs << kMaxObjectObservationFactorLabel
       << SerializableFeatureFactorId(data_.max_object_observation_factor_);
    fs << kMinObjSpecificFactorLabel
       << SerializableFeatureFactorId(data_.min_obj_specific_factor_);
    fs << kMaxObjSpecificFactorLabel
       << SerializableFeatureFactorId(data_.max_obj_specific_factor_);
    fs << kLongTermMapObjectIdsLabel
       << SerializableUnorderedSet<ObjectId, SerializableObjectId>(
              data_.long_term_map_object_ids_);
    fs << kObjectObservationFactorsLabel
       << SerializableMap<FeatureFactorId,
                          SerializableFeatureFactorId,
                          ObjectObservationFactor,
                          SerializableObjectObservationFactor>(
              data_.object_observation_factors_);
    fs << kShapeDimPriorFactorsLabel
       << SerializableMap<FeatureFactorId,
                          SerializableFeatureFactorId,
                          ShapeDimPriorFactor,
                          SerializableShapeDimPriorFactor>(
              data_.shape_dim_prior_factors_);
    fs << kObservationFactorsByFrameLabel
       << SerializableMap<
              FrameId,
              SerializableFrameId,
              util::BoostHashSet<std::pair<FactorType, FeatureFactorId>>,
              SerializableBoostHashSet<
                  std::pair<FactorType, FeatureFactorId>,
                  SerializablePair<FactorType,
                                   SerializableFactorType,
                                   FeatureFactorId,
                                   SerializableFeatureFactorId>>>(
              data_.observation_factors_by_frame_);
    fs << kObservationFactorsByObjectLabel
       << SerializableMap<
              ObjectId,
              SerializableObjectId,
              util::BoostHashSet<std::pair<FactorType, FeatureFactorId>>,
              SerializableBoostHashSet<
                  std::pair<FactorType, FeatureFactorId>,
                  SerializablePair<FactorType,
                                   SerializableFactorType,
                                   FeatureFactorId,
                                   SerializableFeatureFactorId>>>(
              data_.observation_factors_by_object_);
    fs << kObjectOnlyFactorsByObjectLabel
       << SerializableMap<
              ObjectId,
              SerializableObjectId,
              util::BoostHashSet<std::pair<FactorType, FeatureFactorId>>,
              SerializableBoostHashSet<
                  std::pair<FactorType, FeatureFactorId>,
                  SerializablePair<FactorType,
                                   SerializableFactorType,
                                   FeatureFactorId,
                                   SerializableFeatureFactorId>>>(
              data_.object_only_factors_by_object_);
    fs << "}";
  }

  virtual void read(const cv::FileNode &node) override {
    SerializableMap<std::string,
                    SerializableString,
                    std::pair<ObjectDim<double>, Covariance<double, 3>>,
                    SerializablePair<ObjectDim<double>,
                                     SerializableObjectDim<double>,
                                     Covariance<double, 3>,
                                     SerializableCovariance<double, 3>>>
        ser_mean_and_cov_by_semantic_class;
    node[kMeanAndCovBySemanticClassLabel] >> ser_mean_and_cov_by_semantic_class;
    data_.mean_and_cov_by_semantic_class_ =
        ser_mean_and_cov_by_semantic_class.getEntry();
    SerializableObjectId ser_min_object_id;
    node[kMinObjectIdLabel] >> ser_min_object_id;
    data_.min_object_id_ = ser_min_object_id.getEntry();
    SerializableObjectId ser_max_object_id;
    node[kMaxObjectIdLabel] >> ser_max_object_id;
    data_.max_object_id_ = ser_max_object_id.getEntry();
    SerializableMap<ObjectId,
                    SerializableObjectId,
                    RawEllipsoid<double>,
                    SerializableRawEllipsoid<double>>
        ser_ellipsoid_estimates;
    node[kEllipsoidEstimatesLabel] >> ser_ellipsoid_estimates;
    data_.ellipsoid_estimates_ = ser_ellipsoid_estimates.getEntry();
    SerializableMap<ObjectId,
                    SerializableObjectId,
                    std::string,
                    SerializableString>
        ser_semantic_class_for_object;
    node[kSemanticClassForObjectLabel] >> ser_semantic_class_for_object;
    data_.semantic_class_for_object_ = ser_semantic_class_for_object.getEntry();
    SerializableMap<ObjectId,
                    SerializableObjectId,
                    FrameId,
                    SerializableFrameId>
        ser_last_observed_frame_by_object;
    node[kLastObservedFrameByObjectLabel] >> ser_last_observed_frame_by_object;
    data_.last_observed_frame_by_object_ =
        ser_last_observed_frame_by_object.getEntry();
    SerializableMap<ObjectId,
                    SerializableObjectId,
                    FrameId,
                    SerializableFrameId>
        ser_first_observed_frame_by_object;
    node[kFirstObservedFrameByObjectLabel] >>
        ser_first_observed_frame_by_object;
    data_.first_observed_frame_by_object_ =
        ser_first_observed_frame_by_object.getEntry();
    SerializableFeatureFactorId ser_min_object_observation_factor;
    node[kMinObjectObservationFactorLabel] >> ser_min_object_observation_factor;
    data_.min_object_observation_factor_ =
        ser_min_object_observation_factor.getEntry();
    SerializableFeatureFactorId ser_max_object_observation_factor;
    node[kMaxObjectObservationFactorLabel] >> ser_max_object_observation_factor;
    data_.max_object_observation_factor_ =
        ser_max_object_observation_factor.getEntry();
    SerializableFeatureFactorId ser_min_obj_specific_factor;
    node[kMinObjSpecificFactorLabel] >> ser_min_obj_specific_factor;
    data_.min_obj_specific_factor_ = ser_min_obj_specific_factor.getEntry();
    SerializableFeatureFactorId ser_max_obj_specific_factor;
    node[kMaxObjSpecificFactorLabel] >> ser_max_obj_specific_factor;
    data_.max_obj_specific_factor_ = ser_max_obj_specific_factor.getEntry();
    SerializableUnorderedSet<ObjectId, SerializableObjectId>
        ser_long_term_map_object_ids;
    node[kLongTermMapObjectIdsLabel] >> ser_long_term_map_object_ids;
    data_.long_term_map_object_ids_ = ser_long_term_map_object_ids.getEntry();
    SerializableMap<FeatureFactorId,
                    SerializableFeatureFactorId,
                    ObjectObservationFactor,
                    SerializableObjectObservationFactor>
        ser_object_observation_factors;
    node[kObjectObservationFactorsLabel] >> ser_object_observation_factors;
    data_.object_observation_factors_ =
        ser_object_observation_factors.getEntry();
    SerializableMap<FeatureFactorId,
                    SerializableFeatureFactorId,
                    ShapeDimPriorFactor,
                    SerializableShapeDimPriorFactor>
        ser_shape_dim_prior_factors;
    node[kShapeDimPriorFactorsLabel] >> ser_shape_dim_prior_factors;
    data_.shape_dim_prior_factors_ = ser_shape_dim_prior_factors.getEntry();
    SerializableMap<
        FrameId,
        SerializableFrameId,
        util::BoostHashSet<std::pair<FactorType, FeatureFactorId>>,
        SerializableBoostHashSet<std::pair<FactorType, FeatureFactorId>,
                                 SerializablePair<FactorType,
                                                  SerializableFactorType,
                                                  FeatureFactorId,
                                                  SerializableFeatureFactorId>>>
        ser_observation_factors_by_frame;
    node[kObservationFactorsByFrameLabel] >> ser_observation_factors_by_frame;
    data_.observation_factors_by_frame_ =
        ser_observation_factors_by_frame.getEntry();
    SerializableMap<
        ObjectId,
        SerializableObjectId,
        util::BoostHashSet<std::pair<FactorType, FeatureFactorId>>,
        SerializableBoostHashSet<std::pair<FactorType, FeatureFactorId>,
                                 SerializablePair<FactorType,
                                                  SerializableFactorType,
                                                  FeatureFactorId,
                                                  SerializableFeatureFactorId>>>
        ser_observation_factors_by_object;
    node[kObservationFactorsByObjectLabel] >> ser_observation_factors_by_object;
    data_.observation_factors_by_object_ =
        ser_observation_factors_by_object.getEntry();
    SerializableMap<
        ObjectId,
        SerializableObjectId,
        util::BoostHashSet<std::pair<FactorType, FeatureFactorId>>,
        SerializableBoostHashSet<std::pair<FactorType, FeatureFactorId>,
                                 SerializablePair<FactorType,
                                                  SerializableFactorType,
                                                  FeatureFactorId,
                                                  SerializableFeatureFactorId>>>
        ser_object_only_factors_by_object;
    node[kObjectOnlyFactorsByObjectLabel] >> ser_object_only_factors_by_object;
    data_.object_only_factors_by_object_ =
        ser_object_only_factors_by_object.getEntry();
  }

 protected:
  using FileStorageSerializable<ObjOnlyPoseGraphState>::data_;

 private:
  inline static const std::string kMeanAndCovBySemanticClassLabel =
      "mean_and_cov_by_semantic_class";
  inline static const std::string kMinObjectIdLabel = "min_object_id";
  inline static const std::string kMaxObjectIdLabel = "max_object_id";
  inline static const std::string kEllipsoidEstimatesLabel =
      "ellipsoid_estimates";
  inline static const std::string kSemanticClassForObjectLabel =
      "semantic_class_for_object";
  inline static const std::string kLastObservedFrameByObjectLabel =
      "last_observed_frame_by_object";
  inline static const std::string kFirstObservedFrameByObjectLabel =
      "first_observed_frame_by_object";
  inline static const std::string kMinObjectObservationFactorLabel =
      "min_object_observation_factor";
  inline static const std::string kMaxObjectObservationFactorLabel =
      "max_object_observation_factor";
  inline static const std::string kMinObjSpecificFactorLabel =
      "min_obj_specific_factor";
  inline static const std::string kMaxObjSpecificFactorLabel =
      "max_obj_specific_factor";
  inline static const std::string kLongTermMapObjectIdsLabel =
      "long_term_map_object_ids";
  inline static const std::string kObjectObservationFactorsLabel =
      "object_observation_factors";
  inline static const std::string kShapeDimPriorFactorsLabel =
      "shape_dim_prior_factors";
  inline static const std::string kObservationFactorsByFrameLabel =
      "observation_factors_by_frame";
  inline static const std::string kObservationFactorsByObjectLabel =
      "observation_factors_by_object";
  inline static const std::string kObjectOnlyFactorsByObjectLabel =
      "object_only_factors_by_object";
};

static void write(cv::FileStorage &fs,
                  const std::string &,
                  const SerializableObjOnlyPoseGraphState &data) {
  data.write(fs);
}

static void read(const cv::FileNode &node,
                 SerializableObjOnlyPoseGraphState &data,
                 const SerializableObjOnlyPoseGraphState &default_data =
                     SerializableObjOnlyPoseGraphState()) {
  if (node.empty()) {
    data = default_data;
  } else {
    data.read(node);
  }
}

class SerializableObjectAndReprojectionFeaturePoseGraphState
    : public FileStorageSerializable<
          ObjectAndReprojectionFeaturePoseGraphState> {
 public:
  SerializableObjectAndReprojectionFeaturePoseGraphState()
      : FileStorageSerializable<ObjectAndReprojectionFeaturePoseGraphState>() {}
  SerializableObjectAndReprojectionFeaturePoseGraphState(
      const ObjectAndReprojectionFeaturePoseGraphState &data)
      : FileStorageSerializable<ObjectAndReprojectionFeaturePoseGraphState>(
            data) {}

  virtual void write(cv::FileStorage &fs) const override {
    fs << "{";
    fs << kReprojectionLowLevelFeaturePoseGraphStateLabel
       << SerializableReprojectionLowLevelFeaturePoseGraphState(
              data_.reprojection_low_level_feature_pose_graph_state_);
    fs << kObjOnlyPoseGraphStateLabel
       << SerializableObjOnlyPoseGraphState(data_.obj_only_pose_graph_state_);
    fs << "}";
  }

  virtual void read(const cv::FileNode &node) override {
    SerializableReprojectionLowLevelFeaturePoseGraphState
        ser_reprojection_low_level_feature_pose_graph_state;
    node[kReprojectionLowLevelFeaturePoseGraphStateLabel] >>
        ser_reprojection_low_level_feature_pose_graph_state;
    data_.reprojection_low_level_feature_pose_graph_state_ =
        ser_reprojection_low_level_feature_pose_graph_state.getEntry();
    SerializableObjOnlyPoseGraphState ser_obj_only_pose_graph_state;
    node[kObjOnlyPoseGraphStateLabel] >> ser_obj_only_pose_graph_state;
    data_.obj_only_pose_graph_state_ = ser_obj_only_pose_graph_state.getEntry();
  }

 protected:
  using FileStorageSerializable<
      ObjectAndReprojectionFeaturePoseGraphState>::data_;

 private:
  inline static const std::string
      kReprojectionLowLevelFeaturePoseGraphStateLabel =
          "reprojection_low_level_feature_pose_graph_state";
  inline static const std::string kObjOnlyPoseGraphStateLabel =
      "obj_only_pose_graph_state_";
};

static void write(
    cv::FileStorage &fs,
    const std::string &,
    const SerializableObjectAndReprojectionFeaturePoseGraphState &data) {
  data.write(fs);
}

static void read(
    const cv::FileNode &node,
    SerializableObjectAndReprojectionFeaturePoseGraphState &data,
    const SerializableObjectAndReprojectionFeaturePoseGraphState &default_data =
        SerializableObjectAndReprojectionFeaturePoseGraphState()) {
  if (node.empty()) {
    data = default_data;
  } else {
    data.read(node);
  }
}

void outputPoseGraphStateToFile(
    const ObjectAndReprojectionFeaturePoseGraphState &pose_graph_state,
    const std::string &out_file) {
  cv::FileStorage pose_graph_state_fs(out_file, cv::FileStorage::WRITE);

  pose_graph_state_fs << kPoseGraphStateKey
                      << SerializableObjectAndReprojectionFeaturePoseGraphState(
                             pose_graph_state);
  pose_graph_state_fs.release();
}

void readPoseGraphStateFromFile(
    const std::string &in_file,
    ObjectAndReprojectionFeaturePoseGraphState &pose_graph_state) {
  if (!std::filesystem::exists(in_file)) {
    LOG(ERROR) << "Trying to read file " << in_file << " that does not exist";
    return;
  }
  cv::FileStorage pg_state_in_fs(in_file, cv::FileStorage::READ);
  SerializableObjectAndReprojectionFeaturePoseGraphState ser_pose_graph_state;

  pg_state_in_fs[kPoseGraphStateKey] >> ser_pose_graph_state;
  pg_state_in_fs.release();
  pose_graph_state = ser_pose_graph_state.getEntry();
}

void outputPoseGraphToFile(
    const std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph> &pose_graph,
    const std::string &out_file) {
  ObjectAndReprojectionFeaturePoseGraphState pose_graph_state;
  pose_graph->getState(pose_graph_state);
  outputPoseGraphStateToFile(pose_graph_state, out_file);
}
}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_OBJECT_AND_REPROJECTION_FEATURE_POSE_GRAPH_FILE_STORAGE_IO_H
