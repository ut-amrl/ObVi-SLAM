//
// Created by amanda on 2/10/23.
//

#ifndef UT_VSLAM_CONFIG_FILE_FILE_STORAGE_IO_H
#define UT_VSLAM_CONFIG_FILE_FILE_STORAGE_IO_H

#include <file_io/cv_file_storage/file_storage_io_utils.h>
#include <refactoring/configuration/full_ov_slam_config.h>

namespace vslam_types_refactor {

const std::string kConfigEntryKey = "config";

class SerializableFullOVSLAMConfig
    : public FileStorageSerializable<FullOVSLAMConfig> {
 public:
  SerializableFullOVSLAMConfig()
      : FileStorageSerializable<FullOVSLAMConfig>() {}
  SerializableFullOVSLAMConfig(const FullOVSLAMConfig &data)
      : FileStorageSerializable<FullOVSLAMConfig>(data) {}

  virtual void write(cv::FileStorage &fs) const override {
    fs << "{";
    fs << kConfigSchemaVersionLabel << data_.config_schema_version_;
    fs << kConfigVersionIdLabel << data_.config_version_id_;

    fs << kVisualFeatureParamsLabel << SerializableVisualFeatureParams(data_.visual_feature_params_);
    fs << kLocalBaSolverParamsLabel << SerializableOptimizationSolverParams(data_.local_ba_solver_params_);
    fs << kGlobalBaSolverParamsLabel << SerializableOptimizationSolverParams(data_.global_ba_solver_params_);
    fs << kFinalBaSolverParamsLabel << SerializableOptimizationSolverParams(data_.final_ba_solver_params_);
    fs << kLtmTunableParamsLabel << SerializableLongTermMapExtractionTunableParams(data_.ltm_tunable_params_);
    fs << kLtmSolverResidualParamsLabel << SerializableObjectVisualPoseGraphResidualParams(data_.ltm_solver_residual_params_);
    fs << kLtmSolverParamsLabel << SerializableOptimizationSolverParams(data_.ltm_solver_params_);
    fs << kShapeDimensionPriorsLabel << SerializableShapeDimensionPriors(data_.shape_dimension_priors_);
    fs << kCameraInfoLabel << SerializableCameraInfo(data_.camera_info_);
    fs << kBoundingBoxFrontEndParamsLabel << SerializableBoundingBoxFrontEndParams(data_.bounding_box_front_end_params_);
    fs << kBoundingBoxCovarianceGeneratorParamsLabel << SerializableBoundingBoxCovGenParams(data_.bounding_box_covariance_generator_params_);
    fs << kSlidingWindowParamsLabel << SerializableSlidingWindowParams(data_.sliding_window_params_);
    fs << kOptimizationFactorsEnabledParamsLabel << SerializableOptimizationFactorsEnabledParams(data_.optimization_factors_enabled_params_);
    fs << kObjectVisualPoseGraphResidualParamsLabel << SerializableObjectVisualPoseGraphResidualParams(data_.object_visual_pose_graph_residual_params_);
    fs << kLimitTrajEvalParamsLabel << SerializableLimitTrajectoryEvaluationParams(data_.limit_traj_eval_params_);
    fs << kSparsifierParamsLabel << SerializableSparsifierParams(data_.sparsifier_params_);
    fs << "}";
  }

  virtual void read(const cv::FileNode &node) override {
    node[]
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
  using FileStorageSerializable<FullOVSLAMConfig>::data_;

 private:
  inline static const std::string kConfigSchemaVersionLabel =
      "config_schema_version";
  inline static const std::string kConfigVersionIdLabel = "config_version_id";
  inline static const std::string kVisualFeatureParamsLabel =
      "visual_feature_params";
  inline static const std::string kLocalBaSolverParamsLabel =
      "local_ba_solver_params";
  inline static const std::string kGlobalBaSolverParamsLabel =
      "global_ba_solver_params";
  inline static const std::string kFinalBaSolverParamsLabel =
      "final_ba_solver_params";
  inline static const std::string kLtmTunableParamsLabel = "ltm_tunable_params";
  inline static const std::string kLtmSolverResidualParamsLabel =
      "ltm_solver_residual_params";
  inline static const std::string kLtmSolverParamsLabel = "ltm_solver_params";
  inline static const std::string kShapeDimensionPriorsLabel =
      "shape_dimension_priors";
  inline static const std::string kCameraInfoLabel = "camera_info";
  inline static const std::string kBoundingBoxFrontEndParamsLabel =
      "bounding_box_front_end_params";
  inline static const std::string kBoundingBoxCovarianceGeneratorParamsLabel =
      "bounding_box_covariance_generator_params";
  inline static const std::string kSlidingWindowParamsLabel =
      "sliding_window_params";
  inline static const std::string kOptimizationFactorsEnabledParamsLabel =
      "optimization_factors_enabled_params";
  inline static const std::string kObjectVisualPoseGraphResidualParamsLabel =
      "object_visual_pose_graph_residual_params";
  inline static const std::string kLimitTrajEvalParamsLabel =
      "limit_traj_eval_params";
  inline static const std::string kSparsifierParamsLabel = "sparsifier_params";
};

static void write(cv::FileStorage &fs,
                  const std::string &,
                  const SerializableFullOVSLAMConfig &data) {
  data.write(fs);
}

static void read(const cv::FileNode &node,
                 SerializableFullOVSLAMConfig &data,
                 const SerializableFullOVSLAMConfig &default_data =
                     SerializableFullOVSLAMConfig()) {
  if (node.empty()) {
    data = default_data;
  } else {
    data.read(node);
  }
}

void writeConfiguration(const std::string &config_file_name,
                        const FullOVSLAMConfig &configuration) {
  cv::FileStorage config_file_out(config_file_name, cv::FileStorage::WRITE);

  config_file_out << kConfigEntryKey
                  << SerializableFullOVSLAMConfig(configuration);
  config_file_out.release();
}

void readConfiguration(const std::string &config_file_name,
                       FullOVSLAMConfig &configuration) {
  cv::FileStorage config_file_in(config_file_name, cv::FileStorage::READ);
  SerializableFullOVSLAMConfig serializable_config;
  config_file_in[kConfigEntryKey] >> serializable_config;
  config_file_in.release();
  configuration = serializable_config.getEntry();

  if (configuration.config_schema_version_ != kCurrentConfigSchemaVersion) {
    LOG(ERROR) << "Configuration does not match current schema version. Please "
                  "update your config file to the latest format and try again.";
    exit(1);
  }
}

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_CONFIG_FILE_FILE_STORAGE_IO_H
