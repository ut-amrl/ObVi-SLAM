//
// Created by amanda on 2/10/23.
//

#ifndef UT_VSLAM_CONFIG_FILE_STORAGE_IO_H
#define UT_VSLAM_CONFIG_FILE_STORAGE_IO_H

#include <file_io/cv_file_storage/file_storage_io_utils.h>
#include <file_io/cv_file_storage/vslam_basic_types_file_storage_io.h>
#include <refactoring/configuration/full_ov_slam_config.h>

namespace vslam_types_refactor {

const std::string kConfigEntryKey = "config";

class SerializableOptimizationFactorsEnabledParams
    : public FileStorageSerializable<
          pose_graph_optimizer::OptimizationFactorsEnabledParams> {
 public:
  SerializableOptimizationFactorsEnabledParams()
      : FileStorageSerializable<
            pose_graph_optimizer::OptimizationFactorsEnabledParams>() {}
  SerializableOptimizationFactorsEnabledParams(
      const pose_graph_optimizer::OptimizationFactorsEnabledParams &data)
      : FileStorageSerializable<
            pose_graph_optimizer::OptimizationFactorsEnabledParams>(data) {}

  virtual void write(cv::FileStorage &fs) const override {
    fs << "{";
    int include_object_factors_int = data_.include_object_factors_ ? 1 : 0;
    fs << kIncludeObjectFactorsLabel << include_object_factors_int;

    int include_visual_factors_int = data_.include_visual_factors_ ? 1 : 0;
    fs << kIncludeVisualFactorsLabel << include_visual_factors_int;

    int fix_poses_int = data_.fix_poses_ ? 1 : 0;
    fs << kFixPosesLabel << fix_poses_int;

    int fix_objects_int = data_.fix_objects_ ? 1 : 0;
    fs << kFixObjectsLabel << fix_objects_int;

    int fix_visual_features_int = data_.fix_visual_features_ ? 1 : 0;
    fs << kFixVisualFeaturesLabel << fix_visual_features_int;

    int fix_ltm_objects_int = data_.fix_ltm_objects_ ? 1 : 0;
    fs << kFixLtmObjectsLabel << fix_ltm_objects_int;

    int use_pom_int = data_.use_pom_ ? 1 : 0;
    fs << kUsePomLabel << use_pom_int;

    fs << kPosesPriorToWindowToKeepConstantLabel
       << (int)data_.poses_prior_to_window_to_keep_constant_;
    fs << kMinObjectObservationsLabel << (int)data_.min_object_observations_;
    fs << kMinLowLevelFeatureObservationsLabel
       << (int)data_.min_low_level_feature_observations_;
    fs << "}";
  }

  virtual void read(const cv::FileNode &node) override {
    int include_object_factors_int = node[kIncludeObjectFactorsLabel];
    data_.include_object_factors_ = include_object_factors_int != 0;

    int include_visual_factors_int = node[kIncludeVisualFactorsLabel];
    data_.include_visual_factors_ = include_visual_factors_int != 0;

    int fix_poses_int = node[kFixPosesLabel];
    data_.fix_poses_ = fix_poses_int != 0;

    int fix_objects_int = node[kFixObjectsLabel];
    data_.fix_objects_ = fix_objects_int != 0;

    int fix_visual_features_int = node[kFixVisualFeaturesLabel];
    data_.fix_visual_features_ = fix_visual_features_int != 0;

    int fix_ltm_objects_int = node[kFixLtmObjectsLabel];
    data_.fix_ltm_objects_ = fix_ltm_objects_int != 0;

    int use_pom_int = node[kUsePomLabel];
    data_.use_pom_ = use_pom_int != 0;

    data_.poses_prior_to_window_to_keep_constant_ =
        (int)node[kPosesPriorToWindowToKeepConstantLabel];
    data_.min_object_observations_ = (int)node[kMinObjectObservationsLabel];
    data_.min_low_level_feature_observations_ =
        (int)node[kMinLowLevelFeatureObservationsLabel];
  }

 protected:
  using FileStorageSerializable<
      pose_graph_optimizer::OptimizationFactorsEnabledParams>::data_;

 private:
  inline static const std::string kIncludeObjectFactorsLabel =
      "include_object_factors";
  inline static const std::string kIncludeVisualFactorsLabel =
      "include_visual_factors";
  inline static const std::string kFixPosesLabel = "fix_poses";
  inline static const std::string kFixObjectsLabel = "fix_objects";
  inline static const std::string kFixVisualFeaturesLabel =
      "fix_visual_features";
  inline static const std::string kFixLtmObjectsLabel = "fix_ltm_objects";
  inline static const std::string kUsePomLabel = "use_pom";
  inline static const std::string kPosesPriorToWindowToKeepConstantLabel =
      "poses_prior_to_window_to_keep_constant";
  inline static const std::string kMinObjectObservationsLabel =
      "min_object_observations";
  inline static const std::string kMinLowLevelFeatureObservationsLabel =
      "min_low_level_feature_observations";
};

static void write(cv::FileStorage &fs,
                  const std::string &,
                  const SerializableOptimizationFactorsEnabledParams &data) {
  data.write(fs);
}

static void read(
    const cv::FileNode &node,
    SerializableOptimizationFactorsEnabledParams &data,
    const SerializableOptimizationFactorsEnabledParams &default_data =
        SerializableOptimizationFactorsEnabledParams()) {
  if (node.empty()) {
    data = default_data;
  } else {
    data.read(node);
  }
}

class SerializableSlidingWindowParams
    : public FileStorageSerializable<SlidingWindowParams> {
 public:
  SerializableSlidingWindowParams()
      : FileStorageSerializable<SlidingWindowParams>() {}
  SerializableSlidingWindowParams(const SlidingWindowParams &data)
      : FileStorageSerializable<SlidingWindowParams>(data) {}

  virtual void write(cv::FileStorage &fs) const override {
    fs << "{";
    fs << kGlobalBaFrequencyLabel
       << SerializableFrameId(data_.global_ba_frequency_);
    fs << kLocalBaWindowSizeLabel
       << SerializableFrameId(data_.local_ba_window_size_);
    fs << "}";
  }

  virtual void read(const cv::FileNode &node) override {
    SerializableFrameId ser_global_ba_frequency;
    node[kGlobalBaFrequencyLabel] >> ser_global_ba_frequency;
    data_.global_ba_frequency_ = ser_global_ba_frequency.getEntry();
    SerializableFrameId ser_local_ba_window_size;
    node[kLocalBaWindowSizeLabel] >> ser_local_ba_window_size;
    data_.local_ba_window_size_ = ser_local_ba_window_size.getEntry();
  }

 protected:
  using FileStorageSerializable<SlidingWindowParams>::data_;

 private:
  inline static const std::string kGlobalBaFrequencyLabel =
      "global_ba_frequency";
  inline static const std::string kLocalBaWindowSizeLabel =
      "local_ba_window_size";
};

static void write(cv::FileStorage &fs,
                  const std::string &,
                  const SerializableSlidingWindowParams &data) {
  data.write(fs);
}

static void read(const cv::FileNode &node,
                 SerializableSlidingWindowParams &data,
                 const SerializableSlidingWindowParams &default_data =
                     SerializableSlidingWindowParams()) {
  if (node.empty()) {
    data = default_data;
  } else {
    data.read(node);
  }
}

class SerializableBoundingBoxCovGenParams
    : public FileStorageSerializable<BoundingBoxCovGenParams> {
 public:
  SerializableBoundingBoxCovGenParams()
      : FileStorageSerializable<BoundingBoxCovGenParams>() {}
  SerializableBoundingBoxCovGenParams(const BoundingBoxCovGenParams &data)
      : FileStorageSerializable<BoundingBoxCovGenParams>(data) {}

  virtual void write(cv::FileStorage &fs) const override {
    fs << "{";
    fs << kBoundingBoxCovLabel
       << SerializableEigenMat<double, 4, 4>(data_.bounding_box_cov_);
    fs << kNearEdgeThreshold << data_.near_edge_threshold_;
    fs << kImageBoundaryVariance << data_.image_boundary_variance_;
    fs << "}";
  }

  virtual void read(const cv::FileNode &node) override {
    SerializableEigenMat<double, 4, 4> ser_bounding_box_cov;
    node[kBoundingBoxCovLabel] >> ser_bounding_box_cov;
    data_.bounding_box_cov_ = ser_bounding_box_cov.getEntry();

    data_.near_edge_threshold_ = node[kNearEdgeThreshold];
    data_.image_boundary_variance_ = node[kImageBoundaryVariance];
  }

 protected:
  using FileStorageSerializable<BoundingBoxCovGenParams>::data_;

 private:
  inline static const std::string kBoundingBoxCovLabel = "bounding_box_cov";
  inline static const std::string kNearEdgeThreshold = "near_edge_threshold";
  inline static const std::string kImageBoundaryVariance =
      "image_boundary_variance";
};

static void write(cv::FileStorage &fs,
                  const std::string &,
                  const SerializableBoundingBoxCovGenParams &data) {
  data.write(fs);
}

static void read(const cv::FileNode &node,
                 SerializableBoundingBoxCovGenParams &data,
                 const SerializableBoundingBoxCovGenParams &default_data =
                     SerializableBoundingBoxCovGenParams()) {
  if (node.empty()) {
    data = default_data;
  } else {
    data.read(node);
  }
}

class SerializableObjectResidualParams
    : public FileStorageSerializable<
          pose_graph_optimization::ObjectResidualParams> {
 public:
  SerializableObjectResidualParams()
      : FileStorageSerializable<
            pose_graph_optimization::ObjectResidualParams>() {}
  SerializableObjectResidualParams(
      const pose_graph_optimization::ObjectResidualParams &data)
      : FileStorageSerializable<pose_graph_optimization::ObjectResidualParams>(
            data) {}

  virtual void write(cv::FileStorage &fs) const override {
    fs << "{";
    fs << kObjectObservationHuberLossParamLabel
       << data_.object_observation_huber_loss_param_;
    fs << kShapeDimPriorFactorHuberLossParamLabel
       << data_.shape_dim_prior_factor_huber_loss_param_;
    fs << kInvalidEllipsoidErrorValLabel << data_.invalid_ellipsoid_error_val_;
    fs << "}";
  }

  virtual void read(const cv::FileNode &node) override {
    data_.object_observation_huber_loss_param_ =
        node[kObjectObservationHuberLossParamLabel];
    data_.shape_dim_prior_factor_huber_loss_param_ =
        node[kShapeDimPriorFactorHuberLossParamLabel];
    data_.invalid_ellipsoid_error_val_ = node[kInvalidEllipsoidErrorValLabel];
  }

 protected:
  using FileStorageSerializable<
      pose_graph_optimization::ObjectResidualParams>::data_;

 private:
  inline static const std::string kObjectObservationHuberLossParamLabel =
      "object_observation_huber_loss_param";
  inline static const std::string kShapeDimPriorFactorHuberLossParamLabel =
      "shape_dim_prior_factor_huber_loss_param";
  inline static const std::string kInvalidEllipsoidErrorValLabel =
      "invalid_ellipsoid_error_val";
};

static void write(cv::FileStorage &fs,
                  const std::string &,
                  const SerializableObjectResidualParams &data) {
  data.write(fs);
}

static void read(const cv::FileNode &node,
                 SerializableObjectResidualParams &data,
                 const SerializableObjectResidualParams &default_data =
                     SerializableObjectResidualParams()) {
  if (node.empty()) {
    data = default_data;
  } else {
    data.read(node);
  }
}

class SerializableLimitTrajectoryEvaluationParams
    : public FileStorageSerializable<LimitTrajectoryEvaluationParams> {
 public:
  SerializableLimitTrajectoryEvaluationParams()
      : FileStorageSerializable<LimitTrajectoryEvaluationParams>() {}
  SerializableLimitTrajectoryEvaluationParams(
      const LimitTrajectoryEvaluationParams &data)
      : FileStorageSerializable<LimitTrajectoryEvaluationParams>(data) {}

  virtual void write(cv::FileStorage &fs) const override {
    fs << "{";
    int should_limit_trajectory_evaluation_int =
        data_.should_limit_trajectory_evaluation_ ? 1 : 0;
    fs << kShouldLimitTrajectoryEvaluationLabel
       << should_limit_trajectory_evaluation_int;
    fs << kMaxFrameIdLabel << (int)data_.max_frame_id_;
    fs << "}";
  }

  virtual void read(const cv::FileNode &node) override {
    int should_limit_trajectory_evaluation_int =
        node[kShouldLimitTrajectoryEvaluationLabel];
    data_.should_limit_trajectory_evaluation_ =
        should_limit_trajectory_evaluation_int != 0;
    data_.max_frame_id_ = (int)node[kMaxFrameIdLabel];
  }

 protected:
  using FileStorageSerializable<LimitTrajectoryEvaluationParams>::data_;

 private:
  inline static const std::string kShouldLimitTrajectoryEvaluationLabel =
      "should_limit_trajectory_evaluation";
  inline static const std::string kMaxFrameIdLabel = "max_frame_id";
};

static void write(cv::FileStorage &fs,
                  const std::string &,
                  const SerializableLimitTrajectoryEvaluationParams &data) {
  data.write(fs);
}

static void read(
    const cv::FileNode &node,
    SerializableLimitTrajectoryEvaluationParams &data,
    const SerializableLimitTrajectoryEvaluationParams &default_data =
        SerializableLimitTrajectoryEvaluationParams()) {
  if (node.empty()) {
    data = default_data;
  } else {
    data.read(node);
  }
}

class SerializableOptimizationSolverParams
    : public FileStorageSerializable<
          pose_graph_optimization::OptimizationSolverParams> {
 public:
  SerializableOptimizationSolverParams()
      : FileStorageSerializable<
            pose_graph_optimization::OptimizationSolverParams>() {}
  SerializableOptimizationSolverParams(
      const pose_graph_optimization::OptimizationSolverParams &data)
      : FileStorageSerializable<
            pose_graph_optimization::OptimizationSolverParams>(data) {}

  virtual void write(cv::FileStorage &fs) const override {
    fs << "{";
    fs << kMaxNumIterationsLabel << data_.max_num_iterations_;
    fs << kFeatureOutlierPercentageLabel << data_.feature_outlier_percentage;
    int allow_non_monotonic_steps_int =
        data_.allow_non_monotonic_steps_ ? 1 : 0;
    fs << kAllowNonMonotonicStepsLabel << allow_non_monotonic_steps_int;
    fs << kFunctionToleranceLabel << data_.function_tolerance_;
    fs << kGradientToleranceLabel << data_.gradient_tolerance_;
    fs << kParameterToleranceLabel << data_.parameter_tolerance_;
    fs << kInitialTrustRegionRadiusLabel << data_.initial_trust_region_radius_;
    fs << kMaxTrustRegionRadiusLabel << data_.max_trust_region_radius_;
    fs << "}";
  }

  virtual void read(const cv::FileNode &node) override {
    data_.max_num_iterations_ = node[kMaxNumIterationsLabel];
    data_.feature_outlier_percentage = node[kFeatureOutlierPercentageLabel];
    int allow_non_monotonic_steps_int = node[kAllowNonMonotonicStepsLabel];
    data_.allow_non_monotonic_steps_ = allow_non_monotonic_steps_int != 0;
    data_.function_tolerance_ = node[kFunctionToleranceLabel];
    data_.gradient_tolerance_ = node[kGradientToleranceLabel];
    data_.parameter_tolerance_ = node[kParameterToleranceLabel];
    data_.initial_trust_region_radius_ = node[kInitialTrustRegionRadiusLabel];
    data_.max_trust_region_radius_ = node[kMaxTrustRegionRadiusLabel];
  }

 protected:
  using FileStorageSerializable<
      pose_graph_optimization::OptimizationSolverParams>::data_;

 private:
  inline static const std::string kMaxNumIterationsLabel = "max_num_iterations";
  inline static const std::string kFeatureOutlierPercentageLabel =
      "feature_outlier_percentage";
  inline static const std::string kAllowNonMonotonicStepsLabel =
      "allow_non_monotonic_steps";
  inline static const std::string kFunctionToleranceLabel =
      "function_tolerance";
  inline static const std::string kGradientToleranceLabel =
      "gradient_tolerance";
  inline static const std::string kParameterToleranceLabel =
      "parameter_tolerance";
  inline static const std::string kInitialTrustRegionRadiusLabel =
      "initial_trust_region_radius";
  inline static const std::string kMaxTrustRegionRadiusLabel =
      "max_trust_region_radius";
};

static void write(cv::FileStorage &fs,
                  const std::string &,
                  const SerializableOptimizationSolverParams &data) {
  data.write(fs);
}

static void read(const cv::FileNode &node,
                 SerializableOptimizationSolverParams &data,
                 const SerializableOptimizationSolverParams &default_data =
                     SerializableOptimizationSolverParams()) {
  if (node.empty()) {
    data = default_data;
  } else {
    data.read(node);
  }
}

class SerializableGeometricSimilarityScorerParams
    : public FileStorageSerializable<GeometricSimilarityScorerParams> {
 public:
  SerializableGeometricSimilarityScorerParams()
      : FileStorageSerializable<GeometricSimilarityScorerParams>() {}
  SerializableGeometricSimilarityScorerParams(
      const GeometricSimilarityScorerParams &data)
      : FileStorageSerializable<GeometricSimilarityScorerParams>(data) {}

  virtual void write(cv::FileStorage &fs) const override {
    fs << "{";
    fs << kMaxMergeDistanceLabel << data_.max_merge_distance_;
    fs << "}";
  }

  virtual void read(const cv::FileNode &node) override {
    data_.max_merge_distance_ = node[kMaxMergeDistanceLabel];
  }

 protected:
  using FileStorageSerializable<GeometricSimilarityScorerParams>::data_;

 private:
  inline static const std::string kMaxMergeDistanceLabel = "max_merge_distance";
};

static void write(cv::FileStorage &fs,
                  const std::string &,
                  const SerializableGeometricSimilarityScorerParams &data) {
  data.write(fs);
}

static void read(
    const cv::FileNode &node,
    SerializableGeometricSimilarityScorerParams &data,
    const SerializableGeometricSimilarityScorerParams &default_data =
        SerializableGeometricSimilarityScorerParams()) {
  if (node.empty()) {
    data = default_data;
  } else {
    data.read(node);
  }
}

class SerializableSparsifierParams
    : public FileStorageSerializable<SparsifierParams> {
 public:
  SerializableSparsifierParams()
      : FileStorageSerializable<SparsifierParams>() {}
  SerializableSparsifierParams(const SparsifierParams &data)
      : FileStorageSerializable<SparsifierParams>(data) {}

  virtual void write(cv::FileStorage &fs) const override {
    fs << "{";
    fs << kMaxPoseIncThresholdTranslLabel
       << data_.max_pose_inc_threshold_transl_;
    fs << kMaxPoseIncThresholdRotLabel << data_.max_pose_inc_threshold_rot_;
    fs << "}";
  }

  virtual void read(const cv::FileNode &node) override {
    data_.max_pose_inc_threshold_transl_ =
        node[kMaxPoseIncThresholdTranslLabel];
    data_.max_pose_inc_threshold_rot_ = node[kMaxPoseIncThresholdRotLabel];
  }

 protected:
  using FileStorageSerializable<SparsifierParams>::data_;

 private:
  inline static const std::string kMaxPoseIncThresholdTranslLabel =
      "max_pose_inc_threshold_transl";
  inline static const std::string kMaxPoseIncThresholdRotLabel =
      "max_pose_inc_threshold_rot";
};

static void write(cv::FileStorage &fs,
                  const std::string &,
                  const SerializableSparsifierParams &data) {
  data.write(fs);
}

static void read(const cv::FileNode &node,
                 SerializableSparsifierParams &data,
                 const SerializableSparsifierParams &default_data =
                     SerializableSparsifierParams()) {
  if (node.empty()) {
    data = default_data;
  } else {
    data.read(node);
  }
}

class SerializablePendingObjectEstimatorParams
    : public FileStorageSerializable<PendingObjectEstimatorParams> {
 public:
  SerializablePendingObjectEstimatorParams()
      : FileStorageSerializable<PendingObjectEstimatorParams>() {}
  SerializablePendingObjectEstimatorParams(
      const PendingObjectEstimatorParams &data)
      : FileStorageSerializable<PendingObjectEstimatorParams>(data) {}

  virtual void write(cv::FileStorage &fs) const override {
    fs << "{";
    fs << kObjectResidualParamsLabel
       << SerializableObjectResidualParams(data_.object_residual_params_);
    fs << kSolverParamsLabel
       << SerializableOptimizationSolverParams(data_.solver_params_);
    fs << "}";
  }

  virtual void read(const cv::FileNode &node) override {
    SerializableObjectResidualParams ser_object_residual_params;
    node[kObjectResidualParamsLabel] >> ser_object_residual_params;
    data_.object_residual_params_ = ser_object_residual_params.getEntry();
    SerializableOptimizationSolverParams ser_solver_params;
    node[kSolverParamsLabel] >> ser_solver_params;
    data_.solver_params_ = ser_solver_params.getEntry();
  }

 protected:
  using FileStorageSerializable<PendingObjectEstimatorParams>::data_;

 private:
  inline static const std::string kObjectResidualParamsLabel =
      "object_residual_params";
  inline static const std::string kSolverParamsLabel = "solver_params";
};

static void write(cv::FileStorage &fs,
                  const std::string &,
                  const SerializablePendingObjectEstimatorParams &data) {
  data.write(fs);
}

static void read(const cv::FileNode &node,
                 SerializablePendingObjectEstimatorParams &data,
                 const SerializablePendingObjectEstimatorParams &default_data =
                     SerializablePendingObjectEstimatorParams()) {
  if (node.empty()) {
    data = default_data;
  } else {
    data.read(node);
  }
}

class SerializableFeatureBasedBbAssociationParams
    : public FileStorageSerializable<FeatureBasedBbAssociationParams> {
 public:
  SerializableFeatureBasedBbAssociationParams()
      : FileStorageSerializable<FeatureBasedBbAssociationParams>() {}
  SerializableFeatureBasedBbAssociationParams(
      const FeatureBasedBbAssociationParams &data)
      : FileStorageSerializable<FeatureBasedBbAssociationParams>(data) {}

  virtual void write(cv::FileStorage &fs) const override {
    fs << "{";
    fs << kMinObservationsForLocalEstLabel
       << data_.min_observations_for_local_est_;
    fs << kMinObservationsLabel << data_.min_observations_;
    fs << kDiscardCandidateAfterNumFramesLabel
       << SerializableFrameId(data_.discard_candidate_after_num_frames_);
    fs << kMinBbConfidenceLabel << data_.min_bb_confidence_;
    fs << kRequiredMinConfForInitializationLabel
       << data_.required_min_conf_for_initialization_;
    fs << kMinOverlappingFeaturesForMatchLabel
       << data_.min_overlapping_features_for_match_;
    fs << kFeatureValidityWindowLabel
       << SerializableFrameId(data_.feature_validity_window_);
    fs << kPendingObjEstimatorParamsLabel
       << SerializablePendingObjectEstimatorParams(
              data_.pending_obj_estimator_params_);
    fs << kBoundingBoxInflationSizeLabel << data_.bounding_box_inflation_size_;
    fs << "}";
  }

  virtual void read(const cv::FileNode &node) override {
    data_.min_observations_for_local_est_ =
        (int)node[kMinObservationsForLocalEstLabel];
    data_.min_observations_ = (int)node[kMinObservationsLabel];

    SerializableFrameId ser_discard_candidate_after_num_frames;
    node[kDiscardCandidateAfterNumFramesLabel] >>
        ser_discard_candidate_after_num_frames;
    data_.discard_candidate_after_num_frames_ =
        ser_discard_candidate_after_num_frames.getEntry();

    data_.min_bb_confidence_ = node[kMinBbConfidenceLabel];
    data_.required_min_conf_for_initialization_ =
        node[kRequiredMinConfForInitializationLabel];
    data_.min_overlapping_features_for_match_ =
        node[kMinOverlappingFeaturesForMatchLabel];

    SerializableFrameId ser_feature_validity_window;
    node[kFeatureValidityWindowLabel] >> ser_feature_validity_window;
    data_.feature_validity_window_ = ser_feature_validity_window.getEntry();

    SerializablePendingObjectEstimatorParams ser_pending_obj_estimator_params;
    node[kPendingObjEstimatorParamsLabel] >> ser_pending_obj_estimator_params;
    data_.pending_obj_estimator_params_ =
        ser_pending_obj_estimator_params.getEntry();

    data_.bounding_box_inflation_size_ = node[kBoundingBoxInflationSizeLabel];
  }

 protected:
  using FileStorageSerializable<FeatureBasedBbAssociationParams>::data_;

 private:
  inline static const std::string kMinObservationsForLocalEstLabel =
      "min_observations_for_local_est";
  inline static const std::string kMinObservationsLabel = "min_observations";
  inline static const std::string kDiscardCandidateAfterNumFramesLabel =
      "discard_candidate_after_num_frames";
  inline static const std::string kMinBbConfidenceLabel = "min_bb_confidence";
  inline static const std::string kRequiredMinConfForInitializationLabel =
      "required_min_conf_for_initialization";
  inline static const std::string kMinOverlappingFeaturesForMatchLabel =
      "min_overlapping_features_for_match";
  inline static const std::string kFeatureValidityWindowLabel =
      "feature_validity_window";
  inline static const std::string kPendingObjEstimatorParamsLabel =
      "pending_obj_estimator_params";
  inline static const std::string kBoundingBoxInflationSizeLabel =
      "bounding_box_inflation_size";
};

static void write(cv::FileStorage &fs,
                  const std::string &,
                  const SerializableFeatureBasedBbAssociationParams &data) {
  data.write(fs);
}

static void read(
    const cv::FileNode &node,
    SerializableFeatureBasedBbAssociationParams &data,
    const SerializableFeatureBasedBbAssociationParams &default_data =
        SerializableFeatureBasedBbAssociationParams()) {
  if (node.empty()) {
    data = default_data;
  } else {
    data.read(node);
  }
}

class SerializableBoundingBoxFrontEndParams
    : public FileStorageSerializable<BoundingBoxFrontEndParams> {
 public:
  SerializableBoundingBoxFrontEndParams()
      : FileStorageSerializable<BoundingBoxFrontEndParams>() {}
  SerializableBoundingBoxFrontEndParams(const BoundingBoxFrontEndParams &data)
      : FileStorageSerializable<BoundingBoxFrontEndParams>(data) {}

  virtual void write(cv::FileStorage &fs) const override {
    fs << "{";
    fs << kGeometricSimilarityScorerParamsLabel
       << SerializableGeometricSimilarityScorerParams(
              data_.geometric_similarity_scorer_params_);
    fs << kFeatureBasedBbAssociationParamsLabel
       << SerializableFeatureBasedBbAssociationParams(
              data_.feature_based_bb_association_params_);
    fs << "}";
  }

  virtual void read(const cv::FileNode &node) override {
    SerializableGeometricSimilarityScorerParams
        ser_geometric_similarity_scorer_params;
    node[kGeometricSimilarityScorerParamsLabel] >>
        ser_geometric_similarity_scorer_params;
    data_.geometric_similarity_scorer_params_ =
        ser_geometric_similarity_scorer_params.getEntry();
    SerializableFeatureBasedBbAssociationParams
        ser_feature_based_bb_association_params;
    node[kFeatureBasedBbAssociationParamsLabel] >>
        ser_feature_based_bb_association_params;
    data_.feature_based_bb_association_params_ =
        ser_feature_based_bb_association_params.getEntry();
  }

 protected:
  using FileStorageSerializable<BoundingBoxFrontEndParams>::data_;

 private:
  inline static const std::string kGeometricSimilarityScorerParamsLabel =
      "geometric_similarity_scorer_params";
  inline static const std::string kFeatureBasedBbAssociationParamsLabel =
      "feature_based_bb_association_params";
};

static void write(cv::FileStorage &fs,
                  const std::string &,
                  const SerializableBoundingBoxFrontEndParams &data) {
  data.write(fs);
}

static void read(const cv::FileNode &node,
                 SerializableBoundingBoxFrontEndParams &data,
                 const SerializableBoundingBoxFrontEndParams &default_data =
                     SerializableBoundingBoxFrontEndParams()) {
  if (node.empty()) {
    data = default_data;
  } else {
    data.read(node);
  }
}

class SerializableShapeDimensionPriors
    : public FileStorageSerializable<ShapeDimensionPriors> {
 public:
  SerializableShapeDimensionPriors()
      : FileStorageSerializable<ShapeDimensionPriors>() {}
  SerializableShapeDimensionPriors(const ShapeDimensionPriors &data)
      : FileStorageSerializable<ShapeDimensionPriors>(data) {}

  virtual void write(cv::FileStorage &fs) const override {
    fs << "{" << kDimensionPriorMapLabel << "[";
    for (const auto &mean_and_cov_entry :
         data_.mean_and_cov_by_semantic_class_) {
      fs << "{";
      fs << kSemanticClassLabel << mean_and_cov_entry.first;
      fs << kObjectDimMeanLabel
         << SerializableEigenMat<double, 3, 1>(mean_and_cov_entry.second.first);
      fs << kDimCovarianceLabel
         << SerializableEigenMat<double, 3, 3>(
                mean_and_cov_entry.second.second);
      fs << "}";
    }
    fs << "]";
    fs << "}";
  }

  virtual void read(const cv::FileNode &node) override {
    cv::FileNode results_map_data = node[kDimensionPriorMapLabel];
    for (cv::FileNodeIterator it = results_map_data.begin();
         it != results_map_data.end();
         it++) {
      cv::FileNode entry = *it;

      std::string semantic_class;
      entry[kSemanticClassLabel] >> semantic_class;

      SerializableEigenMat<double, 3, 1> ser_mean;
      entry[kObjectDimMeanLabel] >> ser_mean;

      SerializableEigenMat<double, 3, 3> ser_covariance;
      entry[kDimCovarianceLabel] >> ser_covariance;

      data_.mean_and_cov_by_semantic_class_[semantic_class] =
          std::make_pair(ser_mean.getEntry(), ser_covariance.getEntry());
    }
  }

 protected:
  using FileStorageSerializable<ShapeDimensionPriors>::data_;

 private:
  inline static const std::string kSemanticClassLabel = "semantic_class";
  inline static const std::string kObjectDimMeanLabel = "obj_dim_mean";
  inline static const std::string kDimCovarianceLabel = "dim_covariance";
  inline static const std::string kDimensionPriorMapLabel =
      "dimension_prior_label";
};

static void write(cv::FileStorage &fs,
                  const std::string &,
                  const SerializableShapeDimensionPriors &data) {
  data.write(fs);
}

static void read(const cv::FileNode &node,
                 SerializableShapeDimensionPriors &data,
                 const SerializableShapeDimensionPriors &default_data =
                     SerializableShapeDimensionPriors()) {
  if (node.empty()) {
    data = default_data;
  } else {
    data.read(node);
  }
}

class SerializableCameraInfo : public FileStorageSerializable<CameraInfo> {
 public:
  SerializableCameraInfo() : FileStorageSerializable<CameraInfo>() {}
  SerializableCameraInfo(const CameraInfo &data)
      : FileStorageSerializable<CameraInfo>(data) {}

  virtual void write(cv::FileStorage &fs) const override {
    fs << "{" << kCameraTopicToCameraIdLabel << "[";
    for (const auto &camera_topic_to_id : data_.camera_topic_to_camera_id_) {
      fs << "{";
      fs << kCameraTopicLabel << camera_topic_to_id.first;
      fs << kCameraIdLabel << SerializableCameraId(camera_topic_to_id.second);
      fs << "}";
    }
    fs << "]";
    fs << "}";
  }

  virtual void read(const cv::FileNode &node) override {
    cv::FileNode results_map_data = node[kCameraTopicToCameraIdLabel];
    for (cv::FileNodeIterator it = results_map_data.begin();
         it != results_map_data.end();
         it++) {
      cv::FileNode entry = *it;

      std::string camera_topic;
      entry[kCameraTopicLabel] >> camera_topic;

      SerializableCameraId ser_camera_id;
      entry[kCameraIdLabel] >> ser_camera_id;

      data_.camera_topic_to_camera_id_[camera_topic] = ser_camera_id.getEntry();
    }
  }

 protected:
  using FileStorageSerializable<CameraInfo>::data_;

 private:
  inline static const std::string kCameraTopicLabel = "camera_topic";
  inline static const std::string kCameraIdLabel = "camera_id";
  inline static const std::string kCameraTopicToCameraIdLabel =
      "camera_topic_to_camera_id";
};

static void write(cv::FileStorage &fs,
                  const std::string &,
                  const SerializableCameraInfo &data) {
  data.write(fs);
}

static void read(
    const cv::FileNode &node,
    SerializableCameraInfo &data,
    const SerializableCameraInfo &default_data = SerializableCameraInfo()) {
  if (node.empty()) {
    data = default_data;
  } else {
    data.read(node);
  }
}

class SerializableVisualFeatureParams
    : public FileStorageSerializable<VisualFeatureParams> {
 public:
  SerializableVisualFeatureParams()
      : FileStorageSerializable<VisualFeatureParams>() {}
  SerializableVisualFeatureParams(const VisualFeatureParams &data)
      : FileStorageSerializable<VisualFeatureParams>(data) {}

  virtual void write(cv::FileStorage &fs) const override {
    fs << "{";
    fs << kReprojectionErrorStdDevLabel << data_.reprojection_error_std_dev_;
    fs << kMinVisualFeatureParallaxPixelRequirementLabel
       << data_.min_visual_feature_parallax_pixel_requirement_;
    fs << kMinVisualFeatureParallaxRobotTranslRequirementLabel
       << data_.min_visual_feature_parallax_robot_transl_requirement_;
    fs << kMinVisualFeatureParallaxRobotOrientRequirementLabel
       << data_.min_visual_feature_parallax_robot_orient_requirement_;
    fs << kEnforceMinPixelParallaxRequirement
       << data_.enforce_min_pixel_parallax_requirement_;
    fs << kEnforceMinRobotPoseParallaxRequirement
       << data_.enforce_min_robot_pose_parallax_requirement_;
    fs << "}";
  }

  virtual void read(const cv::FileNode &node) override {
    data_.reprojection_error_std_dev_ = node[kReprojectionErrorStdDevLabel];
    data_.min_visual_feature_parallax_pixel_requirement_ =
        node[kMinVisualFeatureParallaxPixelRequirementLabel];
    node[kMinVisualFeatureParallaxRobotTranslRequirementLabel] >>
        data_.min_visual_feature_parallax_robot_transl_requirement_;
    node[kMinVisualFeatureParallaxRobotOrientRequirementLabel] >>
        data_.min_visual_feature_parallax_robot_orient_requirement_;
    node[kEnforceMinPixelParallaxRequirement] >>
        data_.enforce_min_pixel_parallax_requirement_;
    node[kEnforceMinRobotPoseParallaxRequirement] >>
        data_.enforce_min_robot_pose_parallax_requirement_;
  }

 protected:
  using FileStorageSerializable<VisualFeatureParams>::data_;

 private:
  inline static const std::string kReprojectionErrorStdDevLabel =
      "reprojection_error_std_dev";
  inline static const std::string
      kMinVisualFeatureParallaxPixelRequirementLabel =
          "min_visual_feature_parallax_pixel_requirement";
  inline static const std::string
      kMinVisualFeatureParallaxRobotTranslRequirementLabel =
          "min_visual_feature_parallax_robot_transl_requirement";
  inline static const std::string
      kMinVisualFeatureParallaxRobotOrientRequirementLabel =
          "min_visual_feature_parallax_robot_orient_requirement";
  inline static const std::string kEnforceMinPixelParallaxRequirement =
      "enforce_min_pixel_parallax_requirement_";
  inline static const std::string kEnforceMinRobotPoseParallaxRequirement =
      "enforce_min_robot_pose_parallax_requirement_";
};

static void write(cv::FileStorage &fs,
                  const std::string &,
                  const SerializableVisualFeatureParams &data) {
  data.write(fs);
}

static void read(const cv::FileNode &node,
                 SerializableVisualFeatureParams &data,
                 const SerializableVisualFeatureParams &default_data =
                     SerializableVisualFeatureParams()) {
  if (node.empty()) {
    data = default_data;
  } else {
    data.read(node);
  }
}

class SerializableLongTermMapExtractionTunableParams
    : public FileStorageSerializable<LongTermMapExtractionTunableParams> {
 public:
  SerializableLongTermMapExtractionTunableParams()
      : FileStorageSerializable<LongTermMapExtractionTunableParams>() {}
  SerializableLongTermMapExtractionTunableParams(
      const LongTermMapExtractionTunableParams &data)
      : FileStorageSerializable<LongTermMapExtractionTunableParams>(data) {}

  virtual void write(cv::FileStorage &fs) const override {
    fs << "{";
    fs << kFarFeatureThresholdLabel << data_.far_feature_threshold_;
    fs << "}";
  }

  virtual void read(const cv::FileNode &node) override {
    data_.far_feature_threshold_ = node[kFarFeatureThresholdLabel];
  }

 protected:
  using FileStorageSerializable<LongTermMapExtractionTunableParams>::data_;

 private:
  inline static const std::string kFarFeatureThresholdLabel =
      "far_feature_threshold";
};

static void write(cv::FileStorage &fs,
                  const std::string &,
                  const SerializableLongTermMapExtractionTunableParams &data) {
  data.write(fs);
}

static void read(
    const cv::FileNode &node,
    SerializableLongTermMapExtractionTunableParams &data,
    const SerializableLongTermMapExtractionTunableParams &default_data =
        SerializableLongTermMapExtractionTunableParams()) {
  if (node.empty()) {
    data = default_data;
  } else {
    data.read(node);
  }
}

class SerializableVisualFeaturePoseGraphResidualParams
    : public FileStorageSerializable<
          pose_graph_optimization::VisualFeaturePoseGraphResidualParams> {
 public:
  SerializableVisualFeaturePoseGraphResidualParams()
      : FileStorageSerializable<
            pose_graph_optimization::VisualFeaturePoseGraphResidualParams>() {}
  SerializableVisualFeaturePoseGraphResidualParams(
      const pose_graph_optimization::VisualFeaturePoseGraphResidualParams &data)
      : FileStorageSerializable<
            pose_graph_optimization::VisualFeaturePoseGraphResidualParams>(
            data) {}

  virtual void write(cv::FileStorage &fs) const override {
    fs << "{";
    fs << kReprojectionErrorHuberLossParamLabel
       << data_.reprojection_error_huber_loss_param_;
    fs << "}";
  }

  virtual void read(const cv::FileNode &node) override {
    data_.reprojection_error_huber_loss_param_ =
        node[kReprojectionErrorHuberLossParamLabel];
  }

 protected:
  using FileStorageSerializable<
      pose_graph_optimization::VisualFeaturePoseGraphResidualParams>::data_;

 private:
  inline static const std::string kReprojectionErrorHuberLossParamLabel =
      "reprojection_error_huber_loss_param";
};

static void write(
    cv::FileStorage &fs,
    const std::string &,
    const SerializableVisualFeaturePoseGraphResidualParams &data) {
  data.write(fs);
}

static void read(
    const cv::FileNode &node,
    SerializableVisualFeaturePoseGraphResidualParams &data,
    const SerializableVisualFeaturePoseGraphResidualParams &default_data =
        SerializableVisualFeaturePoseGraphResidualParams()) {
  if (node.empty()) {
    data = default_data;
  } else {
    data.read(node);
  }
}

class SerializablePairwiseLongTermMapResidualParams
    : public FileStorageSerializable<
          pose_graph_optimization::PairwiseLongTermMapResidualParams> {
 public:
  SerializablePairwiseLongTermMapResidualParams()
      : FileStorageSerializable<
            pose_graph_optimization::PairwiseLongTermMapResidualParams>() {}
  SerializablePairwiseLongTermMapResidualParams(
      const pose_graph_optimization::PairwiseLongTermMapResidualParams &data)
      : FileStorageSerializable<
            pose_graph_optimization::PairwiseLongTermMapResidualParams>(data) {}

  virtual void write(cv::FileStorage &fs) const override {
    fs << "{";
    fs << kPairHuberLossParamLabel << data_.pair_huber_loss_param_;
    fs << "}";
  }

  virtual void read(const cv::FileNode &node) override {
    data_.pair_huber_loss_param_ = node[kPairHuberLossParamLabel];
  }

 protected:
  using FileStorageSerializable<
      pose_graph_optimization::PairwiseLongTermMapResidualParams>::data_;

 private:
  inline static const std::string kPairHuberLossParamLabel =
      "pair_huber_loss_param";
};

static void write(cv::FileStorage &fs,
                  const std::string &,
                  const SerializablePairwiseLongTermMapResidualParams &data) {
  data.write(fs);
}

static void read(
    const cv::FileNode &node,
    SerializablePairwiseLongTermMapResidualParams &data,
    const SerializablePairwiseLongTermMapResidualParams &default_data =
        SerializablePairwiseLongTermMapResidualParams()) {
  if (node.empty()) {
    data = default_data;
  } else {
    data.read(node);
  }
}

class SerializableObjectVisualPoseGraphResidualParams
    : public FileStorageSerializable<
          pose_graph_optimization::ObjectVisualPoseGraphResidualParams> {
 public:
  SerializableObjectVisualPoseGraphResidualParams()
      : FileStorageSerializable<
            pose_graph_optimization::ObjectVisualPoseGraphResidualParams>() {}
  SerializableObjectVisualPoseGraphResidualParams(
      const pose_graph_optimization::ObjectVisualPoseGraphResidualParams &data)
      : FileStorageSerializable<
            pose_graph_optimization::ObjectVisualPoseGraphResidualParams>(
            data) {}

  virtual void write(cv::FileStorage &fs) const override {
    fs << "{";
    fs << kObjectResidualParamsLabel
       << SerializableObjectResidualParams(data_.object_residual_params_);
    fs << kVisualResidualParamsLabel
       << SerializableVisualFeaturePoseGraphResidualParams(
              data_.visual_residual_params_);
    fs << kLongTermMapParamsLabel
       << SerializablePairwiseLongTermMapResidualParams(
              data_.long_term_map_params_);
    fs << "}";
  }

  virtual void read(const cv::FileNode &node) override {
    SerializableObjectResidualParams ser_object_residual_params;
    node[kObjectResidualParamsLabel] >> ser_object_residual_params;
    data_.object_residual_params_ = ser_object_residual_params.getEntry();
    SerializableVisualFeaturePoseGraphResidualParams ser_visual_residual_params;
    node[kVisualResidualParamsLabel] >> ser_visual_residual_params;
    data_.visual_residual_params_ = ser_visual_residual_params.getEntry();
    SerializablePairwiseLongTermMapResidualParams ser_long_term_map_params;
    node[kLongTermMapParamsLabel] >> ser_long_term_map_params;
    data_.long_term_map_params_ = ser_long_term_map_params.getEntry();
  }

 protected:
  using FileStorageSerializable<
      pose_graph_optimization::ObjectVisualPoseGraphResidualParams>::data_;

 private:
  inline static const std::string kObjectResidualParamsLabel =
      "object_residual_params";
  inline static const std::string kVisualResidualParamsLabel =
      "visual_residual_params";
  inline static const std::string kLongTermMapParamsLabel =
      "long_term_map_params";
};

static void write(cv::FileStorage &fs,
                  const std::string &,
                  const SerializableObjectVisualPoseGraphResidualParams &data) {
  data.write(fs);
}

static void read(
    const cv::FileNode &node,
    SerializableObjectVisualPoseGraphResidualParams &data,
    const SerializableObjectVisualPoseGraphResidualParams &default_data =
        SerializableObjectVisualPoseGraphResidualParams()) {
  if (node.empty()) {
    data = default_data;
  } else {
    data.read(node);
  }
}

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

    fs << kVisualFeatureParamsLabel
       << SerializableVisualFeatureParams(data_.visual_feature_params_);
    fs << kLocalBaSolverParamsLabel
       << SerializableOptimizationSolverParams(data_.local_ba_solver_params_);
    fs << kGlobalBaSolverParamsLabel
       << SerializableOptimizationSolverParams(data_.global_ba_solver_params_);
    fs << kFinalBaSolverParamsLabel
       << SerializableOptimizationSolverParams(data_.final_ba_solver_params_);
    fs << kLtmTunableParamsLabel
       << SerializableLongTermMapExtractionTunableParams(
              data_.ltm_tunable_params_);
    fs << kLtmSolverResidualParamsLabel
       << SerializableObjectVisualPoseGraphResidualParams(
              data_.ltm_solver_residual_params_);
    fs << kLtmSolverParamsLabel
       << SerializableOptimizationSolverParams(data_.ltm_solver_params_);
    fs << kShapeDimensionPriorsLabel
       << SerializableShapeDimensionPriors(data_.shape_dimension_priors_);
    fs << kCameraInfoLabel << SerializableCameraInfo(data_.camera_info_);
    fs << kBoundingBoxFrontEndParamsLabel
       << SerializableBoundingBoxFrontEndParams(
              data_.bounding_box_front_end_params_);
    fs << kBoundingBoxCovarianceGeneratorParamsLabel
       << SerializableBoundingBoxCovGenParams(
              data_.bounding_box_covariance_generator_params_);
    fs << kSlidingWindowParamsLabel
       << SerializableSlidingWindowParams(data_.sliding_window_params_);
    fs << kOptimizationFactorsEnabledParamsLabel
       << SerializableOptimizationFactorsEnabledParams(
              data_.optimization_factors_enabled_params_);
    fs << kObjectVisualPoseGraphResidualParamsLabel
       << SerializableObjectVisualPoseGraphResidualParams(
              data_.object_visual_pose_graph_residual_params_);
    fs << kLimitTrajEvalParamsLabel
       << SerializableLimitTrajectoryEvaluationParams(
              data_.limit_traj_eval_params_);
    fs << kSparsifierParamsLabel
       << SerializableSparsifierParams(data_.sparsifier_params_);
    fs << "}";
  }

  virtual void read(const cv::FileNode &node) override {
    data_.config_schema_version_ = node[kConfigSchemaVersionLabel];
    data_.config_version_id_ = (std::string)node[kConfigVersionIdLabel];

    SerializableVisualFeatureParams ser_visual_feature_params;
    node[kVisualFeatureParamsLabel] >> ser_visual_feature_params;
    data_.visual_feature_params_ = ser_visual_feature_params.getEntry();

    SerializableOptimizationSolverParams ser_local_ba_solver_params;
    node[kLocalBaSolverParamsLabel] >> ser_local_ba_solver_params;
    data_.local_ba_solver_params_ = ser_local_ba_solver_params.getEntry();

    SerializableOptimizationSolverParams ser_global_ba_solver_params;
    node[kGlobalBaSolverParamsLabel] >> ser_global_ba_solver_params;
    data_.global_ba_solver_params_ = ser_global_ba_solver_params.getEntry();

    SerializableOptimizationSolverParams ser_final_ba_solver_params;
    node[kFinalBaSolverParamsLabel] >> ser_final_ba_solver_params;
    data_.final_ba_solver_params_ = ser_final_ba_solver_params.getEntry();

    SerializableLongTermMapExtractionTunableParams ser_ltm_tunable_params;
    node[kLtmTunableParamsLabel] >> ser_ltm_tunable_params;
    data_.ltm_tunable_params_ = ser_ltm_tunable_params.getEntry();

    SerializableObjectVisualPoseGraphResidualParams
        ser_ltm_solver_residual_params;
    node[kLtmSolverResidualParamsLabel] >> ser_ltm_solver_residual_params;
    data_.ltm_solver_residual_params_ =
        ser_ltm_solver_residual_params.getEntry();

    SerializableOptimizationSolverParams ser_ltm_solver_params;
    node[kLtmSolverParamsLabel] >> ser_ltm_solver_params;
    data_.ltm_solver_params_ = ser_ltm_solver_params.getEntry();

    SerializableShapeDimensionPriors ser_shape_dimension_priors;
    node[kShapeDimensionPriorsLabel] >> ser_shape_dimension_priors;
    data_.shape_dimension_priors_ = ser_shape_dimension_priors.getEntry();

    SerializableCameraInfo ser_camera_info;
    node[kCameraInfoLabel] >> ser_camera_info;
    data_.camera_info_ = ser_camera_info.getEntry();

    SerializableBoundingBoxFrontEndParams ser_bounding_box_front_end_params;
    node[kBoundingBoxFrontEndParamsLabel] >> ser_bounding_box_front_end_params;
    data_.bounding_box_front_end_params_ =
        ser_bounding_box_front_end_params.getEntry();

    SerializableBoundingBoxCovGenParams
        ser_bounding_box_covariance_generator_params;
    node[kBoundingBoxCovarianceGeneratorParamsLabel] >>
        ser_bounding_box_covariance_generator_params;
    data_.bounding_box_covariance_generator_params_ =
        ser_bounding_box_covariance_generator_params.getEntry();

    SerializableSlidingWindowParams ser_sliding_window_params;
    node[kSlidingWindowParamsLabel] >> ser_sliding_window_params;
    data_.sliding_window_params_ = ser_sliding_window_params.getEntry();

    SerializableOptimizationFactorsEnabledParams
        ser_optimization_factors_enabled_params;
    node[kOptimizationFactorsEnabledParamsLabel] >>
        ser_optimization_factors_enabled_params;
    data_.optimization_factors_enabled_params_ =
        ser_optimization_factors_enabled_params.getEntry();

    SerializableObjectVisualPoseGraphResidualParams
        ser_object_visual_pose_graph_residual_params;
    node[kObjectVisualPoseGraphResidualParamsLabel] >>
        ser_object_visual_pose_graph_residual_params;
    data_.object_visual_pose_graph_residual_params_ =
        ser_object_visual_pose_graph_residual_params.getEntry();

    SerializableLimitTrajectoryEvaluationParams ser_limit_traj_eval_params;
    node[kLimitTrajEvalParamsLabel] >> ser_limit_traj_eval_params;
    data_.limit_traj_eval_params_ = ser_limit_traj_eval_params.getEntry();

    SerializableSparsifierParams ser_sparsifier_params;
    node[kSparsifierParamsLabel] >> ser_sparsifier_params;
    data_.sparsifier_params_ = ser_sparsifier_params.getEntry();
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
    throw std::invalid_argument(
        "Incorrect schema type; should match current config schema version. "
        "Please update your config to the latest format. ");
  }
}

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_CONFIG_FILE_STORAGE_IO_H
