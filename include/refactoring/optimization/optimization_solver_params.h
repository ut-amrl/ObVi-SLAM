//
// Created by amanda on 6/14/22.
//

#ifndef UT_VSLAM_OPTIMIZATION_SOLVER_PARAMS_H
#define UT_VSLAM_OPTIMIZATION_SOLVER_PARAMS_H

namespace pose_graph_optimization {
struct OptimizationSolverParams {
  // NOTE: If this structure is modified, increment the
  // kCurrentConfigSchemaVersion number in FullOVSLAMConfig.h NOTE: If the
  // default values here are modified, make sure any changes are reflected in
  // the config and if necessary, regenerate the config with a new
  // config_version_id_

  int max_num_iterations_ = 100;
  double feature_outlier_percentage_ = .1;
  bool allow_non_monotonic_steps_ = false;
  double function_tolerance_ = 1e-6;          // Ceres default
  double gradient_tolerance_ = 1e-10;         // Ceres default
  double parameter_tolerance_ = 1e-8;         // Ceres default
  double initial_trust_region_radius_ = 1e4;  // Ceres default
  double max_trust_region_radius_ = 1e16;     // Ceres default
  // TODO

  bool operator==(const OptimizationSolverParams &rhs) const {
    return (max_num_iterations_ == rhs.max_num_iterations_) &&
           (feature_outlier_percentage_ == rhs.feature_outlier_percentage_) &&
           (allow_non_monotonic_steps_ == rhs.allow_non_monotonic_steps_) &&
           (function_tolerance_ == rhs.function_tolerance_) &&
           (gradient_tolerance_ == rhs.gradient_tolerance_) &&
           (parameter_tolerance_ == rhs.parameter_tolerance_);
  }

  bool operator!=(const OptimizationSolverParams &rhs) const {
    return !operator==(rhs);
  }
};

// TOOD revise all residual param defaults
struct ObjectResidualParams {
  // NOTE: If this structure is modified, increment the
  // kCurrentConfigSchemaVersion number in FullOVSLAMConfig.h NOTE: If the
  // default values here are modified, make sure any changes are reflected in
  // the config and if necessary, regenerate the config with a new
  // config_version_id_
  double object_observation_huber_loss_param_ = 1;
  double shape_dim_prior_factor_huber_loss_param_ = 1;
  double invalid_ellipsoid_error_val_ = 1e6;

  bool operator==(const ObjectResidualParams &rhs) const {
    return (object_observation_huber_loss_param_ ==
            rhs.object_observation_huber_loss_param_) &&
           (shape_dim_prior_factor_huber_loss_param_ ==
            rhs.shape_dim_prior_factor_huber_loss_param_) &&
           (invalid_ellipsoid_error_val_ == rhs.invalid_ellipsoid_error_val_);
  }

  bool operator!=(const ObjectResidualParams &rhs) const {
    return !operator==(rhs);
  }
};

struct PairwiseLongTermMapResidualParams {
  // NOTE: If this structure is modified, increment the
  // kCurrentConfigSchemaVersion number in FullOVSLAMConfig.h NOTE: If the
  // default values here are modified, make sure any changes are reflected in
  // the config and if necessary, regenerate the config with a new
  // config_version_id_
  double pair_huber_loss_param_ = 1;

  bool operator==(const PairwiseLongTermMapResidualParams &rhs) const {
    return (pair_huber_loss_param_ == rhs.pair_huber_loss_param_);
  }

  bool operator!=(const PairwiseLongTermMapResidualParams &rhs) const {
    return !operator==(rhs);
  }
};

struct VisualFeaturePoseGraphResidualParams {
  // NOTE: If this structure is modified, increment the
  // kCurrentConfigSchemaVersion number in FullOVSLAMConfig.h NOTE: If the
  // default values here are modified, make sure any changes are reflected in
  // the config and if necessary, regenerate the config with a new
  // config_version_id_
  double reprojection_error_huber_loss_param_ = 1;

  bool operator==(const VisualFeaturePoseGraphResidualParams &rhs) const {
    return (reprojection_error_huber_loss_param_ ==
            rhs.reprojection_error_huber_loss_param_);
  }

  bool operator!=(const VisualFeaturePoseGraphResidualParams &rhs) const {
    return !operator==(rhs);
  }
};

struct PoseResidualParams {
  double rel_pose_huber_loss_param_ = 1.0;

  bool operator==(const PoseResidualParams &rhs) const {
    return (rel_pose_huber_loss_param_ == rhs.rel_pose_huber_loss_param_);
  }

  bool operator!=(const PoseResidualParams &rhs) const {
    return !operator==(rhs);
  }
};

struct ObjectVisualPoseGraphResidualParams {
  // NOTE: If this structure is modified, increment the
  // kCurrentConfigSchemaVersion number in FullOVSLAMConfig.h NOTE: If the
  // default values here are modified, make sure any changes are reflected in
  // the config and if necessary, regenerate the config with a new
  // config_version_id_
  PoseResidualParams pose_residual_params_;
  ObjectResidualParams object_residual_params_;
  VisualFeaturePoseGraphResidualParams visual_residual_params_;
  // TODO maybe make this template so it's flexible to different types of
  // long-term maps
  PairwiseLongTermMapResidualParams long_term_map_params_;

  bool operator==(const ObjectVisualPoseGraphResidualParams &rhs) const {
    return (object_residual_params_ == rhs.object_residual_params_) &&
           (visual_residual_params_ == rhs.visual_residual_params_) &&
           (long_term_map_params_ == rhs.long_term_map_params_) &&
           (pose_residual_params_ == rhs.pose_residual_params_);
  }

  bool operator!=(const ObjectVisualPoseGraphResidualParams &rhs) const {
    return !operator==(rhs);
  }
};
}  // namespace pose_graph_optimization

#endif  // UT_VSLAM_OPTIMIZATION_SOLVER_PARAMS_H
