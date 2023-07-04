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
  bool allow_non_monotonic_steps_ = false;
  double function_tolerance_ = 1e-6;          // Ceres default
  double gradient_tolerance_ = 1e-10;         // Ceres default
  double parameter_tolerance_ = 1e-8;         // Ceres default
  double initial_trust_region_radius_ = 1e4;  // Ceres default
  double max_trust_region_radius_ = 1e16;     // Ceres default
  // TODO

  bool operator==(const OptimizationSolverParams &rhs) const {
    return (max_num_iterations_ == rhs.max_num_iterations_) &&
           (allow_non_monotonic_steps_ == rhs.allow_non_monotonic_steps_) &&
           (function_tolerance_ == rhs.function_tolerance_) &&
           (gradient_tolerance_ == rhs.gradient_tolerance_) &&
           (parameter_tolerance_ == rhs.parameter_tolerance_);
  }

  bool operator!=(const OptimizationSolverParams &rhs) const {
    return !operator==(rhs);
  }
};

struct OptimizationIterationParams {
  bool allow_reversion_after_detecting_jumps_;
  double consecutive_pose_transl_tol_ = 1.0;
  double consecutive_pose_orient_tol_ = M_PI;
  double feature_outlier_percentage_ = .1;
  OptimizationSolverParams phase_one_opt_params_;
  OptimizationSolverParams phase_two_opt_params_;

  bool operator==(const OptimizationIterationParams &rhs) const {
    return (allow_reversion_after_detecting_jumps_ ==
            rhs.allow_reversion_after_detecting_jumps_) &&
           (consecutive_pose_transl_tol_ == rhs.consecutive_pose_transl_tol_) &&
           (consecutive_pose_orient_tol_ == rhs.consecutive_pose_orient_tol_) &&
           (feature_outlier_percentage_ == rhs.feature_outlier_percentage_) &&
           (phase_one_opt_params_ == rhs.phase_one_opt_params_) &&
           (phase_two_opt_params_ == rhs.phase_two_opt_params_);
  }

  bool operator!=(const OptimizationIterationParams &rhs) const {
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

struct RelativePoseCovarianceOdomModelParams {
  double transl_error_mult_for_transl_error_;
  double transl_error_mult_for_rot_error_;
  double rot_error_mult_for_transl_error_;
  double rot_error_mult_for_rot_error_;

  bool operator==(const RelativePoseCovarianceOdomModelParams &rhs) const {
    return (transl_error_mult_for_transl_error_ ==
            rhs.transl_error_mult_for_transl_error_) &&
           (transl_error_mult_for_rot_error_ ==
            rhs.transl_error_mult_for_rot_error_) &&
           (rot_error_mult_for_transl_error_ ==
            rhs.rot_error_mult_for_transl_error_) &&
           (rot_error_mult_for_rot_error_ == rhs.rot_error_mult_for_rot_error_);
  }

  bool operator!=(const RelativePoseCovarianceOdomModelParams &rhs) const {
    return !operator==(rhs);
  }
};

struct ObjectVisualPoseGraphResidualParams {
  // NOTE: If this structure is modified, increment the
  // kCurrentConfigSchemaVersion number in FullOVSLAMConfig.h NOTE: If the
  // default values here are modified, make sure any changes are reflected in
  // the config and if necessary, regenerate the config with a new
  // config_version_id_
  ObjectResidualParams object_residual_params_;
  VisualFeaturePoseGraphResidualParams visual_residual_params_;
  // TODO maybe make this template so it's flexible to different types of
  // long-term maps
  PairwiseLongTermMapResidualParams long_term_map_params_;

  double relative_pose_factor_huber_loss_ = 1.0;
  RelativePoseCovarianceOdomModelParams relative_pose_cov_params_;

  bool operator==(const ObjectVisualPoseGraphResidualParams &rhs) const {
    return (object_residual_params_ == rhs.object_residual_params_) &&
           (visual_residual_params_ == rhs.visual_residual_params_) &&
           (long_term_map_params_ == rhs.long_term_map_params_) &&
           (relative_pose_factor_huber_loss_ ==
            rhs.relative_pose_factor_huber_loss_) &&
           (relative_pose_cov_params_ == rhs.relative_pose_cov_params_);
  }

  bool operator!=(const ObjectVisualPoseGraphResidualParams &rhs) const {
    return !operator==(rhs);
  }
};

struct PoseGraphPlusObjectsOptimizationParams {
  double relative_pose_factor_huber_loss_ = 1.0;

  bool enable_visual_feats_only_opt_post_pgo_ = false;
  bool enable_visual_non_opt_feature_adjustment_post_pgo_ = false;

  RelativePoseCovarianceOdomModelParams relative_pose_cov_params_;
  OptimizationSolverParams pgo_optimization_solver_params_;
  OptimizationSolverParams final_pgo_optimization_solver_params_;

  bool operator==(const PoseGraphPlusObjectsOptimizationParams &rhs) const {
    return (relative_pose_factor_huber_loss_ ==
            rhs.relative_pose_factor_huber_loss_) &&
           (enable_visual_feats_only_opt_post_pgo_ ==
            rhs.enable_visual_feats_only_opt_post_pgo_) &&
           (enable_visual_non_opt_feature_adjustment_post_pgo_ ==
            rhs.enable_visual_non_opt_feature_adjustment_post_pgo_) &&
           (relative_pose_cov_params_ == rhs.relative_pose_cov_params_) &&
           (pgo_optimization_solver_params_ ==
            rhs.pgo_optimization_solver_params_) &&
           (final_pgo_optimization_solver_params_ ==
            rhs.final_pgo_optimization_solver_params_);
  }

  bool operator!=(const PoseGraphPlusObjectsOptimizationParams &rhs) const {
    return !operator==(rhs);
  }
};
}  // namespace pose_graph_optimization

#endif  // UT_VSLAM_OPTIMIZATION_SOLVER_PARAMS_H
