//
// Created by amanda on 2/10/23.
//

#ifndef UT_VSLAM_FULLOVSLAMCONFIG_H
#define UT_VSLAM_FULLOVSLAMCONFIG_H

#include <refactoring/bounding_box_frontend/bounding_box_front_end_creation_utils.h>
#include <refactoring/bounding_box_frontend/feature_based_bounding_box_front_end.h>
#include <refactoring/long_term_map/long_term_map_extraction_tunable_params.h>
#include <refactoring/offline/limit_trajectory_evaluation_params.h>
#include <refactoring/optimization/optimization_factors_enabled_params.h>
#include <refactoring/optimization/optimization_solver_params.h>
#include <refactoring/types/vslam_obj_opt_types_refactor.h>

#include <string>
#include <unordered_map>
#include <utility>

namespace vslam_types_refactor {

// NOTE: This should be incremented every time the format of the configuration
// data changes
const static int kCurrentConfigSchemaVersion = 3;

struct VisualFeatureParams {
  double reprojection_error_std_dev_;
  double min_visual_feature_parallax_pixel_requirement_;
  double min_visual_feature_parallax_robot_transl_requirement_;
  double min_visual_feature_parallax_robot_orient_requirement_;
  bool enforce_min_pixel_parallax_requirement_ = true;
  bool enforce_min_robot_pose_parallax_requirement_ = true;
  bool operator==(const VisualFeatureParams &rhs) const {
    const double kEpsilon = 1e-8;  // TODO FIXME
    return ((reprojection_error_std_dev_ - rhs.reprojection_error_std_dev_ <
             kEpsilon) &&
            (min_visual_feature_parallax_pixel_requirement_ -
                 rhs.min_visual_feature_parallax_pixel_requirement_ <
             kEpsilon) &&
            (min_visual_feature_parallax_robot_transl_requirement_ -
                 rhs.min_visual_feature_parallax_robot_transl_requirement_ <
             kEpsilon) &&
            (min_visual_feature_parallax_robot_orient_requirement_ -
                 rhs.min_visual_feature_parallax_robot_orient_requirement_ <
             kEpsilon) &&
            (enforce_min_pixel_parallax_requirement_ ==
             rhs.enforce_min_pixel_parallax_requirement_) &&
            (enforce_min_robot_pose_parallax_requirement_ ==
             rhs.enforce_min_robot_pose_parallax_requirement_));
    // return (reprojection_error_std_dev_ == rhs.reprojection_error_std_dev_)
    // &&
    //        (min_visual_feature_parallax_pixel_requirement_ ==
    //         rhs.min_visual_feature_parallax_pixel_requirement_) &&
    //        (min_visual_feature_parallax_robot_transl_requirement_ ==
    //         rhs.min_visual_feature_parallax_robot_transl_requirement_) &&
    //        (min_visual_feature_parallax_robot_orient_requirement_ ==
    //         rhs.min_visual_feature_parallax_robot_orient_requirement_);
  }

  bool operator!=(const VisualFeatureParams &rhs) const {
    return !operator==(rhs);
  }
};

struct ShapeDimensionPriors {
  std::unordered_map<std::string,
                     std::pair<ObjectDim<double>, Covariance<double, 3>>>
      mean_and_cov_by_semantic_class_;

  bool operator==(const ShapeDimensionPriors &rhs) const {
    return (mean_and_cov_by_semantic_class_ ==
            rhs.mean_and_cov_by_semantic_class_);
  }

  bool operator!=(const ShapeDimensionPriors &rhs) const {
    return !operator==(rhs);
  }
};

struct CameraInfo {
  std::unordered_map<std::string, CameraId> camera_topic_to_camera_id_;

  bool operator==(const CameraInfo &rhs) const {
    return (camera_topic_to_camera_id_ == rhs.camera_topic_to_camera_id_);
  }

  bool operator!=(const CameraInfo &rhs) const { return !operator==(rhs); }
};

struct SlidingWindowParams {
  FrameId global_ba_frequency_;
  FrameId local_ba_window_size_;

  bool operator==(const SlidingWindowParams &rhs) const {
    return (global_ba_frequency_ == rhs.global_ba_frequency_) &&
           (local_ba_window_size_ == rhs.local_ba_window_size_);
  }

  bool operator!=(const SlidingWindowParams &rhs) const {
    return !operator==(rhs);
  }
};

struct BoundingBoxFrontEndParams {
  GeometricSimilarityScorerParams geometric_similarity_scorer_params_;
  FeatureBasedBbAssociationParams feature_based_bb_association_params_;

  bool operator==(const BoundingBoxFrontEndParams &rhs) const {
    return (geometric_similarity_scorer_params_ ==
            rhs.geometric_similarity_scorer_params_) &&
           (feature_based_bb_association_params_ ==
            rhs.feature_based_bb_association_params_);
  }

  bool operator!=(const BoundingBoxFrontEndParams &rhs) const {
    return !operator==(rhs);
  }
};

struct SparsifierParams {
  double max_pose_inc_threshold_transl_ = 0.2;
  double max_pose_inc_threshold_rot_ = 0.1;  // TODO?

  bool operator==(const SparsifierParams &rhs) const {
    return (max_pose_inc_threshold_transl_ ==
            rhs.max_pose_inc_threshold_transl_) &&
           (max_pose_inc_threshold_rot_ == rhs.max_pose_inc_threshold_rot_);
  }

  bool operator!=(const SparsifierParams &rhs) const {
    return !operator==(rhs);
  }
};

struct FullOVSLAMConfig {
  // DO NOT OVERWRITE THIS WHEN WRITING CONFIG
  int config_schema_version_ = kCurrentConfigSchemaVersion;

  // NOTE: Generally, this should be an integer that is incremented with every
  // config update. See write_configuration for more details
  std::string config_version_id_;

  VisualFeatureParams visual_feature_params_;

  pose_graph_optimization::OptimizationSolverParams local_ba_solver_params_;
  pose_graph_optimization::OptimizationSolverParams global_ba_solver_params_;
  pose_graph_optimization::OptimizationSolverParams final_ba_solver_params_;

  LongTermMapExtractionTunableParams ltm_tunable_params_;
  pose_graph_optimization::ObjectVisualPoseGraphResidualParams
      ltm_solver_residual_params_;
  pose_graph_optimization::OptimizationSolverParams ltm_solver_params_;

  ShapeDimensionPriors shape_dimension_priors_;

  CameraInfo camera_info_;

  BoundingBoxFrontEndParams bounding_box_front_end_params_;
  BoundingBoxCovGenParams bounding_box_covariance_generator_params_;

  SlidingWindowParams sliding_window_params_;

  // The boolean params here should only be altered for debugging
  // In normal circumstances, only the integer parameters should be altered
  // from their default values in the configuration
  pose_graph_optimizer::OptimizationFactorsEnabledParams
      optimization_factors_enabled_params_;

  pose_graph_optimization::ObjectVisualPoseGraphResidualParams
      object_visual_pose_graph_residual_params_;

  LimitTrajectoryEvaluationParams limit_traj_eval_params_;

  SparsifierParams sparsifier_params_;

  // (Maybe add later) ORB-SLAM params (if we want to include)
  //  nFeatures
  //  scaleFactor
  //  nLevels

  // TODO should we make switching to roshan + the roshan params configurable?
  // Not going to bother for now I think

  bool operator==(const FullOVSLAMConfig &rhs) const {
    return (config_schema_version_ == rhs.config_schema_version_) &&
           (config_version_id_ == rhs.config_version_id_) &&
           (visual_feature_params_ == rhs.visual_feature_params_) &&
           (local_ba_solver_params_ == rhs.local_ba_solver_params_) &&
           (global_ba_solver_params_ == rhs.global_ba_solver_params_) &&
           (final_ba_solver_params_ == rhs.final_ba_solver_params_) &&
           (ltm_tunable_params_ == rhs.ltm_tunable_params_) &&
           (ltm_solver_residual_params_ == rhs.ltm_solver_residual_params_) &&
           (ltm_solver_params_ == rhs.ltm_solver_params_) &&
           (shape_dimension_priors_ == rhs.shape_dimension_priors_) &&
           (camera_info_ == rhs.camera_info_) &&
           (bounding_box_front_end_params_ ==
            rhs.bounding_box_front_end_params_) &&
           (bounding_box_covariance_generator_params_ ==
            rhs.bounding_box_covariance_generator_params_) &&
           (sliding_window_params_ == rhs.sliding_window_params_) &&
           (optimization_factors_enabled_params_ ==
            rhs.optimization_factors_enabled_params_) &&
           (object_visual_pose_graph_residual_params_ ==
            rhs.object_visual_pose_graph_residual_params_) &&
           (limit_traj_eval_params_ == rhs.limit_traj_eval_params_) &&
           (sparsifier_params_ == rhs.sparsifier_params_);
  }

  bool operator!=(const FullOVSLAMConfig &rhs) const {
    return !operator==(rhs);
  }
};
}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_FULLOVSLAMCONFIG_H
