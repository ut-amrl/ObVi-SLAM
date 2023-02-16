// Copyright 2019 kvedder@seas.upenn.edu
// School of Engineering and Applied Sciences,
// University of Pennsylvania
//
//
// This software is free: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License Version 3,
// as published by the Free Software Foundation.
//
// This software is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// Version 3 in the file COPYING that came with this distribution.
// If not, see <http://www.gnu.org/licenses/>.
// ========================================================================

#include <file_io/cv_file_storage/config_file_storage_io.h>
#include <gtest/gtest.h>

#include <filesystem>

using namespace vslam_types_refactor;
namespace fs = std::filesystem;

TEST(FullOVSLAMConfigIO, ReadWriteFullOBSLAMConfig) {
  FullOVSLAMConfig orig_config;

  // Populate the config
  orig_config.config_schema_version_ = kCurrentConfigSchemaVersion;
  orig_config.config_version_id_ = "test_version";
  VisualFeatureParams visual_feature_params;
  visual_feature_params.min_visual_feature_parallax_robot_orient_requirement_ =
      1.2;
  visual_feature_params.min_visual_feature_parallax_robot_transl_requirement_ =
      1.3;
  visual_feature_params.min_visual_feature_parallax_pixel_requirement_ = 1.4;
  visual_feature_params.reprojection_error_std_dev_ = 1.5;
  orig_config.visual_feature_params_ = visual_feature_params;

  pose_graph_optimization::OptimizationSolverParams local_ba_solver_params;
  local_ba_solver_params.allow_non_monotonic_steps_ = true;
  local_ba_solver_params.max_num_iterations_ = 5;
  local_ba_solver_params.parameter_tolerance_ = 1e6;
  local_ba_solver_params.gradient_tolerance_ = 2e5;
  local_ba_solver_params.function_tolerance_ = 4.3e2;
  local_ba_solver_params.feature_outlier_percentage = 0.8;
  orig_config.local_ba_solver_params_ = local_ba_solver_params;

  pose_graph_optimization::OptimizationSolverParams global_ba_solver_params;
  global_ba_solver_params.allow_non_monotonic_steps_ = false;
  global_ba_solver_params.max_num_iterations_ = 60;
  global_ba_solver_params.parameter_tolerance_ = 2e6;
  global_ba_solver_params.gradient_tolerance_ = 7.5e5;
  global_ba_solver_params.function_tolerance_ = -9e2;
  global_ba_solver_params.feature_outlier_percentage = 0.7;
  orig_config.global_ba_solver_params_ = global_ba_solver_params;

  pose_graph_optimization::OptimizationSolverParams final_ba_solver_params;
  final_ba_solver_params.allow_non_monotonic_steps_ = false;
  final_ba_solver_params.max_num_iterations_ = 60;
  final_ba_solver_params.parameter_tolerance_ = 3e6;
  final_ba_solver_params.gradient_tolerance_ = -6.4e7;
  final_ba_solver_params.function_tolerance_ = 7e2;
  final_ba_solver_params.feature_outlier_percentage = 0;
  orig_config.final_ba_solver_params_ = final_ba_solver_params;

  LongTermMapExtractionTunableParams ltm_tunable_params;
  ltm_tunable_params.far_feature_threshold_ = 2.4;
  orig_config.ltm_tunable_params_ = ltm_tunable_params;

  pose_graph_optimization::ObjectVisualPoseGraphResidualParams
      ltm_solver_residual_params;
  ltm_solver_residual_params.object_residual_params_
      .object_observation_huber_loss_param_ = 1e3;
  ltm_solver_residual_params.object_residual_params_
      .shape_dim_prior_factor_huber_loss_param_ = 2e4;
  ltm_solver_residual_params.object_residual_params_
      .invalid_ellipsoid_error_val_ = -1e4;
  ltm_solver_residual_params.long_term_map_params_.pair_huber_loss_param_ =
      5e-3;
  ltm_solver_residual_params.visual_residual_params_
      .reprojection_error_huber_loss_param_ = 4.2e3;
  orig_config.ltm_solver_residual_params_ = ltm_solver_residual_params;

  pose_graph_optimization::OptimizationSolverParams ltm_solver_params;
  ltm_solver_params.feature_outlier_percentage = 0.1;
  ltm_solver_params.function_tolerance_ = 1e3;
  ltm_solver_params.gradient_tolerance_ = -2e4;
  ltm_solver_params.parameter_tolerance_ = 3e-4;
  ltm_solver_params.max_num_iterations_ = 20;
  ltm_solver_params.allow_non_monotonic_steps_ = false;
  orig_config.ltm_solver_params_ = ltm_solver_params;

  ShapeDimensionPriors shape_dimension_priors;
  Covariance<double, 3> bench_cov;
  bench_cov << 1.2, 2.3, 3.4, 4.5, 5.6, 6.7, 7.8, 8.9, 9.0;
  Covariance<double, 3> trashcan_cov;
  trashcan_cov << 1.4, 2.5, 3.6, 4.7, 5.8, 6.9, 7.0, 8.1, 9.2;
  shape_dimension_priors.mean_and_cov_by_semantic_class_ = {
      {"bench", std::make_pair(ObjectDim<double>(1.3, 3.5, 5.7), bench_cov)},
      {"trashcan",
       std::make_pair(ObjectDim<double>(2.4, 4.6, 6.8), trashcan_cov)}};
  orig_config.shape_dimension_priors_ = shape_dimension_priors;

  CameraInfo camera_info;
  camera_info.camera_topic_to_camera_id_["zed1"] = 0;
  camera_info.camera_topic_to_camera_id_["zed2"] = 1;
  camera_info.camera_topic_to_camera_id_["zed1_compressed"] = 0;
  camera_info.camera_topic_to_camera_id_["zed2_compressed"] = 1;
  orig_config.camera_info_ = camera_info;

  BoundingBoxFrontEndParams bounding_box_front_end_params;
  bounding_box_front_end_params.geometric_similarity_scorer_params_
      .max_merge_distance_ = 2.3;
  bounding_box_front_end_params.feature_based_bb_association_params_
      .min_overlapping_features_for_match_ = 21;
  bounding_box_front_end_params.feature_based_bb_association_params_
      .bounding_box_inflation_size_ = 13;
  bounding_box_front_end_params.feature_based_bb_association_params_
      .feature_validity_window_ = 24;
  bounding_box_front_end_params.feature_based_bb_association_params_
      .min_bb_confidence_ = 0.234;
  bounding_box_front_end_params.feature_based_bb_association_params_
      .min_observations_for_local_est_ = 43;
  bounding_box_front_end_params.feature_based_bb_association_params_
      .min_observations_ = 50;
  bounding_box_front_end_params.feature_based_bb_association_params_
      .discard_candidate_after_num_frames_ = 41;
  bounding_box_front_end_params.feature_based_bb_association_params_
      .required_min_conf_for_initialization_ = 0.49;

  PendingObjectEstimatorParams pending_est_params;
  pending_est_params.object_residual_params_
      .object_observation_huber_loss_param_ = 12;
  pending_est_params.object_residual_params_
      .shape_dim_prior_factor_huber_loss_param_ = 0.23;
  pending_est_params.object_residual_params_.invalid_ellipsoid_error_val_ =
      1.5e-4;
  pending_est_params.solver_params_.parameter_tolerance_ = 3.4e2;
  pending_est_params.solver_params_.gradient_tolerance_ = -24.3;
  pending_est_params.solver_params_.function_tolerance_ = -2e-3;
  pending_est_params.solver_params_.allow_non_monotonic_steps_ = true;
  pending_est_params.solver_params_.max_num_iterations_ = 12;
  pending_est_params.solver_params_.feature_outlier_percentage = 0.978;
  bounding_box_front_end_params.feature_based_bb_association_params_
      .pending_obj_estimator_params_ = pending_est_params;
  orig_config.bounding_box_front_end_params_ = bounding_box_front_end_params;

  BoundingBoxCovGenParams bounding_box_covariance_generator_params;
  Covariance<double, 4> bb_cov;
  bb_cov << 1.5, 2.6, 3.7, 4.8, 11.15, 12.16, 13.17, 14.18, 1.25, 2.26, 3.27,
      4.28, 51.5, 52.6, 53.7, 54.8;
  bounding_box_covariance_generator_params.bounding_box_cov_ = bb_cov;
  bounding_box_covariance_generator_params.image_boundary_variance_ = 240;
  bounding_box_covariance_generator_params.near_edge_threshold_ = 32;
  orig_config.bounding_box_covariance_generator_params_ =
      bounding_box_covariance_generator_params;

  SlidingWindowParams sliding_window_params;
  sliding_window_params.local_ba_window_size_ = 40;
  sliding_window_params.global_ba_frequency_ = 24;
  orig_config.sliding_window_params_ = sliding_window_params;

  // The boolean params here should only be altered for debugging
  // In normal circumstances, only the integer parameters should be altered
  // from their default values in the configuration
  pose_graph_optimizer::OptimizationFactorsEnabledParams
      optimization_factors_enabled_params;
  optimization_factors_enabled_params.include_object_factors_ = false;
  optimization_factors_enabled_params.min_object_observations_ = 2;
  optimization_factors_enabled_params.min_low_level_feature_observations_ = 32;
  optimization_factors_enabled_params.fix_visual_features_ = true;
  optimization_factors_enabled_params.include_visual_factors_ = false;
  optimization_factors_enabled_params.poses_prior_to_window_to_keep_constant_ =
      2;
  optimization_factors_enabled_params.fix_poses_ = true;
  optimization_factors_enabled_params.fix_ltm_objects_ = false;
  optimization_factors_enabled_params.fix_objects_ = true;
  optimization_factors_enabled_params.use_pom_ = false;
  orig_config.optimization_factors_enabled_params_ =
      optimization_factors_enabled_params;

  pose_graph_optimization::ObjectVisualPoseGraphResidualParams
      object_visual_pose_graph_residual_params;
  object_visual_pose_graph_residual_params.object_residual_params_
      .object_observation_huber_loss_param_ = 3.2;
  object_visual_pose_graph_residual_params.object_residual_params_
      .shape_dim_prior_factor_huber_loss_param_ = 2.42;
  object_visual_pose_graph_residual_params.object_residual_params_
      .invalid_ellipsoid_error_val_ = 1e-4;
  object_visual_pose_graph_residual_params.visual_residual_params_
      .reprojection_error_huber_loss_param_ = 9.3e-3;
  object_visual_pose_graph_residual_params.long_term_map_params_
      .pair_huber_loss_param_ = 3.3;
  orig_config.object_visual_pose_graph_residual_params_ =
      object_visual_pose_graph_residual_params;

  LimitTrajectoryEvaluationParams limit_traj_eval_params;
  limit_traj_eval_params.should_limit_trajectory_evaluation_ = false;
  limit_traj_eval_params.max_frame_id_ = 10;
  orig_config.limit_traj_eval_params_ = limit_traj_eval_params;

  SparsifierParams sparsifier_params;
  sparsifier_params.max_pose_inc_threshold_rot_ = 12;
  sparsifier_params.max_pose_inc_threshold_transl_ = 2.3;
  orig_config.sparsifier_params_ = sparsifier_params;

  orig_config.sliding_window_params_.local_ba_window_size_ = 10;

  std::FILE *tmp_file = std::tmpfile();
  std::string tmp_file_name = fs::read_symlink(
      fs::path("/proc/self/fd") / std::to_string(fileno(tmp_file)));
  writeConfiguration(tmp_file_name, orig_config);
  FullOVSLAMConfig read_config;
  readConfiguration(tmp_file_name, read_config);
  ASSERT_EQ(orig_config, read_config);
}