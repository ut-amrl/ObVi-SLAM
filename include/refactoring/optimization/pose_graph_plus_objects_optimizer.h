//
// Created by amanda on 3/2/23.
//

#ifndef UT_VSLAM_POSE_GRAPH_PLUS_OBJECTS_OPTIMIZER_H
#define UT_VSLAM_POSE_GRAPH_PLUS_OBJECTS_OPTIMIZER_H

#include <ceres/problem.h>
#include <refactoring/factors/relative_pose_factor.h>
#include <refactoring/factors/relative_pose_factor_utils.h>
#include <refactoring/optimization/object_pose_graph.h>
#include <refactoring/optimization/object_pose_graph_optimizer.h>

namespace vslam_types_refactor {

struct RelativePoseFactorInfoWithFrames {
  Pose3D<double> measured_pose_deviation_;
  Covariance<double, 6> pose_deviation_cov_;
  FrameId before_pose_frame_id_;
  FrameId after_pose_frame_id_;
};

template <typename PoseGraphType, typename CachedFactorInfo>
bool runPgoPlusEllipsoids(
    const FrameId &max_frame_id,
    const std::function<bool(
        const std::pair<vslam_types_refactor::FactorType,
                        vslam_types_refactor::FeatureFactorId> &,
        const pose_graph_optimization::ObjectVisualPoseGraphResidualParams &,
        const std::shared_ptr<PoseGraphType> &,
        ceres::Problem *,
        ceres::ResidualBlockId &,
        CachedFactorInfo &)> &residual_creator,
    const pose_graph_optimizer::OptimizationScopeParams
        &optimization_scope_params,
    const std::vector<std::shared_ptr<ceres::IterationCallback>>
        &ceres_callbacks,
    const pose_graph_optimization::ObjectVisualPoseGraphResidualParams
        &residual_params,
    const pose_graph_optimization::PoseGraphPlusObjectsOptimizationParams
        &pgo_solver_params,
    const std::vector<RelativePoseFactorInfoWithFrames>
        &non_local_relative_pose_factors,  // Loop closure
    const bool &final_run,
    std::optional<vslam_types_refactor::OptimizationLogger> &opt_logger,
    std::shared_ptr<PoseGraphType> &pose_graph) {
  // Construct set of relative pose factors
  std::vector<RelativePoseFactorInfoWithFrames> relative_pose_factors;
  relative_pose_factors.insert(relative_pose_factors.end(),
                               non_local_relative_pose_factors.begin(),
                               non_local_relative_pose_factors.end());

  std::unordered_map<FrameId, RawPose3d<double>> raw_robot_pose_estimates;
  pose_graph->getRobotPoseEstimates(raw_robot_pose_estimates);
  std::unordered_map<FeatureId, std::pair<FrameId, Position3d<double>>>
      relative_positions_from_first;

  // Create new optimizer with ellipsoid factors only (use_visual_features =
  // false) Have relative pose factors already in problem
  std::function<bool(
      const std::pair<vslam_types_refactor::FactorType,
                      vslam_types_refactor::FeatureFactorId> &,
      const std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph> &,
      const util::EmptyStruct &)>
      refresh_residual_checker =
          [](const std::pair<vslam_types_refactor::FactorType,
                             vslam_types_refactor::FeatureFactorId> &,
             const std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph> &,
             const util::EmptyStruct &) { return true; };

  pose_graph_optimizer::ObjectPoseGraphOptimizer<
      ReprojectionErrorFactor,
      util::EmptyStruct,
      ObjectAndReprojectionFeaturePoseGraph>
      optimizer(refresh_residual_checker, residual_creator);
  ceres::Problem problem;

  {
#ifdef RUN_TIMERS
    CumulativeFunctionTimer::Invocation build_pgo_invoc(
        CumulativeTimerFactory::getInstance()
            .getOrCreateFunctionTimer(kTimerNameObjOnlyPgoBuildPgo)
            .get());
#endif

    // TODO maybe at some point, we should have connections between other nearby
    // poses
    // Construct relative pose factors from local windows
    for (FrameId frame_num = 1; frame_num <= max_frame_id; frame_num++) {
      if (raw_robot_pose_estimates.find(frame_num) ==
          raw_robot_pose_estimates.end()) {
        LOG(ERROR) << "Could not find current estimate for frame num "
                   << frame_num;
        return false;
      }
      if (raw_robot_pose_estimates.find(frame_num - 1) ==
          raw_robot_pose_estimates.end()) {
        LOG(ERROR) << "Could not find current estimate for frame num "
                   << (frame_num - 1);
        return false;
      }

      Pose3D<double> before_pose =
          convertToPose3D(raw_robot_pose_estimates.at(frame_num - 1));
      Pose3D<double> after_pose =
          convertToPose3D(raw_robot_pose_estimates.at(frame_num));

      Pose3D<double> relative_pose =
          getPose2RelativeToPose1(before_pose, after_pose);

      RelativePoseFactorInfoWithFrames relative_info;
      relative_info.before_pose_frame_id_ = frame_num - 1;
      relative_info.after_pose_frame_id_ = frame_num;
      relative_info.measured_pose_deviation_ = relative_pose;
      relative_info.pose_deviation_cov_ =
          generateOdomCov(relative_pose,
                          pgo_solver_params.relative_pose_cov_params_
                              .transl_error_mult_for_transl_error_,
                          pgo_solver_params.relative_pose_cov_params_
                              .transl_error_mult_for_rot_error_,
                          pgo_solver_params.relative_pose_cov_params_
                              .rot_error_mult_for_transl_error_,
                          pgo_solver_params.relative_pose_cov_params_
                              .rot_error_mult_for_rot_error_);
      relative_pose_factors.emplace_back(relative_info);
    }

    // Add to ceres problem
    for (const RelativePoseFactorInfoWithFrames &factor :
         relative_pose_factors) {
      double *before_pose_block;
      double *after_pose_block;
      if (!pose_graph_optimizer::getParamBlockForPose(
              factor.before_pose_frame_id_, pose_graph, &before_pose_block)) {
        LOG(ERROR) << "Could not find parameter block for frame "
                   << factor.before_pose_frame_id_;
        return false;
      }

      if (!pose_graph_optimizer::getParamBlockForPose(
              factor.after_pose_frame_id_, pose_graph, &after_pose_block)) {
        LOG(ERROR) << "Could not find parameter block for frame "
                   << factor.after_pose_frame_id_;
        return false;
      }

      problem.AddResidualBlock(
          RelativePoseFactor::createRelativePoseFactor(
              factor.measured_pose_deviation_, factor.pose_deviation_cov_),
          new ceres::HuberLoss(
              pgo_solver_params.relative_pose_factor_huber_loss_),
          before_pose_block,
          after_pose_block);
    }

    pose_graph_optimizer::OptimizationScopeParams
        optimization_scope_params_for_pgo = optimization_scope_params;
    optimization_scope_params_for_pgo.include_visual_factors_ = false;
    optimization_scope_params_for_pgo.poses_prior_to_window_to_keep_constant_ =
        1;

    if (pgo_solver_params.enable_visual_non_opt_feature_adjustment_post_pgo_) {
      std::unordered_map<FrameId, RawPose3d<double>> robot_pose_estimates_raw;
      std::unordered_map<FrameId, Pose3D<double>> robot_pose_estimates;
      pose_graph->getRobotPoseEstimates(robot_pose_estimates_raw);
      for (const auto &robot_pose_raw : robot_pose_estimates_raw) {
        robot_pose_estimates[robot_pose_raw.first] =
            convertToPose3D<double>(robot_pose_raw.second);
      }

      std::unordered_map<FeatureId, Position3d<double>>
          visual_feature_estimates;
      pose_graph->getVisualFeatureEstimates(visual_feature_estimates);

      for (const auto &visual_feat : visual_feature_estimates) {
        FrameId first_obs;
        if (pose_graph->getFirstObservedFrameForFeature(visual_feat.first,
                                                        first_obs)) {
          if (robot_pose_estimates.find(first_obs) ==
              robot_pose_estimates.end()) {
            LOG(ERROR) << "Could not find estimate for frame " << first_obs
                       << " even though there were features observed from that "
                          "pose; not adjusting feature "
                       << visual_feat.first;
            continue;
          }
          relative_positions_from_first[visual_feat.first] = std::make_pair(
              first_obs,
              getPositionRelativeToPose(robot_pose_estimates[first_obs],
                                        visual_feat.second));
        }
      }
    }

    if (opt_logger.has_value()) {
      opt_logger->setOptimizationTypeParams(max_frame_id, false, true, true);
    }
    optimizer.buildPoseGraphOptimization(optimization_scope_params_for_pgo,
                                         residual_params,
                                         pose_graph,
                                         &problem,
                                         opt_logger);
  }
  {
#ifdef RUN_TIMERS
    CumulativeFunctionTimer::Invocation solve_pgo_invoc(
        CumulativeTimerFactory::getInstance()
            .getOrCreateFunctionTimer(kTimerNameObjOnlyPgoSolvePgo)
            .get());
#endif
    // Run optimization
    if (!optimizer.solveOptimization(
            &problem,
            final_run ? pgo_solver_params.final_pgo_optimization_solver_params_
                      : pgo_solver_params.pgo_optimization_solver_params_,
            ceres_callbacks,
            opt_logger,
            nullptr)) {
      // TODO do we want to quit or just silently let this iteration fail?
      LOG(ERROR) << "Pose-graph + object optimization failed at max frame id "
                 << max_frame_id;
      return false;
    }
  }
  if (opt_logger.has_value()) {
    opt_logger->writeCurrentOptInfo();
  }

  if (pgo_solver_params.enable_visual_non_opt_feature_adjustment_post_pgo_) {
#ifdef RUN_TIMERS
    CumulativeFunctionTimer::Invocation non_opt_vf_adjust_invoc(
        CumulativeTimerFactory::getInstance()
            .getOrCreateFunctionTimer(kTimerNameObjOnlyPgoManualFeatAdjust)
            .get());
#endif
    if (opt_logger.has_value()) {
      opt_logger->setOptimizationTypeParams(max_frame_id, true, true, false);
    }

    std::unordered_map<FeatureId, VisualFeatureNode> feature_nodes;
    pose_graph->getFeaturePositionPtrs(feature_nodes);

    std::unordered_map<FrameId, RawPose3d<double>> robot_pose_estimates_raw;
    std::unordered_map<FrameId, Pose3D<double>> robot_pose_estimates;
    pose_graph->getRobotPoseEstimates(robot_pose_estimates_raw);
    for (const auto &robot_pose_raw : robot_pose_estimates_raw) {
      robot_pose_estimates[robot_pose_raw.first] =
          convertToPose3D<double>(robot_pose_raw.second);
    }

    for (auto &feat : feature_nodes) {
      if (relative_positions_from_first.find(feat.first) ==
          relative_positions_from_first.end()) {
        LOG(ERROR) << "Did not have adjustment data for feature " << feat.first
                   << "; skipping adjustment";
        continue;
      }

      std::pair<FrameId, Position3d<double>> first_obs_info =
          relative_positions_from_first.at(feat.first);
      if (robot_pose_estimates.find(first_obs_info.first) ==
          robot_pose_estimates.end()) {
        LOG(ERROR) << "Could not find robot pose " << first_obs_info.first
                   << "; not adjusting feature " << feat.first;
        continue;
      }
      Position3d<double> new_position = combinePoseAndPosition(
          robot_pose_estimates.at(first_obs_info.first), first_obs_info.second);
      feat.second.updateVisualPositionParams(new_position);
    }
  }
  if (pgo_solver_params.enable_visual_feats_only_opt_post_pgo_) {
    pose_graph_optimizer::OptimizationScopeParams
        optimization_scope_params_for_vf_adjustment = optimization_scope_params;
    optimization_scope_params_for_vf_adjustment.fix_poses_ = true;
    optimization_scope_params_for_vf_adjustment.fix_objects_ = true;
    optimization_scope_params_for_vf_adjustment.include_object_factors_ = false;
    {
#ifdef RUN_TIMERS
      CumulativeFunctionTimer::Invocation opt_vf_adjust_build_invoc(
          CumulativeTimerFactory::getInstance()
              .getOrCreateFunctionTimer(kTimerNameObjOnlyPgoOptFeatAdjustBuild)
              .get());
#endif
      optimizer.buildPoseGraphOptimization(
          optimization_scope_params_for_vf_adjustment,
          residual_params,
          pose_graph,
          &problem,
          opt_logger);
    }
    {
#ifdef RUN_TIMERS
      CumulativeFunctionTimer::Invocation opt_vf_adjust_solve_invoc(
          CumulativeTimerFactory::getInstance()
              .getOrCreateFunctionTimer(kTimerNameObjOnlyPgoOptFeatAdjustSolve)
              .get());
#endif
      // Run optimization
      if (!optimizer.solveOptimization(
              &problem,
              final_run
                  ? pgo_solver_params.final_pgo_optimization_solver_params_
                  : pgo_solver_params.pgo_optimization_solver_params_,
              ceres_callbacks,
              opt_logger,
              nullptr)) {
        // TODO do we want to quit or just silently let this iteration fail?
        LOG(ERROR) << "Visual feature adjustment after pose-graph optimization "
                      "failed at max frame id "
                   << max_frame_id;
        return false;
      }
    }
    if (opt_logger.has_value()) {
      opt_logger->writeCurrentOptInfo();
    }
  }

  return true;
}
}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_POSE_GRAPH_PLUS_OBJECTS_OPTIMIZER_H
