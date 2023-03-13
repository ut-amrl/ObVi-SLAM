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
  ceres::Problem problem;
  for (const RelativePoseFactorInfoWithFrames &factor : relative_pose_factors) {
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

  pose_graph_optimizer::OptimizationScopeParams
      optimization_scope_params_for_pgo = optimization_scope_params;
  optimization_scope_params_for_pgo.include_visual_factors_ = false;
  optimization_scope_params_for_pgo.poses_prior_to_window_to_keep_constant_ = 1;

  if (opt_logger.has_value()) {
    opt_logger->setOptimizationTypeParams(max_frame_id, false, true, true);
  }
  optimizer.buildPoseGraphOptimization(optimization_scope_params_for_pgo,
                                       residual_params,
                                       pose_graph,
                                       &problem,
                                       opt_logger);

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

  // TODO do we need to readjust visual features after optimization

  if (opt_logger.has_value()) {
    opt_logger->writeCurrentOptInfo();
  }
  return true;
}
}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_POSE_GRAPH_PLUS_OBJECTS_OPTIMIZER_H
