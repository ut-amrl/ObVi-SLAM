//
// Created by amanda on 6/17/22.
//

#ifndef UT_VSLAM_OFFLINE_PROBLEM_RUNNER_H
#define UT_VSLAM_OFFLINE_PROBLEM_RUNNER_H

#include <ceres/problem.h>
#include <refactoring/offline/limit_trajectory_evaluation_params.h>
#include <refactoring/optimization/object_pose_graph.h>
#include <refactoring/optimization/object_pose_graph_optimizer.h>

namespace vslam_types_refactor {

enum VisualizationTypeEnum {
  BEFORE_ANY_OPTIMIZATION,
  BEFORE_EACH_OPTIMIZATION,
  AFTER_EACH_OPTIMIZATION,
  AFTER_ALL_OPTIMIZATION
};

template <typename InputProblemData,
          typename VisualFeatureFactorType,
          typename OutputProblemData,
          typename CachedFactorInfo,
          typename PoseGraphType>
class OfflineProblemRunner {
 public:
  OfflineProblemRunner(
      const pose_graph_optimization::ObjectVisualPoseGraphResidualParams
          &residual_params,
      const LimitTrajectoryEvaluationParams &limit_trajectory_eval_params,
      const std::function<bool()> &continue_opt_checker,
      const std::function<FrameId(const FrameId &)> &window_provider_func,
      const std::function<
          bool(const std::pair<vslam_types_refactor::FactorType,
                               vslam_types_refactor::FeatureFactorId> &,
               const std::shared_ptr<PoseGraphType> &,
               const CachedFactorInfo &)> &refresh_residual_checker,
      const std::function<bool(
          const std::pair<vslam_types_refactor::FactorType,
                          vslam_types_refactor::FeatureFactorId> &,
          const pose_graph_optimization::ObjectVisualPoseGraphResidualParams &,
          const std::shared_ptr<PoseGraphType> &,
          ceres::Problem *,
          ceres::ResidualBlockId &,
          CachedFactorInfo &)> &residual_creator,
      const std::function<void(const InputProblemData &,
                               std::shared_ptr<PoseGraphType> &)>
          &pose_graph_creator,
      const std::function<void(const InputProblemData &,
                               const std::shared_ptr<PoseGraphType> &,
                               const FrameId &,
                               const FrameId &)> &frame_data_adder,
      const std::function<
          void(const InputProblemData &,
               const std::shared_ptr<PoseGraphType> &,
               const pose_graph_optimizer::OptimizationFactorsEnabledParams &,
               OutputProblemData &)> &output_data_extractor,
      const std::function<
          std::vector<std::shared_ptr<ceres::IterationCallback>>(
              const InputProblemData &,
              const std::shared_ptr<PoseGraphType> &,
              const FrameId &,
              const FrameId &)> &ceres_callback_creator,
      const std::function<void(const InputProblemData &,
                               const std::shared_ptr<PoseGraphType> &,
                               const FrameId &,
                               const FrameId &,
                               const VisualizationTypeEnum &)>
          &visualization_callback,
      const std::function<pose_graph_optimization::OptimizationSolverParams(
          const FrameId &)> &solver_params_provider_func)
      : residual_params_(residual_params),
        limit_trajectory_eval_params_(limit_trajectory_eval_params),
        continue_opt_checker_(continue_opt_checker),
        window_provider_func_(window_provider_func),
        optimizer_(refresh_residual_checker, residual_creator),
        pose_graph_creator_(pose_graph_creator),
        frame_data_adder_(frame_data_adder),
        output_data_extractor_(output_data_extractor),
        ceres_callback_creator_(ceres_callback_creator),
        visualization_callback_(visualization_callback),
        solver_params_provider_func_(solver_params_provider_func) {}

  bool runOptimization(
      const InputProblemData &problem_data,
      const pose_graph_optimizer::OptimizationFactorsEnabledParams
          &optimization_factors_enabled_params,
      OutputProblemData &output_problem_data) {
    std::shared_ptr<PoseGraphType> pose_graph;

    ceres::Problem problem;
    LOG(INFO) << "Running pose graph creator";
    pose_graph_creator_(problem_data, pose_graph);
    frame_data_adder_(problem_data, pose_graph, 0, 0);
    FrameId max_frame_id = problem_data.getMaxFrameId();

    if (limit_trajectory_eval_params_.should_limit_trajectory_evaluation_) {
      max_frame_id =
          std::min(limit_trajectory_eval_params_.max_frame_id_, max_frame_id);
    }

    pose_graph_optimizer::OptimizationScopeParams optimization_scope_params;
    optimization_scope_params.allow_reversion_after_dectecting_jumps_ =
        optimization_factors_enabled_params
            .allow_reversion_after_dectecting_jumps_;
    optimization_scope_params.consecutive_pose_transl_tol_ =
        optimization_factors_enabled_params.consecutive_pose_transl_tol_;
    optimization_scope_params.consecutive_pose_orient_tol_ =
        optimization_factors_enabled_params.consecutive_pose_orient_tol_;
    optimization_scope_params.fix_poses_ =
        optimization_factors_enabled_params.fix_poses_;
    optimization_scope_params.fix_objects_ =
        optimization_factors_enabled_params.fix_objects_;
    optimization_scope_params.fix_visual_features_ =
        optimization_factors_enabled_params.fix_visual_features_;
    optimization_scope_params.fix_ltm_objects_ =
        optimization_factors_enabled_params.fix_ltm_objects_;
    optimization_scope_params.include_visual_factors_ =
        optimization_factors_enabled_params.include_visual_factors_;
    optimization_scope_params.include_object_factors_ =
        optimization_factors_enabled_params.include_object_factors_;
    optimization_scope_params.use_pom_ =
        optimization_factors_enabled_params.use_pom_;
    optimization_scope_params.poses_prior_to_window_to_keep_constant_ =
        optimization_factors_enabled_params
            .poses_prior_to_window_to_keep_constant_;
    optimization_scope_params.min_low_level_feature_observations_ =
        optimization_factors_enabled_params.min_low_level_feature_observations_;
    optimization_scope_params.min_object_observations_ =
        optimization_factors_enabled_params.min_object_observations_;

    visualization_callback_(problem_data,
                            pose_graph,
                            0,
                            max_frame_id,  // Should this be max or 0
                            VisualizationTypeEnum::BEFORE_ANY_OPTIMIZATION);
    LOG(INFO) << "Ready to run optimization";

    for (FrameId next_frame_id = 1; next_frame_id <= max_frame_id;
         next_frame_id++) {
      if (!continue_opt_checker_) {
        LOG(WARNING)
            << "Halted optimization due to continue checker reporting false";
        return false;
      }
      // This function is also responsible for adjusting the initial estimate
      // for the pose at next_frame_id given the optimized pose at next_frame_id
      // - 1
      FrameId start_opt_with_frame = window_provider_func_(next_frame_id);
      optimization_scope_params.min_frame_id_ = start_opt_with_frame;
      optimization_scope_params.max_frame_id_ = next_frame_id;
      frame_data_adder_(
          problem_data, pose_graph, start_opt_with_frame, next_frame_id);
      pose_graph_optimization::OptimizationSolverParams solver_params =
          solver_params_provider_func_(next_frame_id);

      // Phase I
      LOG(INFO) << "Building optimization";
      std::unordered_map<ceres::ResidualBlockId,
                         std::pair<vslam_types_refactor::FactorType,
                                   vslam_types_refactor::FeatureFactorId>>
          current_residual_block_info =
              optimizer_.buildPoseGraphOptimization(optimization_scope_params,
                                                    residual_params_,
                                                    pose_graph,
                                                    &problem);
      std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph> pose_graph_copy =
          pose_graph->makeDeepCopy();
      visualization_callback_(problem_data,
                              pose_graph,
                              start_opt_with_frame,
                              next_frame_id,
                              VisualizationTypeEnum::BEFORE_EACH_OPTIMIZATION);
      std::vector<std::shared_ptr<ceres::IterationCallback>> ceres_callbacks =
          ceres_callback_creator_(
              problem_data, pose_graph, start_opt_with_frame, next_frame_id);
      LOG(INFO) << "Solving optimization";
      std::shared_ptr<std::unordered_map<ceres::ResidualBlockId, double>>
          block_ids_and_residuals = std::make_shared<
              std::unordered_map<ceres::ResidualBlockId, double>>();
      if (!optimizer_.solveOptimization(&problem,
                                        solver_params,
                                        ceres_callbacks,
                                        block_ids_and_residuals)) {
        // TODO do we want to quit or just silently let this iteration fail?
        LOG(ERROR) << "Phase I Optimization failed at max frame id "
                   << next_frame_id;
        return false;
      }
      // Phase II
      if (solver_params.feature_outlier_percentage > 0) {
        // revert back to the old pose graph for Phase II optim
        pose_graph->setValuesFromAnotherPoseGraph(pose_graph_copy);
        // use map sort block ids by their corresponding residuals
        std::map<double, ceres::ResidualBlockId, std::greater<double>>
            ordered_residuals_and_block_ids;
        for (const auto &block_id_and_residual : *block_ids_and_residuals) {
          ordered_residuals_and_block_ids.insert(
              {block_id_and_residual.second, block_id_and_residual.first});
        }
        // build excluded_feature_factor_types_and_ids
        util::BoostHashSet<std::pair<FactorType, FeatureFactorId>>
            excluded_feature_factor_types_and_ids;
        size_t n_outliers = (size_t)(ordered_residuals_and_block_ids.size() *
                                     solver_params.feature_outlier_percentage);
        auto it = ordered_residuals_and_block_ids.begin();
        for (size_t i = 0; i < n_outliers; ++i) {
          const ceres::ResidualBlockId &block_id = it->second;
          if ((current_residual_block_info.at(block_id).first !=
               kLongTermMapFactorTypeId) &&
              (current_residual_block_info.at(block_id).first !=
               kShapeDimPriorFactorTypeId)) {
            excluded_feature_factor_types_and_ids.insert(
                current_residual_block_info.at(block_id));
          }
          ++it;
        }

        optimizer_.buildPoseGraphOptimization(
            optimization_scope_params,
            residual_params_,
            pose_graph,
            &problem,
            excluded_feature_factor_types_and_ids);

        if (!optimizer_.solveOptimization(
                &problem, solver_params, ceres_callbacks, nullptr)) {
          // TODO do we want to quit or just silently let this iteration fail?
          LOG(ERROR) << "Phase II Optimization failed at max frame id "
                     << next_frame_id;
          return false;
        }
      }
      if (!isConsecutivePosesStable_(
              pose_graph,
              optimization_scope_params.min_frame_id_,
              optimization_scope_params.max_frame_id_,
              optimization_scope_params.consecutive_pose_transl_tol_,
              optimization_scope_params.consecutive_pose_orient_tol_)) {
        LOG(WARNING) << "Detecting jumps after optimization. Reverting...";
        pose_graph->setValuesFromAnotherPoseGraph(pose_graph_copy);
      }

      visualization_callback_(problem_data,
                              pose_graph,
                              start_opt_with_frame,
                              next_frame_id,
                              VisualizationTypeEnum::AFTER_EACH_OPTIMIZATION);
    }

    visualization_callback_(problem_data,
                            pose_graph,
                            0,
                            max_frame_id,
                            VisualizationTypeEnum::AFTER_ALL_OPTIMIZATION);
    output_data_extractor_(problem_data,
                           pose_graph,
                           optimization_factors_enabled_params,
                           output_problem_data);
    return true;
  }

 private:
  pose_graph_optimization::ObjectVisualPoseGraphResidualParams residual_params_;

  LimitTrajectoryEvaluationParams limit_trajectory_eval_params_;

  std::function<bool()> continue_opt_checker_;
  std::function<FrameId(const FrameId &)> window_provider_func_;
  pose_graph_optimizer::ObjectPoseGraphOptimizer<VisualFeatureFactorType,
                                                 CachedFactorInfo,
                                                 PoseGraphType>
      optimizer_;
  std::function<void(const InputProblemData &,
                     std::shared_ptr<PoseGraphType> &)>
      pose_graph_creator_;

  std::function<void(const InputProblemData &,
                     const std::shared_ptr<PoseGraphType> &,
                     const FrameId &,
                     const FrameId &)>
      frame_data_adder_;
  std::function<void(
      const InputProblemData &,
      const std::shared_ptr<PoseGraphType> &,
      const pose_graph_optimizer::OptimizationFactorsEnabledParams &,
      OutputProblemData &)>
      output_data_extractor_;

  std::function<std::vector<std::shared_ptr<ceres::IterationCallback>>(
      const InputProblemData &,
      const std::shared_ptr<PoseGraphType> &,
      const FrameId &,
      const FrameId &)>
      ceres_callback_creator_;
  std::function<void(const InputProblemData &,
                     const std::shared_ptr<PoseGraphType> &,
                     const FrameId &,
                     const FrameId &,
                     const VisualizationTypeEnum &)>
      visualization_callback_;

  std::function<pose_graph_optimization::OptimizationSolverParams(
      const FrameId &)>
      solver_params_provider_func_;

  bool isConsecutivePosesStable_(
      const std::shared_ptr<PoseGraphType> &pose_graph,
      const FrameId &min_frame_id,
      const FrameId &max_frame_id,
      const double kConsecutiveTranslTol,
      const double kConsecutiveOrientTol) {
    if (max_frame_id == min_frame_id) {
      return true;
    }
    for (FrameId frame_id = min_frame_id + 1; frame_id <= max_frame_id;
         ++frame_id) {
      std::optional<RawPose3d<double>> prev_raw_robot_pose =
          pose_graph->getRobotPose(frame_id - 1);
      std::optional<RawPose3d<double>> raw_robot_pose =
          pose_graph->getRobotPose(frame_id);
      // TODO unsure of how to handle the checks here. Techinically this case
      // should not happen.
      if (!prev_raw_robot_pose.has_value() || !raw_robot_pose.has_value()) {
        LOG(ERROR) << "Couldn't find pose for frame " << frame_id
                   << " in pose graph";
        continue;
      }
      Pose3D<double> prev_robot_pose =
          convertToPose3D(prev_raw_robot_pose.value());
      Pose3D<double> robot_pose = convertToPose3D(raw_robot_pose.value());
      Pose3D<double> relative_pose =
          getPose2RelativeToPose1(prev_robot_pose, robot_pose);
      if (relative_pose.transl_.norm() > kConsecutiveTranslTol ||
          std::fabs(relative_pose.orientation_.angle()) >
              kConsecutiveOrientTol) {
        return false;
      }
    }
    return true;
  }
};
}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_OFFLINE_PROBLEM_RUNNER_H
