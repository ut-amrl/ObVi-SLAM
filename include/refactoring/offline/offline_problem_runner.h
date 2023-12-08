//
// Created by amanda on 6/17/22.
//

#ifndef UT_VSLAM_OFFLINE_PROBLEM_RUNNER_H
#define UT_VSLAM_OFFLINE_PROBLEM_RUNNER_H

#include <analysis/cumulative_timer_constants.h>
#include <analysis/cumulative_timer_factory.h>
#include <ceres/problem.h>
#include <refactoring/offline/limit_trajectory_evaluation_params.h>
#include <refactoring/optimization/object_pose_graph.h>
#include <refactoring/optimization/object_pose_graph_optimizer.h>
#include <refactoring/optimization/pose_graph_plus_objects_optimizer.h>

namespace vslam_types_refactor {

enum VisualizationTypeEnum {
  BEFORE_ANY_OPTIMIZATION,
  BEFORE_EACH_OPTIMIZATION,
  AFTER_EACH_OPTIMIZATION,
  AFTER_PGO_PLUS_OBJ_OPTIMIZATION,
  AFTER_ALL_OPTIMIZATION,
  AFTER_ALL_POSTPROCESSING
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
      const pose_graph_optimization::PoseGraphPlusObjectsOptimizationParams
          &pgo_solver_params,
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
                               const VisualizationTypeEnum &,
                               const int &)> &visualization_callback,
      const std::function<pose_graph_optimization::OptimizationIterationParams(
          const FrameId &)> &iteration_params_provider_func,
      const std::function<bool(const std::shared_ptr<PoseGraphType> &)>
          object_merger,
      const std::function<bool(const FrameId &)> &gba_checker)
      : residual_params_(residual_params),
        limit_trajectory_eval_params_(limit_trajectory_eval_params),
        pgo_solver_params_(pgo_solver_params),
        continue_opt_checker_(continue_opt_checker),
        window_provider_func_(window_provider_func),
        residual_creator_(residual_creator),
        optimizer_(refresh_residual_checker, residual_creator),
        pose_graph_creator_(pose_graph_creator),
        frame_data_adder_(frame_data_adder),
        output_data_extractor_(output_data_extractor),
        ceres_callback_creator_(ceres_callback_creator),
        visualization_callback_(visualization_callback),
        iteration_params_provider_func_(iteration_params_provider_func),
        object_merger_(object_merger),
        gba_checker_(gba_checker) {}

  bool runOptimization(
      const InputProblemData &problem_data,
      const pose_graph_optimizer::OptimizationFactorsEnabledParams
          &optimization_factors_enabled_params,
      std::optional<vslam_types_refactor::OptimizationLogger> &opt_logger,
      OutputProblemData &output_problem_data,
      const FrameId &start_at_frame = 0,
      const bool &add_data_for_starting_frame = true) {
    std::shared_ptr<PoseGraphType> pose_graph;

    if (opt_logger.has_value()) {
      opt_logger->writeOptInfoHeader();
    }
    ceres::Problem problem;
    LOG(INFO) << "Running pose graph creator";

    pose_graph_optimizer::OptimizationScopeParams optimization_scope_params;
    optimization_scope_params.min_low_level_feature_observations_per_frame_ =
        optimization_factors_enabled_params
            .min_low_level_feature_observations_per_frame_;
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

    FrameId max_frame_id = problem_data.getMaxFrameId();

    if (limit_trajectory_eval_params_.should_limit_trajectory_evaluation_) {
      max_frame_id =
          std::min(limit_trajectory_eval_params_.max_frame_id_, max_frame_id);
    }

    {
#ifdef RUN_TIMERS
    CumulativeFunctionTimer::Invocation invoc(
        CumulativeTimerFactory::getInstance()
            .getOrCreateFunctionTimer(
                kTimerNameOfflineProblemRunnerOnlinePortionPlusVis)
            .get());
#endif

    pose_graph_creator_(problem_data, pose_graph);

    if ((start_at_frame == 0) && (add_data_for_starting_frame)) {
      frame_data_adder_(problem_data, pose_graph, 0, 0);
    }

    visualization_callback_(problem_data,
                            pose_graph,
                            0,
                            max_frame_id,  // Should this be max or 0
                            VisualizationTypeEnum::BEFORE_ANY_OPTIMIZATION,
                            0);
    LOG(INFO) << "Ready to run optimization";

    FrameId first_frame = std::max((FrameId)1, start_at_frame);

    for (FrameId next_frame_id = first_frame; next_frame_id <= max_frame_id;
         next_frame_id++) {
      {
#ifdef RUN_TIMERS
      CumulativeFunctionTimer::Invocation invoc(
          CumulativeTimerFactory::getInstance()
              .getOrCreateFunctionTimer(kTimerNameOptimizationIteration)
              .get());
#endif
      if (!continue_opt_checker_) {
        LOG(WARNING)
            << "Halted optimization due to continue checker reporting false";
        return false;
      }
      // This function is also responsible for adjusting the initial estimate
      // for the pose at next_frame_id given the optimized pose at
      // next_frame_id
      // - 1
      FrameId start_opt_with_frame = window_provider_func_(next_frame_id);
      optimization_scope_params.min_frame_id_ = start_opt_with_frame;
      optimization_scope_params.max_frame_id_ = next_frame_id;

      if ((next_frame_id != start_at_frame) || (add_data_for_starting_frame)) {
        frame_data_adder_(
            problem_data, pose_graph, start_opt_with_frame, next_frame_id);
      }

      if (!runOptimizationIteration(start_opt_with_frame,
                                    next_frame_id,
                                    problem_data,
                                    optimization_factors_enabled_params,
                                    optimization_scope_params,
                                    max_frame_id,
                                    opt_logger,
                                    pose_graph,
                                    problem)) {
        return false;
      }
      }
#ifdef RUN_TIMERS
      CumulativeFunctionTimer::Invocation invoc(
          CumulativeTimerFactory::getInstance()
              .getOrCreateFunctionTimer(kTimerNameIterationLoggerWrite)
              .get());
#endif
      IterationLoggerFactory::getInstance().writeAllIterationLoggerStates();
    }
    }

#ifdef RUN_TIMERS
    CumulativeFunctionTimer::Invocation invoc(
        CumulativeTimerFactory::getInstance()
            .getOrCreateFunctionTimer(
                kTimerNameOfflineProblemRunnerOfflinePortionPlusVis)
            .get());
#endif

    // Run the final optimization (refinement)
    if (!runOptimizationIteration(0,
                                  max_frame_id,
                                  problem_data,
                                  optimization_factors_enabled_params,
                                  optimization_scope_params,
                                  max_frame_id,
                                  opt_logger,
                                  pose_graph,
                                  problem,
                                  1)) {
      return false;
    }

    visualization_callback_(problem_data,
                            pose_graph,
                            0,
                            max_frame_id,
                            VisualizationTypeEnum::AFTER_ALL_OPTIMIZATION,
                            1);

    // Until there are no more objects to merge, check if a merge should be
    // performed, and then if so, rerun the optimization to update the estimates
    if (!mergeObjectsAtSessionEnd(max_frame_id,
                                  problem_data,
                                  optimization_factors_enabled_params,
                                  optimization_scope_params,
                                  opt_logger,
                                  pose_graph)) {
      return false;
    }

    visualization_callback_(problem_data,
                            pose_graph,
                            0,
                            max_frame_id,
                            VisualizationTypeEnum::AFTER_ALL_POSTPROCESSING,
                            1);
    output_data_extractor_(problem_data,
                           pose_graph,
                           optimization_factors_enabled_params,
                           output_problem_data);
    return true;
  }

 private:
  pose_graph_optimization::ObjectVisualPoseGraphResidualParams residual_params_;

  LimitTrajectoryEvaluationParams limit_trajectory_eval_params_;

  pose_graph_optimization::PoseGraphPlusObjectsOptimizationParams
      pgo_solver_params_;

  std::function<bool()> continue_opt_checker_;
  std::function<FrameId(const FrameId &)> window_provider_func_;
  std::function<bool(
      const std::pair<vslam_types_refactor::FactorType,
                      vslam_types_refactor::FeatureFactorId> &,
      const pose_graph_optimization::ObjectVisualPoseGraphResidualParams &,
      const std::shared_ptr<PoseGraphType> &,
      ceres::Problem *,
      ceres::ResidualBlockId &,
      CachedFactorInfo &)>
      residual_creator_;
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
                     const VisualizationTypeEnum &,
                     const int &)>
      visualization_callback_;

  std::function<pose_graph_optimization::OptimizationIterationParams(
      const FrameId &)>
      iteration_params_provider_func_;

  std::function<bool(const std::shared_ptr<PoseGraphType> &)> object_merger_;

  std::function<bool(const FrameId &)> gba_checker_;

  bool isConsecutivePosesStable_(
      const std::shared_ptr<PoseGraphType> &pose_graph,
      const FrameId &min_frame_id,
      const FrameId &max_frame_id,
      const double &kConsecutiveTranslTol,
      const double &kConsecutiveOrientTol) {
#ifdef RUN_TIMERS
    CumulativeFunctionTimer::Invocation invoc(
        CumulativeTimerFactory::getInstance()
            .getOrCreateFunctionTimer(kTimerNameConsecutivePosesStable)
            .get());
#endif
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

  bool runOptimizationIteration(
      const FrameId &start_opt_with_frame,
      const FrameId &next_frame_id,
      const InputProblemData &problem_data,
      const pose_graph_optimizer::OptimizationFactorsEnabledParams
          &optimization_factors_enabled_params,
      const pose_graph_optimizer::OptimizationScopeParams
          &optimization_scope_params,
      const FrameId &max_frame_id,
      std::optional<vslam_types_refactor::OptimizationLogger> &opt_logger,
      std::shared_ptr<PoseGraphType> &pose_graph,
      ceres::Problem &problem,
      const int &attempt_num = 0) {
    pose_graph_optimization::OptimizationIterationParams iteration_params =
        iteration_params_provider_func_(next_frame_id);

    visualization_callback_(problem_data,
                            pose_graph,
                            start_opt_with_frame,
                            next_frame_id,
                            VisualizationTypeEnum::BEFORE_EACH_OPTIMIZATION,
                            attempt_num);
    std::vector<std::shared_ptr<ceres::IterationCallback>> ceres_callbacks =
        ceres_callback_creator_(
            problem_data, pose_graph, start_opt_with_frame, next_frame_id);

    if (opt_logger.has_value()) {
      opt_logger->setOptimizationTypeParams(
          next_frame_id, start_opt_with_frame == 0, false, false, attempt_num);
    }

    bool global_ba = gba_checker_(next_frame_id);

    bool run_visual_feature_opt = true;
    if (global_ba) {
      bool run_pgo;
      if ((next_frame_id == max_frame_id) && (attempt_num > 0)) {
        run_pgo = optimization_factors_enabled_params
                      .use_pose_graph_on_final_global_ba_;
        if (run_pgo) {
          run_visual_feature_opt = optimization_factors_enabled_params
                                       .use_visual_features_on_final_global_ba_;
        }
      } else {
        run_pgo =
            optimization_factors_enabled_params.use_pose_graph_on_global_ba_;
        if (run_pgo) {
          run_visual_feature_opt = optimization_factors_enabled_params
                                       .use_visual_features_on_global_ba_;
        }
      }
      if (run_pgo) {
        {
#ifdef RUN_TIMERS
          std::string gba_timer_name =
              attempt_num == 0 ? kTimerNameObjOnlyPgoFullProcess
                               : kTimerNameMapMergeObjOnlyPgoFullProcess;
          CumulativeFunctionTimer::Invocation gba_invoc(
              CumulativeTimerFactory::getInstance()
                  .getOrCreateFunctionTimer(gba_timer_name)
                  .get());
#endif
          // TODO Need to run 1 iteration of tracking first
          LOG(INFO) << "Running tracking before PGO";
          pose_graph_optimizer::OptimizationScopeParams tracking_params =
              optimization_scope_params;
          // TODO do we need more poses than this?
          tracking_params.min_frame_id_ =
              next_frame_id -
              tracking_params.poses_prior_to_window_to_keep_constant_;
          std::optional<OptimizationLogger> null_logger;
          {
            {
#ifdef RUN_TIMERS
              std::string local_track_build_timer_name =
                  attempt_num == 0
                      ? kTimerNameObjOnlyPgoLocalTrackBuild
                      : kTimerNameMapMergeObjOnlyPgoLocalTrackBuild;
              CumulativeFunctionTimer::Invocation local_track_build_invoc(
                  CumulativeTimerFactory::getInstance()
                      .getOrCreateFunctionTimer(local_track_build_timer_name)
                      .get());
#endif
              optimizer_.buildPoseGraphOptimization(tracking_params,
                                                    residual_params_,
                                                    pose_graph,
                                                    &problem,
                                                    null_logger);
            }
            {
#ifdef RUN_TIMERS
              std::string local_track_solve_timer_name =
                  attempt_num == 0
                      ? kTimerNameObjOnlyPgoLocalTrackSolve
                      : kTimerNameMapMergeObjOnlyPgoLocalTrackSolve;
              CumulativeFunctionTimer::Invocation local_track_solve_invoc(
                  CumulativeTimerFactory::getInstance()
                      .getOrCreateFunctionTimer(local_track_solve_timer_name)
                      .get());
#endif
              std::shared_ptr<ceres::Solver::Summary> track_solver_summary =
                  std::make_shared<ceres::Solver::Summary>();
              if (!optimizer_.solveOptimization(
                      &problem,
                      iteration_params.phase_one_opt_params_,
                      ceres_callbacks,
                      null_logger,
                      nullptr,
                      track_solver_summary)) {
                LOG(INFO) << "Tracking failed";
              }
              std::string opt_identifier =
                  std::to_string(next_frame_id);
              std::shared_ptr<IterationLogger> tracking_logger =
                  IterationLoggerFactory::getInstance().getOrCreateLoggerOfType(
                      IterationLoggerFactory::kPrePgoTrackOptimizationType);
              if (tracking_logger != nullptr) {
                tracking_logger->logIterations(
                    opt_identifier, *track_solver_summary);
              }
            }
          }

          LOG(INFO) << "Running PGO+objs";
          runPgoPlusEllipsoids(next_frame_id,
                               residual_creator_,
                               optimization_scope_params,
                               ceres_callbacks,
                               residual_params_,
                               pgo_solver_params_,
                               {},
                               next_frame_id == max_frame_id,
                               opt_logger,
                               pose_graph,
                               attempt_num);
        }
        visualization_callback_(
            problem_data,
            pose_graph,
            start_opt_with_frame,
            next_frame_id,
            VisualizationTypeEnum::AFTER_PGO_PLUS_OBJ_OPTIMIZATION,
            attempt_num);
      }
    }

    if (run_visual_feature_opt) {
      {
#ifdef RUN_TIMERS
        std::string bundle_adjustment_timer_name;
        if (global_ba) {
          if (attempt_num == 0) {
            bundle_adjustment_timer_name = kTimerNameGlobalBundleAdjustment;
          } else {
            bundle_adjustment_timer_name = kTimerNameMapMergeGba;
          }
        } else {
          bundle_adjustment_timer_name = kTimerNameLocalBundleAdjustment;
        }

        CumulativeFunctionTimer::Invocation invoc_ba(
            CumulativeTimerFactory::getInstance()
                .getOrCreateFunctionTimer(bundle_adjustment_timer_name)
                .get());
#endif
        bool visual_feature_opt_enable_two_phase =
            iteration_params.feature_outlier_percentage_ > 0;
        // Phase I
        LOG(INFO) << "Building optimization";
        std::unordered_map<ceres::ResidualBlockId,
                           std::pair<vslam_types_refactor::FactorType,
                                     vslam_types_refactor::FeatureFactorId>>
            current_residual_block_info;
        {
#ifdef RUN_TIMERS
          std::string phase_one_build_opt_timer_name;
          if (global_ba) {
            if (attempt_num == 0) {
              phase_one_build_opt_timer_name = kTimerNamePhaseOneGbaBuildOpt;
            } else {
              phase_one_build_opt_timer_name =
                  kTimerNameMapMergePhaseOneGbaBuildOpt;
            }
          } else {
            phase_one_build_opt_timer_name = kTimerNamePhaseOneLbaBuildOpt;
          }

          CumulativeFunctionTimer::Invocation phase_one_build(
              CumulativeTimerFactory::getInstance()
                  .getOrCreateFunctionTimer(phase_one_build_opt_timer_name)
                  .get());
#endif
          current_residual_block_info =
              optimizer_.buildPoseGraphOptimization(optimization_scope_params,
                                                    residual_params_,
                                                    pose_graph,
                                                    &problem,
                                                    opt_logger);
        }
        std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph> pose_graph_copy;
        {
#ifdef RUN_TIMERS
        std::string pg_copy_timer_name;
        if (global_ba) {
          if (attempt_num == 0) {
            pg_copy_timer_name = kTimerNameGbaPoseGraphCopy;
          } else {
            pg_copy_timer_name = kTimerNameMapMergeGbaPoseGraphCopy;
          }
        } else {
          pg_copy_timer_name = kTimerNameLbaPoseGraphCopy;
        }

        CumulativeFunctionTimer::Invocation invoc_pg_copy(
            CumulativeTimerFactory::getInstance()
                .getOrCreateFunctionTimer(pg_copy_timer_name)
                .get());
#endif
        pose_graph_copy = pose_graph->makeDeepCopy();
        }
        LOG(INFO) << "Solving optimization";
        bool phase1_optim_success;
        std::vector<ceres::ResidualBlockId> residual_block_ids;
        std::vector<double> residuals;
#ifdef RUN_TIMERS
        std::string phase_one_solve_invoc;
        if (global_ba) {
          if (attempt_num == 0) {
            phase_one_solve_invoc = kTimerNamePhaseOneGbaSolveOpt;
          } else {
            phase_one_solve_invoc = kTimerNameMapMergePhaseOneGbaSolveOpt;
          }
        } else {
          phase_one_solve_invoc = kTimerNamePhaseOneLbaSolveOpt;
        }

#endif
        if (visual_feature_opt_enable_two_phase) {
#ifdef RUN_TIMERS
          CumulativeFunctionTimer::Invocation phase_one_invoc(
              CumulativeTimerFactory::getInstance()
                  .getOrCreateFunctionTimer(phase_one_solve_invoc)
                  .get());
#endif
          phase1_optim_success = optimizer_.solveOptimization(
              &problem,
              iteration_params.phase_one_opt_params_,
              ceres_callbacks,
              opt_logger,
              &residual_block_ids,
              &residuals);
        } else {
#ifdef RUN_TIMERS
          CumulativeFunctionTimer::Invocation phase_one_invoc(
              CumulativeTimerFactory::getInstance()
                  .getOrCreateFunctionTimer(phase_one_solve_invoc)
                  .get());
#endif
          phase1_optim_success = optimizer_.solveOptimization(
              &problem,
              iteration_params.phase_one_opt_params_,
              ceres_callbacks,
              opt_logger,
              nullptr,
              nullptr);
        }

        if (!phase1_optim_success) {
          // TODO do we want to quit or just silently let this iteration fail?
          LOG(ERROR) << "Phase I Optimization failed at max frame id "
                     << next_frame_id;
          return false;
        }
        if (opt_logger.has_value()) {
#ifdef RUN_TIMERS
          std::string opt_logger_timer_name;
          if (global_ba) {
            if (attempt_num == 0) {
              opt_logger_timer_name = kTimerNameGbaOptLoggerWrite;
            } else {
              opt_logger_timer_name = kTimerNameMapMergeGbaOptLoggerWrite;
            }
          } else {
            opt_logger_timer_name = kTimerNameLbaOptLoggerWrite;
          }

          CumulativeFunctionTimer::Invocation opt_logger_timer(
              CumulativeTimerFactory::getInstance()
                  .getOrCreateFunctionTimer(opt_logger_timer_name)
                  .get());
#endif
          opt_logger->writeCurrentOptInfo();
        }

        // If two-phase optim is enabled, compute outliers based on residuals
        std::unordered_map<vslam_types_refactor::FactorType,
                           std::unordered_map<ceres::ResidualBlockId, double>>
            factor_types_and_residual_info;
        if (visual_feature_opt_enable_two_phase) {
          std::string post_opt_residual_compute_timer;
          if (attempt_num == 0) {
            post_opt_residual_compute_timer =
                kTimerNamePostOptResidualComputeOnline;
          } else {
            post_opt_residual_compute_timer =
                kTimerNamePostOptResidualComputeOffline;
          }
#ifdef RUN_TIMERS
          CumulativeFunctionTimer::Invocation post_opt_residual_invoc(
              CumulativeTimerFactory::getInstance()
                  .getOrCreateFunctionTimer(post_opt_residual_compute_timer)
                  .get());
#endif
          size_t residual_idx = 0;
          for (size_t block_idx = 0; block_idx < residual_block_ids.size();
               ++block_idx) {
            const ceres::ResidualBlockId &block_id =
                residual_block_ids.at(block_idx);
            // TODO (Taijing) for speed I hardcoded values here
            // We want to use Cere 2.0+ or have a generic API to get residual
            // block lengths
            size_t residual_block_size;
            if (current_residual_block_info.at(block_id).first ==
                kReprojectionErrorFactorTypeId) {
              residual_block_size = 2;
              // vslam_types_refactor::ReprojectionErrorFactor factor;
              // pose_graph->getVisualFactor(
              //     current_residual_block_info.at(block_id).second, factor);
            } else if (current_residual_block_info.at(block_id).first ==
                       kObjectObservationFactorTypeId) {
              residual_block_size = 4;
            } else if (current_residual_block_info.at(block_id).first ==
                       kShapeDimPriorFactorTypeId) {
              residual_block_size = 3;
            } else if (current_residual_block_info.at(block_id).first ==
                       kLongTermMapFactorTypeId) {
              residual_block_size = kEllipsoidParamterizationSize;
            } else if (current_residual_block_info.at(block_id).first ==
                       kPairwiseRobotPoseFactorTypeId) {
              residual_block_size = 6;
            } else if (current_residual_block_info.at(block_id).first ==
                       kPairwiseErrorFactorTypeId) {
              residual_block_size = 1;
            } else {
              LOG(ERROR) << "Unknown factor type "
                         << current_residual_block_info.at(block_id).first
                         << ". Disabling two phase optimization ...";
              visual_feature_opt_enable_two_phase = false;
              break;
            }
            double total_residual = 0;
            for (size_t i = 0; i < residual_block_size; ++i) {
              total_residual +=
                  (residuals.at(residual_idx) * residuals.at(residual_idx));
              ++residual_idx;
            }
            // Only exclude reprojection errors for visual features and bbox
            // observation errors for objects. Modify this check if you want
            // to add two-phase optimization support to other factors.
            if ((current_residual_block_info.at(block_id).first ==
                 kReprojectionErrorFactorTypeId) ||
                (current_residual_block_info.at(block_id).first ==
                 kObjectObservationFactorTypeId)) {
              factor_types_and_residual_info
                  [current_residual_block_info.at(block_id).first][block_id] =
                      total_residual;
            }
          }
          if (residual_idx != residuals.size()) {
            LOG(ERROR) << "Something is wrong with residual block indexing. "
                          "Disabling two phase optimization ...";
            visual_feature_opt_enable_two_phase = false;
          }
        }

        // Build excluded_feature_factor_types_and_ids
        util::BoostHashSet<std::pair<FactorType, FeatureFactorId>>
            excluded_feature_factor_types_and_ids;
        if (visual_feature_opt_enable_two_phase) {
#ifdef RUN_TIMERS
          std::string two_phase_opt_outlier_timer;
          if (attempt_num == 0) {
            two_phase_opt_outlier_timer =
                kTimerNameTwoPhaseOptOutlierIdentificationOnline;
          } else {
            two_phase_opt_outlier_timer =
                kTimerNameTwoPhaseOptOutlierIdentificationOffline;
          }
          CumulativeFunctionTimer::Invocation two_phase_opt_outlier_invoc(
              CumulativeTimerFactory::getInstance()
                  .getOrCreateFunctionTimer(two_phase_opt_outlier_timer)
                  .get());
#endif
          std::unordered_map<
              FactorType,
              std::map<double, ceres::ResidualBlockId, std::greater<double>>>
              factor_types_and_ordered_residual_info;
          for (const auto &factor_type_and_residual_info :
               factor_types_and_residual_info) {
            LOG(INFO) << "Factor type "
                      << (int)factor_type_and_residual_info.first << " has "
                      << factor_type_and_residual_info.second.size()
                      << " factors before outlier exclusion.";
            for (const auto block_id_and_residual :
                 factor_type_and_residual_info.second) {
              factor_types_and_ordered_residual_info
                  [factor_type_and_residual_info.first]
                  [block_id_and_residual.second] = block_id_and_residual.first;
            }
          }
          for (const auto &factor_type_and_ordered_redsidual_info :
               factor_types_and_ordered_residual_info) {
            const auto &ordered_residuals_and_block_ids =
                factor_type_and_ordered_redsidual_info.second;
            size_t n_outliers =
                (size_t)(ordered_residuals_and_block_ids.size() *
                         iteration_params.feature_outlier_percentage_);
            auto it = ordered_residuals_and_block_ids.begin();
            for (size_t i = 0; i < n_outliers; ++i) {
              const ceres::ResidualBlockId &block_id = it->second;
              excluded_feature_factor_types_and_ids.insert(
                  current_residual_block_info.at(block_id));
              ++it;
            }
          }
        }

        // Phase II
        if (visual_feature_opt_enable_two_phase) {
          // revert back to the old pose graph for Phase II optim

          if (opt_logger.has_value()) {
            opt_logger->setOptimizationTypeParams(
                next_frame_id, start_opt_with_frame == 0, false, true, attempt_num);
          }
          pose_graph->setValuesFromAnotherPoseGraph(pose_graph_copy);
          {
#ifdef RUN_TIMERS

            std::string phase_two_build_invoc_timer_name;
            if (global_ba) {
              if (attempt_num == 0) {
                phase_two_build_invoc_timer_name =
                    kTimerNamePhaseTwoGbaBuildOpt;
              } else {
                phase_two_build_invoc_timer_name =
                    kTimerNameMapMergePhaseTwoGbaBuildOpt;
              }
            } else {
              phase_two_build_invoc_timer_name = kTimerNamePhaseTwoLbaBuildOpt;
            }

            CumulativeFunctionTimer::Invocation phase_two_invoc(
                CumulativeTimerFactory::getInstance()
                    .getOrCreateFunctionTimer(phase_two_build_invoc_timer_name)
                    .get());
#endif
            optimizer_.buildPoseGraphOptimization(
                optimization_scope_params,
                residual_params_,
                pose_graph,
                &problem,
                opt_logger,
                excluded_feature_factor_types_and_ids);
          }
          {
#ifdef RUN_TIMERS
            std::string phase_two_solve_invoc_timer_name;
            if (global_ba) {
              if (attempt_num == 0) {
                phase_two_solve_invoc_timer_name =
                    kTimerNamePhaseTwoGbaSolveOpt;
              } else {
                phase_two_solve_invoc_timer_name =
                    kTimerNameMapMergePhaseTwoGbaSolveOpt;
              }
            } else {
              phase_two_solve_invoc_timer_name = kTimerNamePhaseTwoLbaSolveOpt;
            }
            CumulativeFunctionTimer::Invocation phase_two_solve_invoc(
                CumulativeTimerFactory::getInstance()
                    .getOrCreateFunctionTimer(phase_two_solve_invoc_timer_name)
                    .get());
#endif
            if (!optimizer_.solveOptimization(
                    &problem,
                    iteration_params.phase_two_opt_params_,
                    ceres_callbacks,
                    opt_logger,
                    nullptr,
                    nullptr)) {
              // TODO do we want to quit or just silently let this iteration
              // fail?
              LOG(ERROR) << "Phase II Optimization failed at max frame id "
                         << next_frame_id;
              return false;
            }
          }
          if (opt_logger.has_value()) {
#ifdef RUN_TIMERS
            std::string opt_logger_timer_name;
            if (global_ba) {
              if (attempt_num == 0) {
                opt_logger_timer_name = kTimerNameGbaOptLoggerWrite;
              } else {
                opt_logger_timer_name = kTimerNameMapMergeGbaOptLoggerWrite;
              }
            } else {
              opt_logger_timer_name = kTimerNameLbaOptLoggerWrite;
            }

            CumulativeFunctionTimer::Invocation opt_logger_timer(
                CumulativeTimerFactory::getInstance()
                    .getOrCreateFunctionTimer(opt_logger_timer_name)
                    .get());
#endif
            opt_logger->writeCurrentOptInfo();
          }
        }
        if (iteration_params.allow_reversion_after_detecting_jumps_) {
          if (!isConsecutivePosesStable_(
                  pose_graph,
                  optimization_scope_params.min_frame_id_,
                  optimization_scope_params.max_frame_id_,
                  iteration_params.consecutive_pose_transl_tol_,
                  iteration_params.consecutive_pose_orient_tol_)) {
            LOG(WARNING) << "Detecting jumps after optimization. Reverting...";
            pose_graph->setValuesFromAnotherPoseGraph(pose_graph_copy);
          }
        }
      }  // End of BA timer?

      visualization_callback_(problem_data,
                              pose_graph,
                              start_opt_with_frame,
                              next_frame_id,
                              VisualizationTypeEnum::AFTER_EACH_OPTIMIZATION,
                              attempt_num);
    }
    return true;
  }

  bool mergeObjectsAtSessionEnd(
      const FrameId &max_frame_id,
      const InputProblemData &problem_data,
      const pose_graph_optimizer::OptimizationFactorsEnabledParams
          &optimization_factors_enabled_params,
      const pose_graph_optimizer::OptimizationScopeParams
          &optimization_scope_params,
      std::optional<vslam_types_refactor::OptimizationLogger> &opt_logger,
      std::shared_ptr<PoseGraphType> &pose_graph) {
#ifdef RUN_TIMERS
    CumulativeFunctionTimer::Invocation invoc(
        vslam_types_refactor::CumulativeTimerFactory::getInstance()
            .getOrCreateFunctionTimer(
                vslam_types_refactor::kTimerNamePostSessionMapMerge)
            .get());
#endif

    // Until there are no more objects to merge, check if a merge should be
    // performed, and then if so, rerun the optimization to update the estimates
    int post_process_round = 2;
    while (object_merger_(pose_graph)) {
      ceres::Problem merged_problem;
      optimizer_.clearPastOptimizationData();

      // Rerun optimization
      if (!runOptimizationIteration(0,
                                    max_frame_id,
                                    problem_data,
                                    optimization_factors_enabled_params,
                                    optimization_scope_params,
                                    max_frame_id,
                                    opt_logger,
                                    pose_graph,
                                    merged_problem,
                                    post_process_round)) {
        return false;
      }
      post_process_round++;
    }
    return true;
  }
};
}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_OFFLINE_PROBLEM_RUNNER_H
