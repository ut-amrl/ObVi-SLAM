//
// Created by amanda on 6/17/22.
//

#ifndef UT_VSLAM_OFFLINE_PROBLEM_RUNNER_H
#define UT_VSLAM_OFFLINE_PROBLEM_RUNNER_H

#include <ceres/problem.h>
#include <refactoring/optimization/object_pose_graph.h>
#include <refactoring/optimization/object_pose_graph_optimizer.h>

namespace vslam_types_refactor {

enum VisualizationTypeEnum {
  BEFORE_ANY_OPTIMIZATION,
  BEFORE_EACH_OPTIMIZATION,
  AFTER_EACH_OPTIMIZATION,
  AFTER_ALL_OPTIMIZATION
};

enum OptimTypeEnum {
  SLIDING_WINDOW,
  STATIC,
  TWOPAHSE_SLIDING_WINDOW,
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
      const pose_graph_optimization::OptimizationSolverParams &solver_params)
      : residual_params_(residual_params),
        continue_opt_checker_(continue_opt_checker),
        window_provider_func_(window_provider_func),
        optimizer_(refresh_residual_checker, residual_creator),
        pose_graph_creator_(pose_graph_creator),
        frame_data_adder_(frame_data_adder),
        output_data_extractor_(output_data_extractor),
        ceres_callback_creator_(ceres_callback_creator),
        visualization_callback_(visualization_callback),
        solver_params_(solver_params) {}
  
  bool runOptimizationHelper(
      const InputProblemData &problem_data,
      const FrameId start_frame_id,
      const FrameId end_frame_id,
      const std::vector<FrameId> frame_ids_to_add,
      const pose_graph_optimizer::OptimizationScopeParams& optimization_scope_params,
      ceres::Problem& problem,
      std::shared_ptr<PoseGraphType>& pose_graph,
      OutputProblemData &output_problem_data) {
    for (const auto& frame_id_to_add : frame_ids_to_add) {
      frame_data_adder_(problem_data, pose_graph, frame_id_to_add);
    }
    pose_graph_optimizer::OptimizationScopeParams my_optimization_scope_params
        = optimization_scope_params;
    my_optimization_scope_params.min_frame_id_ = start_frame_id;
    my_optimization_scope_params.max_frame_id_ = end_frame_id;
    LOG(INFO) << "Building optimization";
    optimizer_.buildPoseGraphOptimization(
        my_optimization_scope_params, residual_params_, pose_graph, &problem);
    visualization_callback_(problem_data,
                            pose_graph,
                            start_frame_id,
                            end_frame_id,
                            VisualizationTypeEnum::BEFORE_EACH_OPTIMIZATION);
    std::vector<std::shared_ptr<ceres::IterationCallback>> ceres_callbacks =
        ceres_callback_creator_(
            problem_data, pose_graph, start_frame_id, end_frame_id);
    LOG(INFO) << "Solving optimization";
    bool opt_success = optimizer_.solveOptimization(
        &problem, solver_params_, ceres_callbacks);
    visualization_callback_(problem_data,
                            pose_graph,
                            start_frame_id,
                            end_frame_id,
                            VisualizationTypeEnum::AFTER_EACH_OPTIMIZATION);
    if (!opt_success) {
      // TODO do we want to quit or just silently let this iteration fail?
      LOG(ERROR) << "Optimization failed at max frame id " << end_frame_id;
      return false;
    }
    return true;
  }

  bool runOptimization(
      const InputProblemData &problem_data,
      const pose_graph_optimizer::OptimizationFactorsEnabledParams
          &optimization_factors_enabled_params,
      OutputProblemData &output_problem_data, 
      const OptimTypeEnum& optim_type = OptimTypeEnum::SLIDING_WINDOW) {
    
    ceres::Problem problem;
    std::shared_ptr<PoseGraphType> pose_graph;
    LOG(INFO) << "Running pose graph creator";
    pose_graph_creator_(problem_data, pose_graph);
    frame_data_adder_(problem_data, pose_graph, 0);
    FrameId max_frame_id = problem_data.getMaxFrameId();

    pose_graph_optimizer::OptimizationScopeParams optimization_scope_params;
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
    
    visualization_callback_(problem_data,
                            pose_graph,
                            0,
                            max_frame_id,  // Should this be max or 0
                            VisualizationTypeEnum::BEFORE_ANY_OPTIMIZATION);
    LOG(INFO) << "Ready to run optimization";

    switch (optim_type) {
      case OptimTypeEnum::SLIDING_WINDOW: {
        LOG(INFO) << "OptimTypeEnum::SLIDING_WINDOW";
        for (FrameId next_frame_id = 1; next_frame_id <= max_frame_id;
            next_frame_id++) {
          if (!continue_opt_checker_) {
            LOG(WARNING)
                << "Halted optimization due to continue checker reporting false";
            return false;
          }
          FrameId start_frame_id = window_provider_func_(next_frame_id);
          std::vector<FrameId> frame_ids_to_add = {next_frame_id};
          bool optim_success = 
              runOptimizationHelper(
                  problem_data, 
                  start_frame_id,
                  next_frame_id,
                  frame_ids_to_add,
                  optimization_scope_params,
                  problem, 
                  pose_graph,
                  output_problem_data);
          if (!optim_success) { return optim_success; }
        }
        break;
      }
      case OptimTypeEnum::STATIC: {
        FrameId start_frame_id = window_provider_func_(max_frame_id);
        std::vector<FrameId> frame_ids_to_add;
        for (FrameId frame_id = start_frame_id; frame_id <=max_frame_id; ++frame_id) {
          frame_ids_to_add.emplace_back(frame_id);
        }
        bool optim_success = 
              runOptimizationHelper(
                  problem_data, 
                  start_frame_id,
                  max_frame_id,
                  frame_ids_to_add,
                  optimization_scope_params,
                  problem, 
                  pose_graph,
                  output_problem_data);
        if (!optim_success) { return false; }
        break;
      }
      case OptimTypeEnum::TWOPAHSE_SLIDING_WINDOW: {
        for (FrameId next_frame_id = 1; next_frame_id <= max_frame_id;
            next_frame_id++) {
          FrameId start_frame_id = window_provider_func_(next_frame_id);

        }
        break;
      }
      default:
        break;
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
  pose_graph_optimization::OptimizationSolverParams solver_params_;
};
}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_OFFLINE_PROBLEM_RUNNER_H
