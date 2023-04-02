//
// Created by amanda on 6/17/22.
//

#ifndef UT_VSLAM_OFFLINE_PROBLEM_RUNNER_H
#define UT_VSLAM_OFFLINE_PROBLEM_RUNNER_H

#include <ceres/problem.h>
#include <refactoring/offline/limit_trajectory_evaluation_params.h>
#include <refactoring/optimization/object_pose_graph.h>
#include <refactoring/optimization/object_pose_graph_optimizer.h>
#include <refactoring/optimization/pose_graph_plus_objects_optimizer.h>

namespace vslam_types_refactor {

const double kFeatureOutlierPercentage = 0.1;

enum VisualizationTypeEnum {
  BEFORE_ANY_OPTIMIZATION,
  BEFORE_EACH_OPTIMIZATION,
  AFTER_EACH_OPTIMIZATION,
  AFTER_PGO_PLUS_OBJ_OPTIMIZATION,
  AFTER_ALL_OPTIMIZATION
};

enum OptimTypeEnum {
  SLIDING_WINDOW,
  STATIC,
  TWOPHASE_SLIDING_WINDOW,
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
                               const VisualizationTypeEnum &)>
          &visualization_callback,
      const std::function<pose_graph_optimization::OptimizationSolverParams(
          const FrameId &)> &solver_params_provider_func,
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
        solver_params_provider_func_(solver_params_provider_func),
        gba_checker_(gba_checker) {}

  bool runOptimization(
      const InputProblemData &problem_data,
      const pose_graph_optimizer::OptimizationFactorsEnabledParams
          &optimization_factors_enabled_params,
      std::optional<vslam_types_refactor::OptimizationLogger> &opt_logger,
      OutputProblemData &output_problem_data) {
    std::shared_ptr<PoseGraphType> pose_graph;

    if (opt_logger.has_value()) {
      opt_logger->writeOptInfoHeader();
    }
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

    visualization_callback_(problem_data,
                            pose_graph,
                            0,
                            max_frame_id,  // Should this be max or 0
                            VisualizationTypeEnum::BEFORE_ANY_OPTIMIZATION);
    LOG(INFO) << "Ready to run optimization";

    for (FrameId next_frame_id = 1; next_frame_id <= max_frame_id;
         next_frame_id++) {
      if (next_frame_id > 67) {
        exit(0);
      }
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

      visualization_callback_(problem_data,
                              pose_graph,
                              start_opt_with_frame,
                              next_frame_id,
                              VisualizationTypeEnum::BEFORE_EACH_OPTIMIZATION);
      std::vector<std::shared_ptr<ceres::IterationCallback>> ceres_callbacks =
          ceres_callback_creator_(
              problem_data, pose_graph, start_opt_with_frame, next_frame_id);

      if (opt_logger.has_value()) {
        opt_logger->setOptimizationTypeParams(
            next_frame_id, start_opt_with_frame == 0, false, false);
      }

      bool run_visual_feature_opt = true;
      if (gba_checker_(next_frame_id)) {
        bool run_pgo;
        if (next_frame_id == max_frame_id) {
          run_pgo = optimization_factors_enabled_params
                        .use_pose_graph_on_final_global_ba_;
          if (run_pgo) {
            run_visual_feature_opt =
                optimization_factors_enabled_params
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
          // TODO Need to run 1 iteration of tracking first
          runPgoPlusEllipsoids(next_frame_id,
                               residual_creator_,
                               optimization_scope_params,
                               ceres_callbacks,
                               residual_params_,
                               pgo_solver_params_,
                               {},
                               next_frame_id == max_frame_id,
                               opt_logger,
                               pose_graph);
          visualization_callback_(
              problem_data,
              pose_graph,
              start_opt_with_frame,
              next_frame_id,
              VisualizationTypeEnum::AFTER_PGO_PLUS_OBJ_OPTIMIZATION);
        }
      }

      if (run_visual_feature_opt) {
        // Phase I
        LOG(INFO) << "Building optimization";
        std::unordered_map<ceres::ResidualBlockId,
                           std::pair<vslam_types_refactor::FactorType,
                                     vslam_types_refactor::FeatureFactorId>>
            current_residual_block_info =
                optimizer_.buildPoseGraphOptimization(optimization_scope_params,
                                                      residual_params_,
                                                      pose_graph,
                                                      &problem,
                                                      opt_logger);
        std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph> pose_graph_copy =
            pose_graph->makeDeepCopy();
        visualization_callback_(
            problem_data,
            pose_graph,
            start_opt_with_frame,
            next_frame_id,
            VisualizationTypeEnum::BEFORE_EACH_OPTIMIZATION);
        LOG(INFO) << "Solving optimization";
        std::shared_ptr<std::unordered_map<ceres::ResidualBlockId, double>>
            block_ids_and_residuals = std::make_shared<
                std::unordered_map<ceres::ResidualBlockId, double>>();
        if (!optimizer_.solveOptimization(&problem,
                                          solver_params,
                                          ceres_callbacks,
                                          opt_logger,
                                          block_ids_and_residuals)) {
          // TODO do we want to quit or just silently let this iteration fail?
          LOG(ERROR) << "Phase I Optimization failed at max frame id "
                     << next_frame_id;
          return false;
        }
        if (opt_logger.has_value()) {
          opt_logger->writeCurrentOptInfo();
        }
        if (next_frame_id == 67) {
          std::unordered_map<FeatureId, StructuredVisionFeatureTrack>
              visual_features = problem_data.getVisualFeatures();
          std::map<double, ceres::ResidualBlockId, std::greater<double>>
              ordered_residuals_and_block_ids;
          for (const auto &block_id_and_residual : *block_ids_and_residuals) {
            ordered_residuals_and_block_ids.insert(
                {std::fabs(block_id_and_residual.second),
                 block_id_and_residual.first});
          }
          util::BoostHashSet<std::pair<FactorType, FeatureFactorId>>
              feature_factor_types_and_ids;
          std::unordered_map<vslam_types_refactor::FeatureId,
                             ceres::ResidualBlockId>
              feat_ids_and_block_ids;
          std::unordered_set<vslam_types_refactor::FeatureId> feature_ids;
          for (const auto &residual_and_block_id :
               ordered_residuals_and_block_ids) {
            const double residual = residual_and_block_id.first;
            const ceres::ResidualBlockId block_id =
                residual_and_block_id.second;
            if (current_residual_block_info.find(block_id) ==
                current_residual_block_info.end()) {
              LOG(WARNING) << "Somehow cannot find block " << block_id
                           << " in current_residual_block_info";
              continue;
            }
            if (current_residual_block_info.at(block_id).first ==
                kReprojectionErrorFactorTypeId) {
              vslam_types_refactor::FeatureId feature_id;
              pose_graph->getFeatureIdForObservationFactor(
                  current_residual_block_info.at(block_id), feature_id);
              feat_ids_and_block_ids[feature_id] = block_id;
            }
          }

          std::unordered_set<vslam_types_refactor::FeatureId>
              interested_feature_ids;
          std::unordered_map<vslam_types_refactor::CameraId,
                             std::vector<PixelCoord<double>>>
              cam_ids_and_observations;
          for (const auto feat_id_and_block_id : feat_ids_and_block_ids) {
            vslam_types_refactor::FeatureId feat_id =
                feat_id_and_block_id.first;
            if (visual_features.find(feat_id) == visual_features.end()) {
              LOG(INFO) << "Somehow cannot find feature " << feat_id
                        << " in visual_features...";
              continue;
            }
            vslam_types_refactor::FrameId target_frame_id = next_frame_id;
            target_frame_id = 67;
            const std::unordered_map<FrameId, VisionFeature>
                &frame_ids_and_vision_features =
                    visual_features.at(feat_id)
                        .feature_track.feature_observations_;
            if (frame_ids_and_vision_features.find(target_frame_id) ==
                frame_ids_and_vision_features.end()) {
              // std::vector<vslam_types_refactor::FrameId> frame_ids;
              // for (const auto &frame_id_and_vision_feature :
              //      frame_ids_and_vision_features) {
              //   frame_ids.push_back(frame_id_and_vision_feature.first);
              // }
              // std::sort(frame_ids.begin(), frame_ids.end());
              // if (frame_ids.front() < target_frame_id) {
              //   std::cout << "target_frame_id: " << target_frame_id << " - ";
              //   for (const auto frame_id : frame_ids) {
              //     std::cout << frame_id << " ";
              //   }
              //   std::cout << std::endl;
              // }
              continue;
            }
            const std::unordered_map<CameraId, PixelCoord<double>>
                &cam_ids_and_pixels =
                    frame_ids_and_vision_features.at(target_frame_id)
                        .pixel_by_camera_id;
            for (const auto &cam_id_and_pixel : cam_ids_and_pixels) {
              cam_ids_and_observations[cam_id_and_pixel.first].push_back(
                  cam_id_and_pixel.second);
            }
            interested_feature_ids.emplace(feat_id);
          }
          for (const auto &feat_id : interested_feature_ids) {
            LOG(INFO) << "feat_id: " << feat_id
                      << "; residual = " << feat_ids_and_block_ids.at(feat_id);
          }
          for (const auto cam_id_and_observations : cam_ids_and_observations) {
            LOG(INFO) << "camera " << cam_id_and_observations.first << " has "
                      << cam_id_and_observations.second.size()
                      << " observations.";
          }
          std::ofstream ofile;
          const std::string filename = "tmp.txt";
          ofile.open(filename, std::ios::trunc);
          if (!ofile.is_open()) {
            LOG(ERROR) << "failed to open " << filename;
          }
          for (const auto &feat_id : interested_feature_ids) {
            ofile << feat_id << std::endl;
          }
          ofile.close();
        }
        // Phase II
        // if (solver_params.feature_outlier_percentage_ > 0) {
        if (false) {
          LOG(INFO) << "block_ids_and_residuals size: "
                    << block_ids_and_residuals->size();
          if (opt_logger.has_value()) {
            opt_logger->setOptimizationTypeParams(
                next_frame_id, start_opt_with_frame == 0, false, true);
          }
          // revert back to the old pose graph for Phase II optim
          pose_graph->setValuesFromAnotherPoseGraph(pose_graph_copy);
          // use map sort block ids by their corresponding residuals
          std::map<double, ceres::ResidualBlockId, std::greater<double>>
              ordered_residuals_and_block_ids;
          for (const auto &block_id_and_residual : *block_ids_and_residuals) {
            ordered_residuals_and_block_ids.insert(
                {std::fabs(block_id_and_residual.second),
                 block_id_and_residual.first});
          }
          // build excluded_feature_factor_types_and_ids
          util::BoostHashSet<std::pair<FactorType, FeatureFactorId>>
              excluded_feature_factor_types_and_ids;
          size_t n_outliers =
              (size_t)(ordered_residuals_and_block_ids.size() *
                       solver_params.feature_outlier_percentage_);
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
              opt_logger,
              excluded_feature_factor_types_and_ids);

          if (!optimizer_.solveOptimization(&problem,
                                            solver_params,
                                            ceres_callbacks,
                                            opt_logger,
                                            nullptr)) {
            // TODO do we want to quit or just silently let this iteration
            // fail?
            LOG(ERROR) << "Phase II Optimization failed at max frame id "
                       << next_frame_id;
            return false;
          }
          if (opt_logger.has_value()) {
            opt_logger->writeCurrentOptInfo();
          }
        }
        if (optimization_scope_params.allow_reversion_after_dectecting_jumps_) {
          if (!isConsecutivePosesStable_(
                  pose_graph,
                  optimization_scope_params.min_frame_id_,
                  optimization_scope_params.max_frame_id_,
                  optimization_factors_enabled_params
                      .consecutive_pose_transl_tol_,
                  optimization_factors_enabled_params
                      .consecutive_pose_orient_tol_)) {
            LOG(WARNING) << "Detecting jumps after optimization. Reverting...";
            pose_graph->setValuesFromAnotherPoseGraph(pose_graph_copy);
          }
        }

        visualization_callback_(problem_data,
                                pose_graph,
                                start_opt_with_frame,
                                next_frame_id,
                                VisualizationTypeEnum::AFTER_EACH_OPTIMIZATION);
      }
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
                     const VisualizationTypeEnum &)>
      visualization_callback_;

  std::function<pose_graph_optimization::OptimizationSolverParams(
      const FrameId &)>
      solver_params_provider_func_;

  std::function<bool(const FrameId &)> gba_checker_;

  bool isConsecutivePosesStable_(
      const std::shared_ptr<PoseGraphType> &pose_graph,
      const FrameId &min_frame_id,
      const FrameId &max_frame_id,
      const double &kConsecutiveTranslTol,
      const double &kConsecutiveOrientTol) {
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
