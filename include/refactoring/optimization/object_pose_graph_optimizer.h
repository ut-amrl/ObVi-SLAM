//
// Created by amanda on 6/14/22.
//

#ifndef UT_VSLAM_POSE_GRAPH_OPTIMIZER_H
#define UT_VSLAM_POSE_GRAPH_OPTIMIZER_H

#include <base_lib/basic_utils.h>
#include <ceres/ceres.h>
#include <glog/logging.h>
#include <refactoring/optimization/object_pose_graph.h>
#include <refactoring/optimization/optimization_factors_enabled_params.h>
#include <refactoring/optimization/optimization_solver_params.h>

namespace pose_graph_optimizer {

const std::string kPoseTypeStr = "pose";
const std::string kObjTypeStr = "object";
const std::string kFeatureTypeStr = "feature";

template <typename PoseGraphType>
bool getParamBlockForPose(const vslam_types_refactor::FrameId &frame_id,
                          const std::shared_ptr<PoseGraphType> &pose_graph,
                          double **param_block_ptr) {
  return pose_graph->getPosePointers(frame_id, param_block_ptr);
}

template <typename PoseGraphType>
bool getParamBlockForFeature(const vslam_types_refactor::FeatureId &feature_id,
                             const std::shared_ptr<PoseGraphType> &pose_graph,
                             double **param_block_ptr) {
  return pose_graph->getFeaturePointers(feature_id, param_block_ptr);
}

template <typename PoseGraphType>
bool getParamBlockForObject(const vslam_types_refactor::ObjectId &object_id,
                            const std::shared_ptr<PoseGraphType> &pose_graph,
                            double **param_block_ptr) {
  return pose_graph->getObjectParamPointers(object_id, param_block_ptr);
}

template <typename PoseGraphType>
bool getFrameIdsForFrameId(
    const vslam_types_refactor::FrameId &in_frame_id,
    const std::shared_ptr<PoseGraphType> &pose_graph,
    std::pair<vslam_types_refactor::FrameId, vslam_types_refactor::FrameId>
        &min_max_frame_id) {
  min_max_frame_id = std::make_pair(in_frame_id, in_frame_id);
  return true;
}

template <typename PoseGraphType>
bool getMinMaxFramesForFeatureId(
    const vslam_types_refactor::FeatureId &feature_id,
    const std::shared_ptr<PoseGraphType> &pose_graph,
    std::pair<vslam_types_refactor::FrameId, vslam_types_refactor::FrameId>
        &min_max_frame_id) {
  vslam_types_refactor::FrameId max_frame_id;
  vslam_types_refactor::FrameId min_frame_id;
  bool last_frame_result =
      pose_graph->getLastObservedFrameForFeature(feature_id, max_frame_id);
  bool first_frame_result =
      pose_graph->getFirstObservedFrameForFeature(feature_id, min_frame_id);
  if (first_frame_result && last_frame_result) {
    min_max_frame_id = std::make_pair(min_frame_id, max_frame_id);
    return true;
  }
  return false;
}

template <typename PoseGraphType>
bool getLatestObservedFrameForObjectId(
    const vslam_types_refactor::ObjectId &object_id,
    const std::shared_ptr<PoseGraphType> &pose_graph,
    std::pair<vslam_types_refactor::FrameId, vslam_types_refactor::FrameId>
        &min_max_frame_id) {
  vslam_types_refactor::FrameId max_frame_id;
  vslam_types_refactor::FrameId min_frame_id;
  bool last_frame_result =
      pose_graph->getLastObservedFrameForObject(object_id, max_frame_id);
  bool first_frame_result =
      pose_graph->getFirstObservedFrameForObject(object_id, min_frame_id);
  if (first_frame_result && last_frame_result) {
    min_max_frame_id = std::make_pair(min_frame_id, max_frame_id);
    return true;
  }
  return false;
}

template <typename VisualFeatureFactorType,
          typename CachedFactorInfo,
          typename PoseGraphType>
class ObjectPoseGraphOptimizer {
 public:
  ObjectPoseGraphOptimizer(
      const std::function<
          bool(const std::pair<vslam_types_refactor::FactorType,
                               vslam_types_refactor::FeatureFactorId> &,
               const std::shared_ptr<PoseGraphType> &,
               const CachedFactorInfo &)> &refresh_residual_checker,
      const std::function<bool(
          const std::pair<vslam_types_refactor::FactorType,
                          vslam_types_refactor::FeatureFactorId> &,
          const pose_graph_optimization::ObjectVisualPoseGraphResidualParams &,
          const std::shared_ptr<PoseGraphType> &pose_graph,
          ceres::Problem *,
          ceres::ResidualBlockId &,
          CachedFactorInfo &)> &residual_creator)
      : refresh_residual_checker_(refresh_residual_checker),
        residual_creator_(residual_creator) {}

  /**
   *
   * @param optimization_scope
   * @param residual_params
   * @param pose_graph
   * @param problem
   * @return Current residuals in the problem
   */
  std::unordered_map<ceres::ResidualBlockId,
                     std::pair<vslam_types_refactor::FactorType,
                               vslam_types_refactor::FeatureFactorId>>
  buildPoseGraphOptimization(
      const OptimizationScopeParams &optimization_scope,
      const pose_graph_optimization::ObjectVisualPoseGraphResidualParams
          &residual_params,
      std::shared_ptr<PoseGraphType> &pose_graph,
      ceres::Problem *problem,
      const util::BoostHashSet<std::pair<vslam_types_refactor::FactorType,
                                         vslam_types_refactor::FeatureFactorId>>
          &excluded_feature_factor_types_and_ids = {}) {
    // Check for invalid combinations of scope and reject
    CHECK(checkInvalidOptimizationScopeParams(optimization_scope));

    std::unordered_set<vslam_types_refactor::FrameId> optimized_frames;
    std::unordered_set<vslam_types_refactor::ObjectId> ltm_object_ids;

    std::unordered_map<
        vslam_types_refactor::FeatureId,
        util::BoostHashSet<std::pair<vslam_types_refactor::FactorType,
                                     vslam_types_refactor::FeatureFactorId>>>
        features_to_include;
    std::unordered_map<
        vslam_types_refactor::ObjectId,
        util::BoostHashSet<std::pair<vslam_types_refactor::FactorType,
                                     vslam_types_refactor::FeatureFactorId>>>
        objects_to_include;

    bool use_object_only_factors = false;  // POM, shape prior, etc
    if (optimization_scope.include_object_factors_) {
      if (!optimization_scope.fix_objects_) {
        use_object_only_factors = true;
      }
    }

    bool use_feature_pose_factors = optimization_scope.include_visual_factors_;
    bool use_object_pose_factors = optimization_scope.include_object_factors_;
    bool use_object_param_blocks = optimization_scope.include_object_factors_;
    bool fix_object_param_blocks = optimization_scope.fix_objects_;
    bool fix_ltm_param_blocks =
        optimization_scope.fix_objects_ || optimization_scope.fix_ltm_objects_;
    bool use_visual_feature_param_blocks =
        optimization_scope.include_visual_factors_;
    bool fix_visual_feature_param_blocks =
        optimization_scope.fix_visual_features_;
    bool fix_pose_param_blocks = optimization_scope.fix_poses_;

    std::unordered_map<
        vslam_types_refactor::FactorType,
        std::unordered_set<vslam_types_refactor::FeatureFactorId>>
        required_feature_factors;

    // TODO do we run into a problem of unstability of the min node is the
    // only one that has observed the feature (do we loose all of that past
    // information?)
    std::unordered_set<vslam_types_refactor::FrameId> all_frames =
        pose_graph->getFrameIds();
    for (const vslam_types_refactor::FrameId &frame_id : all_frames) {
      if ((frame_id >= optimization_scope.min_frame_id_) &&
          (frame_id <= optimization_scope.max_frame_id_)) {
        optimized_frames.insert(frame_id);
      }
    }

    if (use_feature_pose_factors) {
      //      LOG(INFO) << "Using feature-pose factors";
      util::BoostHashSet<std::pair<vslam_types_refactor::FactorType,
                                   vslam_types_refactor::FeatureFactorId>>
          matching_visual_feature_factors;

      pose_graph->getVisualFeatureFactorIdsBetweenFrameIdsInclusive(
          optimization_scope.min_frame_id_,
          optimization_scope.max_frame_id_,
          matching_visual_feature_factors);
      std::function<bool(
          const std::pair<vslam_types_refactor::FactorType,
                          vslam_types_refactor::FeatureFactorId> &,
          vslam_types_refactor::FeatureId &)>
          id_for_factor_retriever =
              [&](const std::pair<vslam_types_refactor::FactorType,
                                  vslam_types_refactor::FeatureFactorId>
                      &factor_info,
                  vslam_types_refactor::FeatureId &feature_id) {
                return pose_graph->getFeatureIdForObservationFactor(factor_info,
                                                                    feature_id);
              };
      extractObservationFactorsForEntityTypeToInclude(
          pose_graph,
          matching_visual_feature_factors,
          optimization_scope.factor_types_to_exclude,
          id_for_factor_retriever,
          features_to_include,
          excluded_feature_factor_types_and_ids);
      applyMinObservationRequirementsToIncludedFactors(
          optimization_scope.min_low_level_feature_observations_,
          features_to_include,
          required_feature_factors);
    }

    if (use_object_param_blocks) {
      // TODO -- if no objects in the long-term map are observed, then nothing
      // in the long-term map will change,
      //  so we could omit those objects, the object only factors, and the
      //  factors relating objects in the long-term map
      pose_graph->getLongTermMapObjects(ltm_object_ids);
    }
    if (use_object_pose_factors) {
      util::BoostHashSet<std::pair<vslam_types_refactor::FactorType,
                                   vslam_types_refactor::FeatureFactorId>>
          matching_observation_factor_ids;
      pose_graph->getObservationFactorsBetweenFrameIdsInclusive(
          optimization_scope.min_frame_id_,
          optimization_scope.max_frame_id_,
          matching_observation_factor_ids);
      std::function<bool(
          const std::pair<vslam_types_refactor::FactorType,
                          vslam_types_refactor::FeatureFactorId> &,
          vslam_types_refactor::ObjectId &)>
          id_for_factor_retriever =
              [&](const std::pair<vslam_types_refactor::FactorType,
                                  vslam_types_refactor::FeatureFactorId>
                      &factor_info,
                  vslam_types_refactor::FeatureId &object_id) {
                return pose_graph->getObjectIdForObjObservationFactor(
                    factor_info, object_id);
              };
      extractObservationFactorsForEntityTypeToInclude(
          pose_graph,
          matching_observation_factor_ids,
          optimization_scope.factor_types_to_exclude,
          id_for_factor_retriever,
          objects_to_include,
          excluded_feature_factor_types_and_ids);
      applyMinObservationRequirementsToIncludedFactors(
          optimization_scope.min_object_observations_,
          objects_to_include,
          required_feature_factors);
    }

    if (use_object_only_factors) {
      //      LOG(INFO) << "Using object only factors";
      // Get the object-only factors that are required
      std::unordered_map<
          vslam_types_refactor::ObjectId,
          util::BoostHashSet<std::pair<vslam_types_refactor::FactorType,
                                       vslam_types_refactor::FeatureFactorId>>>
          matching_obj_only_factors_by_obj;
      std::unordered_set<vslam_types_refactor::ObjectId>
          objects_with_object_only_factors;
      if (fix_ltm_param_blocks) {
        // If the long term map is fixed, we only need the object-only factors
        // for the objects not in the long-term map
        // If at some point we marginalize mid-run, we may need to look for the
        // LTM objects that are connected to the observed ones (rather than just
        // the observed ones)
        for (const auto &observed_obj_data : objects_to_include) {
          vslam_types_refactor::ObjectId observed_obj = observed_obj_data.first;
          if (ltm_object_ids.find(observed_obj) == ltm_object_ids.end()) {
            objects_with_object_only_factors.insert(observed_obj);
          }
        }
      } else {
        // If the long term map is not fixed, we need the object-only factors
        // for the observed objects and those in the long-term map
        for (const auto &observed_obj_data : objects_to_include) {
          vslam_types_refactor::ObjectId observed_obj = observed_obj_data.first;
          objects_with_object_only_factors.insert(observed_obj);
        }
        objects_with_object_only_factors.insert(ltm_object_ids.begin(),
                                                ltm_object_ids.end());
      }

      pose_graph->getOnlyObjectFactorsForObjects(
          objects_with_object_only_factors,
          optimization_scope.use_pom_,
          true,
          matching_obj_only_factors_by_obj);
      // This assumes we're not having the option to exclude object only factors
      // due to outliers. If this becomes the case, then we'll have to revise
      // this
      for (const auto &obj_only_factors_by_obj :
           matching_obj_only_factors_by_obj) {
        objects_to_include[obj_only_factors_by_obj.first].insert(
            obj_only_factors_by_obj.second.begin(),
            obj_only_factors_by_obj.second.end());
        for (const std::pair<vslam_types_refactor::FactorType,
                             vslam_types_refactor::FeatureFactorId>
                 &obj_only_factor : obj_only_factors_by_obj.second) {
          required_feature_factors[obj_only_factor.first].insert(
              obj_only_factor.second);
        }
      }
    }

    for (const vslam_types_refactor::FactorType &type_to_exclude :
         optimization_scope.factor_types_to_exclude) {
      required_feature_factors.erase(type_to_exclude);
    }
    // Remove unused residual blocks from the problem
    removeUnnecessaryResidualBlocks(required_feature_factors, problem);
    // Add residual blocks that are required but not added and refresh any that
    // exist but are stale
    addOrRefreshResidualBlocksForRequiredFactors(
        residual_params, required_feature_factors, pose_graph, problem);
    addParamBlocksWithIdentifiers(optimized_frames,
                                  kPoseTypeStr,
                                  getParamBlockForPose<PoseGraphType>,
                                  pose_graph,
                                  6,
                                  problem);

    if (fix_pose_param_blocks) {
      // Set all pose param blocks constant
      setVariabilityForParamBlocks(optimized_frames,
                                   true,
                                   kPoseTypeStr,
                                   getParamBlockForPose<PoseGraphType>,
                                   pose_graph,
                                   problem);
    } else {
      // Set only first pose param block constant (or add Gaussian prior
      // later...?)
      std::unordered_set<vslam_types_refactor::FrameId> constant_frames;
      // Set all others not-constant (if were set constant)
      std::unordered_set<vslam_types_refactor::FrameId> variable_frames =
          optimized_frames;

      if (optimization_scope.min_frame_id_ == 0) {
        constant_frames.insert(optimization_scope.min_frame_id_);
        variable_frames.erase(optimization_scope.min_frame_id_);
      } else {
        //         TODO should this be poses in addition to the window, or the
        //         number of poses from the prescribed window to keep constant
        uint32_t min_const_frames = std::max(
            (uint32_t)1,
            optimization_scope.poses_prior_to_window_to_keep_constant_);
        for (uint32_t const_frame_idx = 0; const_frame_idx < min_const_frames;
             const_frame_idx++) {
          uint64_t frame_to_keep_const =
              optimization_scope.min_frame_id_ + const_frame_idx;
          if (frame_to_keep_const > optimization_scope.max_frame_id_) {
            break;
          }
          constant_frames.insert(frame_to_keep_const);
          variable_frames.erase(frame_to_keep_const);
        }
      }
      setVariabilityForParamBlocks(constant_frames,
                                   true,
                                   kPoseTypeStr,
                                   getParamBlockForPose<PoseGraphType>,
                                   pose_graph,
                                   problem);
      setVariabilityForParamBlocks(variable_frames,
                                   false,
                                   kPoseTypeStr,
                                   getParamBlockForPose<PoseGraphType>,
                                   pose_graph,
                                   problem);
    }

    // Remove unused poses
    std::unordered_set<vslam_types_refactor::FrameId> frames_to_remove;
    //    std::set_difference(
    //        last_optimized_nodes_.begin(),
    //        last_optimized_nodes_.end(),
    //        optimized_frames.begin(),
    //        optimized_frames.end(),
    //        std::inserter(frames_to_remove, frames_to_remove.end()));
    for (const vslam_types_refactor::ObjectId &last_opt :
         last_optimized_nodes_) {
      if (optimized_frames.find(last_opt) == optimized_frames.end()) {
        frames_to_remove.insert(last_opt);
      }
    }
    removeParamBlocksWithIdentifiers(frames_to_remove,
                                     kPoseTypeStr,
                                     getParamBlockForPose<PoseGraphType>,
                                     pose_graph,
                                     problem);
    last_optimized_nodes_ = optimized_frames;

    std::unordered_set<vslam_types_refactor::FeatureId> features_to_remove;
    std::unordered_set<vslam_types_refactor::FeatureId>
        next_last_optimized_features;
    if (use_visual_feature_param_blocks) {
      //      LOG(INFO) << "Setting variability of feature param blocks";
      bool set_constant;
      if (fix_visual_feature_param_blocks) {
        // Fix all remaining visual feature param blocks
        set_constant = true;
      } else {
        // Set all others not-constant (if were set constant)
        set_constant = false;
      }

      //      std::set_difference(
      //          last_optimized_features_.begin(),
      //          last_optimized_features_.end(),
      //          optimized_features.begin(),
      //          optimized_features.end(),
      //          std::inserter(features_to_remove, features_to_remove.end()));
      for (const vslam_types_refactor::FeatureId &last_opt :
           last_optimized_features_) {
        if (features_to_include.find(last_opt) == features_to_include.end()) {
          features_to_remove.insert(last_opt);
        }
      }
      for (const auto &feature : features_to_include) {
        next_last_optimized_features.insert(feature.first);
      }
      setVariabilityForParamBlocks(next_last_optimized_features,
                                   set_constant,
                                   kFeatureTypeStr,
                                   getParamBlockForFeature<PoseGraphType>,
                                   pose_graph,
                                   problem);
    } else {
      // Remove all visual feature param blocks
      features_to_remove = last_optimized_features_;
      next_last_optimized_features = {};
    }
    removeParamBlocksWithIdentifiers(features_to_remove,
                                     kFeatureTypeStr,
                                     getParamBlockForFeature<PoseGraphType>,
                                     pose_graph,
                                     problem);
    last_optimized_features_ = next_last_optimized_features;

    std::unordered_set<vslam_types_refactor::ObjectId> objects_to_remove;
    std::unordered_set<vslam_types_refactor::ObjectId>
        next_last_optimized_objects;
    if (use_object_param_blocks) {
      std::unordered_set<vslam_types_refactor::ObjectId>
          constant_object_param_blocks;
      std::unordered_set<vslam_types_refactor::ObjectId>
          variable_object_param_blocks;
      std::unordered_set<vslam_types_refactor::ObjectId> observed_objects;
      for (const auto &obj_to_include : objects_to_include) {
        observed_objects.insert(obj_to_include.first);
      }

      if (fix_object_param_blocks) {
        constant_object_param_blocks = observed_objects;
        next_last_optimized_objects = observed_objects;
      } else if (fix_ltm_param_blocks) {
        // The variable ones are those that are observed but not in the
        // long-term map
        //        std::set_difference(observed_objects.begin(),
        //                            observed_objects.end(),
        //                            ltm_object_ids.begin(),
        //                            ltm_object_ids.end(),
        //                            std::inserter(variable_object_param_blocks,
        //                                          variable_object_param_blocks.end()));
        for (const vslam_types_refactor::ObjectId &obs_obj : observed_objects) {
          if (ltm_object_ids.find(obs_obj) == ltm_object_ids.end()) {
            variable_object_param_blocks.insert(obs_obj);
          } else {
            constant_object_param_blocks.insert(obs_obj);
          }
        }
        // The constant ones are the ones that are observed and in the long-term
        // map
        //        std::set_intersection(
        //            ltm_object_ids.begin(),
        //            ltm_object_ids.end(),
        //            observed_objects.begin(),
        //            observed_objects.end(),
        //            std::inserter(constant_object_param_blocks,
        //                          constant_object_param_blocks.end()));
        next_last_optimized_objects = observed_objects;
      } else {
        //        std::set_union(observed_objects.begin(),
        //                       observed_objects.end(),
        //                       ltm_object_ids.begin(),
        //                       ltm_object_ids.end(),
        //                       std::inserter(variable_object_param_blocks,
        //                                     variable_object_param_blocks.end()));
        for (const vslam_types_refactor::ObjectId &observed_obj :
             observed_objects) {
          variable_object_param_blocks.insert(observed_obj);
        }
        for (const vslam_types_refactor::ObjectId &ltm_obj : ltm_object_ids) {
          variable_object_param_blocks.insert(ltm_obj);
        }
        next_last_optimized_objects = variable_object_param_blocks;
      }

      // TODO we could also say that the next_last_optimized_objects is just a
      // union of the constant and variable param blocks, but having it defined
      // per case seems like less need for union when it may not be necessary
      // Similarly, this next line could be done for all cases, but it seems
      // wasteful to run when we know next_last... will be empty (as in the case
      // where we don't have the object param blocks at all)
      //      std::set_difference(
      //          last_optimized_objects_.begin(),
      //          last_optimized_objects_.end(),
      //          next_last_optimized_objects.begin(),
      //          next_last_optimized_objects.end(),
      //          std::inserter(objects_to_remove, objects_to_remove.end()));
      for (const vslam_types_refactor::ObjectId &last_opt :
           last_optimized_objects_) {
        if (next_last_optimized_objects.find(last_opt) ==
            next_last_optimized_objects.end()) {
          objects_to_remove.insert(last_opt);
        }
      }

      if (!constant_object_param_blocks.empty()) {
        setVariabilityForParamBlocks(constant_object_param_blocks,
                                     true,
                                     kObjTypeStr,
                                     getParamBlockForObject<PoseGraphType>,
                                     pose_graph,
                                     problem);
      }
      if (!variable_object_param_blocks.empty()) {
        setVariabilityForParamBlocks(variable_object_param_blocks,
                                     false,
                                     kObjTypeStr,
                                     getParamBlockForObject<PoseGraphType>,
                                     pose_graph,
                                     problem);
      }
    } else {
      objects_to_remove = last_optimized_objects_;
      next_last_optimized_objects = {};
    }
    removeParamBlocksWithIdentifiers(objects_to_remove,
                                     kObjTypeStr,
                                     getParamBlockForObject<PoseGraphType>,
                                     pose_graph,
                                     problem);
    last_optimized_objects_ = next_last_optimized_objects;
    std::unordered_map<ceres::ResidualBlockId,
                       std::pair<vslam_types_refactor::FactorType,
                                 vslam_types_refactor::FeatureFactorId>>
        current_residual_block_info;
    for (const auto &factor_type_and_residuals :
         residual_blocks_and_cached_info_by_factor_id_) {
      for (const auto &factor_id_and_info : factor_type_and_residuals.second) {
        current_residual_block_info[factor_id_and_info.second.first] =
            std::make_pair(factor_type_and_residuals.first,
                           factor_id_and_info.first);
      }
    }
    return current_residual_block_info;
  }

  bool solveOptimization(
      ceres::Problem *problem,
      const pose_graph_optimization::OptimizationSolverParams &solver_params,
      const std::vector<std::shared_ptr<ceres::IterationCallback>> callbacks,
      std::shared_ptr<std::unordered_map<ceres::ResidualBlockId, double>>
          block_ids_and_residuals_ptr = nullptr) {
    CHECK(problem != NULL);
    ceres::Solver::Options options;
    // TODO configure options

    // Set up callbacks
    //    options.callbacks = callbacks;
    for (const std::shared_ptr<ceres::IterationCallback> &callback_smart_ptr :
         callbacks) {
      options.callbacks.emplace_back(callback_smart_ptr.get());
    }
    if (!callbacks.empty()) {
      options.update_state_every_iteration = true;
    }
    options.max_num_iterations = solver_params.max_num_iterations_;
    options.num_threads = 10;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.use_nonmonotonic_steps = solver_params.allow_non_monotonic_steps_;
    options.function_tolerance = solver_params.function_tolerance_;
    options.gradient_tolerance = solver_params.gradient_tolerance_;
    options.parameter_tolerance = solver_params.parameter_tolerance_;

    ceres::Solver::Summary summary;
    ceres::Solve(options, problem, &summary);
    LOG(INFO) << summary.FullReport();

    if (block_ids_and_residuals_ptr != nullptr) {
      std::vector<ceres::ResidualBlockId> residual_block_ids;
      problem->GetResidualBlocks(&residual_block_ids);
      ceres::Problem::EvaluateOptions eval_options;
      eval_options.apply_loss_function = false;
      eval_options.residual_blocks = residual_block_ids;
      std::vector<double> residuals;
      problem->Evaluate(eval_options, nullptr, &residuals, nullptr, nullptr);
      for (size_t i = 0; i < residual_block_ids.size(); ++i) {
        const ceres::ResidualBlockId &block_id = residual_block_ids[i];
        const double &residual = residuals[i];
        block_ids_and_residuals_ptr->insert({block_id, residual});
      }
    }

    if ((summary.termination_type == ceres::TerminationType::FAILURE) ||
        (summary.termination_type == ceres::TerminationType::USER_FAILURE)) {
      LOG(ERROR) << "Ceres optimization failed";
    }
    LOG(INFO) << "Optimization complete";
    return summary.IsSolutionUsable();
  }

  /**
   * Clear the data stored related to optimization in this optimizer. Note that
   * the ceres problem will also need to be cleaned up (or just construct a new
   * one).
   */
  void clearPastOptimizationData() {
    last_optimized_objects_.clear();
    last_optimized_features_.clear();
    last_optimized_nodes_.clear();
    residual_blocks_and_cached_info_by_factor_id_.clear();
  }

 protected:
 private:
  std::unordered_set<vslam_types_refactor::ObjectId> last_optimized_objects_;
  std::unordered_set<vslam_types_refactor::FeatureId> last_optimized_features_;
  std::unordered_set<vslam_types_refactor::FrameId> last_optimized_nodes_;
  std::unordered_map<
      vslam_types_refactor::FactorType,
      std::unordered_map<vslam_types_refactor::FeatureFactorId,
                         std::pair<ceres::ResidualBlockId, CachedFactorInfo>>>
      residual_blocks_and_cached_info_by_factor_id_;

  // TODO include pose graph in the signature?
  std::function<bool(const std::pair<vslam_types_refactor::FactorType,
                                     vslam_types_refactor::FeatureFactorId> &,
                     const std::shared_ptr<PoseGraphType> &,
                     const CachedFactorInfo &)>
      refresh_residual_checker_;
  std::function<bool(
      const std::pair<vslam_types_refactor::FactorType,
                      vslam_types_refactor::FeatureFactorId> &,
      const pose_graph_optimization::ObjectVisualPoseGraphResidualParams &,
      const std::shared_ptr<PoseGraphType> &,
      ceres::Problem *,
      ceres::ResidualBlockId &,
      CachedFactorInfo &)>
      residual_creator_;

  template <typename IdType>
  void applyMinObservationRequirementsToIncludedFactors(
      const size_t &min_obs_requirement,
      std::unordered_map<
          IdType,
          util::BoostHashSet<std::pair<vslam_types_refactor::FactorType,
                                       vslam_types_refactor::FeatureFactorId>>>
          &factors_to_include_by_id,
      std::unordered_map<
          vslam_types_refactor::FactorType,
          std::unordered_set<vslam_types_refactor::FeatureFactorId>>
          &required_feature_factors) {
    std::unordered_map<
        IdType,
        util::BoostHashSet<std::pair<vslam_types_refactor::FactorType,
                                     vslam_types_refactor::FeatureFactorId>>>
        new_factors_to_include;
    // TODO (Taijing) As this is an unordered_map, erase should be faster
    for (const auto &factor_to_include : factors_to_include_by_id) {
      if (factor_to_include.second.size() >= min_obs_requirement) {
        new_factors_to_include[factor_to_include.first] =
            factor_to_include.second;
      }
    }
    for (const auto &factor_ids_and_factor_sets : new_factors_to_include) {
      for (const auto &factor_type_and_factor_id :
           factor_ids_and_factor_sets.second) {
        required_feature_factors[factor_type_and_factor_id.first].insert(
            factor_type_and_factor_id.second);
      }
    }
    factors_to_include_by_id = new_factors_to_include;
  }

  template <typename IdType>
  void extractObservationFactorsForEntityTypeToInclude(
      const std::shared_ptr<PoseGraphType> &pose_graph,
      const util::BoostHashSet<std::pair<vslam_types_refactor::FactorType,
                                         vslam_types_refactor::FeatureFactorId>>
          &matching_factors,
      const std::unordered_set<vslam_types_refactor::FactorType>
          &factor_types_to_exclude,
      const std::function<
          bool(const std::pair<vslam_types_refactor::FactorType,
                               vslam_types_refactor::FeatureFactorId> &,
               IdType &)> &id_for_factor_retriever,
      std::unordered_map<
          IdType,
          util::BoostHashSet<std::pair<vslam_types_refactor::FactorType,
                                       vslam_types_refactor::FeatureFactorId>>>
          &factors_to_include,
      const util::BoostHashSet<std::pair<vslam_types_refactor::FactorType,
                                         vslam_types_refactor::FeatureFactorId>>
          &excluded_feature_factor_types_and_ids = {}) {
    size_t excluded_count = 0;
    for (const auto &matching_factor : matching_factors) {
      const vslam_types_refactor::FactorType &factor_type =
          matching_factor.first;
      if (factor_types_to_exclude.find(factor_type) !=
          factor_types_to_exclude.end()) {
        continue;
      }
      // If we want to exclude this individual feature
      if (excluded_feature_factor_types_and_ids.find(matching_factor) !=
          excluded_feature_factor_types_and_ids.end()) {
        if (factor_type == vslam_types_refactor::kLongTermMapFactorTypeId ||
            factor_type == vslam_types_refactor::kShapeDimPriorFactorTypeId) {
          LOG(ERROR)
              << "You shouldn't exclude long-term map objects or shape priors "
              << "when building pose graph!";
        } else {
          excluded_count++;
          continue;
        }
      }

      // This assumes all observations only affect one type of entity. If code
      // evolves, this could potentially change.
      IdType id;
      if (!id_for_factor_retriever(matching_factor, id)) {
        LOG(ERROR) << "Could not find id for factor " << matching_factor.first
                   << ", " << matching_factor.second;
        continue;
      }
      factors_to_include[id].insert(matching_factor);
    }
    if (!excluded_feature_factor_types_and_ids.empty()) {
      LOG(INFO) << "Excluded " << excluded_count << " out of "
                << excluded_feature_factor_types_and_ids.size()
                << " to exclude";
    }
  }

  template <typename IdentifierType>
  void setVariabilityForParamBlocks(
      const std::unordered_set<IdentifierType> &param_identifiers,
      const bool &set_constant,
      const std::string &identifier_type,
      const std::function<bool(const IdentifierType &,
                               const std::shared_ptr<PoseGraphType> &,
                               double **)> &param_block_retriever,
      const std::shared_ptr<PoseGraphType> &pose_graph,
      ceres::Problem *problem) {
    for (const IdentifierType &param_identifier : param_identifiers) {
      double *param_ptr = NULL;
      if (param_block_retriever(param_identifier, pose_graph, &param_ptr)) {
        if (set_constant) {
          problem->SetParameterBlockConstant(param_ptr);
        } else {
          problem->SetParameterBlockVariable(param_ptr);
        }
      } else {
        LOG(WARNING) << "No parameter block found for object with id "
                     << identifier_type
                     << "; not able to set variability in optimization problem";
      }
    }
  }

  template <typename IdentifierType>
  void removeParamBlocksWithIdentifiers(
      const std::unordered_set<IdentifierType> &identifiers,
      const std::string &identifier_type,
      const std::function<bool(const IdentifierType &,
                               const std::shared_ptr<PoseGraphType> &,
                               double **)> &param_block_retriever,
      const std::shared_ptr<PoseGraphType> &pose_graph,
      ceres::Problem *problem) {
    for (const IdentifierType &id_to_remove : identifiers) {
      double *param_block;
      if (param_block_retriever(id_to_remove, pose_graph, &param_block)) {
        problem->RemoveParameterBlock(param_block);
      } else {
        LOG(WARNING) << "No parameter block found for " << identifier_type
                     << " with id " << id_to_remove
                     << "; not removing from optimization problem";
      }
    }
  }

  template <typename IdentifierType>
  void addParamBlocksWithIdentifiers(
      const std::unordered_set<IdentifierType> &identifiers,
      const std::string &identifier_type,
      const std::function<bool(const IdentifierType &,
                               const std::shared_ptr<PoseGraphType> &,
                               double **)> &param_block_retriever,
      const std::shared_ptr<PoseGraphType> &pose_graph,
      const int &param_block_size,
      ceres::Problem *problem) {
    for (const IdentifierType &id_to_add : identifiers) {
      double *param_block;
      if (param_block_retriever(id_to_add, pose_graph, &param_block)) {
        problem->AddParameterBlock(param_block, param_block_size);
      } else {
        LOG(WARNING) << "No parameter block found for " << identifier_type
                     << " with id " << id_to_add
                     << "; not adding from optimization problem";
      }
    }
  }

  void addOrRefreshResidualBlocksForRequiredFactors(
      const pose_graph_optimization::ObjectVisualPoseGraphResidualParams
          &residual_params,
      const std::unordered_map<
          vslam_types_refactor::FactorType,
          std::unordered_set<vslam_types_refactor::FeatureFactorId>>
          &required_feature_factors,
      const std::shared_ptr<PoseGraphType> &pose_graph,
      ceres::Problem *problem) {
    for (const auto &factor_type_and_required_factors :
         required_feature_factors) {
      vslam_types_refactor::FactorType factor_type =
          factor_type_and_required_factors.first;
      bool residuals_for_factor_type_exist =
          residual_blocks_and_cached_info_by_factor_id_.find(factor_type) !=
          residual_blocks_and_cached_info_by_factor_id_.end();
      for (const auto &factor_id : factor_type_and_required_factors.second) {
        std::pair<vslam_types_refactor::FactorType,
                  vslam_types_refactor::FeatureFactorId>
            factor_type_id_pair = std::make_pair(factor_type, factor_id);
        bool create_new = false;
        if (!residuals_for_factor_type_exist ||
            (residual_blocks_and_cached_info_by_factor_id_[factor_type].find(
                 factor_id) ==
             residual_blocks_and_cached_info_by_factor_id_[factor_type]
                 .end())) {
          create_new = true;
        } else {
          // Check if it should be refreshed
          std::pair<ceres::ResidualBlockId, CachedFactorInfo> existing_info =
              residual_blocks_and_cached_info_by_factor_id_[factor_type]
                                                           [factor_id];
          bool refresh = refresh_residual_checker_(
              factor_type_id_pair, pose_graph, existing_info.second);
          if (refresh) {
            // TODO verify that this should go here
            problem->RemoveResidualBlock(existing_info.first);
            residual_blocks_and_cached_info_by_factor_id_[factor_type].erase(
                factor_id);
            create_new = true;
          }
        }
        if (create_new) {
          ceres::ResidualBlockId residual_id;
          CachedFactorInfo cached_info;
          bool residual_success = residual_creator_(factor_type_id_pair,
                                                    residual_params,
                                                    pose_graph,
                                                    problem,
                                                    residual_id,
                                                    cached_info);
          if (residual_success) {
            residual_blocks_and_cached_info_by_factor_id_[factor_type]
                                                         [factor_id] =
                                                             std::make_pair(
                                                                 residual_id,
                                                                 cached_info);
          } else {
            LOG(ERROR) << "Could not make residual for factor type "
                       << factor_type << " and factor id " << factor_id;
          }
        }
      }
    }
  }

  bool checkInvalidOptimizationScopeParams(
      const OptimizationScopeParams &optimization_scope) {
    if (!optimization_scope.include_object_factors_ &&
        !optimization_scope.include_visual_factors_) {
      LOG(ERROR) << "Include object factors and include visual factors were "
                    "both false -- invalid optimization scope";
      return false;
    }
    if (!optimization_scope.include_object_factors_) {
      if (optimization_scope.fix_visual_features_ &&
          optimization_scope.fix_poses_) {
        LOG(ERROR) << "Not including objects and all other parameters are "
                      "fixed -- invalid optimization scope";
        return false;
      }
    } else {
      if (optimization_scope.fix_objects_ && optimization_scope.fix_poses_) {
        LOG(ERROR) << "Want to include object factors, but all parameters "
                      "used by them are fixed  -- invalid optimization scope.";
        return false;
      }
    }
    if (!optimization_scope.include_visual_factors_) {
      if (optimization_scope.fix_objects_ && optimization_scope.fix_poses_) {
        LOG(ERROR) << "Not including visual features and all other parameters "
                      "are fixed -- invalid optimization scope";
        return false;
      }
    } else {
      if (optimization_scope.fix_visual_features_ &&
          optimization_scope.fix_poses_) {
        LOG(ERROR)
            << "Want to include visual feature factors, but all parameters "
               "used by them are fixed  -- invalid optimization scope.";
        return false;
      }
    }

    if (optimization_scope.fix_objects_ &&
        optimization_scope.fix_visual_features_ &&
        optimization_scope.fix_poses_) {
      LOG(ERROR) << "All parameters are fixed -- invalid optimization scope";
      return false;
    }

    return true;
  }

  void removeUnnecessaryResidualBlocks(
      const std::unordered_map<
          vslam_types_refactor::FactorType,
          std::unordered_set<vslam_types_refactor::FeatureFactorId>>
          &required_feature_factors,
      ceres::Problem *problem) {
    std::unordered_map<
        vslam_types_refactor::FactorType,
        std::unordered_set<vslam_types_refactor::FeatureFactorId>>
        removed_factors;
    std::unordered_set<ceres::ResidualBlockId> residual_blocks_to_remove;
    for (const auto &factor_type_and_factor_ids :
         residual_blocks_and_cached_info_by_factor_id_) {
      vslam_types_refactor::FactorType factor_type =
          factor_type_and_factor_ids.first;
      if (required_feature_factors.find(factor_type) ==
          required_feature_factors.end()) {
        for (const auto &factor_id_and_residual :
             factor_type_and_factor_ids.second) {
          removed_factors[factor_type].insert(factor_id_and_residual.first);
          residual_blocks_to_remove.insert(factor_id_and_residual.second.first);
        }
      } else {
        std::unordered_set<vslam_types_refactor::FeatureFactorId>
            required_factors_for_type =
                required_feature_factors.at(factor_type);
        for (const auto &factor_id_and_residual :
             factor_type_and_factor_ids.second) {
          if (required_factors_for_type.find(factor_id_and_residual.first) ==
              required_factors_for_type.end()) {
            removed_factors[factor_type].insert(factor_id_and_residual.first);
            residual_blocks_to_remove.insert(
                factor_id_and_residual.second.first);
          }
        }
      }
    }

    for (const ceres::ResidualBlockId &residual_block :
         residual_blocks_to_remove) {
      problem->RemoveResidualBlock(residual_block);
    }
    for (const auto &factor_type_and_removed_features : removed_factors) {
      for (const vslam_types_refactor::FeatureFactorId &factor_id :
           factor_type_and_removed_features.second) {
        residual_blocks_and_cached_info_by_factor_id_
            [factor_type_and_removed_features.first]
                .erase(factor_id);
      }
    }
  }
};
}  // namespace pose_graph_optimizer

#endif  // UT_VSLAM_POSE_GRAPH_OPTIMIZER_H
