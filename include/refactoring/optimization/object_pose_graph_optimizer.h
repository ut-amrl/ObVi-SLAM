//
// Created by amanda on 6/14/22.
//

#ifndef UT_VSLAM_POSE_GRAPH_OPTIMIZER_H
#define UT_VSLAM_POSE_GRAPH_OPTIMIZER_H

#include <base_lib/basic_utils.h>
#include <ceres/ceres.h>
#include <glog/logging.h>
#include <refactoring/optimization/object_pose_graph.h>
#include <refactoring/optimization/optimization_solver_params.h>

namespace pose_graph_optimizer {

const std::string kPoseTypeStr = "pose";
const std::string kObjTypeStr = "object";
const std::string kFeatureTypeStr = "feature";

struct OptimizationScopeParams {
  bool include_object_factors_;
  bool include_visual_factors_;
  bool fix_poses_;
  bool fix_objects_;
  bool fix_visual_features_;
  bool use_pom_;  // Effectively false if fix_objects_ is true
  vslam_types_refactor::FrameId min_frame_id_;
  vslam_types_refactor::FrameId max_frame_id_;
  // TODO consider adding set of nodes to optimize -- for now, we'll just assume
  // that we have min_frame_id (held constant) and all poses above that
};

template <typename VisualFeatureFactorType>
bool getParamBlockForPose(
    const vslam_types_refactor::FrameId &frame_id,
    const std::shared_ptr<vslam_types_refactor::ObjAndLowLevelFeaturePoseGraph<
        VisualFeatureFactorType>> &pose_graph,
    double **param_block_ptr) {
  return pose_graph->getPosePointers(frame_id, param_block_ptr);
}

template <typename VisualFeatureFactorType>
bool getParamBlockForFeature(
    const vslam_types_refactor::FeatureId &feature_id,
    const std::shared_ptr<vslam_types_refactor::ObjAndLowLevelFeaturePoseGraph<
        VisualFeatureFactorType>> &pose_graph,
    double **param_block_ptr) {
  return pose_graph->getFeaturePointers(feature_id, param_block_ptr);
}

template <typename VisualFeatureFactorType>
bool getParamBlockForObject(
    const vslam_types_refactor::ObjectId &object_id,
    const std::shared_ptr<vslam_types_refactor::ObjAndLowLevelFeaturePoseGraph<
        VisualFeatureFactorType>> &pose_graph,
    double **param_block_ptr) {
  return pose_graph->getObjectParamPointers(object_id, param_block_ptr);
}

template <typename VisualFeatureFactorType>
bool getFrameIdForFrameId(
    const vslam_types_refactor::FrameId &in_frame_id,
    const std::shared_ptr<vslam_types_refactor::ObjAndLowLevelFeaturePoseGraph<
        VisualFeatureFactorType>> &pose_graph,
    std::pair<vslam_types_refactor::FrameId, vslam_types_refactor::FrameId>
        &min_max_frame_id) {
  min_max_frame_id = std::make_pair(in_frame_id, in_frame_id);
  return true;
}

template <typename VisualFeatureFactorType>
bool getMinMaxFramesForFeatureId(
    const vslam_types_refactor::FeatureId &feature_id,
    const std::shared_ptr<vslam_types_refactor::ObjAndLowLevelFeaturePoseGraph<
        VisualFeatureFactorType>> &pose_graph,
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

template <typename VisualFeatureFactorType>
bool getLatestObservedFrameForObjectId(
    const vslam_types_refactor::ObjectId &object_id,
    const std::shared_ptr<vslam_types_refactor::ObjAndLowLevelFeaturePoseGraph<
        VisualFeatureFactorType>> &pose_graph,
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

template <typename VisualFeatureFactorType, typename CachedFactorInfo>
class ObjectPoseGraphOptimizer {
 public:
  ObjectPoseGraphOptimizer(
      const std::function<
          bool(const std::pair<vslam_types_refactor::FactorType,
                               vslam_types_refactor::FeatureFactorId> &,
               const std::shared_ptr<
                   vslam_types_refactor::ObjAndLowLevelFeaturePoseGraph<
                       VisualFeatureFactorType>> &,
               const CachedFactorInfo &)> &refresh_residual_checker,
      const std::function<bool(
          const std::pair<vslam_types_refactor::FactorType,
                          vslam_types_refactor::FeatureFactorId> &,
          const pose_graph_optimization::ObjectVisualPoseGraphResidualParams &,
          const std::shared_ptr<
              vslam_types_refactor::ObjAndLowLevelFeaturePoseGraph<
                  VisualFeatureFactorType>> &pose_graph,
          ceres::Problem *,
          ceres::ResidualBlockId &,
          CachedFactorInfo &)> &residual_creator)
      : refresh_residual_checker_(refresh_residual_checker),
        residual_creator_(residual_creator) {}

  void buildPoseGraphOptimization(
      const OptimizationScopeParams &optimization_scope,
      const pose_graph_optimization::ObjectVisualPoseGraphResidualParams
          &residual_params,
      std::shared_ptr<vslam_types_refactor::ObjAndLowLevelFeaturePoseGraph<
          VisualFeatureFactorType>> &pose_graph,
      ceres::Problem *problem) {
    // Check for invalid combinations of scope and reject
    CHECK(checkInvalidOptimizationScopeParams(optimization_scope));

    std::unordered_set<vslam_types_refactor::ObjectId> optimized_objects;
    std::unordered_set<vslam_types_refactor::FeatureId> optimized_features;
    std::unordered_set<vslam_types_refactor::FrameId> optimized_frames;

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

    if (use_object_param_blocks) {
      pose_graph->getObjectsViewedBetweenFramesInclusive(
          optimization_scope.min_frame_id_,
          optimization_scope.max_frame_id_,
          optimized_objects);
    }
    if (use_visual_feature_param_blocks) {
      pose_graph->getFeaturesViewedBetweenFramesInclusive(
          optimization_scope.min_frame_id_,
          optimization_scope.max_frame_id_,
          optimized_features);
    }
    std::unordered_set<vslam_types_refactor::FrameId> all_frames =
        pose_graph->getFrameIds();
    for (const vslam_types_refactor::FrameId &frame_id : all_frames) {
      if ((frame_id >= optimization_scope.min_frame_id_) &&
          (frame_id <= optimization_scope.max_frame_id_)) {
        optimized_frames.insert(frame_id);
      }
    }

    if (use_object_only_factors) {
      // Get the object-only factors that are required
      std::unordered_set<
          std::pair<vslam_types_refactor::FactorType,
                    vslam_types_refactor::FeatureFactorId>,
          boost::hash<std::pair<vslam_types_refactor::FactorType,
                                vslam_types_refactor::FeatureFactorId>>>
          matching_obj_only_factors;

      pose_graph->getOnlyObjectFactorsForObjects(optimized_objects,
                                                 optimization_scope.use_pom_,
                                                 matching_obj_only_factors);
      for (const std::pair<vslam_types_refactor::FactorType,
                           vslam_types_refactor::FeatureFactorId>
               &obj_only_factor : matching_obj_only_factors) {
        required_feature_factors[obj_only_factor.first].insert(
            obj_only_factor.second);
      }
    }

    if (use_object_pose_factors) {
      vslam_types_refactor::FrameId min_frame_id;
      if (fix_object_param_blocks) {
        // If the objects are fixed, including those attached to the min frame
        // id will do nothing, so only get observation factors corresponding to
        min_frame_id = optimization_scope.min_frame_id_ + 1;
      } else {
        min_frame_id = optimization_scope.min_frame_id_;
      }
      util::BoostHashSet<std::pair<vslam_types_refactor::FactorType,
                                   vslam_types_refactor::FeatureFactorId>>
          matching_observation_factor_ids;
      pose_graph->getObservationFactorsBetweenFrameIdsInclusive(
          min_frame_id,
          optimization_scope.max_frame_id_,
          matching_observation_factor_ids);
      for (const std::pair<vslam_types_refactor::FactorType,
                           vslam_types_refactor::FeatureFactorId> &factor :
           matching_observation_factor_ids) {
        required_feature_factors[factor.first].insert(factor.second);
      }
    }

    if (use_feature_pose_factors) {
      vslam_types_refactor::FrameId min_visual_feature_frame_id;
      if (fix_visual_feature_param_blocks) {
        min_visual_feature_frame_id = optimization_scope.min_frame_id_ + 1;
      } else {
        min_visual_feature_frame_id = optimization_scope.min_frame_id_;
      }
      std::vector<std::pair<vslam_types_refactor::FactorType,
                            vslam_types_refactor::FeatureFactorId>>
          matching_visual_feature_factors;
      pose_graph->getVisualFeatureFactorIdsBetweenFrameIdsInclusive(
          min_visual_feature_frame_id,
          optimization_scope.max_frame_id_,
          matching_visual_feature_factors);
      for (const std::pair<vslam_types_refactor::FactorType,
                           vslam_types_refactor::FeatureFactorId>
               &matching_factor : matching_visual_feature_factors) {
        required_feature_factors[matching_factor.first].insert(
            matching_factor.second);
      }
    }

    // Remove unused residual blocks from the problem
    removeUnnecessaryResidualBlocks(required_feature_factors, problem);
    // Add residual blocks that are required but not added and refresh any that
    // exist but are stale
    addOrRefreshResidualBlocksForRequiredFactors(
        residual_params, required_feature_factors, pose_graph, problem);

    if (fix_pose_param_blocks) {
      // Set all pose param blocks constant
      setVariabilityForParamBlocks(optimized_frames,
                                   true,
                                   kPoseTypeStr,
                                   getParamBlockForPose,
                                   pose_graph,
                                   problem);
    } else {
      // Set only first pose param block constant (or add Gaussian prior
      // later...?)
      setVariabilityForParamBlocks({optimization_scope.min_frame_id_},
                                   true,
                                   kPoseTypeStr,
                                   getParamBlockForPose,
                                   pose_graph,
                                   problem);
      // Set all others not-constant (if were set constant)
      std::unordered_set<vslam_types_refactor::FrameId> variable_frames =
          optimized_frames;
      variable_frames.erase(optimization_scope.min_frame_id_);
      setVariabilityForParamBlocks(variable_frames,
                                   false,
                                   kPoseTypeStr,
                                   getParamBlockForPose,
                                   pose_graph,
                                   problem);
    }
    // Remove unused poses
    removeParamBlocksWithFrameIdOutsideWindowInclusive(
        optimization_scope.min_frame_id_,
        optimization_scope.max_frame_id_,
        last_optimized_nodes_,
        getFrameIdForFrameId,
        kPoseTypeStr,
        getParamBlockForPose,
        pose_graph,
        problem);
    last_optimized_nodes_ = optimized_frames;

    if (use_visual_feature_param_blocks) {
      bool set_constant;
      vslam_types_refactor::FrameId min_latest_observation;
      if (fix_visual_feature_param_blocks) {
        // Fix all remaining visual feature param blocks
        set_constant = true;
        // Remove old visual feature param blocks (visual features with
        // last_sighted <= min_frame_id)
        min_latest_observation = optimization_scope.min_frame_id_ + 1;

      } else {
        // Remove old visual feature param blocks (visual features with
        // last_sighted < min_frame_id)
        min_latest_observation = optimization_scope.min_frame_id_;
        // Set all others not-constant (if were set constant)
        set_constant = false;
      }
      removeParamBlocksWithMinFrameId(min_latest_observation,
                                      last_optimized_features_,
                                      getMinMaxFramesForFeatureId,
                                      kFeatureTypeStr,
                                      getParamBlockForFeature,
                                      pose_graph,
                                      problem);
      setVariabilityForParamBlocks(optimized_features,
                                   set_constant,
                                   kFeatureTypeStr,
                                   getParamBlockForFeature,
                                   pose_graph,
                                   problem);
      last_optimized_features_ = optimized_features;
    } else {
      // Remove all visual feature param blocks
      removeParamBlocksWithIdentifiers(last_optimized_features_,
                                       kFeatureTypeStr,
                                       getParamBlockForFeature,
                                       pose_graph,
                                       problem);
      last_optimized_features_ = {};
    }

    if (use_object_param_blocks) {
      bool set_constant;
      vslam_types_refactor::FrameId min_latest_observation;
      if (fix_object_param_blocks) {
        // Fix all included object param blocks
        set_constant = true;
        // Remove old object param blocks (objects with last_sighted
        // <= min_frame_id)
        min_latest_observation = optimization_scope.min_frame_id_ + 1;

      } else {
        // Remove old visual feature param blocks (visual features with
        // last_sighted < min_frame_id)
        min_latest_observation = optimization_scope.min_frame_id_;
        // Set all used objects non-constant
        set_constant = false;
      }

      removeParamBlocksWithMinFrameId(min_latest_observation,
                                      last_optimized_objects_,
                                      getLatestObservedFrameForObjectId,
                                      kObjTypeStr,
                                      getParamBlockForObject,
                                      pose_graph,
                                      problem);
      setVariabilityForParamBlocks(optimized_objects,
                                   set_constant,
                                   kObjTypeStr,
                                   getParamBlockForObject,
                                   pose_graph,
                                   problem);
      last_optimized_objects_ = optimized_objects;
    } else {
      // Remove all object param blocks
      removeParamBlocksWithIdentifiers(last_optimized_objects_,
                                       kObjTypeStr,
                                       getParamBlockForObject,
                                       pose_graph,
                                       problem);
      last_optimized_objects_ = {};
    }
  }

  bool solveOptimization(
      ceres::Problem *problem,
      const pose_graph_optimization::OptimizationSolverParams &solver_params,
      const std::vector<ceres::IterationCallback *> callbacks) {
    CHECK(problem != NULL);
    ceres::Solver::Options options;
    // TODO configure options

    // Set up callbacks
    options.callbacks = callbacks;
    if (!callbacks.empty()) {
      options.update_state_every_iteration = true;
    }

    ceres::Solver::Summary summary;
    ceres::Solve(options, problem, &summary);
    std::cout << summary.FullReport() << '\n';

    if ((summary.termination_type == ceres::TerminationType::FAILURE) ||
        (summary.termination_type == ceres::TerminationType::USER_FAILURE)) {
      LOG(ERROR) << "Ceres optimization failed";
    }
    return summary.IsSolutionUsable();
  }

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
                     const std::shared_ptr<
                         vslam_types_refactor::ObjAndLowLevelFeaturePoseGraph<
                             VisualFeatureFactorType>> &,
                     const CachedFactorInfo &)>
      refresh_residual_checker_;
  std::function<bool(
      const std::pair<vslam_types_refactor::FactorType,
                      vslam_types_refactor::FeatureFactorId> &,
      const pose_graph_optimization::ObjectVisualPoseGraphResidualParams &,
      const std::shared_ptr<
          vslam_types_refactor::ObjAndLowLevelFeaturePoseGraph<
              VisualFeatureFactorType>> &,
      ceres::Problem *,
      ceres::ResidualBlockId &,
      CachedFactorInfo &)>
      residual_creator_;

  template <typename IdentifierType>
  void removeParamBlocksWithFrameIdOutsideWindowInclusive(
      const vslam_types_refactor::FrameId &min_frame_id,
      const vslam_types_refactor::FrameId &max_frame_id,
      const std::unordered_set<IdentifierType> &identifiers_to_check,
      const std::function<
          bool(const IdentifierType &,
               const std::shared_ptr<
                   vslam_types_refactor::ObjAndLowLevelFeaturePoseGraph<
                       VisualFeatureFactorType>> &,
               std::pair<vslam_types_refactor::FrameId,
                         vslam_types_refactor::FrameId> &)> &frame_retriever,
      const std::string &identifier_type,
      const std::function<
          bool(const IdentifierType &,
               const std::shared_ptr<
                   vslam_types_refactor::ObjAndLowLevelFeaturePoseGraph<
                       VisualFeatureFactorType>> &,
               double **)> &param_block_retriever,
      const std::shared_ptr<
          vslam_types_refactor::ObjAndLowLevelFeaturePoseGraph<
              VisualFeatureFactorType>> &pose_graph,
      ceres::Problem *problem) {
    std::unordered_set<IdentifierType> identifiers_to_remove;
    for (const IdentifierType &identifier_to_check : identifiers_to_check) {
      bool remove = true;
      std::pair<vslam_types_refactor::FrameId, vslam_types_refactor::FrameId>
          min_max_frame_id;
      if (frame_retriever(identifier_to_check, pose_graph, min_max_frame_id)) {
        if ((min_frame_id > min_max_frame_id.second) ||
            (max_frame_id < min_max_frame_id.first)) {
          remove = true;
        } else {
          remove = false;
        }
      }
      if (remove) {
        identifiers_to_remove.insert(identifier_to_check);
      }
    }
    removeParamBlocksWithIdentifiers(identifiers_to_remove,
                                     identifier_type,
                                     param_block_retriever,
                                     pose_graph,
                                     problem);
  }

  template <typename IdentifierType>
  void setVariabilityForParamBlocks(
      const std::unordered_set<IdentifierType> &param_identifiers,
      const bool &set_constant,
      const std::string &identifier_type,
      const std::function<
          bool(const IdentifierType &,
               const std::shared_ptr<
                   vslam_types_refactor::ObjAndLowLevelFeaturePoseGraph<
                       VisualFeatureFactorType>> &,
               double **)> &param_block_retriever,
      const std::shared_ptr<
          vslam_types_refactor::ObjAndLowLevelFeaturePoseGraph<
              VisualFeatureFactorType>> &pose_graph,
      ceres::Problem *problem) {
    for (const IdentifierType &param_identifier : param_identifiers) {
      double *param_ptr = NULL;
      if (param_block_retriever(param_identifier, pose_graph, param_ptr)) {
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
      const std::function<
          bool(const IdentifierType &,
               const std::shared_ptr<
                   vslam_types_refactor::ObjAndLowLevelFeaturePoseGraph<
                       VisualFeatureFactorType>> &,
               double **)> &param_block_retriever,
      const std::shared_ptr<
          vslam_types_refactor::ObjAndLowLevelFeaturePoseGraph<
              VisualFeatureFactorType>> &pose_graph,
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

  void addOrRefreshResidualBlocksForRequiredFactors(
      const pose_graph_optimization::ObjectVisualPoseGraphResidualParams
          &residual_params,
      const std::unordered_map<
          vslam_types_refactor::FactorType,
          std::unordered_set<vslam_types_refactor::FeatureFactorId>>
          &required_feature_factors,
      const std::shared_ptr<
          vslam_types_refactor::ObjAndLowLevelFeaturePoseGraph<
              VisualFeatureFactorType>> &pose_graph,
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
