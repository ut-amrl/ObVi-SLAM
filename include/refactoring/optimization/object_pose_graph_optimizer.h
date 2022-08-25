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
  bool use_pom_;          // Effectively false if fix_objects_ is true
  bool fix_ltm_objects_;  // Effectively true if fix_objects_ is true
  vslam_types_refactor::FrameId min_frame_id_;
  vslam_types_refactor::FrameId max_frame_id_;
  // TODO consider adding set of nodes to optimize -- for now, we'll just assume
  // that we have min_frame_id (held constant) and all poses above that
};

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

  void buildPoseGraphOptimization(
      const OptimizationScopeParams &optimization_scope,
      const pose_graph_optimization::ObjectVisualPoseGraphResidualParams
          &residual_params,
      std::shared_ptr<PoseGraphType> &pose_graph,
      ceres::Problem *problem) {
    // Check for invalid combinations of scope and reject
    CHECK(checkInvalidOptimizationScopeParams(optimization_scope));

    std::unordered_set<vslam_types_refactor::ObjectId> observed_objects;
    std::unordered_set<vslam_types_refactor::FeatureId> optimized_features;
    std::unordered_set<vslam_types_refactor::FrameId> optimized_frames;
    std::unordered_set<vslam_types_refactor::ObjectId> ltm_object_ids;

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

    if (use_object_param_blocks) {
      vslam_types_refactor::FrameId min_frame_for_obj;
      if (fix_object_param_blocks) {
        min_frame_for_obj = optimization_scope.min_frame_id_ + 1;
      } else {
        min_frame_for_obj = optimization_scope.min_frame_id_;
      }
      pose_graph->getObjectsViewedBetweenFramesInclusive(
          min_frame_for_obj,
          optimization_scope.max_frame_id_,
          observed_objects);
      // TODO -- if no objects in the long-term map are observed, then nothing
      // in the long-term map will change,
      //  so we could omit those objects, the object only factors, and the
      //  factors relating objects in the long-term map
      pose_graph->getLongTermMapObjects(ltm_object_ids);
    }
    if (use_visual_feature_param_blocks) {
      vslam_types_refactor::FrameId min_frame_for_feats;
      if (fix_visual_feature_param_blocks) {
        min_frame_for_feats = optimization_scope.min_frame_id_ + 1;
      } else {
        min_frame_for_feats = optimization_scope.min_frame_id_;
      }
      pose_graph->getFeaturesViewedBetweenFramesInclusive(
          min_frame_for_feats,
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
      //      LOG(INFO) << "Using object only factors";
      // Get the object-only factors that are required
      util::BoostHashSet<std::pair<vslam_types_refactor::FactorType,
                                   vslam_types_refactor::FeatureFactorId>>
          matching_obj_only_factors;
      std::unordered_set<vslam_types_refactor::ObjectId>
          objects_with_object_only_factors;
      if (fix_ltm_param_blocks) {
        // If the long term map is fixed, we only need the object-only factors
        // for the objects not in the long-term map
        // If at some point we marginalize mid-run, we may need to look for the
        // LTM objects that are connected to the observed ones (rather than just
        // the observed ones)
        for (const vslam_types_refactor::ObjectId &observed_obj :
             observed_objects) {
          if (ltm_object_ids.find(observed_obj) == ltm_object_ids.end()) {
            objects_with_object_only_factors.insert(observed_obj);
          }
        }
        //        std::set_difference(
        //            observed_objects.begin(),
        //            observed_objects.end(),
        //            ltm_object_ids.begin(),
        //            ltm_object_ids.end(),
        //            std::inserter(objects_with_object_only_factors,
        //                          objects_with_object_only_factors.end()));
      } else {
        // If the long term map is not fixed, we need the object-only factors
        // for the observed objects and those in the long-term map
        objects_with_object_only_factors.insert(observed_objects.begin(),
                                                observed_objects.end());
        objects_with_object_only_factors.insert(ltm_object_ids.begin(),
                                                ltm_object_ids.end());
      }

      pose_graph->getOnlyObjectFactorsForObjects(
          objects_with_object_only_factors,
          optimization_scope.use_pom_,
          !fix_ltm_param_blocks,
          matching_obj_only_factors);
      for (const std::pair<vslam_types_refactor::FactorType,
                           vslam_types_refactor::FeatureFactorId>
               &obj_only_factor : matching_obj_only_factors) {
        required_feature_factors[obj_only_factor.first].insert(
            obj_only_factor.second);
      }
    }

    if (use_object_pose_factors) {
      //      LOG(INFO) << "Using object-pose factors";
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
      LOG(INFO) << "Num observation factors "
                << matching_observation_factor_ids.size();
      for (const std::pair<vslam_types_refactor::FactorType,
                           vslam_types_refactor::FeatureFactorId> &factor :
           matching_observation_factor_ids) {
        required_feature_factors[factor.first].insert(factor.second);
      }
    }

    if (use_feature_pose_factors) {
      //      LOG(INFO) << "Using feature-pose factors";
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
    addParamBlocksWithIdentifiers(optimized_frames,
                                  kPoseTypeStr,
                                  getParamBlockForPose<PoseGraphType>,
                                  pose_graph,
                                  6,
                                  problem);

    if (fix_pose_param_blocks) {
      LOG(INFO) << "Fixing pose blocks";
      // Set all pose param blocks constant
      setVariabilityForParamBlocks(optimized_frames,
                                   true,
                                   kPoseTypeStr,
                                   getParamBlockForPose<PoseGraphType>,
                                   pose_graph,
                                   problem);
    } else {
      LOG(INFO) << "Fixing first pose block and setting remaining variable";
      // Set only first pose param block constant (or add Gaussian prior
      // later...?)
      setVariabilityForParamBlocks(
          (std::unordered_set<vslam_types_refactor::FrameId>){
              optimization_scope.min_frame_id_},
          true,
          kPoseTypeStr,
          getParamBlockForPose<PoseGraphType>,
          pose_graph,
          problem);
      // Set all others not-constant (if were set constant)
      std::unordered_set<vslam_types_refactor::FrameId> variable_frames =
          optimized_frames;
      variable_frames.erase(optimization_scope.min_frame_id_);
      LOG(INFO) << "Setting pose param blocks variable";
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

    std::unordered_set<vslam_types_refactor::FrameId> features_to_remove;
    std::unordered_set<vslam_types_refactor::FrameId>
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
      for (const vslam_types_refactor::ObjectId &last_opt :
           last_optimized_features_) {
        if (optimized_features.find(last_opt) == optimized_features.end()) {
          features_to_remove.insert(last_opt);
        }
      }
      setVariabilityForParamBlocks(optimized_features,
                                   set_constant,
                                   kFeatureTypeStr,
                                   getParamBlockForFeature<PoseGraphType>,
                                   pose_graph,
                                   problem);
      next_last_optimized_features = optimized_features;
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
        LOG(INFO) << "Getting variable objects";
        LOG(INFO) << "Observed objects size " << observed_objects.size();
        LOG(INFO) << "Ltm objects size " << ltm_object_ids.size();
//        std::set_union(observed_objects.begin(),
//                       observed_objects.end(),
//                       ltm_object_ids.begin(),
//                       ltm_object_ids.end(),
//                       std::inserter(variable_object_param_blocks,
//                                     variable_object_param_blocks.end()));
        for (const vslam_types_refactor::ObjectId &observed_obj : observed_objects) {
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
      LOG(INFO) << "Objects to remove size " << objects_to_remove.size();

      if (!objects_to_remove.empty()) {
        LOG(INFO) << "Last optimized ";
        for (const vslam_types_refactor::ObjectId &last_opt :
             last_optimized_objects_) {
          LOG(INFO) << last_opt;
        }
        LOG(INFO) << "Currently optimized";
        for (const vslam_types_refactor::ObjectId &next_opt :
             next_last_optimized_objects) {
          LOG(INFO) << "Next opt: " << next_opt;
          for (const vslam_types_refactor::ObjectId &to_remove :
               objects_to_remove) {
            LOG(INFO) << "To Remove " << to_remove << " equal? "
                      << (to_remove == next_opt);
          }
        }
        LOG(INFO) << "Not expecting objects to be removed";
        for (const vslam_types_refactor::ObjectId &to_remove :
             objects_to_remove) {
          LOG(INFO) << to_remove;
        }
        exit(1);
      }

      if (!constant_object_param_blocks.empty()) {
        LOG(INFO) << "Setting object param blocks constant";
        setVariabilityForParamBlocks(constant_object_param_blocks,
                                     true,
                                     kObjTypeStr,
                                     getParamBlockForObject<PoseGraphType>,
                                     pose_graph,
                                     problem);
      }
      if (!variable_object_param_blocks.empty()) {
        LOG(INFO) << "Setting object param blocks variable";
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
  }

  bool solveOptimization(
      ceres::Problem *problem,
      const pose_graph_optimization::OptimizationSolverParams &solver_params,
      const std::vector<std::shared_ptr<ceres::IterationCallback>> callbacks) {
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

    ceres::Solver::Summary summary;
    ceres::Solve(options, problem, &summary);
    std::cout << summary.FullReport() << '\n';

    if ((summary.termination_type == ceres::TerminationType::FAILURE) ||
        (summary.termination_type == ceres::TerminationType::USER_FAILURE)) {
      LOG(ERROR) << "Ceres optimization failed";
    }
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
      LOG(INFO) << "Identifier " << param_identifier;
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
            LOG(INFO) << "Removing factor " << factor_id << " of type "
                      << factor_type;
            exit(1);
            problem->RemoveResidualBlock(existing_info.first);
            residual_blocks_and_cached_info_by_factor_id_[factor_type].erase(
                factor_id);
            create_new = true;
          } else {
            LOG(INFO) << "Not refreshing";
          }
        }
        if (create_new) {
          LOG(INFO) << "Creating new residual";
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
            LOG(INFO) << "Removing factor " << factor_id_and_residual.first
                      << " of type " << factor_type;
            exit(1);
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
