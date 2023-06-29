//
// Created by amanda on 1/4/23.
//

#ifndef UT_VSLAM_BOUNDING_BOX_FRONT_END_HELPERS_H
#define UT_VSLAM_BOUNDING_BOX_FRONT_END_HELPERS_H

#include <refactoring/types/vslam_obj_opt_types_refactor.h>
#include <refactoring/types/vslam_types_math_util.h>

namespace vslam_types_refactor {
std::vector<RawBoundingBox> filterBoundingBoxesWithMinConfidence(
    const FrameId &frame_id,
    const CameraId &camera_id,
    const std::vector<RawBoundingBox> &original_bounding_boxes,
    const double &min_detection_confidence) {
  std::vector<RawBoundingBox> bbs_to_keep;
  for (const RawBoundingBox &bounding_box : original_bounding_boxes) {
    if (bounding_box.detection_confidence_ > min_detection_confidence) {
      bbs_to_keep.emplace_back(bounding_box);
    }
  }
  return bbs_to_keep;
}

template <typename VisualFeatureFactorType,
          typename ObjectAssociationInfo,
          typename PendingObjInfo>
void identifyMergeCandidates(
    const std::shared_ptr<vslam_types_refactor::ObjAndLowLevelFeaturePoseGraph<
        VisualFeatureFactorType>> &pose_graph,
    const std::unordered_set<ObjectId> &existing_associated_to_objects,
    const std::unordered_set<ObjectId> &possible_pending_objs_to_merge,
    const std::vector<
        UninitializedEllispoidInfo<ObjectAssociationInfo, PendingObjInfo>>
        &uninitialized_object_info,
    const bool &only_match_semantic_class,
    std::unordered_map<ObjectId, std::vector<ObjectId>> &merge_candidates) {
  // Only populated for the relevant case
  std::unordered_map<std::string, std::unordered_set<ObjectId>>
      objs_by_semantic_class;
  std::unordered_set<ObjectId> all_objs;

  std::unordered_map<ObjectId, util::BoostHashSet<std::pair<FrameId, CameraId>>>
      observation_sources_by_obj;  // Lazily initialized

  std::function<std::unordered_set<ObjectId>(const std::string &)>
      plausible_candidate_getter;
  if (only_match_semantic_class) {
    plausible_candidate_getter = [&](const std::string &semantic_class) {
      if (objs_by_semantic_class.find(semantic_class) ==
          objs_by_semantic_class.end()) {
        std::unordered_set<ObjectId> matching_objs =
            pose_graph->getObjectsWithSemanticClass(semantic_class);

        std::unordered_set<ObjectId> filtered_matching_objs;
        std::copy_if(matching_objs.begin(),
                     matching_objs.end(),
                     std::inserter(filtered_matching_objs,
                                   filtered_matching_objs.begin()),
                     [&existing_associated_to_objects](ObjectId obj_id) {
                       return existing_associated_to_objects.find(obj_id) ==
                              existing_associated_to_objects.end();
                     });
        objs_by_semantic_class[semantic_class] = filtered_matching_objs;
      }
      return objs_by_semantic_class.at(semantic_class);
    };
  } else {
    std::unordered_map<ObjectId, std::pair<std::string, RawEllipsoid<double>>>
        object_estimates;
    pose_graph->getObjectEstimates(object_estimates);
    for (const auto &obj_est : object_estimates) {
      if (existing_associated_to_objects.find(obj_est.first) ==
          existing_associated_to_objects.end()) {
        all_objs.insert(obj_est.first);
      }
    }
    plausible_candidate_getter = [&](const std::string &semantic_class) {
      return all_objs;
    };
  }

  for (const ObjectId &pending_obj_to_maybe_merge :
       possible_pending_objs_to_merge) {
    UninitializedEllispoidInfo<ObjectAssociationInfo, PendingObjInfo>
        pending_obj_info =
            uninitialized_object_info[pending_obj_to_maybe_merge];
    std::unordered_set<ObjectId> possible_obj_ids =
        plausible_candidate_getter(pending_obj_info.semantic_class_);
    for (const ObjectId &possible_merge_with : possible_obj_ids) {
      util::BoostHashSet<std::pair<FrameId, CameraId>> observations_for_obj;
      if (observation_sources_by_obj.find(possible_merge_with) ==
          observation_sources_by_obj.end()) {
        std::vector<ObjectObservationFactor> observation_factors;
        pose_graph->getObservationFactorsForObjId(possible_merge_with,
                                                  observation_factors);
        for (const ObjectObservationFactor &obs_factor : observation_factors) {
          observations_for_obj.insert(
              std::make_pair(obs_factor.frame_id_, obs_factor.camera_id_));
        }
        observation_sources_by_obj[possible_merge_with] = observations_for_obj;
      } else {
        observations_for_obj =
            observation_sources_by_obj.at(possible_merge_with);
      }
      bool no_overlap = true;
      for (const UninitializedObjectFactor &pending_obj_obs :
           pending_obj_info.observation_factors_) {
        if (observations_for_obj.find(std::make_pair(
                pending_obj_obs.frame_id_, pending_obj_obs.camera_id_)) !=
            observations_for_obj.end()) {
          no_overlap = false;
          break;
        }
      }
      if (no_overlap) {
        merge_candidates[pending_obj_to_maybe_merge].emplace_back(
            possible_merge_with);
      }
    }
  }
}

std::vector<AssociatedObjectIdentifier> greedilyAssignBoundingBoxes(
    const std::vector<
        std::vector<std::pair<AssociatedObjectIdentifier, double>>>
        &match_candidates_with_scores,
    const ObjectId &next_free_obj) {
  std::vector<
      std::pair<uint64_t, std::pair<AssociatedObjectIdentifier, double>>>
      flattened_scores;
  for (size_t bb_idx = 0; bb_idx < match_candidates_with_scores.size();
       bb_idx++) {
    for (const std::pair<AssociatedObjectIdentifier, double> &candidate_for_bb :
         match_candidates_with_scores[bb_idx]) {
      flattened_scores.emplace_back(std::make_pair(bb_idx, candidate_for_bb));
    }
  }

  std::sort(
      flattened_scores.begin(),
      flattened_scores.end(),
      [](const std::pair<uint64_t,
                         std::pair<AssociatedObjectIdentifier, double>> &lhs,
         const std::pair<uint64_t,
                         std::pair<AssociatedObjectIdentifier, double>> &rhs) {
        return lhs.second.second > rhs.second.second;
      });

  util::BoostHashSet<AssociatedObjectIdentifier> claimed_objs;
  std::unordered_map<uint64_t, AssociatedObjectIdentifier> assignments_map;

  for (const std::pair<uint64_t, std::pair<AssociatedObjectIdentifier, double>>
           &possible_assignment : flattened_scores) {
    uint64_t bb_idx = possible_assignment.first;
    if (assignments_map.find(bb_idx) != assignments_map.end()) {
      continue;
    }
    if (claimed_objs.find(possible_assignment.second.first) !=
        claimed_objs.end()) {
      continue;
    }
    claimed_objs.insert(possible_assignment.second.first);
    assignments_map[possible_assignment.first] =
        possible_assignment.second.first;
  }

  ObjectId next_free_assignment = next_free_obj;
  std::vector<AssociatedObjectIdentifier> final_assignments;
  for (size_t bb_idx = 0; bb_idx < match_candidates_with_scores.size();
       bb_idx++) {
    if (assignments_map.find(bb_idx) != assignments_map.end()) {
      final_assignments.emplace_back(assignments_map.at(bb_idx));
    } else {
      AssociatedObjectIdentifier new_ellipsoid_assoc;
      new_ellipsoid_assoc.initialized_ellipsoid_ = false;
      new_ellipsoid_assoc.object_id_ = next_free_assignment;
      final_assignments.emplace_back(new_ellipsoid_assoc);
      next_free_assignment++;
    }
  }
  return final_assignments;
}

template <typename AssociationInfo, typename PendingObjInfo>
void removeStalePendingObjects(
    const std::vector<
        UninitializedEllispoidInfo<AssociationInfo, PendingObjInfo>>
        &full_uninitialized_object_info,
    const FrameId &curr_frame_id,
    const FrameId &discard_candidate_after_num_frames,
    std::vector<UninitializedEllispoidInfo<AssociationInfo, PendingObjInfo>>
        &uninitialized_object_info_to_keep) {
  for (const UninitializedEllispoidInfo<AssociationInfo, PendingObjInfo>
           &uninitialized_info : full_uninitialized_object_info) {
    if (curr_frame_id <= (uninitialized_info.max_frame_id_ +
                          discard_candidate_after_num_frames)) {
      uninitialized_object_info_to_keep.emplace_back(uninitialized_info);
    }
  }
}

double getObjectDepthGivenHeight(const BbCornerPair<double> &bb,
                                 const double &height,
                                 const double &fy) {
  double y_diff = bb.second.y() - bb.first.y();
  // TODO this isn't a valid way to do it, because this assumes the points
  // corresponding to the top and bottom of the bounding box correspond to the
  // min/max points in the z direction on an upright ellipsoid, which would
  // only be the case if the depth was infinite (as we get closer, the tangent
  // points are closer to the tip of the ellipsoid facing the camera
  return height * fy / y_diff;
}

template <typename VisualFeatureFactorType>
bool generateSingleViewEllipsoidEstimate(
    const std::shared_ptr<vslam_types_refactor::ObjAndLowLevelFeaturePoseGraph<
        VisualFeatureFactorType>> &pose_graph,
    const FrameId &frame_id,
    const CameraId &camera_id,
    const std::string &semantic_class,
    const BbCorners<double> &bb,
    vslam_types_refactor::EllipsoidState<double> &ellipsoid_est) {
  std::optional<ObjectDim<double>> mean_dim_for_class =
      pose_graph->getShapeDimMean(semantic_class);

  if (!mean_dim_for_class.has_value()) {
    LOG(ERROR) << "Could not initialize; no mean dimension provided for class "
               << semantic_class;
    return false;
  }
  ObjectDim<double> dim = mean_dim_for_class.value();
  CameraIntrinsicsMat<double> intrinsics;
  pose_graph->getIntrinsicsForCamera(camera_id, intrinsics);
  BbCornerPair<double> bb_pair = cornerLocationsVectorToPair(bb);
  double depth = getObjectDepthGivenHeight(bb_pair, dim.z(), intrinsics(1, 1));
  PixelCoord<double> bb_center = (bb_pair.first + bb_pair.second) / 2;
  Eigen::Vector3d bb_center_homogeneous(bb_center.x(), bb_center.y(), 1);
  Position3d<double> position_rel_cam =
      depth * intrinsics.inverse() * bb_center_homogeneous;

  std::optional<RawPose3d<double>> raw_robot_pose =
      pose_graph->getRobotPose(frame_id);
  if (!raw_robot_pose.has_value()) {
    return false;
  }
  Pose3D<double> robot_pose = convertToPose3D(raw_robot_pose.value());
  CameraExtrinsics<double> extrinsics;
  pose_graph->getExtrinsicsForCamera(camera_id, extrinsics);
  Pose3D<double> camera_pose = combinePoses(robot_pose, extrinsics);

#ifdef CONSTRAIN_ELLIPSOID_ORIENTATION
  Position3d<double> global_position =
      combinePoseAndPosition(camera_pose, position_rel_cam);
  Pose3DYawOnly<double> global_pose(global_position, 0);
#else
  Pose3D<double> pose_rel_to_cam(position_rel_cam, Orientation3D<double>());
  Pose3D<double> global_pose = combinePoses(camera_pose, pose_rel_to_cam);
#endif

  ellipsoid_est = EllipsoidState<double>(global_pose, dim);
  return true;
}

template <typename ObjectPoseGraphType>
void identifyMergeObjectsBasedOnCenterProximity(
    const std::shared_ptr<ObjectPoseGraphType> &pose_graph,
    const double &max_distance_for_merge,
    std::unordered_map<ObjectId, std::unordered_set<ObjectId>> &merge_results) {
  if (max_distance_for_merge < 0) {
    return;
  }

  // Doing most rudimentary version of this right now -- no checks about
  // duplicate associations for a single frame or anything class or size
  // dependent

  // Getting long-term map objects so we can merge into these and also don't
  // merge two together -- may want to enable merging long-term objects
  // eventually, but for now that's just going to add complexity for an
  // unlikely case
  std::unordered_set<ObjectId> long_term_map_objects;
  pose_graph->getLongTermMapObjects(long_term_map_objects);
  std::unordered_set<ObjectId> involved_in_merge;

  std::unordered_map<ObjectId, std::pair<std::string, RawEllipsoid<double>>>
      object_estimates_raw;
  std::unordered_map<std::string,
                     std::vector<std::pair<ObjectId, Position3d<double>>>>
      object_centers_by_semantic_class;
  pose_graph->getObjectEstimates(object_estimates_raw);
  for (const auto &raw_est : object_estimates_raw) {
    object_centers_by_semantic_class[raw_est.second.first].emplace_back(
        std::make_pair(raw_est.first, extractPosition(raw_est.second.second)));
  }
  std::vector<std::pair<double, std::pair<ObjectId, ObjectId>>>
      possible_merge_objs_with_dist;

  for (const auto &sem_class_and_objs : object_centers_by_semantic_class) {
    for (size_t obj_idx = 0; obj_idx < sem_class_and_objs.second.size();
         obj_idx++) {
      std::pair<ObjectId, Position3d<double>> first_obj =
          sem_class_and_objs.second.at(obj_idx);
      for (size_t compare_obj_idx = obj_idx + 1;
           compare_obj_idx < sem_class_and_objs.second.size();
           compare_obj_idx++) {
        std::pair<ObjectId, Position3d<double>> second_obj =
            sem_class_and_objs.second.at(compare_obj_idx);
        double center_dist = (first_obj.second - second_obj.second).norm();
        if (center_dist <= max_distance_for_merge) {
          if ((long_term_map_objects.find(first_obj.first) ==
               long_term_map_objects.end()) ||
              (long_term_map_objects.find(second_obj.first) ==
               long_term_map_objects.end())) {
            possible_merge_objs_with_dist.emplace_back(std::make_pair(
                center_dist,
                std::make_pair(first_obj.first, second_obj.first)));
          }
        }
      }
    }
  }

  // Sort possible merge objects with dist
  std::sort(possible_merge_objs_with_dist.begin(),
            possible_merge_objs_with_dist.end(),
            util::sort_pair_by_first<double, std::pair<ObjectId, ObjectId>>());

  for (const std::pair<double, std::pair<ObjectId, ObjectId>> &merge_candidate :
       possible_merge_objs_with_dist) {
    if ((involved_in_merge.find(merge_candidate.second.first) ==
         involved_in_merge.end()) &&
        (involved_in_merge.find(merge_candidate.second.second) ==
         involved_in_merge.end())) {
      involved_in_merge.insert(merge_candidate.second.first);
      involved_in_merge.insert(merge_candidate.second.second);
      if (long_term_map_objects.find(merge_candidate.second.first) !=
          long_term_map_objects.end()) {
        merge_results[merge_candidate.second.first] = {
            merge_candidate.second.second};
      } else {
        merge_results[merge_candidate.second.second] = {
            merge_candidate.second.first};
      }
    }
  }
}

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_BOUNDING_BOX_FRONT_END_HELPERS_H
