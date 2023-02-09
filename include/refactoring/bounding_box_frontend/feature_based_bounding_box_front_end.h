//
// Created by amanda on 1/4/23.
//

#ifndef UT_VSLAM_FEATURE_BASED_BOUNDING_BOX_FRONT_END_H
#define UT_VSLAM_FEATURE_BASED_BOUNDING_BOX_FRONT_END_H

#include <refactoring/bounding_box_frontend/bounding_box_front_end.h>
#include <refactoring/bounding_box_frontend/bounding_box_front_end_helpers.h>
#include <refactoring/bounding_box_frontend/pending_object_estimator.h>
#include <refactoring/types/ellipsoid_utils.h>

namespace vslam_types_refactor {

struct FeatureBasedFrontEndPendingObjInfo {
  std::optional<EllipsoidState<double>> object_estimate_ = std::nullopt;
  double max_confidence_;
  bool ready_for_merge = false;
};

struct FeatureBasedFrontEndObjAssociationInfo {
  std::unordered_map<
      FrameId,
      std::unordered_map<CameraId, std::unordered_set<FeatureId>>>
      observed_feats_;
};

struct FeatureBasedSingleBbContextInfo {
  std::unordered_set<FeatureId> features_in_bb_;
  double detection_confidence_;
};

struct FeatureBasedContextInfo {
  std::unordered_map<FeatureId, PixelCoord<double>> observed_features_;
};

struct FeatureBasedBbCandidateMatchInfo {
  std::unordered_map<FrameId, std::unordered_map<CameraId, int>>
      feature_overlap_count_per_obs_;
};

const int kNoBbDiscardingConst = INT_MIN;

struct FeatureBasedBbAssociationParams {
  double max_distance_for_associated_ellipsoids_;

  // TODO tune/override in calling function
  uint16_t min_observations_for_local_est_ = 3;
  uint16_t min_observations_ = 10;
  FrameId discard_candidate_after_num_frames_ = kNoBbDiscardingConst;
  double min_bb_confidence_ = 0.3;
  double required_min_conf_for_initialization = 0;
  double min_overlapping_features_for_match_ =
      2;  // TODO should this be ratio or absolute quantity? Going with absolute
          // quantity for now
  // When we're looking to match bounding boxes to pending or full objects
  // we should consider feature matches at most this many frames ago
  FrameId feature_validity_window_ = INT_MAX;

  PendingObjectEstimatorParams pending_obj_estimator_params_;
  double bounding_box_inflation_size_ = 0;
};

template <typename VisualFeatureFactorType>
class FeatureBasedBoundingBoxFrontEnd
    : public AbstractUnknownDataAssociationBbFrontEnd<
          VisualFeatureFactorType,
          FeatureBasedFrontEndObjAssociationInfo,
          FeatureBasedFrontEndPendingObjInfo,
          FeatureBasedContextInfo,
          FeatureBasedContextInfo,
          FeatureBasedSingleBbContextInfo,
          util::EmptyStruct,
          FeatureBasedBbCandidateMatchInfo> {
 public:
  FeatureBasedBoundingBoxFrontEnd(
      const std::shared_ptr<
          vslam_types_refactor::ObjAndLowLevelFeaturePoseGraph<
              VisualFeatureFactorType>> &pose_graph,
      const FeatureBasedBbAssociationParams &association_params,
      const std::function<Covariance<double, 4>(
          const RawBoundingBox &,
          const FrameId &,
          const CameraId &,
          const FeatureBasedContextInfo &)> &covariance_generator,
      const std::function<bool(const EllipsoidState<double> &,
                               const EllipsoidState<double> &,
                               double &)>
          &geometric_similarity_scorer,  // Consider adding uncertainty in here?
      const std::shared_ptr<std::unordered_map<
          FrameId,
          std::unordered_map<CameraId,
                             std::vector<std::pair<BbCornerPair<double>,
                                                   std::optional<double>>>>>>
          &all_filtered_corner_locations,
      const std::shared_ptr<std::unordered_map<
          FrameId,
          std::unordered_map<
              CameraId,
              std::unordered_map<
                  ObjectId,
                  std::pair<BbCornerPair<double>, std::optional<double>>>>>>
          &observed_corner_locations,
      const std::shared_ptr<std::vector<std::unordered_map<
          FrameId,
          std::unordered_map<CameraId,
                             std::pair<BbCornerPair<double>, double>>>>>
          &bounding_boxes_for_pending_object,
      const std::shared_ptr<std::vector<
          std::pair<std::string, std::optional<EllipsoidState<double>>>>>
          &pending_objects)
      : AbstractUnknownDataAssociationBbFrontEnd<
            VisualFeatureFactorType,
            FeatureBasedFrontEndObjAssociationInfo,
            FeatureBasedFrontEndPendingObjInfo,
            FeatureBasedContextInfo,
            FeatureBasedContextInfo,
            FeatureBasedSingleBbContextInfo,
            util::EmptyStruct,
            FeatureBasedBbCandidateMatchInfo>(pose_graph),
        association_params_(association_params),
        covariance_generator_(covariance_generator),
        geometric_similarity_scorer_(geometric_similarity_scorer),
        all_filtered_corner_locations_(all_filtered_corner_locations),
        observed_corner_locations_(observed_corner_locations),
        bounding_boxes_for_pending_object_(bounding_boxes_for_pending_object),
        pending_objects_(pending_objects) {}

 protected:
  virtual FeatureBasedFrontEndObjAssociationInfo objAssocInfoFromMapData(
      const ObjectId &obj_id,
      const util::EmptyStruct &front_end_data_for_obj) override {
    return FeatureBasedFrontEndObjAssociationInfo();
  }

  virtual util::EmptyStruct mapDataFromObjAssociationInfo(
      const ObjectId &obj_id,
      const FeatureBasedFrontEndObjAssociationInfo &obj_association_info)
      override {
    return util::EmptyStruct();
  }

  virtual void updateAppearanceInfoWithObjectIdAssignmentAndInitialization(
      const ObjectId &obj_id,
      const EllipsoidState<double> &est,
      FeatureBasedFrontEndObjAssociationInfo &info_to_update) override {
  }  // I don't think we need anything here but not sure

  virtual std::vector<RawBoundingBox> filterBoundingBoxes(
      const FrameId &frame_id,
      const CameraId &camera_id,
      const std::vector<RawBoundingBox> &original_bounding_boxes) override {
    std::vector<RawBoundingBox> bbs_to_keep =
        filterBoundingBoxesWithMinConfidence(
            frame_id,
            camera_id,
            original_bounding_boxes,
            association_params_.min_bb_confidence_);
    for (const RawBoundingBox &bb_to_keep : bbs_to_keep) {
      (*all_filtered_corner_locations_)[frame_id][camera_id].emplace_back(
          std::make_pair(bb_to_keep.pixel_corner_locations_,
                         bb_to_keep.detection_confidence_));
    }
    return bbs_to_keep;
  }

  virtual FeatureBasedSingleBbContextInfo generateSingleBoundingBoxContextInfo(
      const RawBoundingBox &bb,
      const FrameId &frame_id,
      const CameraId &camera_id,
      const FeatureBasedContextInfo &refined_context) override {
    // Note: could do this with a KD tree, but we likely don't have enough
    // bounding boxes to warrant optimizing the search in that way
    FeatureBasedSingleBbContextInfo single_bb_info;
    BbCornerPair<double> original_bb = bb.pixel_corner_locations_;
    BbCornerPair<double> inflated_bounding_box = inflateBoundingBox(
        original_bb, association_params_.bounding_box_inflation_size_);
    for (const auto &feats_and_coord : refined_context.observed_features_) {
      if (pixelInBoundingBoxClosedSet(inflated_bounding_box,
                                      feats_and_coord.second)) {
        single_bb_info.features_in_bb_.insert(feats_and_coord.first);
      }
    }
    single_bb_info.detection_confidence_ = bb.detection_confidence_;
    return single_bb_info;
  }

  virtual UninitializedEllispoidInfo<FeatureBasedFrontEndObjAssociationInfo,
                                     FeatureBasedFrontEndPendingObjInfo>
  createObjectInfoFromSingleBb(
      const std::string &semantic_class,
      const FrameId &frame_id,
      const CameraId &camera_id,
      const UninitializedObjectFactor &uninitialized_factor,
      const FeatureBasedContextInfo &refined_context,
      const FeatureBasedSingleBbContextInfo &single_bb_context_info) override {
    UninitializedEllispoidInfo<FeatureBasedFrontEndObjAssociationInfo,
                               FeatureBasedFrontEndPendingObjInfo>
        uninitalized_info;
    uninitalized_info.semantic_class_ = semantic_class;
    uninitalized_info.observation_factors_.emplace_back(uninitialized_factor);
    uninitalized_info.appearance_info_.observed_feats_[frame_id][camera_id] =
        single_bb_context_info.features_in_bb_;
    uninitalized_info.pending_info_.max_confidence_ =
        single_bb_context_info.detection_confidence_;
    uninitalized_info.max_frame_id_ = frame_id;
    uninitalized_info.min_frame_id_ = frame_id;
    EllipsoidState<double> ellipsoid_est;
    if (generateSingleViewEllipsoidEstimate(
            FeatureBasedBoundingBoxFrontEnd::pose_graph_,
            frame_id,
            camera_id,
            semantic_class,
            uninitialized_factor.bounding_box_corners_,
            ellipsoid_est)) {
      uninitalized_info.pending_info_.object_estimate_ = ellipsoid_est;
    }
    return uninitalized_info;
  }

  virtual void mergeObjectAssociationInfo(
      const FeatureBasedFrontEndObjAssociationInfo
          &association_info_to_merge_in,
      FeatureBasedFrontEndObjAssociationInfo &association_info_to_update)
      override {
    for (const auto &feats_for_frame :
         association_info_to_merge_in.observed_feats_) {
      for (const auto &feats_for_cam : feats_for_frame.second) {
        if (association_info_to_update.observed_feats_.find(
                feats_for_frame.first) !=
            association_info_to_update.observed_feats_.end()) {
          if (association_info_to_update.observed_feats_
                  .at(feats_for_frame.first)
                  .find(feats_for_cam.first) !=
              association_info_to_update.observed_feats_
                  .at(feats_for_frame.first)
                  .end()) {
            LOG(WARNING) << "Association info to merge in and association info "
                            "to update have multiple bounding boxes for the "
                            "same frame+camera. Overwriting with new one?";
          }
        }
        association_info_to_update
            .observed_feats_[feats_for_frame.first][feats_for_cam.first] =
            feats_for_cam.second;
      }
    }
    // TODO update estimate here? right now, merge is only used once an object
    // is initialized, and in this case, the estimate isn't used.
  }

  virtual void mergeSingleBbContextIntoObjectAssociationInfo(
      const RawBoundingBox &raw_bb,
      const FrameId &frame_id,
      const CameraId &camera_id,
      const FeatureBasedContextInfo &refined_bb_context_info,
      const FeatureBasedSingleBbContextInfo &single_bb_context_info,
      FeatureBasedFrontEndObjAssociationInfo &association_info_to_update,
      FeatureBasedFrontEndPendingObjInfo *pending_obj_info = nullptr) override {
    if (pending_obj_info != nullptr) {
      pending_obj_info->max_confidence_ =
          std::max(pending_obj_info->max_confidence_,
                   single_bb_context_info.detection_confidence_);
    }
    association_info_to_update.observed_feats_[frame_id][camera_id] =
        single_bb_context_info.features_in_bb_;
    if (pending_obj_info != nullptr) {
      if (!pending_obj_info->object_estimate_.has_value()) {
        EllipsoidState<double> ellipsoid_est;
        if (generateSingleViewEllipsoidEstimate(
                FeatureBasedBoundingBoxFrontEnd::pose_graph_,
                frame_id,
                camera_id,
                raw_bb.semantic_class_,
                cornerLocationsPairToVector(raw_bb.pixel_corner_locations_),
                ellipsoid_est)) {
          pending_obj_info->object_estimate_ = ellipsoid_est;
        }
      }
    }
  }

  virtual Covariance<double, 4> generateBoundingBoxCovariance(
      const RawBoundingBox &raw_bb,
      const FrameId &frame_id,
      const CameraId &camera_id,
      const FeatureBasedContextInfo &refined_bb_context) override {
    return covariance_generator_(
        raw_bb, frame_id, camera_id, refined_bb_context);
  }

  virtual std::vector<
      std::pair<AssociatedObjectIdentifier, FeatureBasedBbCandidateMatchInfo>>
  identifyCandidateMatches(
      const FrameId &frame_id,
      const CameraId &camera_id,
      const RawBoundingBox &bounding_box,
      const FeatureBasedSingleBbContextInfo &bb_context) override {
    std::vector<
        std::pair<AssociatedObjectIdentifier, FeatureBasedBbCandidateMatchInfo>>
        associated_obj_candidates;
    for (size_t uninitialized_obj_idx = 0;
         uninitialized_obj_idx <
         FeatureBasedBoundingBoxFrontEnd::uninitialized_object_info_.size();
         uninitialized_obj_idx++) {
      UninitializedEllispoidInfo<FeatureBasedFrontEndObjAssociationInfo,
                                 FeatureBasedFrontEndPendingObjInfo>
          uninitialized_obj = FeatureBasedBoundingBoxFrontEnd::
              uninitialized_object_info_[uninitialized_obj_idx];
      if (uninitialized_obj.semantic_class_ == bounding_box.semantic_class_) {
        AssociatedObjectIdentifier assoc_obj;
        assoc_obj.initialized_ellipsoid_ = false;
        assoc_obj.object_id_ = uninitialized_obj_idx;
        associated_obj_candidates.emplace_back(
            std::make_pair(assoc_obj, FeatureBasedBbCandidateMatchInfo()));
      }
    }

    std::unordered_set<ObjectId> pg_candidates =
        FeatureBasedBoundingBoxFrontEnd::pose_graph_
            ->getObjectsWithSemanticClass(bounding_box.semantic_class_);
    for (const ObjectId &candidate : pg_candidates) {
      AssociatedObjectIdentifier assoc_obj;
      assoc_obj.initialized_ellipsoid_ = true;
      assoc_obj.object_id_ = candidate;
      associated_obj_candidates.emplace_back(
          std::make_pair(assoc_obj, FeatureBasedBbCandidateMatchInfo()));
    }
    return associated_obj_candidates;
  }

  virtual std::vector<
      std::pair<AssociatedObjectIdentifier, FeatureBasedBbCandidateMatchInfo>>
  pruneCandidateMatches(
      const FrameId &frame_id,
      const CameraId &camera_id,
      const RawBoundingBox &bounding_box,
      const std::vector<std::pair<AssociatedObjectIdentifier,
                                  FeatureBasedBbCandidateMatchInfo>>
          &candidate_matches,
      const FeatureBasedSingleBbContextInfo &bb_context) override {
    if (candidate_matches.empty()) {
      LOG(WARNING) << "No candidate matches. This is to be expected for the "
                      "first occurrence of the object for a given semantic "
                      "class, but otherwise could indicate a problem.";
      return {};
    }
    std::vector<
        std::pair<AssociatedObjectIdentifier, FeatureBasedBbCandidateMatchInfo>>
        pruned_candidates;
    for (const std::pair<AssociatedObjectIdentifier,
                         FeatureBasedBbCandidateMatchInfo> &candidate :
         candidate_matches) {
      std::unordered_map<
          FrameId,
          std::unordered_map<CameraId, std::unordered_set<FeatureId>>>
          candidate_observed_feats;
      if (candidate.first.initialized_ellipsoid_) {
        if (FeatureBasedBoundingBoxFrontEnd::object_appearance_info_.find(
                candidate.first.object_id_) ==
            FeatureBasedBoundingBoxFrontEnd::object_appearance_info_.end()) {
          LOG(WARNING)
              << "Could not find appearance info for initialized object "
              << candidate.first.object_id_;
          continue;
        }
        candidate_observed_feats =
            FeatureBasedBoundingBoxFrontEnd::object_appearance_info_
                .at(candidate.first.object_id_)
                .observed_feats_;
      } else {
        if (FeatureBasedBoundingBoxFrontEnd::uninitialized_object_info_
                .size() <= candidate.first.object_id_) {
          LOG(WARNING) << "Invalid identifier " << candidate.first.object_id_
                       << " for pending object";
          continue;
        }
        candidate_observed_feats =
            FeatureBasedBoundingBoxFrontEnd::uninitialized_object_info_
                [candidate.first.object_id_]
                    .appearance_info_.observed_feats_;
      }
      std::unordered_map<FrameId, std::unordered_map<CameraId, int>>
          feature_overlap_count_per_obs;
      if (getMaxFeatureIntersection(bb_context.features_in_bb_,
                                    candidate_observed_feats,
                                    feature_overlap_count_per_obs) >=
          association_params_.min_overlapping_features_for_match_) {
        std::pair<AssociatedObjectIdentifier, FeatureBasedBbCandidateMatchInfo>
            candidate_copy = candidate;
        candidate_copy.second.feature_overlap_count_per_obs_ =
            feature_overlap_count_per_obs;
        pruned_candidates.emplace_back(candidate_copy);
      }
    }
    return pruned_candidates;
  }

  // IOU?
  virtual double scoreCandidateMatch(
      const FrameId &frame_id,
      const CameraId &camera_id,
      const RawBoundingBox &bounding_box,
      const std::pair<AssociatedObjectIdentifier,
                      FeatureBasedBbCandidateMatchInfo> &candidate,
      const FeatureBasedSingleBbContextInfo &bounding_box_appearance_info)
      override {
    std::unordered_map<
        FrameId,
        std::unordered_map<CameraId, std::unordered_set<FeatureId>>>
        candidate_observed_feats;
    if (candidate.first.initialized_ellipsoid_) {
      if (FeatureBasedBoundingBoxFrontEnd::object_appearance_info_.find(
              candidate.first.object_id_) ==
          FeatureBasedBoundingBoxFrontEnd::object_appearance_info_.end()) {
        LOG(WARNING) << "Could not find appearance info for initialized object "
                     << candidate.first.object_id_;
        return (-1 * std::numeric_limits<double>::infinity());
      }
      candidate_observed_feats =
          FeatureBasedBoundingBoxFrontEnd::object_appearance_info_
              .at(candidate.first.object_id_)
              .observed_feats_;
    } else {
      if (FeatureBasedBoundingBoxFrontEnd::uninitialized_object_info_.size() <=
          candidate.first.object_id_) {
        LOG(WARNING) << "Invalid identifier " << candidate.first.object_id_
                     << " for pending object";
        return (-1 * std::numeric_limits<double>::infinity());
      }
      candidate_observed_feats =
          FeatureBasedBoundingBoxFrontEnd::uninitialized_object_info_
              [candidate.first.object_id_]
                  .appearance_info_.observed_feats_;
    }
    double average_iou = 0;  // TODO do we want average or p90 or something?
    int total_obs = 0;
    for (const auto &feats_by_frame_id : candidate_observed_feats) {
      for (const auto &feats_by_cam_id : feats_by_frame_id.second) {
        total_obs++;
        int feats_intersection = candidate.second.feature_overlap_count_per_obs_
                                     .at(feats_by_frame_id.first)
                                     .at(feats_by_cam_id.first);
        if (feats_intersection != 0) {
          double iou =
              ((double)feats_intersection) /
              ((double)(bounding_box_appearance_info.features_in_bb_.size() +
                        feats_by_cam_id.second.size() - feats_intersection));
          average_iou += iou;
        }
      }
    }
    average_iou = average_iou / total_obs;
    // TODO do we want a minimum threshold here too?
    return average_iou;
  }

  virtual std::vector<AssociatedObjectIdentifier> assignBoundingBoxes(
      const std::vector<
          std::vector<std::pair<AssociatedObjectIdentifier, double>>>
          &match_candidates_with_scores) override {
    // TODO for now we're just doing this more or less greedily. Should change
    //  to find best overall matching
    return greedilyAssignBoundingBoxes(
        match_candidates_with_scores,
        FeatureBasedBoundingBoxFrontEnd::uninitialized_object_info_.size());
  }

  virtual FeatureBasedContextInfo generateRefinedBbContextInfo(
      const FeatureBasedContextInfo &bb_context,
      const FrameId &frame_id,
      const CameraId &camera_id) override {
    return bb_context;
  }

  virtual void setupBbAssociationRound(
      const vslam_types_refactor::FrameId &frame_id,
      const vslam_types_refactor::CameraId &camera_id,
      const std::vector<RawBoundingBox> &bounding_boxes,
      const FeatureBasedContextInfo &raw_bb_context,
      const FeatureBasedContextInfo &refined_bb_context) override {}

  virtual void cleanupBbAssociationRound(
      const vslam_types_refactor::FrameId &frame_id,
      const vslam_types_refactor::CameraId &camera_id) override {
    std::vector<
        UninitializedEllispoidInfo<FeatureBasedFrontEndObjAssociationInfo,
                                   FeatureBasedFrontEndPendingObjInfo>>
        uninitialized_object_info_to_keep;
    if (association_params_.discard_candidate_after_num_frames_ > 0) {
      removeStalePendingObjects(
          FeatureBasedBoundingBoxFrontEnd::uninitialized_object_info_,
          frame_id,
          association_params_.discard_candidate_after_num_frames_,
          uninitialized_object_info_to_keep);
      // TODO maybe also remove if it hasn't been initialized N frames after
      // first viewing (or maybe remove old viewings)
    }

    for (UninitializedEllispoidInfo<FeatureBasedFrontEndObjAssociationInfo,
                                    FeatureBasedFrontEndPendingObjInfo>
             &uninitialized_info : uninitialized_object_info_to_keep) {
      std::unordered_map<
          FrameId,
          std::unordered_map<CameraId, std::unordered_set<FeatureId>>>
          observed_feats_copy =
              uninitialized_info.appearance_info_.observed_feats_;
      for (const auto &frame_id_and_obs : observed_feats_copy) {
        if ((frame_id_and_obs.first +
             association_params_.feature_validity_window_) < frame_id) {
          uninitialized_info.appearance_info_.observed_feats_.erase(
              frame_id_and_obs.first);

          // TODO very that this modifies the copy stored in uninitialized
          // object info to keep
        }
      }
    }

    FeatureBasedBoundingBoxFrontEnd::uninitialized_object_info_ =
        uninitialized_object_info_to_keep;

    std::unordered_map<vslam_types_refactor::ObjectId,
                       FeatureBasedFrontEndObjAssociationInfo>
        updated_object_appearance_info;
    for (const auto &obj_id_and_appearance :
         FeatureBasedBoundingBoxFrontEnd::object_appearance_info_) {
      FeatureBasedFrontEndObjAssociationInfo association_info =
          obj_id_and_appearance.second;
      std::unordered_map<
          FrameId,
          std::unordered_map<CameraId, std::unordered_set<FeatureId>>>
          observed_feats_copy = association_info.observed_feats_;
      for (const auto &frame_id_and_obs : observed_feats_copy) {
        if ((frame_id_and_obs.first +
             association_params_.feature_validity_window_) < frame_id) {
          association_info.observed_feats_.erase(frame_id_and_obs.first);
          // TODO very that this modifies the copy stored in uninitialized
          // object info to keep
        }
      }
      updated_object_appearance_info[obj_id_and_appearance.first] =
          association_info;
    }
    FeatureBasedBoundingBoxFrontEnd::object_appearance_info_ =
        updated_object_appearance_info;
    bounding_boxes_for_pending_object_->clear();
    pending_objects_->clear();
    for (const UninitializedEllispoidInfo<
             FeatureBasedFrontEndObjAssociationInfo,
             FeatureBasedFrontEndPendingObjInfo> &uninitialized_obj :
         FeatureBasedBoundingBoxFrontEnd::uninitialized_object_info_) {
      std::unordered_map<
          FrameId,
          std::unordered_map<CameraId, std::pair<BbCornerPair<double>, double>>>
          obs_for_obj;
      for (const UninitializedObjectFactor &obj_factor :
           uninitialized_obj.observation_factors_) {
        obs_for_obj[obj_factor.frame_id_][obj_factor.camera_id_] =
            std::make_pair(
                cornerLocationsVectorToPair(obj_factor.bounding_box_corners_),
                obj_factor.detection_confidence_);
      }
      bounding_boxes_for_pending_object_->emplace_back(obs_for_obj);
      pending_objects_->emplace_back(
          std::make_pair(uninitialized_obj.semantic_class_,
                         uninitialized_obj.pending_info_.object_estimate_));
    }
  }

  virtual void setupInitialEstimateGeneration(
      const std::vector<AssociatedObjectIdentifier> &bounding_box_assignments)
      override {
    std::unordered_map<ObjectId, EllipsoidState<double>>
        rough_initial_estimates;
    std::unordered_map<
        ObjectId,
        UninitializedEllispoidInfo<FeatureBasedFrontEndObjAssociationInfo,
                                   FeatureBasedFrontEndPendingObjInfo>>
        uninitialized_obj_info_for_pending_objs;
    for (const AssociatedObjectIdentifier &association :
         bounding_box_assignments) {
      if (association.initialized_ellipsoid_) {
        continue;
      }
      if (association.object_id_ >=
          FeatureBasedBoundingBoxFrontEnd::uninitialized_object_info_.size()) {
        LOG(ERROR) << "No pending object for id " << association.object_id_;
        exit(1);
      }
      UninitializedEllispoidInfo<FeatureBasedFrontEndObjAssociationInfo,
                                 FeatureBasedFrontEndPendingObjInfo>
          association_info = FeatureBasedBoundingBoxFrontEnd::
              uninitialized_object_info_[association.object_id_];
      if (association_info.pending_info_.object_estimate_.has_value()) {
        rough_initial_estimates[association.object_id_] =
            association_info.pending_info_.object_estimate_.value();
        uninitialized_obj_info_for_pending_objs[association.object_id_] =
            association_info;
      }
    }
    for (size_t pending_obj_id = 0;
         pending_obj_id <
         FeatureBasedBoundingBoxFrontEnd::uninitialized_object_info_.size();
         pending_obj_id++) {
      if (rough_initial_estimates.find(pending_obj_id) !=
          rough_initial_estimates.end()) {
        continue;
      }
      UninitializedEllispoidInfo<FeatureBasedFrontEndObjAssociationInfo,
                                 FeatureBasedFrontEndPendingObjInfo>
          uninitialized_obj_data = FeatureBasedBoundingBoxFrontEnd::
              uninitialized_object_info_[pending_obj_id];
      if (uninitialized_obj_data.pending_info_.ready_for_merge) {
        if (uninitialized_obj_data.pending_info_.object_estimate_.has_value()) {
          rough_initial_estimates[pending_obj_id] =
              uninitialized_obj_data.pending_info_.object_estimate_.value();
          uninitialized_obj_info_for_pending_objs[pending_obj_id] =
              uninitialized_obj_data;
        }
      }
    }
    std::unordered_map<ObjectId, EllipsoidState<double>>
        refined_initial_estimates = refineInitialEstimateForPendingObjects(
            rough_initial_estimates,
            uninitialized_obj_info_for_pending_objs,
            FeatureBasedBoundingBoxFrontEnd::pose_graph_,
            association_params_.pending_obj_estimator_params_);
    for (const auto &pending_obj_initial_est : refined_initial_estimates) {
      FeatureBasedBoundingBoxFrontEnd::uninitialized_object_info_
          [pending_obj_initial_est.first]
              .pending_info_.object_estimate_ = pending_obj_initial_est.second;
      UninitializedEllispoidInfo<FeatureBasedFrontEndObjAssociationInfo,
                                 FeatureBasedFrontEndPendingObjInfo>
          association_info = FeatureBasedBoundingBoxFrontEnd::
              uninitialized_object_info_[pending_obj_initial_est.first];
      FeatureBasedBoundingBoxFrontEnd::uninitialized_object_info_
          [pending_obj_initial_est.first]
              .pending_info_.ready_for_merge =
          (association_info.observation_factors_.size() >=
           association_params_.min_observations_for_local_est_) &&
          (association_info.pending_info_.max_confidence_ >=
           association_params_.required_min_conf_for_initialization) &&
          (association_info.pending_info_.object_estimate_.has_value());
    }
  }

  virtual ObjectInitializationStatus tryInitializeEllipsoid(
      const FeatureBasedContextInfo &refined_bb_context,
      const UninitializedEllispoidInfo<FeatureBasedFrontEndObjAssociationInfo,
                                       FeatureBasedFrontEndPendingObjInfo>
          &uninitialized_bb_info,
      vslam_types_refactor::EllipsoidState<double> &ellipsoid_est) override {
    if (!uninitialized_bb_info.pending_info_.ready_for_merge) {
      return NOT_INITIALIZED;
    }

    ellipsoid_est =
        uninitialized_bb_info.pending_info_.object_estimate_.value();
    if (uninitialized_bb_info.observation_factors_.size() <
        association_params_.min_observations_) {
      return ENOUGH_VIEWS_FOR_MERGE;
    } else {
      return SUFFICIENT_VIEWS_FOR_NEW;
    }
  }

  virtual std::vector<ObjectId> mergeExistingPendingObjects() {
    std::unordered_map<
        ObjectId,
        std::pair<ObjectInitializationStatus, EllipsoidState<double>>>
        mergable_objects_and_ests;
    for (ObjectId pending_obj_id = 0;
         pending_obj_id <
         FeatureBasedBoundingBoxFrontEnd::uninitialized_object_info_.size();
         pending_obj_id++) {
      UninitializedEllispoidInfo<FeatureBasedFrontEndObjAssociationInfo,
                                 FeatureBasedFrontEndPendingObjInfo>
          pending_obj = FeatureBasedBoundingBoxFrontEnd::
              uninitialized_object_info_[pending_obj_id];

      // See if there are enough views to merge or create
      if (pending_obj.pending_info_.ready_for_merge) {
        // If enough views to merge or enough views for full object add to
        // later processing list
        mergable_objects_and_ests[pending_obj_id] =
            std::make_pair(ENOUGH_VIEWS_FOR_MERGE,
                           pending_obj.pending_info_.object_estimate_.value());
      }
    }
    LOG(INFO) << mergable_objects_and_ests.size()
              << " pending objects that have enough views for merge";

    std::vector<std::pair<ObjectId, ObjectId>>
        pending_and_initialized_objects_to_merge;
    std::vector<std::pair<ObjectId, EllipsoidState<double>>>
        pending_objects_to_add;
    searchForObjectMerges(mergable_objects_and_ests,
                          {},
                          pending_and_initialized_objects_to_merge,
                          pending_objects_to_add);
    if (!pending_objects_to_add.empty()) {
      LOG(WARNING) << "When searching through pending objects not modified "
                      "that should be added, found objects that should be "
                      "added. This probably shouldn't happen";
    }
    LOG(INFO) << "Merging " << pending_and_initialized_objects_to_merge.size()
              << " pending objects";

    std::vector<ObjectId> merged_indices =
        FeatureBasedBoundingBoxFrontEnd::mergePending(
            pending_and_initialized_objects_to_merge);
    return merged_indices;
  }

  virtual void searchForObjectMerges(
      const std::unordered_map<
          ObjectId,
          std::pair<ObjectInitializationStatus, EllipsoidState<double>>>
          &mergable_objects_and_ests,
      const std::unordered_set<ObjectId> &existing_associated_to_objects,
      std::vector<std::pair<ObjectId, ObjectId>>
          &pending_and_initialized_objects_to_merge,
      std::vector<std::pair<ObjectId, EllipsoidState<double>>>
          &pending_objects_to_add) override {
    std::unordered_map<ObjectId, std::vector<ObjectId>> merge_candidates;
    std::unordered_set<ObjectId> possible_pending_objs_to_merge;
    for (const auto &mergable_obj : mergable_objects_and_ests) {
      possible_pending_objs_to_merge.insert(mergable_obj.first);
    }
    identifyMergeCandidates(
        FeatureBasedBoundingBoxFrontEnd<VisualFeatureFactorType>::pose_graph_,
        existing_associated_to_objects,
        possible_pending_objs_to_merge,
        FeatureBasedBoundingBoxFrontEnd<
            VisualFeatureFactorType>::uninitialized_object_info_,
        true,
        merge_candidates);

    // Filter and score merge candidates based on geometric similarity
    // First entry is the pending object id and the mapped object id
    // Second is the score for the match
    std::vector<std::pair<std::pair<ObjectId, ObjectId>, double>>
        flattened_merge_candidates;
    for (const auto &mergable_obj : mergable_objects_and_ests) {
      for (const ObjectId &merge_candidate :
           merge_candidates[mergable_obj.first]) {
        std::optional<EllipsoidState<double>> merge_candidate_est =
            FeatureBasedBoundingBoxFrontEnd::pose_graph_->getEllipsoidEst(
                merge_candidate);
        if (merge_candidate_est.has_value()) {
          double score;
          if (geometric_similarity_scorer_(mergable_obj.second.second,
                                           merge_candidate_est.value(),
                                           score)) {
            flattened_merge_candidates.emplace_back(std::make_pair(
                std::make_pair(mergable_obj.first, merge_candidate), score));
          }
        }
      }
    }

    // Sort the scores
    std::sort(flattened_merge_candidates.begin(),
              flattened_merge_candidates.end(),
              [](const std::pair<std::pair<ObjectId, ObjectId>, double> &lhs,
                 const std::pair<std::pair<ObjectId, ObjectId>, double> &rhs) {
                return lhs.second > rhs.second;
              });

    std::unordered_set<ObjectId> unmerged_objects =
        possible_pending_objs_to_merge;
    std::unordered_set<ObjectId> newly_matched_existing_objects;
    for (const std::pair<std::pair<ObjectId, ObjectId>, double>
             &merge_pair_and_score : flattened_merge_candidates) {
      // If pending object already has match, skip it
      if (unmerged_objects.find(merge_pair_and_score.first.first) ==
          unmerged_objects.end()) {
        continue;
      }

      // If existing object is already matched to, skip it
      // TODO this doesn't actually work when we're searching through the
      // existing objects, but we'd have to search through all of the bounding
      // boxes to see if there are any that are in the same frame, which seems a
      // bit expensive to verify here, so leaving it as is for now, and using
      // the fact that each pending object will be revisited on each round, so
      // it should eventually merge all in
      // TODO -- could also put merge pending in a loop until there are none to
      // merge.
      if (newly_matched_existing_objects.find(
              merge_pair_and_score.first.second) !=
          newly_matched_existing_objects.end()) {
        continue;
      }

      // Otherwise save the merge
      unmerged_objects.erase(merge_pair_and_score.first.first);
      pending_and_initialized_objects_to_merge.emplace_back(
          merge_pair_and_score.first);

      // If we don't have any more objects to merge, then we should quit the
      // loop
      if (unmerged_objects.empty()) {
        break;
      }
    }

    for (const ObjectId &unmerged_obj : unmerged_objects) {
      std::pair<ObjectInitializationStatus, EllipsoidState<double>>
          info_for_obj = mergable_objects_and_ests.at(unmerged_obj);
      if (info_for_obj.first == SUFFICIENT_VIEWS_FOR_NEW) {
        pending_objects_to_add.emplace_back(
            std::make_pair(unmerged_obj, info_for_obj.second));
      }
    }
  }

  virtual void addObservationForObject(
      const FrameId &frame_id,
      const CameraId &camera_id,
      const ObjectId &object_id,
      const BbCorners<double> &bb_corners,
      const Covariance<double, 4> &bb_cov,
      const double &detection_confidence) override {
    AbstractUnknownDataAssociationBbFrontEnd<
        VisualFeatureFactorType,
        FeatureBasedFrontEndObjAssociationInfo,
        FeatureBasedFrontEndPendingObjInfo,
        FeatureBasedContextInfo,
        FeatureBasedContextInfo,
        FeatureBasedSingleBbContextInfo,
        util::EmptyStruct,
        FeatureBasedBbCandidateMatchInfo>::
        addObservationForObject(frame_id,
                                camera_id,
                                object_id,
                                bb_corners,
                                bb_cov,
                                detection_confidence);
    (*observed_corner_locations_)[frame_id][camera_id][object_id] =
        std::make_pair(cornerLocationsVectorToPair(bb_corners),
                       std::make_optional<double>(detection_confidence));
  }

 private:
  FeatureBasedBbAssociationParams association_params_;

  std::function<Covariance<double, 4>(const RawBoundingBox &,
                                      const FrameId &,
                                      const CameraId &,
                                      const FeatureBasedContextInfo &)>
      covariance_generator_;

  // Scores returned should have higher scores for better matches and lower
  // scores for worse
  std::function<bool(
      const EllipsoidState<double> &, const EllipsoidState<double> &, double &)>
      geometric_similarity_scorer_;

  std::shared_ptr<std::unordered_map<
      FrameId,
      std::unordered_map<
          CameraId,
          std::vector<std::pair<BbCornerPair<double>, std::optional<double>>>>>>
      all_filtered_corner_locations_;

  /**
   * Observed corner locations that have been associated to an object stored by
   * frame then camera, then object id.
   *
   * Optional double is the uncertainty of the bounding box.
   */
  std::shared_ptr<std::unordered_map<
      FrameId,
      std::unordered_map<CameraId,
                         std::unordered_map<ObjectId,
                                            std::pair<BbCornerPair<double>,
                                                      std::optional<double>>>>>>
      observed_corner_locations_;

  std::shared_ptr<std::vector<std::unordered_map<
      FrameId,
      std::unordered_map<CameraId, std::pair<BbCornerPair<double>, double>>>>>
      bounding_boxes_for_pending_object_;

  std::shared_ptr<std::vector<
      std::pair<std::string, std::optional<EllipsoidState<double>>>>>
      pending_objects_;

  int getMaxFeatureIntersection(
      const std::unordered_set<FeatureId> &features_in_bb,
      const std::unordered_map<
          FrameId,
          std::unordered_map<CameraId, std::unordered_set<FeatureId>>>
          &candidate_observed_feats,
      std::unordered_map<FrameId, std::unordered_map<CameraId, int>>
          &feature_overlap_count_per_obs) {
    int max_common_feats = 0;
    for (const auto &frame_id_and_feats : candidate_observed_feats) {
      for (const auto &cam_id_and_feats : frame_id_and_feats.second) {
        int common_feats = 0;
        for (const FeatureId &feat_id : cam_id_and_feats.second) {
          if (features_in_bb.find(feat_id) != features_in_bb.end()) {
            common_feats++;
          }
        }
        feature_overlap_count_per_obs[frame_id_and_feats.first]
                                     [cam_id_and_feats.first] = common_feats;
        max_common_feats = std::max(max_common_feats, common_feats);
      }
    }
    return max_common_feats;
  }
};

template <typename VisualFeatureFactorType>
class FeatureBasedBoundingBoxFrontEndCreator {
 public:
  FeatureBasedBoundingBoxFrontEndCreator(
      const FeatureBasedBbAssociationParams &association_params,
      const std::function<Covariance<double, 4>(
          const RawBoundingBox &,
          const FrameId &,
          const CameraId &,
          const FeatureBasedContextInfo &)> &covariance_generator,
      const std::function<bool(const EllipsoidState<double> &,
                               const EllipsoidState<double> &,
                               double &)>
          &geometric_similarity_scorer,  // Consider adding uncertainty in here?
      const std::shared_ptr<std::unordered_map<
          FrameId,
          std::unordered_map<CameraId,
                             std::vector<std::pair<BbCornerPair<double>,
                                                   std::optional<double>>>>>>
          &all_filtered_corner_locations,
      const std::shared_ptr<std::unordered_map<
          FrameId,
          std::unordered_map<
              CameraId,
              std::unordered_map<
                  ObjectId,
                  std::pair<BbCornerPair<double>, std::optional<double>>>>>>
          &observed_corner_locations,
      const std::shared_ptr<std::vector<std::unordered_map<
          FrameId,
          std::unordered_map<CameraId,
                             std::pair<BbCornerPair<double>, double>>>>>
          &bounding_boxes_for_pending_object,
      const std::shared_ptr<std::vector<
          std::pair<std::string, std::optional<EllipsoidState<double>>>>>
          &pending_objects,
      const std::unordered_map<ObjectId, util::EmptyStruct> &front_end_data)
      : association_params_(association_params),
        covariance_generator_(covariance_generator),
        geometric_similarity_scorer_(geometric_similarity_scorer),
        all_filtered_corner_locations_(all_filtered_corner_locations),
        observed_corner_locations_(observed_corner_locations),
        bounding_boxes_for_pending_object_(bounding_boxes_for_pending_object),
        pending_objects_(pending_objects),
        front_end_data_(front_end_data),
        initialized_(false) {}

  std::shared_ptr<AbstractUnknownDataAssociationBbFrontEnd<
      VisualFeatureFactorType,
      FeatureBasedFrontEndObjAssociationInfo,
      FeatureBasedFrontEndPendingObjInfo,
      FeatureBasedContextInfo,
      FeatureBasedContextInfo,
      FeatureBasedSingleBbContextInfo,
      util::EmptyStruct,
      FeatureBasedBbCandidateMatchInfo>>
  getDataAssociator(const std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph>
                        &pose_graph) {
    if (!initialized_) {
      feature_based_front_end_ = std::make_shared<
          FeatureBasedBoundingBoxFrontEnd<VisualFeatureFactorType>>(
          pose_graph,
          association_params_,
          covariance_generator_,
          geometric_similarity_scorer_,  // Consider adding uncertainty in here?
          all_filtered_corner_locations_,
          observed_corner_locations_,
          bounding_boxes_for_pending_object_,
          pending_objects_);
      feature_based_front_end_->initializeWithLongTermMapFrontEndData(
          front_end_data_);
      initialized_ = true;
    }
    return feature_based_front_end_;
  }

 private:
  FeatureBasedBbAssociationParams association_params_;

  std::function<Covariance<double, 4>(const RawBoundingBox &,
                                      const FrameId &,
                                      const CameraId &,
                                      const FeatureBasedContextInfo &)>
      covariance_generator_;

  // Scores returned should have higher scores for better matches and lower
  // scores for worse
  std::function<bool(
      const EllipsoidState<double> &, const EllipsoidState<double> &, double &)>
      geometric_similarity_scorer_;

  std::shared_ptr<std::unordered_map<
      FrameId,
      std::unordered_map<
          CameraId,
          std::vector<std::pair<BbCornerPair<double>, std::optional<double>>>>>>
      all_filtered_corner_locations_;

  /**
   * Observed corner locations that have been associated to an object stored by
   * frame then camera, then object id.
   *
   * Optional double is the uncertainty of the bounding box.
   */
  std::shared_ptr<std::unordered_map<
      FrameId,
      std::unordered_map<CameraId,
                         std::unordered_map<ObjectId,
                                            std::pair<BbCornerPair<double>,
                                                      std::optional<double>>>>>>
      observed_corner_locations_;

  std::shared_ptr<std::vector<std::unordered_map<
      FrameId,
      std::unordered_map<CameraId, std::pair<BbCornerPair<double>, double>>>>>
      bounding_boxes_for_pending_object_;

  std::shared_ptr<std::vector<
      std::pair<std::string, std::optional<EllipsoidState<double>>>>>
      pending_objects_;

  std::unordered_map<ObjectId, util::EmptyStruct> front_end_data_;

  bool initialized_;
  std::shared_ptr<AbstractUnknownDataAssociationBbFrontEnd<
      VisualFeatureFactorType,
      FeatureBasedFrontEndObjAssociationInfo,
      FeatureBasedFrontEndPendingObjInfo,
      FeatureBasedContextInfo,
      FeatureBasedContextInfo,
      FeatureBasedSingleBbContextInfo,
      util::EmptyStruct,
      FeatureBasedBbCandidateMatchInfo>>
      feature_based_front_end_;
};

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_FEATURE_BASED_BOUNDING_BOX_FRONT_END_H
