//
// Created by amanda on 6/21/22.
//

#ifndef UT_VSLAM_ROSHAN_BOUNDING_BOX_FRONT_END_H
#define UT_VSLAM_ROSHAN_BOUNDING_BOX_FRONT_END_H

#include <cv_bridge/cv_bridge.h>
#include <refactoring/bounding_box_frontend/bounding_box_front_end.h>
#include <refactoring/bounding_box_frontend/bounding_box_front_end_helpers.h>
#include <refactoring/types/vslam_types_math_util.h>
#include <sensor_msgs/Image.h>

namespace vslam_types_refactor {

const float kHRanges[] = {0, 180};
const float kSRanges[] = {0, 256};
const int kHAndSChannels[] = {0, 1};

const int kNoDiscardingConst = INT_MIN;  // Should htis be min?

struct RoshanBbAssociationParams {
  double max_distance_for_associated_ellipsoids_;
  int hue_histogram_bins_;
  int saturation_histogram_bins_;
  uint16_t min_observations_ = 1;
  int discard_candidate_after_num_frames_ = kNoDiscardingConst;
  double min_bb_confidence_ = 0.3;
  double required_min_conf_for_initialization = 0;
};

struct RoshanImageSummaryInfo {
  std::optional<cv::Mat> hsv_img_ = std::nullopt;
};

struct RoshanBbInfo {
  // TODO what's the best way to store/initialize this so we're not moving
  // around big matrices
  cv::Mat hue_sat_histogram_;
  bool est_generated_;
  EllipsoidState<double> single_bb_init_est_;
  double detection_confidence_;
};

struct RoshanAggregateBbInfo {
  std::vector<RoshanBbInfo> infos_for_observed_bbs_;
  double max_confidence_;
};

template <typename VisualFeatureFactorType>
class RoshanBbFrontEnd
    : public AbstractUnknownDataAssociationBbFrontEnd<
          VisualFeatureFactorType,
          RoshanAggregateBbInfo,
          util::EmptyStruct,  // TODO at some point, it might be worth
                              // separating the RoshanAggregateBbInfo into the
                              // stuff only needed for pending objects and the
                              // stuff needed for existing objects, but we're
                              // not really using that functionality now, so
                              // don't want to spend time making changes that
                              // would take time to do right
          std::optional<sensor_msgs::Image::ConstPtr>,
          RoshanImageSummaryInfo,
          RoshanBbInfo,
          std::unordered_map<ObjectId, RoshanAggregateBbInfo>,
          util::EmptyStruct> {
 public:
  RoshanBbFrontEnd(
      const std::shared_ptr<
          vslam_types_refactor::ObjAndLowLevelFeaturePoseGraph<
              VisualFeatureFactorType>> &pose_graph,
      const RoshanBbAssociationParams &association_params,
      const std::function<Covariance<double, 4>(const RawBoundingBox &,
                                                const FrameId &,
                                                const CameraId &,
                                                const RoshanImageSummaryInfo &)>
          &covariance_generator,
      const std::shared_ptr<std::unordered_map<
          FrameId,
          std::unordered_map<
              CameraId,
              std::unordered_map<
                  ObjectId,
                  std::pair<BbCornerPair<double>, std::optional<double>>>>>>
          &observed_corner_locations,
      const std::shared_ptr<std::unordered_map<
          FrameId,
          std::unordered_map<CameraId,
                             std::vector<std::pair<BbCornerPair<double>,
                                                   std::optional<double>>>>>>
          &all_filtered_corner_locations)
      : AbstractUnknownDataAssociationBbFrontEnd<
            VisualFeatureFactorType,
            RoshanAggregateBbInfo,
            util::EmptyStruct,
            std::optional<sensor_msgs::Image::ConstPtr>,
            RoshanImageSummaryInfo,
            RoshanBbInfo,
            std::unordered_map<ObjectId, RoshanAggregateBbInfo>,
            util::EmptyStruct>(pose_graph),
        association_params_(association_params),
        covariance_generator_(covariance_generator),
        observed_corner_locations_(observed_corner_locations),
        all_filtered_corner_locations_(all_filtered_corner_locations) {}

 protected:
  virtual RoshanAggregateBbInfo objAssocInfoFromMapData(
      const ObjectId &obj_id,
      const RoshanAggregateBbInfo &front_end_data_for_obj) override {
    return front_end_data_for_obj;
  }

  virtual RoshanAggregateBbInfo mapDataFromObjAssociationInfo(
      const ObjectId &obj_id,
      const RoshanAggregateBbInfo &obj_association_info) override {
    return obj_association_info;
  }

  virtual std::vector<ObjectId> mergeExistingPendingObjects() override {
    return {};
  }

  virtual void updateAppearanceInfoWithObjectIdAssignmentAndInitialization(
      const ObjectId &obj_id,
      const EllipsoidState<double> &est,
      RoshanAggregateBbInfo &info_to_update) override {}

  virtual std::vector<RawBoundingBox> filterBoundingBoxes(
      const FrameId &frame_id,
      const CameraId &camera_id,
      const std::vector<RawBoundingBox> &original_bounding_boxes) override {
    // Note: could couple these in the helper for a tiny speed up
    // (only 1 for loop instead of 2 that way) -- keeping like this for now
    // for better modularity
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

  virtual RoshanBbInfo generateSingleBoundingBoxContextInfo(
      const RawBoundingBox &bb,
      const FrameId &frame_id,
      const CameraId &camera_id,
      const RoshanImageSummaryInfo &refined_context) override {
    Eigen::Vector2d bb_dim =
        bb.pixel_corner_locations_.second - bb.pixel_corner_locations_.first;
    cv::Rect bb_rect(bb.pixel_corner_locations_.first.x(),
                     bb.pixel_corner_locations_.first.y(),
                     bb_dim.x(),
                     bb_dim.y());
    cv::Mat bb_img = (refined_context.hsv_img_.value())(bb_rect);
    RoshanBbInfo bb_info;
    int hist_size[] = {association_params_.hue_histogram_bins_,
                       association_params_.saturation_histogram_bins_};
    const float *ranges[] = {kHRanges, kSRanges};
    cv::calcHist(&bb_img,
                 1,
                 kHAndSChannels,
                 cv::Mat(),
                 bb_info.hue_sat_histogram_,
                 2,
                 hist_size,
                 ranges,
                 true,
                 false);
    cv::normalize(bb_info.hue_sat_histogram_,
                  bb_info.hue_sat_histogram_,
                  0,
                  1,
                  cv::NORM_MINMAX,
                  -1,
                  cv::Mat());

    bb_info.est_generated_ = initializeEllipsoid(
        frame_id,
        camera_id,
        bb.semantic_class_,
        cornerLocationsPairToVector(bb.pixel_corner_locations_),
        bb_info.single_bb_init_est_);

    bb_info.detection_confidence_ = bb.detection_confidence_;

    return bb_info;
  }

  virtual UninitializedEllispoidInfo<RoshanAggregateBbInfo, util::EmptyStruct>
  createObjectInfoFromSingleBb(
      const std::string &semantic_class,
      const FrameId &frame_id,
      const CameraId &camera_id,
      const UninitializedObjectFactor &uninitialized_factor,
      const RoshanImageSummaryInfo &refined_context,
      const RoshanBbInfo &single_bb_context_info) override {
    UninitializedEllispoidInfo<RoshanAggregateBbInfo, util::EmptyStruct>
        uninitalized_info;
    uninitalized_info.semantic_class_ = semantic_class;
    uninitalized_info.observation_factors_.emplace_back(uninitialized_factor);
    uninitalized_info.appearance_info_.infos_for_observed_bbs_.emplace_back(
        single_bb_context_info);
    uninitalized_info.appearance_info_.max_confidence_ =
        single_bb_context_info.detection_confidence_;
    uninitalized_info.min_frame_id_ = frame_id;
    uninitalized_info.max_frame_id_ = frame_id;
    return uninitalized_info;
  }

  virtual void mergeSingleBbContextIntoObjectAssociationInfo(
      const RawBoundingBox &raw_bb,
      const FrameId &frame_id,
      const CameraId &camera_id,
      const RoshanImageSummaryInfo &refined_bb_context_info,
      const RoshanBbInfo &single_bb_context_info,
      RoshanAggregateBbInfo &association_info_to_update,
      util::EmptyStruct *pending_obj_info = nullptr) override {
    association_info_to_update.infos_for_observed_bbs_.emplace_back(
        single_bb_context_info);
    association_info_to_update.max_confidence_ =
        std::max(association_info_to_update.max_confidence_,
                 single_bb_context_info.detection_confidence_);
  }

  virtual void mergeObjectAssociationInfo(
      const RoshanAggregateBbInfo &association_info_to_merge_in,
      RoshanAggregateBbInfo &association_info_to_update) {
    association_info_to_update.infos_for_observed_bbs_.insert(
        association_info_to_update.infos_for_observed_bbs_.end(),
        association_info_to_merge_in.infos_for_observed_bbs_.begin(),
        association_info_to_merge_in.infos_for_observed_bbs_.end());
    association_info_to_update.max_confidence_ =
        std::max(association_info_to_update.max_confidence_,
                 association_info_to_merge_in.max_confidence_);
  }

  virtual Covariance<double, 4> generateBoundingBoxCovariance(
      const RawBoundingBox &raw_bb,
      const FrameId &frame_id,
      const CameraId &camera_id,
      const RoshanImageSummaryInfo &refined_bb_context) override {
    // TODO do we need additional parameters here?
    return covariance_generator_(
        raw_bb, frame_id, camera_id, refined_bb_context);
  }

  virtual std::vector<std::pair<AssociatedObjectIdentifier, util::EmptyStruct>>
  identifyCandidateMatches(const FrameId &frame_id,
                           const CameraId &camera_id,
                           const RawBoundingBox &bounding_box,
                           const RoshanBbInfo &bb_context) override {
    std::vector<std::pair<AssociatedObjectIdentifier, util::EmptyStruct>>
        associated_obj_candidates;
    for (size_t uninitialized_obj_idx = 0;
         uninitialized_obj_idx <
         RoshanBbFrontEnd::uninitialized_object_info_.size();
         uninitialized_obj_idx++) {
      UninitializedEllispoidInfo<RoshanAggregateBbInfo, util::EmptyStruct>
          uninitialized_obj = RoshanBbFrontEnd::uninitialized_object_info_
              [uninitialized_obj_idx];
      if (uninitialized_obj.semantic_class_ == bounding_box.semantic_class_) {
        AssociatedObjectIdentifier assoc_obj;
        assoc_obj.initialized_ellipsoid_ = false;
        assoc_obj.object_id_ = uninitialized_obj_idx;
        associated_obj_candidates.emplace_back(
            std::make_pair(assoc_obj, util::EmptyStruct()));
      }
    }

    std::unordered_set<ObjectId> pg_candidates =
        RoshanBbFrontEnd::pose_graph_->getObjectsWithSemanticClass(
            bounding_box.semantic_class_);
    for (const ObjectId &candidate : pg_candidates) {
      AssociatedObjectIdentifier assoc_obj;
      assoc_obj.initialized_ellipsoid_ = true;
      assoc_obj.object_id_ = candidate;
      associated_obj_candidates.emplace_back(
          std::make_pair(assoc_obj, util::EmptyStruct()));
    }
    return associated_obj_candidates;
  }

  virtual std::vector<std::pair<AssociatedObjectIdentifier, util::EmptyStruct>>
  pruneCandidateMatches(
      const FrameId &frame_id,
      const CameraId &camera_id,
      const RawBoundingBox &bounding_box,
      const std::vector<std::pair<AssociatedObjectIdentifier,
                                  util::EmptyStruct>> &candidate_matches,
      const RoshanBbInfo &bb_context) override {
    if (candidate_matches.empty()) {
      return {};
    }
    if (!bb_context.est_generated_) {
      return {};
    }
    std::vector<std::pair<AssociatedObjectIdentifier, util::EmptyStruct>>
        pruned_candidates;
    for (const std::pair<AssociatedObjectIdentifier, util::EmptyStruct>
             &candidate : candidate_matches) {
      double centroid_dist = std::numeric_limits<double>::max();
      if (candidate.first.initialized_ellipsoid_) {
        std::optional<EllipsoidState<double>> candidate_ellispoid_state_opt =
            RoshanBbFrontEnd::pose_graph_->getEllipsoidEst(
                candidate.first.object_id_);
        if (!candidate_ellispoid_state_opt.has_value()) {
          continue;
        }

        centroid_dist = (bb_context.single_bb_init_est_.pose_.transl_ -
                         candidate_ellispoid_state_opt.value().pose_.transl_)
                            .norm();
      } else {
        UninitializedEllispoidInfo<RoshanAggregateBbInfo, util::EmptyStruct>
            uninitialized_obj =
                RoshanBbFrontEnd::uninitialized_object_info_[candidate.first
                                                                 .object_id_];
        for (const RoshanBbInfo &single_bb_info :
             uninitialized_obj.appearance_info_.infos_for_observed_bbs_) {
          double obs_centroid_dist =
              (bb_context.single_bb_init_est_.pose_.transl_ -
               single_bb_info.single_bb_init_est_.pose_.transl_)
                  .norm();
          centroid_dist = std::min(centroid_dist, obs_centroid_dist);
        }
      }
      if (centroid_dist <=
          association_params_.max_distance_for_associated_ellipsoids_) {
        pruned_candidates.emplace_back(candidate);
      }
    }
    return pruned_candidates;
  }

  virtual double scoreCandidateMatch(
      const FrameId &frame_id,
      const CameraId &camera_id,
      const RawBoundingBox &bounding_box,
      const std::pair<AssociatedObjectIdentifier, util::EmptyStruct> &candidate,
      const RoshanBbInfo &bounding_box_appearance_info) override {
    RoshanAggregateBbInfo aggregate_bb_info;
    if (candidate.first.initialized_ellipsoid_) {
      aggregate_bb_info =
          RoshanBbFrontEnd::object_appearance_info_[candidate.first.object_id_];
    } else {
      aggregate_bb_info =
          RoshanBbFrontEnd::uninitialized_object_info_[candidate.first
                                                           .object_id_]
              .appearance_info_;
    }

    std::vector<double> correlation_scores;
    for (const RoshanBbInfo &single_bb_info :
         aggregate_bb_info.infos_for_observed_bbs_) {
      correlation_scores.emplace_back(
          cv::compareHist(single_bb_info.hue_sat_histogram_,
                          bounding_box_appearance_info.hue_sat_histogram_,
                          cv::HISTCMP_CORREL));
    }

    // TODO how do we want to combine the correlation scores? Right now just
    // taking the max
    double max_score =
        *std::max_element(correlation_scores.begin(), correlation_scores.end());
    return max_score;
  }

  virtual std::vector<AssociatedObjectIdentifier> assignBoundingBoxes(
      const std::vector<
          std::vector<std::pair<AssociatedObjectIdentifier, double>>>
          &match_candidates_with_scores) override {
    // TODO for now we're just doing this more or less greedily. Should change
    //  to find best overall matching
    return greedilyAssignBoundingBoxes(
        match_candidates_with_scores,
        RoshanBbFrontEnd::uninitialized_object_info_.size());
  }

  virtual RoshanImageSummaryInfo generateRefinedBbContextInfo(
      const std::optional<sensor_msgs::Image::ConstPtr> &bb_context,
      const FrameId &frame_id,
      const CameraId &camera_id) override {
    RoshanImageSummaryInfo summary_info;
    if (bb_context.has_value()) {
      // TODO is this the right way to specify encoding?
      cv_bridge::CvImageConstPtr cv_img =
          cv_bridge::toCvShare(bb_context.value(), "bgr8");
      summary_info.hsv_img_ = std::make_optional(cv::Mat());
      // TODO is this the right code to switch to HSV?
      cv::cvtColor(cv_img->image, *(summary_info.hsv_img_), cv::COLOR_BGR2HSV);
    }
    return summary_info;
  }

  virtual void setupBbAssociationRound(
      const vslam_types_refactor::FrameId &frame_id,
      const vslam_types_refactor::CameraId &camera_id,
      const std::vector<RawBoundingBox> &bounding_boxes,
      const std::optional<sensor_msgs::Image::ConstPtr> &raw_bb_context,
      const RoshanImageSummaryInfo &refined_bb_context) override {}

  virtual void cleanupBbAssociationRound(
      const vslam_types_refactor::FrameId &frame_id,
      const vslam_types_refactor::CameraId &camera_id) override {
    if (association_params_.discard_candidate_after_num_frames_ > 0) {
      std::vector<
          UninitializedEllispoidInfo<RoshanAggregateBbInfo, util::EmptyStruct>>
          uninitialized_object_info_to_keep;
      removeStalePendingObjects(
          RoshanBbFrontEnd::uninitialized_object_info_,
          frame_id,
          association_params_.discard_candidate_after_num_frames_,
          uninitialized_object_info_to_keep);
      // TODO maybe also remove if it hasn't been initialized N frames after
      // first viewing (or maybe remove old viewings)
      RoshanBbFrontEnd::uninitialized_object_info_ =
          uninitialized_object_info_to_keep;
    }
  }

  virtual void setupInitialEstimateGeneration(
      const std::vector<AssociatedObjectIdentifier> &bounding_box_assignments,
      const FrameId &frame_id,
      const CameraId &camera_id) override {}

  virtual ObjectInitializationStatus tryInitializeEllipsoid(
      const RoshanImageSummaryInfo &refined_bb_context,
      const UninitializedEllispoidInfo<RoshanAggregateBbInfo, util::EmptyStruct>
          &uninitialized_bb_info,
      vslam_types_refactor::EllipsoidState<double> &ellipsoid_est) override {
    if (uninitialized_bb_info.observation_factors_.empty()) {
      return NOT_INITIALIZED;
    }
    if (association_params_.min_observations_ >
        uninitialized_bb_info.observation_factors_.size()) {
      return NOT_INITIALIZED;
    }
    if (association_params_.required_min_conf_for_initialization >
        uninitialized_bb_info.appearance_info_.max_confidence_) {
      return NOT_INITIALIZED;
    }

    RoshanBbInfo single_info =
        uninitialized_bb_info.appearance_info_.infos_for_observed_bbs_.back();
    ellipsoid_est = single_info.single_bb_init_est_;
    return single_info.est_generated_ ? SUFFICIENT_VIEWS_FOR_NEW
                                      : NOT_INITIALIZED;
  }

  virtual void addObservationForObject(
      const FrameId &frame_id,
      const CameraId &camera_id,
      const ObjectId &object_id,
      const BbCorners<double> &bb_corners,
      const Covariance<double, 4> &bb_cov,
      const double &detection_confidence) override {
    AbstractBoundingBoxFrontEnd<
        VisualFeatureFactorType,
        RoshanAggregateBbInfo,
        util::EmptyStruct,
        std::optional<sensor_msgs::Image::ConstPtr>,
        RoshanImageSummaryInfo,
        RoshanBbInfo,
        std::unordered_map<ObjectId, RoshanAggregateBbInfo>>::
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
  RoshanBbAssociationParams association_params_;

  std::function<Covariance<double, 4>(const RawBoundingBox &,
                                      const FrameId &,
                                      const CameraId &,
                                      const RoshanImageSummaryInfo &)>
      covariance_generator_;

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

  bool initializeEllipsoid(
      const FrameId &frame_id,
      const CameraId &camera_id,
      const std::string &semantic_class,
      const BbCorners<double> &bb,
      vslam_types_refactor::EllipsoidState<double> &ellipsoid_est) {
    return generateSingleViewEllipsoidEstimate(RoshanBbFrontEnd::pose_graph_,
                                               frame_id,
                                               camera_id,
                                               semantic_class,
                                               bb,
                                               ellipsoid_est);
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
    for (const auto &mergable_obj : mergable_objects_and_ests) {
      pending_objects_to_add.emplace_back(
          std::make_pair(mergable_obj.first, mergable_obj.second.second));
    }
  }
};

template <typename VisualFeatureFactorType>
class RoshanBbFrontEndCreator {
 public:
  RoshanBbFrontEndCreator(
      const RoshanBbAssociationParams &association_params,
      const std::shared_ptr<std::unordered_map<
          FrameId,
          std::unordered_map<
              CameraId,
              std::unordered_map<
                  ObjectId,
                  std::pair<BbCornerPair<double>, std::optional<double>>>>>>
          &observed_corner_locations,
      const std::shared_ptr<std::unordered_map<
          FrameId,
          std::unordered_map<CameraId,
                             std::vector<std::pair<BbCornerPair<double>,
                                                   std::optional<double>>>>>>
          &all_filtered_corner_locations,
      const std::function<Covariance<double, 4>(const RawBoundingBox &,
                                                const FrameId &,
                                                const CameraId &,
                                                const RoshanImageSummaryInfo &)>
          &covariance_generator,
      const std::unordered_map<vslam_types_refactor::ObjectId,
                               RoshanAggregateBbInfo>
          &long_term_map_front_end_data)
      : association_params_(association_params),
        covariance_generator_(covariance_generator),
        initialized_(false),
        observed_corner_locations_(observed_corner_locations),
        all_filtered_corner_locations_(all_filtered_corner_locations),
        long_term_map_front_end_data_(long_term_map_front_end_data) {}

  std::shared_ptr<AbstractBoundingBoxFrontEnd<
      VisualFeatureFactorType,
      RoshanAggregateBbInfo,
      util::EmptyStruct,
      std::optional<sensor_msgs::Image::ConstPtr>,
      RoshanImageSummaryInfo,
      RoshanBbInfo,
      std::unordered_map<ObjectId, RoshanAggregateBbInfo>>>
  getDataAssociator(const std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph>
                        &pose_graph) {
    if (!initialized_) {
      roshan_front_end_ =
          std::make_shared<RoshanBbFrontEnd<VisualFeatureFactorType>>(
              pose_graph,
              association_params_,
              covariance_generator_,
              observed_corner_locations_,
              all_filtered_corner_locations_);
      roshan_front_end_->initializeWithLongTermMapFrontEndData(
          long_term_map_front_end_data_);
      initialized_ = true;
    }
    return roshan_front_end_;
  }

 private:
  RoshanBbAssociationParams association_params_;
  std::function<Covariance<double, 4>(const RawBoundingBox &,
                                      const FrameId &,
                                      const CameraId &,
                                      const RoshanImageSummaryInfo &)>
      covariance_generator_;
  bool initialized_;

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
  std::shared_ptr<std::unordered_map<
      FrameId,
      std::unordered_map<
          CameraId,
          std::vector<std::pair<BbCornerPair<double>, std::optional<double>>>>>>
      all_filtered_corner_locations_;

  std::unordered_map<vslam_types_refactor::ObjectId, RoshanAggregateBbInfo>
      long_term_map_front_end_data_;
  std::shared_ptr<AbstractBoundingBoxFrontEnd<
      VisualFeatureFactorType,
      RoshanAggregateBbInfo,
      util::EmptyStruct,
      std::optional<sensor_msgs::Image::ConstPtr>,
      RoshanImageSummaryInfo,
      RoshanBbInfo,
      std::unordered_map<ObjectId, RoshanAggregateBbInfo>>>
      roshan_front_end_;
};

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_ROSHAN_BOUNDING_BOX_FRONT_END_H
