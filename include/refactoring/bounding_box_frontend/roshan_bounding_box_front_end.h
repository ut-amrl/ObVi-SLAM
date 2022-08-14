//
// Created by amanda on 6/21/22.
//

#ifndef UT_VSLAM_ROSHAN_BOUNDING_BOX_FRONT_END_H
#define UT_VSLAM_ROSHAN_BOUNDING_BOX_FRONT_END_H

#include <cv_bridge/cv_bridge.h>
#include <refactoring/bounding_box_frontend/bounding_box_front_end.h>
#include <refactoring/types/vslam_types_math_util.h>
#include <sensor_msgs/Image.h>

namespace vslam_types_refactor {

const float kHRanges[] = {0, 180};
const float kSRanges[] = {0, 256};
const int kHAndSChannels[] = {0, 1};

const int kNoDiscardingConst = INT_MIN;

struct RoshanBbAssociationParams {
  double max_distance_for_associated_ellipsoids_;
  int hue_histogram_bins_;
  int saturation_histogram_bins_;
  uint16_t min_observations_ = 1;
  int discard_candidate_after_num_frames_ = kNoDiscardingConst;
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
};

struct RoshanAggregateBbInfo {
  std::vector<RoshanBbInfo> infos_for_observed_bbs_;
};

template <typename VisualFeatureFactorType>
class RoshanBbFrontEnd
    : public AbstractUnknownDataAssociationBbFrontEnd<
          VisualFeatureFactorType,
          RoshanAggregateBbInfo,
          std::optional<sensor_msgs::Image::ConstPtr>,
          RoshanImageSummaryInfo,
          RoshanBbInfo,
          std::unordered_map<ObjectId, RoshanAggregateBbInfo>> {
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
              std::unordered_map<ObjectId, BbCornerPair<double>>>>>
          &observed_corner_locations,
      const std::unordered_map<vslam_types_refactor::ObjectId,
                               RoshanAggregateBbInfo>
          &long_term_map_front_end_data)
      : AbstractUnknownDataAssociationBbFrontEnd<
            VisualFeatureFactorType,
            RoshanAggregateBbInfo,
            std::optional<sensor_msgs::Image::ConstPtr>,
            RoshanImageSummaryInfo,
            RoshanBbInfo,
            std::unordered_map<ObjectId, RoshanAggregateBbInfo>>(pose_graph),
        association_params_(association_params),
        covariance_generator_(covariance_generator),
        observed_corner_locations_(observed_corner_locations) {
    AbstractBoundingBoxFrontEnd<
        VisualFeatureFactorType,
        RoshanAggregateBbInfo,
        std::optional<sensor_msgs::Image::ConstPtr>,
        RoshanImageSummaryInfo,
        RoshanBbInfo,
        std::unordered_map<vslam_types_refactor::ObjectId,
                           RoshanAggregateBbInfo>>::object_appearance_info_ =
        long_term_map_front_end_data;
  }

  virtual bool getFrontEndObjMapData(
      std::unordered_map<vslam_types_refactor::ObjectId, RoshanAggregateBbInfo>
          &map_data) override {
    map_data = AbstractBoundingBoxFrontEnd<
        VisualFeatureFactorType,
        RoshanAggregateBbInfo,
        std::optional<sensor_msgs::Image::ConstPtr>,
        RoshanImageSummaryInfo,
        RoshanBbInfo,
        std::unordered_map<vslam_types_refactor::ObjectId,
                           RoshanAggregateBbInfo>>::object_appearance_info_;
    return true;
  }

 protected:
  virtual void updateAppearanceInfoWithObjectIdAssignmentAndInitialization(
      const ObjectId &obj_id,
      const EllipsoidEstimateNode &est,
      RoshanAggregateBbInfo &info_to_update) override {}

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

    EllipsoidEstimateNode raw_est;
    bb_info.est_generated_ = initializeEllipsoid(
        frame_id,
        camera_id,
        bb.semantic_class_,
        cornerLocationsPairToVector(bb.pixel_corner_locations_),
        raw_est);

    bb_info.single_bb_init_est_ =
        convertToEllipsoidState(*(raw_est.ellipsoid_));

    return bb_info;
  }

  virtual UninitializedEllispoidInfo<RoshanAggregateBbInfo>
  createObjectInfoFromSingleBb(
      const std::string &semantic_class,
      const FrameId &frame_id,
      const CameraId &camera_id,
      const UninitializedObjectFactor &uninitialized_factor,
      const RoshanImageSummaryInfo &refined_context,
      const RoshanBbInfo &single_bb_context_info) override {
    UninitializedEllispoidInfo<RoshanAggregateBbInfo> uninitalized_info;
    uninitalized_info.semantic_class_ = semantic_class;
    uninitalized_info.observation_factors_.emplace_back(uninitialized_factor);
    uninitalized_info.appearance_info_.infos_for_observed_bbs_.emplace_back(
        single_bb_context_info);
    return uninitalized_info;
  }

  virtual void mergeSingleBbContextIntoObjectAssociationInfo(
      const RawBoundingBox &raw_bb,
      const FrameId &frame_id,
      const CameraId &camera_id,
      const RoshanImageSummaryInfo &refined_bb_context_info,
      const RoshanBbInfo &single_bb_context_info,
      RoshanAggregateBbInfo &association_info_to_update) override {
    association_info_to_update.infos_for_observed_bbs_.emplace_back(
        single_bb_context_info);
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

  virtual std::vector<AssociatedObjectIdentifier> identifyCandidateMatches(
      const FrameId &frame_id,
      const CameraId &camera_id,
      const RawBoundingBox &bounding_box,
      const RoshanBbInfo &bb_context) override {
    std::vector<AssociatedObjectIdentifier> associated_obj_candidates;
    for (size_t uninitialized_obj_idx = 0;
         uninitialized_obj_idx <
         RoshanBbFrontEnd::uninitialized_object_info_.size();
         uninitialized_obj_idx++) {
      UninitializedEllispoidInfo<RoshanAggregateBbInfo> uninitialized_obj =
          RoshanBbFrontEnd::uninitialized_object_info_[uninitialized_obj_idx];
      if (uninitialized_obj.semantic_class_ == bounding_box.semantic_class_) {
        AssociatedObjectIdentifier assoc_obj;
        assoc_obj.initialized_ellipsoid_ = false;
        assoc_obj.object_id_ = uninitialized_obj_idx;
        associated_obj_candidates.emplace_back(assoc_obj);
      }
    }

    std::unordered_set<ObjectId> pg_candidates =
        RoshanBbFrontEnd::pose_graph_->getObjectsWithSemanticClass(
            bounding_box.semantic_class_);
    for (const ObjectId &candidate : pg_candidates) {
      AssociatedObjectIdentifier assoc_obj;
      assoc_obj.initialized_ellipsoid_ = true;
      assoc_obj.object_id_ = candidate;
      associated_obj_candidates.emplace_back(assoc_obj);
    }
    return associated_obj_candidates;
  }

  virtual std::vector<AssociatedObjectIdentifier>
  pruneCandidateMatchesBasedOnGeometry(
      const FrameId &frame_id,
      const CameraId &camera_id,
      const RawBoundingBox &bounding_box,
      const std::vector<AssociatedObjectIdentifier> &candidate_matches,
      const RoshanBbInfo &bb_context) override {
    if (candidate_matches.empty()) {
      return {};
    }
    if (!bb_context.est_generated_) {
      return {};
    }
    std::vector<AssociatedObjectIdentifier> pruned_candidates;
    for (const AssociatedObjectIdentifier &candidate : candidate_matches) {
      double centroid_dist = std::numeric_limits<double>::max();
      if (candidate.initialized_ellipsoid_) {
        std::optional<EllipsoidState<double>> candidate_ellispoid_state_opt =
            RoshanBbFrontEnd::pose_graph_->getEllipsoidEst(
                candidate.object_id_);
        if (!candidate_ellispoid_state_opt.has_value()) {
          continue;
        }

        centroid_dist = (bb_context.single_bb_init_est_.pose_.transl_ -
                         candidate_ellispoid_state_opt.value().pose_.transl_)
                            .norm();
      } else {
        UninitializedEllispoidInfo<RoshanAggregateBbInfo> uninitialized_obj =
            RoshanBbFrontEnd::uninitialized_object_info_[candidate.object_id_];
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
      const AssociatedObjectIdentifier &candidate,
      const RoshanBbInfo &bounding_box_appearance_info) override {
    RoshanAggregateBbInfo aggregate_bb_info;
    if (candidate.initialized_ellipsoid_) {
      aggregate_bb_info =
          RoshanBbFrontEnd::object_appearance_info_[candidate.object_id_];
    } else {
      aggregate_bb_info =
          RoshanBbFrontEnd::uninitialized_object_info_[candidate.object_id_]
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
    std::vector<
        std::pair<uint64_t, std::pair<AssociatedObjectIdentifier, double>>>
        flattened_scores;
    for (size_t bb_idx = 0; bb_idx < match_candidates_with_scores.size();
         bb_idx++) {
      for (const std::pair<AssociatedObjectIdentifier, double>
               &candidate_for_bb : match_candidates_with_scores[bb_idx]) {
        flattened_scores.emplace_back(std::make_pair(bb_idx, candidate_for_bb));
      }
    }

    std::sort(
        flattened_scores.begin(),
        flattened_scores.end(),
        [](const std::pair<uint64_t,
                           std::pair<AssociatedObjectIdentifier, double>> &lhs,
           const std::pair<uint64_t,
                           std::pair<AssociatedObjectIdentifier, double>>
               &rhs) { return lhs.second.second > rhs.second.second; });

    util::BoostHashSet<AssociatedObjectIdentifier> claimed_objs;
    std::unordered_map<uint64_t, AssociatedObjectIdentifier> assignments_map;

    for (const std::pair<uint64_t,
                         std::pair<AssociatedObjectIdentifier, double>>
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

    ObjectId next_free_assignment =
        RoshanBbFrontEnd::uninitialized_object_info_.size();
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
      std::vector<UninitializedEllispoidInfo<RoshanAggregateBbInfo>>
          uninitialized_object_info_to_keep;
      for (const UninitializedEllispoidInfo<RoshanAggregateBbInfo>
               &uninitialized_info :
           RoshanBbFrontEnd::uninitialized_object_info_) {
        FrameId max_frame_for_obj = 0;
        for (const UninitializedObjectFactor &bb_info :
             uninitialized_info.observation_factors_) {
          max_frame_for_obj = std::max(bb_info.frame_id_, max_frame_for_obj);
        }
        if (frame_id <=
            (max_frame_for_obj +
             association_params_.discard_candidate_after_num_frames_)) {
          uninitialized_object_info_to_keep.emplace_back(uninitialized_info);
        }
      }
      RoshanBbFrontEnd::uninitialized_object_info_ =
          uninitialized_object_info_to_keep;
    }
  }

  bool tryInitializeEllipsoid(
      const RoshanImageSummaryInfo &refined_bb_context,
      const RoshanAggregateBbInfo &association_info,
      const std::vector<UninitializedObjectFactor> &factors,
      const std::string &semantic_class,
      const bool &already_init,
      vslam_types_refactor::EllipsoidEstimateNode &ellipsoid_est) override {
    if (factors.empty()) {
      return false;
    }
    if (association_params_.min_observations_ > factors.size()) {
      return false;
    }

    RoshanBbInfo single_info = association_info.infos_for_observed_bbs_.back();
    ellipsoid_est.updateEllipsoidParams(
        convertPoseToArray(single_info.single_bb_init_est_.pose_),
        single_info.single_bb_init_est_.dimensions_);
    return single_info.est_generated_;
  }

  virtual void addObservationForObject(
      const FrameId &frame_id,
      const CameraId &camera_id,
      const ObjectId &object_id,
      const BbCorners<double> &bb_corners,
      const Covariance<double, 4> &bb_cov) override {
    AbstractBoundingBoxFrontEnd<
        VisualFeatureFactorType,
        RoshanAggregateBbInfo,
        std::optional<sensor_msgs::Image::ConstPtr>,
        RoshanImageSummaryInfo,
        RoshanBbInfo,
        std::unordered_map<ObjectId, RoshanAggregateBbInfo>>::
        addObservationForObject(
            frame_id, camera_id, object_id, bb_corners, bb_cov);
    (*observed_corner_locations_)[frame_id][camera_id][object_id] =
        cornerLocationsVectorToPair(bb_corners);
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
      std::unordered_map<CameraId,
                         std::unordered_map<ObjectId, BbCornerPair<double>>>>>
      observed_corner_locations_;

  double getObjectDepth(const BbCornerPair<double> &bb,
                        const ObjectDim<double> &mean_dim_for_class,
                        const double &fy) {
    double y_diff = bb.second.y() - bb.first.y();
    // TODO this isn't a valid way to do it, because this assumes the points
    // corresponding to the top and bottom of the bounding box correspond to the
    // min/max points in the z direction on an upright ellipsoid, which would
    // only be the case if the depth was infinite (as we get closer, the tangent
    // points are closer to the tip of the ellipsoid facing the camera
    return mean_dim_for_class.z() * fy / y_diff;
  }

  bool initializeEllipsoid(
      const FrameId &frame_id,
      const CameraId &camera_id,
      const std::string &semantic_class,
      const BbCorners<double> &bb,
      vslam_types_refactor::EllipsoidEstimateNode &ellipsoid_est) {
    std::optional<ObjectDim<double>> mean_dim_for_class =
        RoshanBbFrontEnd::pose_graph_->getShapeDimMean(semantic_class);

    if (!mean_dim_for_class.has_value()) {
      LOG(ERROR)
          << "Could not initialize; no mean dimension provided for class "
          << semantic_class;
      return false;
    }
    ObjectDim<double> dim = mean_dim_for_class.value();
    CameraIntrinsicsMat<double> intrinsics;
    RoshanBbFrontEnd::pose_graph_->getIntrinsicsForCamera(camera_id,
                                                          intrinsics);
    BbCornerPair<double> bb_pair = cornerLocationsVectorToPair(bb);
    double depth = getObjectDepth(bb_pair, dim, intrinsics(1, 1));
    PixelCoord<double> bb_center = (bb_pair.first + bb_pair.second) / 2;
    Eigen::Vector3d bb_center_homogeneous(bb_center.x(), bb_center.y(), 1);
    Position3d<double> position_rel_cam =
        depth * intrinsics.inverse() * bb_center_homogeneous;

    Pose3D<double> pose_rel_to_cam(position_rel_cam, Orientation3D<double>());

    std::optional<RawPose3d<double>> raw_robot_pose =
        RoshanBbFrontEnd::pose_graph_->getRobotPose(frame_id);
    if (!raw_robot_pose.has_value()) {
      return false;
    }
    Pose3D<double> robot_pose = convertToPose3D(raw_robot_pose.value());
    CameraExtrinsics<double> extrinsics;
    RoshanBbFrontEnd::pose_graph_->getExtrinsicsForCamera(camera_id,
                                                          extrinsics);
    Pose3D<double> camera_pose = combinePoses(robot_pose, extrinsics);
    Pose3D<double> global_pose = combinePoses(camera_pose, pose_rel_to_cam);
    ellipsoid_est.updateEllipsoidParams(convertPoseToArray(global_pose), dim);
    return true;
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
              std::unordered_map<ObjectId, BbCornerPair<double>>>>>
          &observed_corner_locations,
      const std::function<Covariance<double, 4>(const RawBoundingBox &,
                                                const FrameId &,
                                                const CameraId &,
                                                const RoshanImageSummaryInfo &)>
          &covariance_generator,
      std::unordered_map<vslam_types_refactor::ObjectId, RoshanAggregateBbInfo>
          &long_term_map_front_end_data)
      : association_params_(association_params),
        covariance_generator_(covariance_generator),
        initialized_(false),
        observed_corner_locations_(observed_corner_locations),
        long_term_map_front_end_data_(long_term_map_front_end_data) {}

  std::shared_ptr<AbstractUnknownDataAssociationBbFrontEnd<
      VisualFeatureFactorType,
      RoshanAggregateBbInfo,
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
  std::shared_ptr<std::unordered_map<
      FrameId,
      std::unordered_map<CameraId,
                         std::unordered_map<ObjectId, BbCornerPair<double>>>>>
      observed_corner_locations_;
  std::unordered_map<vslam_types_refactor::ObjectId, RoshanAggregateBbInfo>
      long_term_map_front_end_data_;
  std::shared_ptr<AbstractUnknownDataAssociationBbFrontEnd<
      VisualFeatureFactorType,
      RoshanAggregateBbInfo,
      std::optional<sensor_msgs::Image::ConstPtr>,
      RoshanImageSummaryInfo,
      RoshanBbInfo,
      std::unordered_map<ObjectId, RoshanAggregateBbInfo>>>
      roshan_front_end_;
};

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_ROSHAN_BOUNDING_BOX_FRONT_END_H
