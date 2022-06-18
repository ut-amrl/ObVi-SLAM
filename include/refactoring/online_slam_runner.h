//
// Created by amanda on 6/9/22.
//

#ifndef UT_VSLAM_ONLINE_SLAM_RUNNER_H
#define UT_VSLAM_ONLINE_SLAM_RUNNER_H

#include <refactoring/object_pose_graph.h>
#include <sensor_msgs/Image.h>

namespace online_runner {

typedef uint64_t UninitializedEllispoidId;

struct OnlineBoundingBoxDetection {
  std::string semantic_class_;
  vslam_types_refactor::BbCorners<double> bounding_box_corners_;
};

template <typename ObjectAppearanceInfo>
struct UninitializedEllispoidInfo {
  ObjectAppearanceInfo appearance_info_;

  std::unordered_map<vslam_types_refactor::FrameId,
                     std::unordered_map<vslam_types_refactor::CameraId,
                                        OnlineBoundingBoxDetection>>
      bounding_box_detections_;
};

struct AssociatedObjectIdentifier {
  bool initialized_ellipsoid_;

  // If initialized object, this will be the object's id
  // If uninitialized, this will the index for the uninitialized object info
  // in the list of uninitialized objects
  vslam_types_refactor::ObjectId object_id_;
};

template <typename ObjectAppearanceInfo,
          typename VisualFeatureFactorType,
          typename SingleBbAppearanceInfo,
          typename NecessaryImageData,
          typename FrameDependentEllipsoidInitData>
class ObjectDataAssociator {
 public:
  void performBoundingBoxDataAssociationAndUpdatePoseGraph(
      const vslam_types_refactor::FrameId &frame_id,
      const vslam_types_refactor::CameraId &camera_id,
      const vslam_types_refactor::CameraIntrinsicsMat<double> &intrinsics,
      const vslam_types_refactor::CameraExtrinsics<double> &extrinsics,
      const std::vector<OnlineBoundingBoxDetection> &bounding_boxes,
      const sensor_msgs::Image::ConstPtr &image,
      std::unordered_map<vslam_types_refactor::ObjectId,
                         ObjectAppearanceInfo> &object_appearance_info,
      std::vector<UninitializedEllispoidInfo<ObjectAppearanceInfo>>
          &uninitialized_object_info,
      std::shared_ptr<vslam_types_refactor::ObjAndLowLevelFeaturePoseGraph<
          VisualFeatureFactorType>> &pose_graph) {
    NecessaryImageData necessary_image_data = extractNecessaryImageData(image);

    FrameDependentEllipsoidInitData frame_dep_object_init_data =
        extractFrameDependentInitializationData(
            frame_id, camera_id, image, pose_graph);

    // For each bounding box, generate single bb appearance info
    std::vector<SingleBbAppearanceInfo> single_bb_appearance_infos;
    for (const OnlineBoundingBoxDetection &bb : bounding_boxes) {
      single_bb_appearance_infos.emplace_back(
          generateSingleBoundingBoxAppearanceInfo(necessary_image_data,
                                                  bounding_boxes));
    }

    std::vector<std::vector<std::pair<AssociatedObjectIdentifier, double>>>
        match_candidates_with_scores;
    for (size_t i = 0; i < bounding_boxes.size(); i++) {
      std::vector<std::pair<AssociatedObjectIdentifier, double>>
          candidates_with_scores_for_bb;
      OnlineBoundingBoxDetection bb = bounding_boxes[i];

      // For each bounding box, identify candidates
      std::vector<AssociatedObjectIdentifier> candidates =
          identifyCandidateMatches(
              bb, pose_graph, uninitialized_object_info);

      // For each bounding box, prune candidates using geometric checks
      candidates = pruneCandidateMatchesBasedOnGeometry(
          bb, candidates, pose_graph, uninitialized_object_info);

      // For each bounding box, calculate data association scores for each
      // candidate
      SingleBbAppearanceInfo bb_appearance_info = single_bb_appearance_infos[i];
      for (const AssociatedObjectIdentifier &candidate : candidates) {
        double score = scoreCandidateMatch(bb,
                                           candidate,
                                           bb_appearance_info,
                                           object_appearance_info,
                                           pose_graph,
                                           uninitialized_object_info);
        // If the score is negative infinity, this should not be matched even if
        // there are no better matches (better to make new object)
        if (score != (-1 * std::numeric_limits<double>::infinity())) {
          candidates_with_scores_for_bb.emplace_back(
              std::make_pair(candidate, score));
        }
      }
    }

    // Assign bounding boxes to objects/uninitialized objects based on
    // scores/determine which need new bbs

    std::vector<AssociatedObjectIdentifier> bounding_box_assignments =
        assignBoundingBoxes(match_candidates_with_scores,
                            pose_graph,
                            uninitialized_object_info.size());

    // Combine appearance infos
    combineAppearanceInfosAndAddNewEllipsoids(single_bb_appearance_infos,
                                              bounding_box_assignments,
                                              object_appearance_info,
                                              uninitialized_object_info);

    // For each object, add information and either initialize, delay
    // initialization (until we have more information, or refine estimate (if
    // already initialized, but with little data)
    std::vector<vslam_types_refactor::ObjectId>
        newly_initialized_object_init_indices;
    for (size_t i = 0; i < bounding_box_assignments.size(); i++) {
      AssociatedObjectIdentifier bounding_box_assignment =
          bounding_box_assignments[i];
      OnlineBoundingBoxDetection bounding_box = bounding_boxes[i];

      if (bounding_box_assignment.initialized_ellipsoid_) {
        // If initialized, add bb info to pose graph
        addObservationForExistingObject(bounding_box,
                                        necessary_image_data,
                                        bounding_box_assignment.object_id_,
                                        frame_id,
                                        camera_id,
                                        pose_graph);
        // TODO try refining estimate by initializing with all new data and
        // seeing if the cost is lower than current estimate
      } else {
        // If not initialized,
        // add bounding box info to data
        uninitialized_object_info[bounding_box_assignment.object_id_]
            .bounding_box_detections_[frame_id][camera_id] = bounding_box;

        // Check if it can be initialized and if so, what to initialize to
        vslam_types_refactor::EllipsoidEstimateNode object_est;
        bool initialized = tryInitializeEllipsoid(
            uninitialized_object_info[bounding_box_assignment.object_id_]
                .bounding_box_detections_,
            false,
            object_est);
        if (initialized) {
          pose_graph->addNewEllipsoid(object_est,
                                      bounding_box.semantic_class_);
          newly_initialized_object_init_indices.emplace_back(
              bounding_box_assignment.object_id_);
          for (const OnlineBoundingBoxDetection &bb_det :
               uninitialized_object_info[bounding_box_assignment
                                                .object_id_]
                   .bounding_box_detections_) {
            addObservationForExistingObject(
                bb_det,
                necessary_image_data,
                bounding_box_assignment.object_id_,
                frame_id,
                camera_id,
                pose_graph);
          }
        }
      }
    }

    // Sort the objects that just have been added to the pose graph in
    // descending order and delete them from the uninitialized objects list
    // Descending order needed so the indices to delete don't change as they're
    // removed
    std::sort(newly_initialized_object_init_indices.begin(),
              newly_initialized_object_init_indices.end(),
              std::greater<size_t>());
    for (const size_t &pending_init_object_index :
         newly_initialized_object_init_indices) {
      uninitialized_object_info.erase(uninitialized_object_info.begin() +
                                         pending_init_object_index);
    }
  }

 protected:
  virtual FrameDependentEllipsoidInitData
  extractFrameDependentInitializationData(
      const vslam_types_refactor::FrameId &frame_id,
      const vslam_types_refactor::CameraId &camera_id,
      const sensor_msgs::Image &image,
      const std::shared_ptr<
          vslam_types_refactor::ObjAndLowLevelFeaturePoseGraph<
              VisualFeatureFactorType>> &pose_graph) = 0;

  bool tryInitializeEllipsoid(
      const std::unordered_map<
          vslam_types_refactor::FrameId,
          std::unordered_map<vslam_types_refactor::CameraId,
                             OnlineBoundingBoxDetection>>
          bounding_box_detections,
      const bool &already_init,
      vslam_types_refactor::EllipsoidEstimateNode &ellipsoid_est) = 0;

  virtual vslam_types_refactor::Covariance<double, 4>
  generateBoundingBoxDetectionCovariance(
      const OnlineBoundingBoxDetection &bounding_box_detection,
      const NecessaryImageData &image_data) = 0;

  void addObservationForExistingObject(
      const OnlineBoundingBoxDetection &bounding_box,
      const NecessaryImageData &image_data,
      const vslam_types_refactor::ObjectId &object,
      const vslam_types_refactor::FrameId &observed_at_frame,
      const vslam_types_refactor::CameraId &observed_with_camera,
      std::shared_ptr<vslam_types_refactor::ObjAndLowLevelFeaturePoseGraph<
          VisualFeatureFactorType>> &pose_graph) {
    vslam_types_refactor::ObjectObservationFactor factor;
    factor.frame_id_ = observed_at_frame;
    factor.camera_id_ = observed_with_camera;
    factor.object_id_ = object;
    factor.bounding_box_corners_ = bounding_box.bounding_box_corners_;
    factor.bounding_box_corners_covariance_ =
        generateBoundingBoxDetectionCovariance(bounding_box, image_data);

    pose_graph->addObjectObservation(factor);
  }

  virtual NecessaryImageData extractNecessaryImageData(
      const sensor_msgs::Image::ConstPtr &image) = 0;

  virtual SingleBbAppearanceInfo generateSingleBoundingBoxAppearanceInfo(
      const NecessaryImageData &necessary_image_data,
      const OnlineBoundingBoxDetection &bounding_box_detection) = 0;

  virtual std::vector<AssociatedObjectIdentifier> identifyCandidateMatches(
      const OnlineBoundingBoxDetection &bounding_box,
      const std::shared_ptr<
          const vslam_types_refactor::ObjAndLowLevelFeaturePoseGraph<
              VisualFeatureFactorType>> &pose_graph,
      const std::vector<UninitializedEllispoidInfo<ObjectAppearanceInfo>>
          &uninitialized_object_info) = 0;

  virtual std::vector<AssociatedObjectIdentifier>
  pruneCandidateMatchesBasedOnGeometry(
      const OnlineBoundingBoxDetection &bounding_box,
      const std::vector<AssociatedObjectIdentifier> &candidate_matches,
      const std::shared_ptr<
          const vslam_types_refactor::ObjAndLowLevelFeaturePoseGraph<
              VisualFeatureFactorType>> &pose_graph,
      std::vector<UninitializedEllispoidInfo<ObjectAppearanceInfo>>
          &uninitialized_object_info) = 0;

  virtual double scoreCandidateMatch(
      const OnlineBoundingBoxDetection &bounding_box,
      const AssociatedObjectIdentifier &candidate,
      const SingleBbAppearanceInfo &bounding_box_appearance_info,
      const std::unordered_map<vslam_types_refactor::ObjectId,
                               ObjectAppearanceInfo> &object_appearance_info,
      const std::shared_ptr<
          const vslam_types_refactor::ObjAndLowLevelFeaturePoseGraph<
              VisualFeatureFactorType>> &pose_graph,
      const std::vector<UninitializedEllispoidInfo<ObjectAppearanceInfo>>
          &uninitialized_object_info) = 0;

  /**
   * For bounding boxes assigned to new objects, a bb earlier in the list should
   * have a lower (future) id than one later on.
   * @param match_candidates_with_scores
   * @param pose_graph
   * @param num_uninitialized_objects
   * @return
   */
  virtual std::vector<AssociatedObjectIdentifier> assignBoundingBoxes(
      const std::vector<
          std::vector<std::pair<AssociatedObjectIdentifier, double>>>
          &match_candidates_with_scores,
      const std::shared_ptr<
          vslam_types_refactor::ObjAndLowLevelFeaturePoseGraph<
              VisualFeatureFactorType>> &pose_graph,
      const size_t &num_uninitialized_objects) = 0;

  virtual void combineAppearanceInfosAndAddNewEllipsoids(
      const std::vector<SingleBbAppearanceInfo> single_bb_appearance_infos,
      const std::vector<AssociatedObjectIdentifier> bb_assignments,
      std::unordered_map<vslam_types_refactor::ObjectId,
                         ObjectAppearanceInfo> &object_appearance_info,
      std::vector<UninitializedEllispoidInfo<ObjectAppearanceInfo>>
          &uninitialized_object_info) {
    for (size_t i = 0; i < bb_assignments.size(); i++) {
      SingleBbAppearanceInfo single_bb_appearance_info =
          single_bb_appearance_infos[i];
      AssociatedObjectIdentifier assoc_identifier = bb_assignments[i];

      if (assoc_identifier.initialized_ellipsoid_) {
        updateAppearance(
            single_bb_appearance_info,
            object_appearance_info[assoc_identifier.object_id_]);
      } else {
        if (assoc_identifier.object_id_ >=
            uninitialized_object_info.size()) {
          uninitialized_object_info.emplace_back(
              createNewAppearanceInfo(single_bb_appearance_info));
        } else {
          updateAppearance(
              single_bb_appearance_info,
              uninitialized_object_info[assoc_identifier.object_id_]);
        }
      }
    }
  }

  virtual void updateAppearance(
      const SingleBbAppearanceInfo &single_bb_appearance_info,
      ObjectAppearanceInfo &combined_appearance_info) = 0;

  virtual ObjectAppearanceInfo createNewAppearanceInfo(
      const SingleBbAppearanceInfo &single_bb_appearance_info) = 0;

 private:
};

template <typename ObjectAppearanceInfo,
          typename VisualFeatureAppearanceInfo,
          typename VisualFeatureFactorType,
          typename SingleBbAppearanceInfo,
          typename NecessaryImageData,
          typename FrameDependentEllipsoidInitData>
class OnlineObjectSlamRunner {
 public:
  void addBoundingBoxObservations(
      const vslam_types_refactor::FrameId &frame_id,
      const vslam_types_refactor::CameraId &camera_id,
      const std::vector<OnlineBoundingBoxDetection> &bounding_boxes,
      const sensor_msgs::Image::ConstPtr &image) {
    vslam_types_refactor::CameraIntrinsicsMat<double> intrinsics;
    if (!pose_graph_->getIntrinsicsForCamera(camera_id, intrinsics)) {
      LOG(ERROR) << "No intrinsics found for camera with id " << camera_id
                 << "; dropping bounding boxes for camera.";
      return;
    }

    vslam_types_refactor::CameraExtrinsics<double> extrinsics;
    if (!pose_graph_->getExtrinsicsForCamera(camera_id, extrinsics)) {
      LOG(ERROR) << "No extrinsics found for camera with id " << camera_id
                 << "; dropping bounding boxes for camera.";
      return;
    }

    object_data_associator_.performBoundingBoxDataAssociationAndUpdatePoseGraph(
        frame_id,
        camera_id,
        intrinsics,
        extrinsics,
        bounding_boxes,
        image,
        object_appearance_info_,
        uninitialized_object_info_,
        pose_graph_);
  }

 private:
  std::shared_ptr<vslam_types_refactor::ObjAndLowLevelFeaturePoseGraph<
      VisualFeatureFactorType>>
      pose_graph_;

  ObjectDataAssociator<ObjectAppearanceInfo,
                       VisualFeatureFactorType,
                       SingleBbAppearanceInfo,
                       NecessaryImageData,
                       FrameDependentEllipsoidInitData>
      object_data_associator_;

  std::vector<UninitializedEllispoidInfo<ObjectAppearanceInfo>>
      uninitialized_object_info_;

  std::unordered_map<vslam_types_refactor::ObjectId, ObjectAppearanceInfo>
      object_appearance_info_;
};

}  // namespace online_runner

#endif  // UT_VSLAM_ONLINE_SLAM_RUNNER_H
