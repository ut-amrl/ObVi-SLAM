//
// Created by amanda on 6/21/22.
//

#ifndef UT_VSLAM_BOUNDING_BOX_FRONT_END_H
#define UT_VSLAM_BOUNDING_BOX_FRONT_END_H

#include <analysis/cumulative_timer_constants.h>
#include <analysis/cumulative_timer_factory.h>
#include <refactoring/optimization/object_pose_graph.h>
#include <refactoring/types/vslam_obj_opt_types_refactor.h>

namespace vslam_types_refactor {

struct UninitializedObjectFactor {
  FrameId frame_id_;
  CameraId camera_id_;

  BbCorners<double> bounding_box_corners_;

  double detection_confidence_;

  Covariance<double, 4> bounding_box_corners_covariance_;
};

template <typename ObjectAppearanceInfo, typename PendingObjectInfo>
struct UninitializedEllispoidInfo {
  ObjectAppearanceInfo appearance_info_;
  PendingObjectInfo pending_info_;
  std::string semantic_class_;

  std::vector<UninitializedObjectFactor> observation_factors_;
  FrameId min_frame_id_;
  FrameId max_frame_id_;
};

struct AssociatedObjectIdentifier {
  bool initialized_ellipsoid_;

  // If initialized object, this will be the object's id
  // If uninitialized, this will the index for the uninitialized object info
  // in the list of uninitialized objects
  ObjectId object_id_;
};

bool operator==(const AssociatedObjectIdentifier &assoc_1,
                const AssociatedObjectIdentifier &assoc_2) {
  return std::make_pair(assoc_1.initialized_ellipsoid_, assoc_1.object_id_) ==
         std::make_pair(assoc_2.initialized_ellipsoid_, assoc_2.object_id_);
}

std::size_t hash_value(const AssociatedObjectIdentifier &assoc) {
  boost::hash<std::pair<bool, ObjectId>> hasher;
  return hasher(std::make_pair(assoc.initialized_ellipsoid_, assoc.object_id_));
}

enum ObjectInitializationStatus {
  NOT_INITIALIZED,
  ENOUGH_VIEWS_FOR_MERGE,
  SUFFICIENT_VIEWS_FOR_NEW
};

template <typename VisualFeatureFactorType,
          typename ObjectAssociationInfo,
          typename PendingObjectInfo,
          typename RawBoundingBoxContextInfo,
          typename RefinedBoundingBoxContextInfo,
          typename SingleBbContextInfo,
          typename FrontEndObjMapData>
class AbstractBoundingBoxFrontEnd {
 public:
  AbstractBoundingBoxFrontEnd(
      const std::shared_ptr<
          vslam_types_refactor::ObjAndLowLevelFeaturePoseGraph<
              VisualFeatureFactorType>> &pose_graph)
      : pose_graph_(pose_graph) {}

  void addBoundingBoxObservations(
      const vslam_types_refactor::FrameId &frame_id,
      const vslam_types_refactor::CameraId &camera_id,
      const std::vector<RawBoundingBox> &bounding_boxes,
      const RawBoundingBoxContextInfo &bb_context) {
#ifdef RUN_TIMERS
    CumulativeFunctionTimer::Invocation invoc(
        CumulativeTimerFactory::getInstance()
            .getOrCreateFunctionTimer(kTimerNameBbFrontEndAddBbObs)
            .get());
#endif
    CHECK(initialized_)
        << "Bounding box front end not initialized properly. Make "
           "sure to call initializeWithLongTermMapFrontEndData "
           "before using the front end";
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

    LOG(INFO) << "Num bbs: " << bounding_boxes.size();
    std::vector<RawBoundingBox> filtered_bounding_boxes =
        filterBoundingBoxes(frame_id, camera_id, bounding_boxes);
    LOG(INFO) << "Num filtered bbs: " << filtered_bounding_boxes.size();
    RefinedBoundingBoxContextInfo refined_context =
        generateRefinedBbContextInfo(bb_context, frame_id, camera_id);
    setupBbAssociationRound(frame_id,
                            camera_id,
                            filtered_bounding_boxes,
                            bb_context,
                            refined_context);

    std::vector<SingleBbContextInfo> single_bb_appearance_infos;
    for (const RawBoundingBox &bb : filtered_bounding_boxes) {
      single_bb_appearance_infos.emplace_back(
          generateSingleBoundingBoxContextInfo(
              bb, frame_id, camera_id, refined_context));
    }

    // Get info from current box needed for association
    // Get associations
    std::vector<AssociatedObjectIdentifier> bounding_box_assignments;
    getBoundingBoxAssignments(frame_id,
                              camera_id,
                              filtered_bounding_boxes,
                              refined_context,
                              single_bb_appearance_infos,
                              bounding_box_assignments);

    // Add bbs and update the association info with the context for this bb
    for (size_t bb_index = 0; bb_index < bounding_box_assignments.size();
         bb_index++) {
      RawBoundingBox bb = filtered_bounding_boxes[bb_index];
      AssociatedObjectIdentifier associated_obj =
          bounding_box_assignments[bb_index];
      Covariance<double, 4> bb_cov = generateBoundingBoxCovariance(
          bb, frame_id, camera_id, refined_context);
      BbCorners<double> bb_corners =
          vslam_types_refactor::cornerLocationsPairToVector(
              bb.pixel_corner_locations_);
      SingleBbContextInfo single_bb_info = single_bb_appearance_infos[bb_index];
      if (associated_obj.initialized_ellipsoid_) {
        addObservationForObject(frame_id,
                                camera_id,
                                associated_obj.object_id_,
                                bb_corners,
                                bb_cov,
                                bb.detection_confidence_);
        mergeSingleBbContextIntoObjectAssociationInfo(
            bb,
            frame_id,
            camera_id,
            refined_context,
            single_bb_info,
            object_appearance_info_[associated_obj.object_id_]);
      } else {
        UninitializedObjectFactor uninitialized_object_factor;
        uninitialized_object_factor.frame_id_ = frame_id;
        uninitialized_object_factor.camera_id_ = camera_id;
        uninitialized_object_factor.bounding_box_corners_ = bb_corners;
        uninitialized_object_factor.bounding_box_corners_covariance_ = bb_cov;
        uninitialized_object_factor.detection_confidence_ =
            bb.detection_confidence_;

        if (associated_obj.object_id_ >= uninitialized_object_info_.size()) {
          uninitialized_object_info_.emplace_back(
              createObjectInfoFromSingleBb(bb.semantic_class_,
                                           frame_id,
                                           camera_id,
                                           uninitialized_object_factor,
                                           refined_context,
                                           single_bb_info));
        } else {
          uninitialized_object_info_[associated_obj.object_id_]
              .observation_factors_.emplace_back(uninitialized_object_factor);
          uninitialized_object_info_[associated_obj.object_id_].max_frame_id_ =
              std::max(frame_id,
                       uninitialized_object_info_[associated_obj.object_id_]
                           .max_frame_id_);
          uninitialized_object_info_[associated_obj.object_id_].min_frame_id_ =
              std::min(frame_id,
                       uninitialized_object_info_[associated_obj.object_id_]
                           .min_frame_id_);
          mergeSingleBbContextIntoObjectAssociationInfo(
              bb,
              frame_id,
              camera_id,
              refined_context,
              single_bb_info,
              uninitialized_object_info_[associated_obj.object_id_]
                  .appearance_info_,
              &(uninitialized_object_info_[associated_obj.object_id_]
                    .pending_info_));
        }
      }
    }

    setupInitialEstimateGeneration(bounding_box_assignments);
    // Initialize or refine estimates
    // For each object, add information and either initialize, delay
    // initialization (until we have more information, or refine estimate (if
    // already initialized, but with little data)
    std::vector<ObjectId> added_pending_object_indices;
    std::unordered_set<ObjectId> existing_associated_to_objects;

    // List of pending objects that can be merged
    // Id is the index in the unassociated objects list
    // Estimate is the initial estimate produced
    std::unordered_map<
        ObjectId,
        std::pair<ObjectInitializationStatus, EllipsoidState<double>>>
        mergable_objects_and_ests;
    // for each assignment
    for (size_t i = 0; i < bounding_box_assignments.size(); i++) {
      AssociatedObjectIdentifier bounding_box_assignment =
          bounding_box_assignments[i];
      RawBoundingBox bounding_box = filtered_bounding_boxes[i];

      // If matched to initialized
      if (bounding_box_assignment.initialized_ellipsoid_) {
        existing_associated_to_objects.insert(
            bounding_box_assignment.object_id_);

        // Consider refining
        // TODO try refining estimate by initializing with all new data and
        // seeing if the cost is lower than current estimate
      } else {
        UninitializedEllispoidInfo<ObjectAssociationInfo, PendingObjectInfo>
            uninitialized_bb_info =
                uninitialized_object_info_[bounding_box_assignment.object_id_];
        vslam_types_refactor::EllipsoidState<double> object_est;
        ObjectInitializationStatus initialization_status =
            tryInitializeEllipsoid(
                refined_context, uninitialized_bb_info, object_est);

        // See if there are enough views to merge or create
        if ((initialization_status == ENOUGH_VIEWS_FOR_MERGE) ||
            (initialization_status == SUFFICIENT_VIEWS_FOR_NEW)) {
          // If enough views to merge or enough views for full object add to
          // later processing list
          mergable_objects_and_ests[bounding_box_assignment.object_id_] =
              std::make_pair(initialization_status, object_est);
        }
      }
    }

    // Search for objects that can be merged and which should be created as
    // a new object
    std::vector<std::pair<ObjectId, ObjectId>>
        pending_and_initialized_objects_to_merge;
    std::vector<std::pair<ObjectId, EllipsoidState<double>>>
        pending_objects_to_add;
    searchForObjectMerges(mergable_objects_and_ests,
                          existing_associated_to_objects,
                          pending_and_initialized_objects_to_merge,
                          pending_objects_to_add);

    std::vector<ObjectId> merged_indices =
        mergePending(pending_and_initialized_objects_to_merge);
    added_pending_object_indices.insert(added_pending_object_indices.end(),
                                        merged_indices.begin(),
                                        merged_indices.end());
    for (const std::pair<ObjectId, EllipsoidState<double>> &add_obj :
         pending_objects_to_add) {
      ObjectId pending_obj_id = add_obj.first;

      UninitializedEllispoidInfo<ObjectAssociationInfo, PendingObjectInfo>
          uninitialized_bb_info = uninitialized_object_info_[pending_obj_id];
      EllipsoidEstimateNode estimate_node;
      estimate_node.updateEllipsoidParams(
          convertToRawEllipsoid(add_obj.second));
      ObjectId new_obj_id = pose_graph_->addNewEllipsoid(
          estimate_node, uninitialized_bb_info.semantic_class_);
      LOG(INFO) << "Creating new ellipsoid with id " << new_obj_id;
      object_appearance_info_[new_obj_id] =
          uninitialized_bb_info.appearance_info_;
      updateAppearanceInfoWithObjectIdAssignmentAndInitialization(
          new_obj_id, add_obj.second, object_appearance_info_[new_obj_id]);
      for (const UninitializedObjectFactor &bb_factor :
           uninitialized_bb_info.observation_factors_) {
        addObservationForObject(bb_factor.frame_id_,
                                bb_factor.camera_id_,
                                new_obj_id,
                                bb_factor.bounding_box_corners_,
                                bb_factor.bounding_box_corners_covariance_,
                                bb_factor.detection_confidence_);
      }
      added_pending_object_indices.emplace_back(pending_obj_id);
    }
    // Sort the objects that just have been added to the pose graph in
    // descending order and delete them from the uninitialized objects list
    // Descending order needed so the indices to delete don't change as
    // they're removed
    std::sort(added_pending_object_indices.begin(),
              added_pending_object_indices.end(),
              std::greater<size_t>());
    for (const size_t &pending_init_object_index :
         added_pending_object_indices) {
      uninitialized_object_info_.erase(uninitialized_object_info_.begin() +
                                       pending_init_object_index);
    }

    std::vector<ObjectId> merged_existing_pending_object_indices =
        mergeExistingPendingObjects();
    std::sort(merged_existing_pending_object_indices.begin(),
              merged_existing_pending_object_indices.end(),
              std::greater<size_t>());
    for (const size_t &pending_init_object_index :
         merged_existing_pending_object_indices) {
      uninitialized_object_info_.erase(uninitialized_object_info_.begin() +
                                       pending_init_object_index);
    }

    cleanupBbAssociationRound(frame_id, camera_id);
    LOG(INFO) << "Done with bounding box data association for frame "
              << frame_id << " and camera " << camera_id;
  }

  /**
   * Get the data from the bounding box front end that should be saved with the
   * map.
   *
   * @param map_data[out]   Populate this variable with data to be saved with
   *                        the map.
   *
   * @return True if the map data retrieval succeeded, false otherwise.
   */
  virtual bool getFrontEndObjMapData(
      std::unordered_map<ObjectId, FrontEndObjMapData> &map_data) {
    for (const auto &obj_assoc_info : object_appearance_info_) {
      map_data[obj_assoc_info.first] = mapDataFromObjAssociationInfo(
          obj_assoc_info.first, obj_assoc_info.second);
    }
    return true;
  }

  /**
   * NOTE FOR SUBCLASSES: This function should set initialized_ to true.
   * @param map_data
   * @return
   */
  virtual bool initializeWithLongTermMapFrontEndData(
      const std::unordered_map<ObjectId, FrontEndObjMapData> &map_data) {
    for (const auto &obj_map_data : map_data) {
      object_appearance_info_[obj_map_data.first] =
          objAssocInfoFromMapData(obj_map_data.first, obj_map_data.second);
    }
    initialized_ = true;
    return true;
  }

  /**
   * Merge objects.
   *
   * @param objects_to_merge Map of object id to merge into to objects to merge
   * into that object. If there is not a key for the object and it isn't
   * included in one of the values, then it should be left alone
   *
   * @return True if the merge object succeeded, false otherwise.
   */
  virtual bool mergeObjects(
      const std::unordered_map<ObjectId, std::unordered_set<ObjectId>>
          &objects_to_merge) {
    std::unordered_set<ObjectId> objects_to_remove;
    for (const auto &merge_group : objects_to_merge) {
      for (const ObjectId &merge_obj : merge_group.second) {
        if (objects_to_remove.find(merge_obj) != objects_to_remove.end()) {
          LOG(ERROR)
              << "Object " << merge_obj
              << " expected to be merged into multiple objects (second was "
              << merge_group.first
              << "). This will cause unexpected behavior. ";
        }
        if (objects_to_merge.find(merge_obj) != objects_to_merge.end()) {
          LOG(ERROR) << "Object " << merge_obj
                     << " is supposed to be merged into " << merge_group.first
                     << " and have other objects merged into it. This will "
                        "cause unexpected behavior. ";
        }
        if (object_appearance_info_.find(merge_obj) ==
            object_appearance_info_.end()) {
          LOG(ERROR) << "Could not find appearance info for object to merge "
                     << merge_obj << "; not merging";
        } else if (object_appearance_info_.find(merge_group.first) ==
                   object_appearance_info_.end()) {
          LOG(ERROR)
              << "Could not find appearance info for object to merge into ("
              << merge_group.first
              << ") ; using info for object to merge into it (" << merge_obj
              << ")";
          object_appearance_info_[merge_group.first] =
              object_appearance_info_.at(merge_obj);
        } else {
          // TODO make sure this actually updates the reference
          mergeObjectAssociationInfo(
              object_appearance_info_.at(merge_obj),
              object_appearance_info_[merge_group.first]);
        }
      }
      objects_to_remove.insert(merge_group.second.begin(),
                               merge_group.second.end());
    }

    for (const ObjectId &object_to_remove : objects_to_remove) {
      object_appearance_info_.erase(object_to_remove);
    }

    return true;
  }

 protected:
  std::shared_ptr<vslam_types_refactor::ObjAndLowLevelFeaturePoseGraph<
      VisualFeatureFactorType>>
      pose_graph_;
  std::vector<
      UninitializedEllispoidInfo<ObjectAssociationInfo, PendingObjectInfo>>
      uninitialized_object_info_;

  std::unordered_map<vslam_types_refactor::ObjectId, ObjectAssociationInfo>
      object_appearance_info_;

  bool initialized_ = false;

  virtual ObjectAssociationInfo objAssocInfoFromMapData(
      const ObjectId &obj_id,
      const FrontEndObjMapData &front_end_data_for_obj) = 0;

  virtual FrontEndObjMapData mapDataFromObjAssociationInfo(
      const ObjectId &obj_id,
      const ObjectAssociationInfo &obj_association_info) = 0;

  virtual std::vector<ObjectId> mergePending(
      const std::vector<std::pair<ObjectId, ObjectId>>
          &pending_and_initialized_objects_to_merge) {
    std::vector<ObjectId> merged_indices;
    for (const std::pair<ObjectId, ObjectId> &merge_obj :
         pending_and_initialized_objects_to_merge) {
      ObjectId pending_obj_id = merge_obj.first;
      ObjectId obj_to_merge_with_id = merge_obj.second;

      UninitializedEllispoidInfo<ObjectAssociationInfo, PendingObjectInfo>
          uninitialized_bb_info = uninitialized_object_info_[pending_obj_id];

      // Merge observed bounding boxes from the pending one with the existing
      // object it should be merged with
      for (const UninitializedObjectFactor &bb_factor :
           uninitialized_bb_info.observation_factors_) {
        addObservationForObject(bb_factor.frame_id_,
                                bb_factor.camera_id_,
                                obj_to_merge_with_id,
                                bb_factor.bounding_box_corners_,
                                bb_factor.bounding_box_corners_covariance_,
                                bb_factor.detection_confidence_);
      }
      // Merge the association info for the objects
      mergeObjectAssociationInfo(
          uninitialized_bb_info.appearance_info_,
          object_appearance_info_.at(obj_to_merge_with_id));
      merged_indices.emplace_back(pending_obj_id);
    }
    return merged_indices;
  }

  virtual std::vector<ObjectId> mergeExistingPendingObjects() = 0;

  virtual void updateAppearanceInfoWithObjectIdAssignmentAndInitialization(
      const ObjectId &obj_id,
      const EllipsoidState<double> &est,
      ObjectAssociationInfo &info_to_update) = 0;

  virtual SingleBbContextInfo generateSingleBoundingBoxContextInfo(
      const RawBoundingBox &bb,
      const FrameId &frame_id,
      const CameraId &camera_id,
      const RefinedBoundingBoxContextInfo &refined_context) = 0;

  virtual UninitializedEllispoidInfo<ObjectAssociationInfo, PendingObjectInfo>
  createObjectInfoFromSingleBb(
      const std::string &semantic_class,
      const FrameId &frame_id,
      const CameraId &camera_id,
      const UninitializedObjectFactor &uninitialized_factor,
      const RefinedBoundingBoxContextInfo &refined_context,
      const SingleBbContextInfo &single_bb_context_info) = 0;

  virtual void mergeSingleBbContextIntoObjectAssociationInfo(
      const RawBoundingBox &raw_bb,
      const FrameId &frame_id,
      const CameraId &camera_id,
      const RefinedBoundingBoxContextInfo &refined_bb_context_info,
      const SingleBbContextInfo &single_bb_context_info,
      ObjectAssociationInfo &association_info_to_update,
      PendingObjectInfo *pending_obj_info = nullptr) = 0;

  virtual void mergeObjectAssociationInfo(
      const ObjectAssociationInfo &association_info_to_merge_in,
      ObjectAssociationInfo &association_info_to_update) = 0;

  virtual Covariance<double, 4> generateBoundingBoxCovariance(
      const RawBoundingBox &raw_bb,
      const FrameId &frame_id,
      const CameraId &camera_id,
      const RefinedBoundingBoxContextInfo &refined_bb_context) = 0;

  virtual void getBoundingBoxAssignments(
      const vslam_types_refactor::FrameId &frame_id,
      const vslam_types_refactor::CameraId &camera_id,
      const std::vector<RawBoundingBox> &bounding_boxes,
      const RefinedBoundingBoxContextInfo &refined_bb_context,
      const std::vector<SingleBbContextInfo> &indiv_bb_contexts,
      std::vector<AssociatedObjectIdentifier> &bounding_box_assignments) = 0;

  virtual std::vector<RawBoundingBox> filterBoundingBoxes(
      const FrameId &frame_id,
      const CameraId &camera_id,
      const std::vector<RawBoundingBox> &original_bounding_boxes) = 0;

  virtual RefinedBoundingBoxContextInfo generateRefinedBbContextInfo(
      const RawBoundingBoxContextInfo &bb_context,
      const FrameId &frame_id,
      const CameraId &camera_id) = 0;

  virtual void setupBbAssociationRound(
      const vslam_types_refactor::FrameId &frame_id,
      const vslam_types_refactor::CameraId &camera_id,
      const std::vector<RawBoundingBox> &bounding_boxes,
      const RawBoundingBoxContextInfo &raw_bb_context,
      const RefinedBoundingBoxContextInfo &refined_bb_context) = 0;

  virtual void cleanupBbAssociationRound(
      const vslam_types_refactor::FrameId &frame_id,
      const vslam_types_refactor::CameraId &camera_id) = 0;

  virtual void setupInitialEstimateGeneration(
      const std::vector<AssociatedObjectIdentifier>
          &bounding_box_assignments) = 0;

  virtual ObjectInitializationStatus tryInitializeEllipsoid(
      const RefinedBoundingBoxContextInfo &refined_bb_context,
      const UninitializedEllispoidInfo<ObjectAssociationInfo, PendingObjectInfo>
          &uninitialized_bb_info,
      vslam_types_refactor::EllipsoidState<double> &ellipsoid_est) = 0;

  virtual void searchForObjectMerges(
      const std::unordered_map<
          ObjectId,
          std::pair<ObjectInitializationStatus, EllipsoidState<double>>>
          &mergable_objects_and_ests,
      const std::unordered_set<ObjectId> &existing_associated_to_objects,
      std::vector<std::pair<ObjectId, ObjectId>>
          &pending_and_initialized_objects_to_merge,
      std::vector<std::pair<ObjectId, EllipsoidState<double>>>
          &pending_objects_to_add) = 0;

  virtual void addObservationForObject(const FrameId &frame_id,
                                       const CameraId &camera_id,
                                       const ObjectId &object_id,
                                       const BbCorners<double> &bb_corners,
                                       const Covariance<double, 4> &bb_cov,
                                       const double &detection_confidence) {
    ObjectObservationFactor factor;
    factor.frame_id_ = frame_id;
    factor.camera_id_ = camera_id;
    factor.object_id_ = object_id;
    factor.bounding_box_corners_ = bb_corners;
    factor.bounding_box_corners_covariance_ = bb_cov;
    factor.detection_confidence_ = detection_confidence;
    pose_graph_->addObjectObservation(factor);
  }

 private:
};

struct KnownAssociationObjAssociationInfo {
  ObjectId input_obj_id_;
  std::optional<ObjectId> pg_obj_id_;
};

template <typename VisualFeatureFactorType,
          typename RawBoundingBoxContextInfo,
          typename RefinedBoundingBoxContextInfo>
class KnownAssociationsOptionalEllipsoidEstBoundingBoxFrontEnd
    : public AbstractBoundingBoxFrontEnd<VisualFeatureFactorType,
                                         KnownAssociationObjAssociationInfo,
                                         util::EmptyStruct,
                                         RawBoundingBoxContextInfo,
                                         RefinedBoundingBoxContextInfo,
                                         util::EmptyStruct,
                                         KnownAssociationObjAssociationInfo> {
 public:
  KnownAssociationsOptionalEllipsoidEstBoundingBoxFrontEnd(
      const std::shared_ptr<
          vslam_types_refactor::ObjAndLowLevelFeaturePoseGraph<
              VisualFeatureFactorType>> &pose_graph,
      const std::unordered_map<ObjectId,
                               vslam_types_refactor::RawEllipsoid<double>>
          &optional_initial_estimates,
      const std::function<bool(
          const std::shared_ptr<
              vslam_types_refactor::ObjAndLowLevelFeaturePoseGraph<
                  VisualFeatureFactorType>> &,
          const RefinedBoundingBoxContextInfo &,
          const UninitializedEllispoidInfo<KnownAssociationObjAssociationInfo,
                                           util::EmptyStruct> &,
          EllipsoidState<double> &)> &initializer_function,
      const std::function<Covariance<double, 4>(
          const RawBoundingBox &,
          const FrameId &,
          const CameraId &,
          const RefinedBoundingBoxContextInfo &)> &covariance_generator,
      const std::function<RefinedBoundingBoxContextInfo(
          const RawBoundingBoxContextInfo &, const FrameId &, CameraId &)>
          &bb_context_refiner)
      : AbstractBoundingBoxFrontEnd<
            VisualFeatureFactorType,
            KnownAssociationObjAssociationInfo,
            util::EmptyStruct,
            RawBoundingBoxContextInfo,
            RefinedBoundingBoxContextInfo,
            util::EmptyStruct,
            std::unordered_map<vslam_types_refactor::ObjectId,
                               KnownAssociationObjAssociationInfo>>(pose_graph),
        optional_initial_estimates_(optional_initial_estimates),
        initializer_function_(initializer_function),
        covariance_generator_(covariance_generator),
        bb_context_refiner_(bb_context_refiner) {}

 protected:
  virtual KnownAssociationObjAssociationInfo objAssocInfoFromMapData(
      const ObjectId &obj_id,
      const KnownAssociationObjAssociationInfo &front_end_data_for_obj)
      override {
    return front_end_data_for_obj;
  }

  virtual KnownAssociationObjAssociationInfo mapDataFromObjAssociationInfo(
      const ObjectId &obj_id,
      const KnownAssociationObjAssociationInfo &obj_association_info) override {
    return obj_association_info;
  }

  virtual std::vector<ObjectId> mergeExistingPendingObjects() override {
    return {};
  }

  virtual std::vector<RawBoundingBox> filterBoundingBoxes(
      const FrameId &frame_id,
      const CameraId &camera_id,
      const std::vector<RawBoundingBox> &original_bounding_boxes) override {
    return original_bounding_boxes;
  }

  virtual void updateAppearanceInfoWithObjectIdAssignmentAndInitialization(
      const ObjectId &obj_id,
      const EllipsoidState<double> &est,
      KnownAssociationObjAssociationInfo &info_to_update) override {
    info_to_update.pg_obj_id_ = obj_id;
    input_to_pg_object_id_map_[info_to_update.input_obj_id_] = obj_id;
    pg_to_input_object_id_map_[obj_id] = info_to_update.input_obj_id_;
  }

  virtual util::EmptyStruct generateSingleBoundingBoxContextInfo(
      const RawBoundingBox &bb,
      const FrameId &frame_id,
      const CameraId &camera_id,
      const RefinedBoundingBoxContextInfo &refined_context) {
    return util::EmptyStruct();
  }

  virtual UninitializedEllispoidInfo<KnownAssociationObjAssociationInfo,
                                     util::EmptyStruct>
  createObjectInfoFromSingleBb(
      const std::string &semantic_class,
      const FrameId &frame_id,
      const CameraId &camera_id,
      const UninitializedObjectFactor &uninitialized_factor,
      const RefinedBoundingBoxContextInfo &refined_context,
      const util::EmptyStruct &single_bb_context_info) {
    UninitializedEllispoidInfo<KnownAssociationObjAssociationInfo,
                               util::EmptyStruct>
        uninitialized_ellipsoid_info;
    uninitialized_ellipsoid_info.semantic_class_ = semantic_class;
    uninitialized_ellipsoid_info.observation_factors_.emplace_back(
        uninitialized_factor);
    std::unordered_map<CameraId, util::BoostHashMap<RawBoundingBox, ObjectId>>
        associations_for_frame = associations_.at(frame_id);
    util::BoostHashMap<RawBoundingBox, ObjectId>
        associations_for_frame_and_cam = associations_for_frame.at(camera_id);
    RawBoundingBox raw_bb;
    raw_bb.semantic_class_ = semantic_class;
    raw_bb.pixel_corner_locations_ =
        cornerLocationsVectorToPair(uninitialized_factor.bounding_box_corners_);
    uninitialized_ellipsoid_info.appearance_info_.input_obj_id_ =
        associations_for_frame_and_cam.at(raw_bb);
    uninitialized_ellipsoid_info.min_frame_id_ = frame_id;
    uninitialized_ellipsoid_info.max_frame_id_ = frame_id;
    return uninitialized_ellipsoid_info;
  }

  virtual void mergeSingleBbContextIntoObjectAssociationInfo(
      const RawBoundingBox &raw_bb,
      const FrameId &frame_id,
      const CameraId &camera_id,
      const RefinedBoundingBoxContextInfo &refined_bb_context_info,
      const util::EmptyStruct &single_bb_context_info,
      KnownAssociationObjAssociationInfo &association_info_to_update,
      util::EmptyStruct *pending_obj_info = nullptr) override {}

  virtual void mergeObjectAssociationInfo(
      const KnownAssociationObjAssociationInfo &association_info_to_merge_in,
      KnownAssociationObjAssociationInfo &association_info_to_update)
      override{};

  virtual Covariance<double, 4> generateBoundingBoxCovariance(
      const RawBoundingBox &raw_bb,
      const FrameId &frame_id,
      const CameraId &camera_id,
      const RefinedBoundingBoxContextInfo &refined_bb_context) {
    return covariance_generator_(
        raw_bb, frame_id, camera_id, refined_bb_context);
  };

  virtual void getBoundingBoxAssignments(
      const vslam_types_refactor::FrameId &frame_id,
      const vslam_types_refactor::CameraId &camera_id,
      const std::vector<RawBoundingBox> &bounding_boxes,
      const RefinedBoundingBoxContextInfo &refined_bb_context,
      const std::vector<util::EmptyStruct> &indiv_bb_contexts,
      std::vector<AssociatedObjectIdentifier> &bounding_box_assignments) {
    ObjectId next_new_obj_temp_id =
        KnownAssociationsOptionalEllipsoidEstBoundingBoxFrontEnd::
            uninitialized_object_info_.size();
    for (size_t bb_index = 0; bb_index < bounding_boxes.size(); bb_index++) {
      RawBoundingBox bb = bounding_boxes[bb_index];
      AssociatedObjectIdentifier identifier;
      ObjectId input_obj_id =
          ((associations_.at(frame_id)).at(camera_id)).at(bb);
      if (input_to_pg_object_id_map_.find(input_obj_id) !=
          input_to_pg_object_id_map_.end()) {
        identifier.initialized_ellipsoid_ = true;
        identifier.object_id_ = input_to_pg_object_id_map_.at(input_obj_id);
      } else {
        identifier.initialized_ellipsoid_ = false;
        if (curr_frame_input_obj_id_to_uninit_index_.find(input_obj_id) !=
            curr_frame_input_obj_id_to_uninit_index_.end()) {
          identifier.object_id_ =
              curr_frame_input_obj_id_to_uninit_index_.at(input_obj_id);
        } else {
          identifier.object_id_ = next_new_obj_temp_id;
          next_new_obj_temp_id++;
        }
      }
      bounding_box_assignments.emplace_back(identifier);
    }
  }

  virtual RefinedBoundingBoxContextInfo generateRefinedBbContextInfo(
      const RawBoundingBoxContextInfo &bb_context,
      const FrameId &frame_id,
      const CameraId &camera_id) {
    return bb_context_refiner_(bb_context, frame_id, camera_id);
  }

  virtual void setupBbAssociationRound(
      const vslam_types_refactor::FrameId &frame_id,
      const vslam_types_refactor::CameraId &camera_id,
      const std::vector<RawBoundingBox> &bounding_boxes,
      const RawBoundingBoxContextInfo &raw_bb_context,
      const RefinedBoundingBoxContextInfo &refined_bb_context) {
    for (ObjectId uninit_obj_info_idx = 0;
         uninit_obj_info_idx <
         KnownAssociationsOptionalEllipsoidEstBoundingBoxFrontEnd::
             uninitialized_object_info_.size();
         uninit_obj_info_idx++) {
      UninitializedEllispoidInfo<KnownAssociationObjAssociationInfo,
                                 util::EmptyStruct>
          obj_info = KnownAssociationsOptionalEllipsoidEstBoundingBoxFrontEnd::
              uninitialized_object_info_[uninit_obj_info_idx];
      curr_frame_uninit_index_to_input_obj_id_[uninit_obj_info_idx] =
          obj_info.appearance_info_.input_obj_id_;
      curr_frame_input_obj_id_to_uninit_index_[obj_info.appearance_info_
                                                   .input_obj_id_] =
          uninit_obj_info_idx;
    }
  }

  virtual void cleanupBbAssociationRound(
      const vslam_types_refactor::FrameId &frame_id,
      const vslam_types_refactor::CameraId &camera_id) {
    curr_frame_input_obj_id_to_uninit_index_.clear();
    curr_frame_uninit_index_to_input_obj_id_.clear();
  }

  virtual void setupInitialEstimateGeneration(
      const std::vector<AssociatedObjectIdentifier> &bounding_box_assignments)
      override {}

  virtual ObjectInitializationStatus tryInitializeEllipsoid(
      const RefinedBoundingBoxContextInfo &refined_bb_context,
      const UninitializedEllispoidInfo<KnownAssociationObjAssociationInfo,
                                       util::EmptyStruct>
          &uninitialized_bb_info,
      vslam_types_refactor::EllipsoidState<double> &ellipsoid_est) override {
    if (optional_initial_estimates_.find(
            uninitialized_bb_info.appearance_info_.input_obj_id_) !=
        optional_initial_estimates_.end()) {
      ellipsoid_est = convertToEllipsoidState(optional_initial_estimates_.at(
          uninitialized_bb_info.appearance_info_.input_obj_id_));
      return SUFFICIENT_VIEWS_FOR_NEW;
    }
    bool init_result = initializer_function_(
        KnownAssociationsOptionalEllipsoidEstBoundingBoxFrontEnd::pose_graph_,
        refined_bb_context,
        uninitialized_bb_info,
        ellipsoid_est);
    return init_result ? SUFFICIENT_VIEWS_FOR_NEW : NOT_INITIALIZED;
  }

  virtual bool mergeObjects(
      const std::unordered_map<ObjectId, std::unordered_set<ObjectId>>
          &objects_to_merge) override {
    LOG(WARNING) << "Merge has no effect in known data association front-end "
                    "since there shouldn't be any merges";
    return true;
  }

 private:
  std::unordered_map<ObjectId, vslam_types_refactor::RawEllipsoid<double>>
      optional_initial_estimates_;

  std::unordered_map<ObjectId, ObjectId> input_to_pg_object_id_map_;
  std::unordered_map<ObjectId, ObjectId> pg_to_input_object_id_map_;

  std::unordered_map<
      FrameId,
      std::unordered_map<CameraId,
                         util::BoostHashMap<RawBoundingBox, ObjectId>>>
      associations_;

  std::unordered_map<ObjectId, ObjectId>
      curr_frame_uninit_index_to_input_obj_id_;
  std::unordered_map<ObjectId, ObjectId>
      curr_frame_input_obj_id_to_uninit_index_;

  std::function<bool(
      const std::shared_ptr<
          vslam_types_refactor::ObjAndLowLevelFeaturePoseGraph<
              VisualFeatureFactorType>> &,
      const RefinedBoundingBoxContextInfo &,
      const UninitializedEllispoidInfo<KnownAssociationObjAssociationInfo,
                                       util::EmptyStruct> &,
      EllipsoidState<double> &)>
      initializer_function_;

  std::function<Covariance<double, 4>(const RawBoundingBox &,
                                      const FrameId &,
                                      const CameraId &,
                                      const RefinedBoundingBoxContextInfo &)>
      covariance_generator_;

  std::function<RefinedBoundingBoxContextInfo(
      const RawBoundingBoxContextInfo &, const FrameId &, CameraId &)>
      bb_context_refiner_;
};

template <typename VisualFeatureFactorType,
          typename ObjectAssociationInfo,
          typename PendingObjectInfo,
          typename RawBoundingBoxContextInfo,
          typename RefinedBoundingBoxContextInfo,
          typename SingleBbContextInfo,
          typename FrontEndObjMapData,
          typename CandidateInfo>
class AbstractUnknownDataAssociationBbFrontEnd
    : public AbstractBoundingBoxFrontEnd<VisualFeatureFactorType,
                                         ObjectAssociationInfo,
                                         PendingObjectInfo,
                                         RawBoundingBoxContextInfo,
                                         RefinedBoundingBoxContextInfo,
                                         SingleBbContextInfo,
                                         FrontEndObjMapData> {
 public:
  AbstractUnknownDataAssociationBbFrontEnd(
      const std::shared_ptr<
          vslam_types_refactor::ObjAndLowLevelFeaturePoseGraph<
              VisualFeatureFactorType>> &pose_graph)
      : AbstractBoundingBoxFrontEnd<VisualFeatureFactorType,
                                    ObjectAssociationInfo,
                                    PendingObjectInfo,
                                    RawBoundingBoxContextInfo,
                                    RefinedBoundingBoxContextInfo,
                                    SingleBbContextInfo,
                                    FrontEndObjMapData>(pose_graph) {}

 protected:
  virtual void updateAppearanceInfoWithObjectIdAssignmentAndInitialization(
      const ObjectId &obj_id,
      const EllipsoidState<double> &est,
      ObjectAssociationInfo &info_to_update) = 0;

  virtual SingleBbContextInfo generateSingleBoundingBoxContextInfo(
      const RawBoundingBox &bb,
      const FrameId &frame_id,
      const CameraId &camera_id,
      const RefinedBoundingBoxContextInfo &refined_context) = 0;

  virtual UninitializedEllispoidInfo<ObjectAssociationInfo, PendingObjectInfo>
  createObjectInfoFromSingleBb(
      const std::string &semantic_class,
      const FrameId &frame_id,
      const CameraId &camera_id,
      const UninitializedObjectFactor &uninitialized_factor,
      const RefinedBoundingBoxContextInfo &refined_context,
      const SingleBbContextInfo &single_bb_context_info) = 0;

  virtual void mergeSingleBbContextIntoObjectAssociationInfo(
      const RawBoundingBox &raw_bb,
      const FrameId &frame_id,
      const CameraId &camera_id,
      const RefinedBoundingBoxContextInfo &refined_bb_context_info,
      const SingleBbContextInfo &single_bb_context_info,
      ObjectAssociationInfo &association_info_to_update,
      PendingObjectInfo *pending_obj_info = nullptr) = 0;

  virtual Covariance<double, 4> generateBoundingBoxCovariance(
      const RawBoundingBox &raw_bb,
      const FrameId &frame_id,
      const CameraId &camera_id,
      const RefinedBoundingBoxContextInfo &refined_bb_context) = 0;

  virtual void getBoundingBoxAssignments(
      const vslam_types_refactor::FrameId &frame_id,
      const vslam_types_refactor::CameraId &camera_id,
      const std::vector<RawBoundingBox> &bounding_boxes,
      const RefinedBoundingBoxContextInfo &refined_bb_context,
      const std::vector<SingleBbContextInfo> &indiv_bb_contexts,
      std::vector<AssociatedObjectIdentifier> &bounding_box_assignments) {
    std::vector<std::vector<std::pair<AssociatedObjectIdentifier, double>>>
        match_candidates_with_scores;
    for (size_t i = 0; i < bounding_boxes.size(); i++) {
      std::vector<std::pair<AssociatedObjectIdentifier, double>>
          candidates_with_scores_for_bb;
      RawBoundingBox bb = bounding_boxes[i];
      SingleBbContextInfo bb_context = indiv_bb_contexts[i];

      // For each bounding box, identify candidates
      std::vector<std::pair<AssociatedObjectIdentifier, CandidateInfo>>
          candidates =
              identifyCandidateMatches(frame_id, camera_id, bb, bb_context);

      // For each bounding box, prune candidates using geometric checks
      candidates = pruneCandidateMatches(
          frame_id, camera_id, bb, candidates, bb_context);

      // For each bounding box, calculate data association scores for each
      // candidate
      for (const std::pair<AssociatedObjectIdentifier, CandidateInfo>
               &candidate : candidates) {
        double score =
            scoreCandidateMatch(frame_id, camera_id, bb, candidate, bb_context);
        // If the score is negative infinity, this should not be matched even
        // if there are no better matches (better to make new object)
        if (score != (-1 * std::numeric_limits<double>::infinity())) {
          candidates_with_scores_for_bb.emplace_back(
              std::make_pair(candidate.first, score));
        }
      }
      match_candidates_with_scores.emplace_back(candidates_with_scores_for_bb);
    }

    // Assign bounding boxes to objects/uninitialized objects based on
    // scores/determine which need new bbs
    bounding_box_assignments =
        assignBoundingBoxes(match_candidates_with_scores);
  }

  virtual std::vector<std::pair<AssociatedObjectIdentifier, CandidateInfo>>
  identifyCandidateMatches(const FrameId &frame_id,
                           const CameraId &camera_id,
                           const RawBoundingBox &bounding_box,
                           const SingleBbContextInfo &bb_context) = 0;

  virtual std::vector<std::pair<AssociatedObjectIdentifier, CandidateInfo>>
  pruneCandidateMatches(
      const FrameId &frame_id,
      const CameraId &camera_id,
      const RawBoundingBox &bounding_box,
      const std::vector<std::pair<AssociatedObjectIdentifier, CandidateInfo>>
          &candidate_matches,
      const SingleBbContextInfo &bb_context) = 0;

  virtual double scoreCandidateMatch(
      const FrameId &frame_id,
      const CameraId &camera_id,
      const RawBoundingBox &bounding_box,
      const std::pair<AssociatedObjectIdentifier, CandidateInfo> &candidate,
      const SingleBbContextInfo &bounding_box_appearance_info) = 0;

  virtual std::vector<AssociatedObjectIdentifier> assignBoundingBoxes(
      const std::vector<
          std::vector<std::pair<AssociatedObjectIdentifier, double>>>
          &match_candidates_with_scores) = 0;

  virtual RefinedBoundingBoxContextInfo generateRefinedBbContextInfo(
      const RawBoundingBoxContextInfo &bb_context,
      const FrameId &frame_id,
      const CameraId &camera_id) = 0;

  virtual void setupBbAssociationRound(
      const vslam_types_refactor::FrameId &frame_id,
      const vslam_types_refactor::CameraId &camera_id,
      const std::vector<RawBoundingBox> &bounding_boxes,
      const RawBoundingBoxContextInfo &raw_bb_context,
      const RefinedBoundingBoxContextInfo &refined_bb_context) = 0;

  virtual void cleanupBbAssociationRound(
      const vslam_types_refactor::FrameId &frame_id,
      const vslam_types_refactor::CameraId &camera_id) = 0;

  virtual void setupInitialEstimateGeneration(
      const std::vector<AssociatedObjectIdentifier>
          &bounding_box_assignments) = 0;

  virtual ObjectInitializationStatus tryInitializeEllipsoid(
      const RefinedBoundingBoxContextInfo &refined_bb_context,
      const UninitializedEllispoidInfo<ObjectAssociationInfo, PendingObjectInfo>
          &uninitialized_bb_info,
      vslam_types_refactor::EllipsoidState<double> &ellipsoid_est) = 0;

  virtual void searchForObjectMerges(
      const std::unordered_map<
          ObjectId,
          std::pair<ObjectInitializationStatus, EllipsoidState<double>>>
          &mergable_objects_and_ests,
      const std::unordered_set<ObjectId> &existing_associated_to_objects,
      std::vector<std::pair<ObjectId, ObjectId>>
          &pending_and_initialized_objects_to_merge,
      std::vector<std::pair<ObjectId, EllipsoidState<double>>>
          &pending_objects_to_add) = 0;

 private:
};

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_BOUNDING_BOX_FRONT_END_H
