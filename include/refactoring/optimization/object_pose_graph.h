//
// Created by amanda on 6/9/22.
//

#ifndef UT_VSLAM_OBJECT_POSE_GRAPH_H
#define UT_VSLAM_OBJECT_POSE_GRAPH_H

#include <base_lib/basic_utils.h>
#include <glog/logging.h>
#include <refactoring/optimization/low_level_feature_pose_graph.h>
#include <refactoring/types/vslam_basic_types_refactor.h>
#include <refactoring/types/vslam_obj_opt_types_refactor.h>

namespace vslam_types_refactor {

static const FactorType kObjectObservationFactorTypeId = 2;
static const FactorType kShapeDimPriorFactorTypeId = 3;
static const FactorType kLongTermMapFactorTypeId = 4;

struct EllipsoidEstimateNode {
  RawEllipsoidPtr<double> ellipsoid_;

  EllipsoidEstimateNode()
      : ellipsoid_(std::make_shared<RawEllipsoid<double>>()) {}

#ifdef CONSTRAIN_ELLIPSOID_ORIENTATION
  EllipsoidEstimateNode(const RawPose3dYawOnly<double> &pose,
#else
  EllipsoidEstimateNode(const RawPose3d<double> &pose,
#endif
                        const ObjectDim<double> &dimensions)
      : EllipsoidEstimateNode() {
    updateEllipsoidParams(pose, dimensions);
  }

  EllipsoidEstimateNode(const RawEllipsoid<double> &raw_ellipsoid_data)
      : ellipsoid_(std::make_shared<RawEllipsoid<double>>(raw_ellipsoid_data)) {
  }

#ifdef CONSTRAIN_ELLIPSOID_ORIENTATION
  void updatePoseData(const RawPose3dYawOnly<double> &pose) {
#else
  void updatePoseData(const RawPose3d<double> &pose) {
#endif
    // TODO verify that this is changing the contents of the pointer
    //  (with a copy) and not the pointer
    ellipsoid_->topRows(kEllipsoidPoseParameterizationSize) = pose;
  }

  void updateDimensionData(const ObjectDim<double> &dim) {
    // TODO verify that this is changing the contents of the pointer (with a
    //  copy) and not the pointer
    ellipsoid_->bottomRows(3) = dim;
  }

#ifdef CONSTRAIN_ELLIPSOID_ORIENTATION
  void updateEllipsoidParams(const RawPose3dYawOnly<double> &pose,
#else
  void updateEllipsoidParams(const RawPose3d<double> &pose,
#endif
                             const ObjectDim<double> &dim) {
    updatePoseData(pose);
    updateDimensionData(dim);
  }

  void updateEllipsoidParams(const RawEllipsoid<double> &ellipsoid) {
    ellipsoid_->topRows(kEllipsoidParamterizationSize) = ellipsoid;
  }

#ifdef CONSTRAIN_ELLIPSOID_ORIENTATION
  void updatePoseData(const RawPose3dYawOnlyPtr<double> &pose_ptr){
#else
  void updatePoseData(const RawPose3dPtr<double> &pose_ptr) {
#endif
      updatePoseData(*pose_ptr);
}

void updateEllipsoidParams(const RawEllipsoidPtr<double> &ellipsoid_ptr) {
  updateEllipsoidParams(*ellipsoid_ptr);
}

EllipsoidEstimateNode makeDeepCopy() const {
  RawEllipsoid<double> ellipsoid_copy(*ellipsoid_);
  return EllipsoidEstimateNode(ellipsoid_copy);
}
};  // namespace vslam_types_refactor

struct ObjectObservationFactor {
  FrameId frame_id_;
  CameraId camera_id_;
  ObjectId object_id_;

  BbCorners<double> bounding_box_corners_;

  Covariance<double, 4> bounding_box_corners_covariance_;

  double detection_confidence_;

  ObjectObservationFactor() = default;
  ObjectObservationFactor(
      const FrameId &frame_id,
      const CameraId &camera_id,
      const ObjectId &object_id,
      const BbCorners<double> &bounding_box_corners,
      const Covariance<double, 4> &bounding_box_corners_covariance,
      const double &detection_confidence)
      : frame_id_(frame_id),
        camera_id_(camera_id),
        object_id_(object_id),
        bounding_box_corners_(bounding_box_corners),
        bounding_box_corners_covariance_(bounding_box_corners_covariance),
        detection_confidence_(detection_confidence) {}

  // TODO should semantic class go in here?

  bool operator==(const ObjectObservationFactor &rhs) const {
    return (frame_id_ == rhs.frame_id_) && (camera_id_ == rhs.camera_id_) &&
           (object_id_ == rhs.object_id_) &&
           (bounding_box_corners_ == rhs.bounding_box_corners_) &&
           (bounding_box_corners_covariance_ ==
            rhs.bounding_box_corners_covariance_) &&
           (detection_confidence_ == rhs.detection_confidence_);
  }
};

struct ShapeDimPriorFactor {
  ObjectId object_id_;
  // TODO put anything here?
  ObjectDim<double> mean_shape_dim_;
  Covariance<double, 3> shape_dim_cov_;

  ShapeDimPriorFactor() = default;

  ShapeDimPriorFactor(const ObjectId &object_id,
                      const ObjectDim<double> &mean_shape_dim,
                      const Covariance<double, 3> &shape_dim_cov)
      : object_id_(object_id),
        mean_shape_dim_(mean_shape_dim),
        shape_dim_cov_(shape_dim_cov) {}

  bool operator==(const ShapeDimPriorFactor &rhs) const {
    return (object_id_ == rhs.object_id_) &&
           (mean_shape_dim_ == rhs.mean_shape_dim_) &&
           (shape_dim_cov_ == rhs.shape_dim_cov_);
  }
};

struct ObjOnlyPoseGraphState {
  std::unordered_map<std::string,
                     std::pair<ObjectDim<double>, Covariance<double, 3>>>
      mean_and_cov_by_semantic_class_;

  ObjectId min_object_id_;
  ObjectId max_object_id_;

  std::unordered_map<ObjectId, RawEllipsoid<double>> ellipsoid_estimates_;
  std::unordered_map<ObjectId, std::string> semantic_class_for_object_;
  std::unordered_map<ObjectId, FrameId> last_observed_frame_by_object_;
  std::unordered_map<ObjectId, FrameId> first_observed_frame_by_object_;

  FeatureFactorId min_object_observation_factor_;
  FeatureFactorId max_object_observation_factor_;

  FeatureFactorId min_obj_specific_factor_;
  FeatureFactorId max_obj_specific_factor_;

  std::unordered_set<ObjectId> long_term_map_object_ids_;

  std::unordered_map<FeatureFactorId, ObjectObservationFactor>
      object_observation_factors_;
  std::unordered_map<FeatureFactorId, ShapeDimPriorFactor>
      shape_dim_prior_factors_;

  std::unordered_map<FrameId,
                     util::BoostHashSet<std::pair<FactorType, FeatureFactorId>>>
      observation_factors_by_frame_;

  std::unordered_map<ObjectId,
                     util::BoostHashSet<std::pair<FactorType, FeatureFactorId>>>
      observation_factors_by_object_;
  std::unordered_map<ObjectId,
                     util::BoostHashSet<std::pair<FactorType, FeatureFactorId>>>
      object_only_factors_by_object_;

  bool operator==(const ObjOnlyPoseGraphState &rhs) const {
    return (mean_and_cov_by_semantic_class_ ==
            rhs.mean_and_cov_by_semantic_class_) &&
           (min_object_id_ == rhs.min_object_id_) &&
           (max_object_id_ == rhs.max_object_id_) &&
           (ellipsoid_estimates_ == rhs.ellipsoid_estimates_) &&
           (semantic_class_for_object_ == rhs.semantic_class_for_object_) &&
           (last_observed_frame_by_object_ ==
            rhs.last_observed_frame_by_object_) &&
           (first_observed_frame_by_object_ ==
            rhs.first_observed_frame_by_object_) &&
           (min_object_observation_factor_ ==
            rhs.min_object_observation_factor_) &&
           (max_object_observation_factor_ ==
            rhs.max_object_observation_factor_) &&
           (min_obj_specific_factor_ == rhs.min_obj_specific_factor_) &&
           (max_obj_specific_factor_ == rhs.max_obj_specific_factor_) &&
           (long_term_map_object_ids_ == rhs.long_term_map_object_ids_) &&
           (object_observation_factors_ == rhs.object_observation_factors_) &&
           (shape_dim_prior_factors_ == rhs.shape_dim_prior_factors_) &&
           (observation_factors_by_frame_ ==
            rhs.observation_factors_by_frame_) &&
           (observation_factors_by_object_ ==
            rhs.observation_factors_by_object_) &&
           (object_only_factors_by_object_ ==
            rhs.object_only_factors_by_object_);
  }

  // Not including long-term map factors in state -- have to load
  // long-term map for long-term map factors
};

struct ObjectAndReprojectionFeaturePoseGraphState {
  ReprojectionLowLevelFeaturePoseGraphState
      reprojection_low_level_feature_pose_graph_state_;
  ObjOnlyPoseGraphState obj_only_pose_graph_state_;

  bool operator==(const ObjectAndReprojectionFeaturePoseGraphState &rhs) const {
    return (reprojection_low_level_feature_pose_graph_state_ ==
            rhs.reprojection_low_level_feature_pose_graph_state_) &&
           (obj_only_pose_graph_state_ == rhs.obj_only_pose_graph_state_);
  }
};

// TODO consider making generic to other object representations
template <typename VisualFeatureFactorType>
class ObjAndLowLevelFeaturePoseGraph
    : public virtual LowLevelFeaturePoseGraph<VisualFeatureFactorType> {
 public:
  ObjAndLowLevelFeaturePoseGraph(
      const std::unordered_map<
          std::string,
          std::pair<ObjectDim<double>, Covariance<double, 3>>>
          &mean_and_cov_by_semantic_class,
      const std::unordered_map<CameraId, CameraExtrinsics<double>>
          &camera_extrinsics_by_camera,
      const std::unordered_map<CameraId, CameraIntrinsicsMat<double>>
          &camera_intrinsics_by_camera,
      const std::unordered_map<ObjectId,
                               std::pair<std::string, RawEllipsoid<double>>>
          &long_term_map_objects_with_semantic_class,
      const std::function<
          bool(const std::unordered_set<ObjectId> &,
               util::BoostHashMap<std::pair<FactorType, FeatureFactorId>,
                                  std::unordered_set<ObjectId>> &)>
          &long_term_map_factor_provider,
      const FactorType &visual_feature_factor_type)
      : LowLevelFeaturePoseGraph<VisualFeatureFactorType>(
            camera_extrinsics_by_camera,
            camera_intrinsics_by_camera,
            visual_feature_factor_type),
        mean_and_cov_by_semantic_class_(mean_and_cov_by_semantic_class),
        max_object_id_(0),
        max_object_observation_factor_(0),
        max_obj_specific_factor_(0),
        long_term_map_factor_provider_(long_term_map_factor_provider) {
    for (const auto &ltm_object : long_term_map_objects_with_semantic_class) {
      long_term_map_object_ids_.insert(ltm_object.first);
      max_object_id_ = std::max(max_object_id_, ltm_object.first);
      initializeEllipsoidWithId(EllipsoidEstimateNode(ltm_object.second.second),
                                ltm_object.first,
                                ltm_object.second.first);
    }
  }

  virtual ~ObjAndLowLevelFeaturePoseGraph() = default;

  /**
   * Get the ids of the objects that are in the long-term map.
   *
   * @param ltm_object_ids[out] This variable will be updated with the ids of
   * the long-term map objects.
   */
  virtual void getLongTermMapObjects(
      std::unordered_set<ObjectId> &ltm_object_ids) {
    ltm_object_ids = long_term_map_object_ids_;
  }

  ObjectId addNewEllipsoid(const ObjectDim<double> &object_dim,
                           const RawPose3d<double> &object_pose,
                           const std::string &semantic_class) {
    EllipsoidEstimateNode new_node(object_pose, object_dim);
    return addNewEllipsoid(new_node, semantic_class);
  }

  virtual ObjectId addNewEllipsoid(const EllipsoidEstimateNode &new_node,
                                   const std::string &semantic_class) {
    ObjectId new_object_id = max_object_id_ + 1;
    max_object_id_ = new_object_id;

    initializeEllipsoidWithId(new_node, new_object_id, semantic_class);

    return new_object_id;
  }

  void initializeEllipsoidWithId(const EllipsoidEstimateNode &new_node,
                                 const ObjectId &obj_id,
                                 const std::string &semantic_class) {
    // TODO is it a problem that this function isn't virtual?

    ellipsoid_estimates_[obj_id] = new_node;
    semantic_class_for_object_[obj_id] = semantic_class;
    object_only_factors_by_object_[obj_id] = {};
    observation_factors_by_object_[obj_id] = {};

    addShapeDimPriorBasedOnSemanticClass(obj_id);
  }

  virtual void updateEllipsoid(const ObjectId &object_id,
                               const EllipsoidEstimateNode &ellipsoid_node) {
    // TODO need to make sure nothing records the pointers of the old one -- if
    // so, need to copy the data into the existing one
    ellipsoid_estimates_[object_id].updateEllipsoidParams(
        ellipsoid_node.ellipsoid_);
  }

  virtual FeatureFactorId addShapeDimPriorBasedOnSemanticClass(
      const ObjectId &object_id) {
    std::pair<ObjectDim<double>, Covariance<double, 3>> shape_prior_params =
        mean_and_cov_by_semantic_class_[semantic_class_for_object_[object_id]];
    ShapeDimPriorFactor shape_dim_prior(
        object_id, shape_prior_params.first, shape_prior_params.second);
    FeatureFactorId new_shape_prior_factor_id = max_obj_specific_factor_ + 1;
    max_obj_specific_factor_ = new_shape_prior_factor_id;
    shape_dim_prior_factors_[new_shape_prior_factor_id] = shape_dim_prior;
    object_only_factors_by_object_[object_id].insert(
        std::make_pair(kShapeDimPriorFactorTypeId, new_shape_prior_factor_id));
    return new_shape_prior_factor_id;
  }

  virtual FeatureFactorId addObjectObservation(
      const ObjectObservationFactor &observation_factor) {
    ObjectId object_id = observation_factor.object_id_;
    FrameId frame_id = observation_factor.frame_id_;
    if (last_observed_frame_by_object_.find(object_id) ==
        last_observed_frame_by_object_.end()) {
      last_observed_frame_by_object_[object_id] = frame_id;
    } else {
      if (last_observed_frame_by_object_[object_id] < frame_id) {
        last_observed_frame_by_object_[object_id] = frame_id;
      }
    }
    if (first_observed_frame_by_object_.find(object_id) ==
        first_observed_frame_by_object_.end()) {
      first_observed_frame_by_object_[object_id] = frame_id;
    } else {
      if (first_observed_frame_by_object_[object_id] > frame_id) {
        first_observed_frame_by_object_[object_id] = frame_id;
      }
    }
    FeatureFactorId new_feature_factor_id = max_object_observation_factor_ + 1;
    max_object_observation_factor_ = new_feature_factor_id;
    std::pair<FactorType, FeatureFactorId> factor_type_id_pair =
        std::make_pair(kObjectObservationFactorTypeId, new_feature_factor_id);
    observation_factors_by_object_[object_id].insert(factor_type_id_pair);
    object_observation_factors_[new_feature_factor_id] = observation_factor;
    observation_factors_by_frame_[frame_id].insert(factor_type_id_pair);

    return new_feature_factor_id;
  }

  virtual std::vector<std::pair<FactorType, FeatureFactorId>> pruneFrame(
      const FrameId &frame_id) {
    // TODO need to remove parameter blocks from ceres problem before doing this
    // Should this optionally take in a ceres problem to clean it up? or just
    // note in documentation that this needs to be done
    std::vector<std::pair<FactorType, FeatureFactorId>> removed_factor_ids;
    // TODO finish filling in
    util::BoostHashSet<std::pair<FactorType, FeatureFactorId>>
        observation_factors_to_remove = observation_factors_by_frame_[frame_id];
    std::unordered_map<std::pair<FactorType, FeatureFactorId>,
                       ObjectObservationFactor,
                       boost::hash<std::pair<FactorType, FeatureFactorId>>>
        removed_factors;
    for (const std::pair<FactorType, FeatureFactorId> &feature_factor_id :
         observation_factors_to_remove) {
      if (kObjectObservationFactorTypeId == feature_factor_id.first) {
        removed_factors[feature_factor_id] =
            object_observation_factors_[feature_factor_id.second];
        object_observation_factors_.erase(feature_factor_id.second);
      } else {
        LOG(WARNING) << "Unexpected factor type in observation factors list";
      }
      removed_factor_ids.emplace_back(feature_factor_id);
    }
    // TODO if we're resetting the mins/maxes for factors, need to make sure
    // that the optimizer's state is cleaned up before reusing factor ids
    if (observation_factors_to_remove.find(std::make_pair(
            kObjectObservationFactorTypeId, min_object_observation_factor_)) !=
        observation_factors_to_remove.end()) {
      // TODO reset min_ellipsoid_observation_factor_ maybe?
    }
    if (observation_factors_to_remove.find(std::make_pair(
            kObjectObservationFactorTypeId, max_object_observation_factor_)) !=
        observation_factors_to_remove.end()) {
      // TODO reset max_ellipsoid_observation_factor_
    }

    for (const auto &feature_factor_and_obs : removed_factors) {
      std::pair<FactorType, FeatureFactorId> removed_factor_id =
          feature_factor_and_obs.first;
      ObjectObservationFactor factor = feature_factor_and_obs.second;

      observation_factors_by_object_[factor.object_id_].erase(
          removed_factor_id);
      // TODO consider deleting shape prior and ellipsoid if there are no
      // observations of the ellipsoid left
      // Would need to check for emptiness, find ellispoid, remove it, find
      // shape prior, remove it, update the min/maxes and pass the removal info
      // up
      // Could also add it to a queue or something to indicate that it should be
      // removed if no observations are seen in X frames
    }

    observation_factors_by_frame_.erase(frame_id);

    // TODO call super class function (which should also change min frame id)

    return removed_factor_ids;
  }

  virtual bool getObjectParamPointers(const ObjectId &object_id,
                                      double **ellipsoid_ptr) const {
    if (ellipsoid_estimates_.find(object_id) == ellipsoid_estimates_.end()) {
      return false;
    }

    *ellipsoid_ptr = ellipsoid_estimates_.at(object_id).ellipsoid_->data();
    return true;
  }

  virtual bool getLastObservedFrameForObject(const ObjectId &object_id,
                                             FrameId &frame_id) {
    if (last_observed_frame_by_object_.find(object_id) !=
        last_observed_frame_by_object_.end()) {
      frame_id = last_observed_frame_by_object_[object_id];
      return true;
    }
    return false;
  }

  virtual bool getFirstObservedFrameForObject(const ObjectId &object_id,
                                              FrameId &frame_id) {
    if (first_observed_frame_by_object_.find(object_id) !=
        first_observed_frame_by_object_.end()) {
      frame_id = first_observed_frame_by_object_[object_id];
      return true;
    }
    return false;
  }

  virtual void getObjectsViewedBetweenFramesInclusive(
      const FrameId &min_frame_id,
      const FrameId &max_frame_id,
      std::unordered_set<ObjectId> &matching_objects) {
    for (const auto &object_id_and_most_recent_frame :
         last_observed_frame_by_object_) {
      if (object_id_and_most_recent_frame.second >= min_frame_id) {
        if (first_observed_frame_by_object_.find(
                object_id_and_most_recent_frame.first) !=
            first_observed_frame_by_object_.end()) {
          if (first_observed_frame_by_object_[object_id_and_most_recent_frame
                                                  .first] <= max_frame_id) {
            matching_objects.insert(object_id_and_most_recent_frame.first);
          }
        }
      }
    }
  }

  virtual void getObservationFactorsBetweenFrameIdsInclusive(
      const FrameId &min_frame_id,
      const FrameId &max_frame_id,
      util::BoostHashSet<std::pair<FactorType, FeatureFactorId>>
          &matching_observation_factor_ids) {
    for (const auto &frame_id_and_factors : observation_factors_by_frame_) {
      if ((frame_id_and_factors.first >= min_frame_id) &&
          (frame_id_and_factors.first <= max_frame_id)) {
        matching_observation_factor_ids.insert(
            frame_id_and_factors.second.begin(),
            frame_id_and_factors.second.end());
      }
    }
  }

  virtual void getOnlyObjectFactorsForObjects(
      const std::unordered_set<ObjectId> &objects,
      const bool &use_pom,
      const bool &include_ltm_factors,
      std::unordered_map<
          ObjectId,
          util::BoostHashSet<std::pair<FactorType, FeatureFactorId>>>
          &matching_factors) {
    for (const ObjectId &object_id : objects) {
      if (object_only_factors_by_object_.find(object_id) !=
          object_only_factors_by_object_.end()) {
        // This assumes that all non-LTM object only factors are for a single
        // object. If this changes, this will have to be revised
        util::BoostHashSet<std::pair<FactorType, FeatureFactorId>>
            factors_for_obj = object_only_factors_by_object_[object_id];
        matching_factors[object_id].insert(factors_for_obj.begin(),
                                           factors_for_obj.end());
      }
    }
    if (include_ltm_factors) {
      util::BoostHashMap<std::pair<FactorType, FeatureFactorId>,
                         std::unordered_set<ObjectId>>
          ltm_factors;
      if (!long_term_map_factor_provider_(objects, ltm_factors)) {
        LOG(ERROR) << "Could not add long term map factors to the set to "
                      "include in the optimization. Skipping them.";
      }
      for (const auto &ltm_factor : ltm_factors) {
        for (const auto &objs_for_factor : ltm_factor.second) {
          matching_factors[objs_for_factor].insert(ltm_factor.first);
        }
      }
    }
    // TODO add pom someday
  }

  virtual void getObjectEstimates(
      std::unordered_map<ObjectId, std::pair<std::string, RawEllipsoid<double>>>
          &object_estimates) const {
    for (const auto &pg_obj_est : ellipsoid_estimates_) {
      object_estimates[pg_obj_est.first] =
          std::make_pair(semantic_class_for_object_.at(pg_obj_est.first),
                         *(pg_obj_est.second.ellipsoid_));
    }
  }

  virtual bool getObjectObservationFactor(
      const FeatureFactorId &factor_id,
      ObjectObservationFactor &obs_factor) const {
    if (object_observation_factors_.find(factor_id) ==
        object_observation_factors_.end()) {
      return false;
    }
    obs_factor = object_observation_factors_.at(factor_id);
    return true;
  }

  virtual bool getShapeDimPriorFactor(
      const FeatureFactorId &factor_id,
      ShapeDimPriorFactor &shape_dim_factor) const {
    if (shape_dim_prior_factors_.find(factor_id) ==
        shape_dim_prior_factors_.end()) {
      return false;
    }
    shape_dim_factor = shape_dim_prior_factors_.at(factor_id);
    return true;
  }

  virtual void addFrame(const FrameId &frame_id,
                        const Pose3D<double> &initial_pose_estimate) override {
    LowLevelFeaturePoseGraph<VisualFeatureFactorType>::addFrame(
        frame_id, initial_pose_estimate);
    if (observation_factors_by_frame_.find(frame_id) ==
        observation_factors_by_frame_.end()) {
      observation_factors_by_frame_[frame_id] = {};
    }
  }

  virtual std::optional<ObjectDim<double>> getShapeDimMean(
      const std::string &semantic_class) {
    if (mean_and_cov_by_semantic_class_.find(semantic_class) ==
        mean_and_cov_by_semantic_class_.end()) {
      return {};
    }
    return mean_and_cov_by_semantic_class_.at(semantic_class).first;
  }

  virtual std::unordered_map<
      std::string,
      std::pair<ObjectDim<double>, Covariance<double, 3>>>
  getMeanAndCovBySemanticClass() {
    return mean_and_cov_by_semantic_class_;
  }

  virtual std::unordered_set<ObjectId> getObjectsWithSemanticClass(
      const std::string &semantic_class) const {
    std::unordered_set<ObjectId> objs;
    for (const auto &obj_id_and_class : semantic_class_for_object_) {
      if (obj_id_and_class.second == semantic_class) {
        objs.insert(obj_id_and_class.first);
      }
    }
    return objs;
  }

  std::optional<EllipsoidState<double>> getEllipsoidEst(
      const ObjectId &obj_id) {
    if (ellipsoid_estimates_.find(obj_id) == ellipsoid_estimates_.end()) {
      return {};
    }
    return convertToEllipsoidState(
        RawEllipsoid<double>(*(ellipsoid_estimates_.at(obj_id).ellipsoid_)));
  }

  bool getObservationFactorsForObjId(
      const ObjectId &obj_id,
      std::vector<ObjectObservationFactor> &observation_factors) {
    if (observation_factors_by_object_.find(obj_id) ==
        observation_factors_by_object_.end()) {
      return false;
    }

    util::BoostHashSet<std::pair<FactorType, FeatureFactorId>>
        observation_factor_infos_for_obj =
            observation_factors_by_object_.at(obj_id);
    for (const std::pair<FactorType, FeatureFactorId> &obs_factor_info :
         observation_factor_infos_for_obj) {
      ObjectObservationFactor obs_factor;
      if (getObjectObservationFactor(obs_factor_info.second, obs_factor)) {
        observation_factors.emplace_back(
            object_observation_factors_.at(obs_factor_info.second));
      } else {
        LOG(WARNING) << "Found id " << obs_factor_info.second
                     << " for object observation factor, but there was "
                        "no matching factor";
      }
    }

    return true;
  }

  bool getObjectIdForObjObservationFactor(
      const std::pair<vslam_types_refactor::FactorType,
                      vslam_types_refactor::FeatureFactorId> &factor_info,
      vslam_types_refactor::FeatureId &object_id) const {
    if (factor_info.first != kObjectObservationFactorTypeId) {
      LOG(WARNING) << "Unsupported object observation factor type "
                   << factor_info.first;
      return false;
    }
    if (object_observation_factors_.find(factor_info.second) ==
        object_observation_factors_.end()) {
      return false;
    }
    object_id = object_observation_factors_.at(factor_info.second).object_id_;
    return true;
  }

  void setEllipsoidEstimatePtrs(
      const std::unordered_map<ObjectId, EllipsoidEstimateNode>
          &ellipsoid_estimates) {
    ellipsoid_estimates_ = ellipsoid_estimates;
  }

  void getEllipsoidEstimatePtrs(
      std::unordered_map<ObjectId, EllipsoidEstimateNode> &ellipsoid_estimates)
      const {
    ellipsoid_estimates = ellipsoid_estimates_;
  }

  void initializeFromState(const ObjOnlyPoseGraphState &pose_graph_state) {
    mean_and_cov_by_semantic_class_ =
        pose_graph_state.mean_and_cov_by_semantic_class_;
    min_object_id_ = pose_graph_state.min_object_id_;
    max_object_id_ = pose_graph_state.max_object_id_;
    semantic_class_for_object_ = pose_graph_state.semantic_class_for_object_;
    last_observed_frame_by_object_ =
        pose_graph_state.last_observed_frame_by_object_;
    first_observed_frame_by_object_ =
        pose_graph_state.first_observed_frame_by_object_;
    min_object_observation_factor_ =
        pose_graph_state.min_object_observation_factor_;
    max_object_observation_factor_ =
        pose_graph_state.max_object_observation_factor_;
    min_obj_specific_factor_ = pose_graph_state.min_obj_specific_factor_;
    max_obj_specific_factor_ = pose_graph_state.max_obj_specific_factor_;
    long_term_map_object_ids_ = pose_graph_state.long_term_map_object_ids_;
    object_observation_factors_ = pose_graph_state.object_observation_factors_;
    shape_dim_prior_factors_ = pose_graph_state.shape_dim_prior_factors_;
    observation_factors_by_frame_ =
        pose_graph_state.observation_factors_by_frame_;
    observation_factors_by_object_ =
        pose_graph_state.observation_factors_by_object_;
    object_only_factors_by_object_ =
        pose_graph_state.object_only_factors_by_object_;

    for (const auto &ellipsoid_est : pose_graph_state.ellipsoid_estimates_) {
      ellipsoid_estimates_[ellipsoid_est.first] =
          EllipsoidEstimateNode(ellipsoid_est.second);
    }

    // Long term map factors not included in state -- need to separately load
    // long-term map
  }

  void getState(ObjOnlyPoseGraphState &pose_graph_state) {
    pose_graph_state.mean_and_cov_by_semantic_class_ =
        mean_and_cov_by_semantic_class_;
    pose_graph_state.min_object_id_ = min_object_id_;
    pose_graph_state.max_object_id_ = max_object_id_;
    pose_graph_state.semantic_class_for_object_ = semantic_class_for_object_;
    pose_graph_state.last_observed_frame_by_object_ =
        last_observed_frame_by_object_;
    pose_graph_state.first_observed_frame_by_object_ =
        first_observed_frame_by_object_;
    pose_graph_state.min_object_observation_factor_ =
        min_object_observation_factor_;
    pose_graph_state.max_object_observation_factor_ =
        max_object_observation_factor_;
    pose_graph_state.min_obj_specific_factor_ = min_obj_specific_factor_;
    pose_graph_state.max_obj_specific_factor_ = max_obj_specific_factor_;
    pose_graph_state.long_term_map_object_ids_ = long_term_map_object_ids_;
    pose_graph_state.object_observation_factors_ = object_observation_factors_;
    pose_graph_state.shape_dim_prior_factors_ = shape_dim_prior_factors_;
    pose_graph_state.observation_factors_by_frame_ =
        observation_factors_by_frame_;
    pose_graph_state.observation_factors_by_object_ =
        observation_factors_by_object_;
    pose_graph_state.object_only_factors_by_object_ =
        object_only_factors_by_object_;

    for (const auto &ellipsoid_est : ellipsoid_estimates_) {
      pose_graph_state.ellipsoid_estimates_[ellipsoid_est.first] =
          RawEllipsoid<double>(*(ellipsoid_est.second.ellipsoid_));
    }

    // Long term map factors not included in state -- need to separately load
    // long-term map
  }

 protected:
  ObjAndLowLevelFeaturePoseGraph(
      const std::function<
          bool(const std::unordered_set<ObjectId> &,
               util::BoostHashMap<std::pair<FactorType, FeatureFactorId>,
                                  std::unordered_set<ObjectId>> &)>
          &long_term_map_factor_provider)
      : long_term_map_factor_provider_(long_term_map_factor_provider) {}

  std::unordered_map<std::string,
                     std::pair<ObjectDim<double>, Covariance<double, 3>>>
      mean_and_cov_by_semantic_class_;

  ObjectId min_object_id_;
  ObjectId max_object_id_;

  std::unordered_map<ObjectId, EllipsoidEstimateNode> ellipsoid_estimates_;
  std::unordered_map<ObjectId, std::string> semantic_class_for_object_;
  std::unordered_map<ObjectId, FrameId> last_observed_frame_by_object_;
  std::unordered_map<ObjectId, FrameId> first_observed_frame_by_object_;

  FeatureFactorId min_object_observation_factor_;
  FeatureFactorId max_object_observation_factor_;

  FeatureFactorId min_obj_specific_factor_;
  FeatureFactorId max_obj_specific_factor_;

  std::unordered_set<ObjectId> long_term_map_object_ids_;

  std::unordered_map<FeatureFactorId, ObjectObservationFactor>
      object_observation_factors_;
  std::unordered_map<FeatureFactorId, ShapeDimPriorFactor>
      shape_dim_prior_factors_;

  std::unordered_map<FrameId,
                     util::BoostHashSet<std::pair<FactorType, FeatureFactorId>>>
      observation_factors_by_frame_;

  std::unordered_map<ObjectId,
                     util::BoostHashSet<std::pair<FactorType, FeatureFactorId>>>
      observation_factors_by_object_;
  std::unordered_map<ObjectId,
                     util::BoostHashSet<std::pair<FactorType, FeatureFactorId>>>
      object_only_factors_by_object_;

  std::function<bool(const std::unordered_set<ObjectId> &,
                     util::BoostHashMap<std::pair<FactorType, FeatureFactorId>,
                                        std::unordered_set<ObjectId>> &)>
      long_term_map_factor_provider_;

  ObjAndLowLevelFeaturePoseGraph(const ObjAndLowLevelFeaturePoseGraph &other) =
      default;
};

class ObjectAndReprojectionFeaturePoseGraph
    : public ReprojectionLowLevelFeaturePoseGraph,
      public ObjAndLowLevelFeaturePoseGraph<ReprojectionErrorFactor> {
 public:
  ObjectAndReprojectionFeaturePoseGraph(
      const std::unordered_map<
          std::string,
          std::pair<ObjectDim<double>, Covariance<double, 3>>>
          &mean_and_cov_by_semantic_class,
      const std::unordered_map<CameraId, CameraExtrinsics<double>>
          &camera_extrinsics_by_camera,
      const std::unordered_map<CameraId, CameraIntrinsicsMat<double>>
          &camera_intrinsics_by_camera,
      const std::unordered_map<ObjectId,
                               std::pair<std::string, RawEllipsoid<double>>>
          &long_term_map_objects_with_semantic_class,
      const std::function<
          bool(const std::unordered_set<ObjectId> &,
               util::BoostHashMap<std::pair<FactorType, FeatureFactorId>,
                                  std::unordered_set<ObjectId>> &)>
          &long_term_map_factor_provider)
      : ReprojectionLowLevelFeaturePoseGraph(camera_extrinsics_by_camera,
                                             camera_intrinsics_by_camera),
        LowLevelFeaturePoseGraph<ReprojectionErrorFactor>(
            camera_extrinsics_by_camera,
            camera_intrinsics_by_camera,
            kReprojectionErrorFactorTypeId),
        ObjAndLowLevelFeaturePoseGraph<ReprojectionErrorFactor>(
            mean_and_cov_by_semantic_class,
            camera_extrinsics_by_camera,
            camera_intrinsics_by_camera,
            long_term_map_objects_with_semantic_class,
            long_term_map_factor_provider,
            kReprojectionErrorFactorTypeId) {}
  virtual ~ObjectAndReprojectionFeaturePoseGraph() = default;

  /**
   * WARNING: This only makes a shallow copy, so make sure that's what you want.
   * If you want a deep(er) copy (some things are assumed to be unchanging even
   * if there is a reference), call makeDeepCopy instead.
   * @param other
   */
  ObjectAndReprojectionFeaturePoseGraph(
      const ObjectAndReprojectionFeaturePoseGraph &other) = default;

  void setValuesFromAnotherPoseGraph(
      const std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph>
          &pose_graph) {
    std::unordered_map<ObjectId, EllipsoidEstimateNode> ellipsoid_estimates;
    pose_graph->getEllipsoidEstimatePtrs(ellipsoid_estimates);
    for (const auto &ellipsoid_est : ellipsoid_estimates) {
      this->ellipsoid_estimates_[ellipsoid_est.first].updateEllipsoidParams(
          ellipsoid_est.second.ellipsoid_);
    }
    std::unordered_map<FrameId, RobotPoseNode> robot_poses;
    pose_graph->getRobotPosePtrs(robot_poses);
    for (const auto &robot_pose : robot_poses) {
      this->robot_poses_[robot_pose.first].updateRobotPoseParams(
          robot_pose.second.pose_);
    }
    std::unordered_map<FeatureId, VisualFeatureNode> feature_positions;
    pose_graph->getFeaturePositionPtrs(feature_positions);
    for (const auto &feature_position : feature_positions) {
      this->feature_positions_[feature_position.first]
          .updateVisualPositionParams(feature_position.second.position_);
    }
  }

  std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph> makeDeepCopy() const {
    std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph> copy =
        std::make_shared<ObjectAndReprojectionFeaturePoseGraph>(*this);
    // Reset the fields that we don't just want a shallow copy of
    std::unordered_map<ObjectId, EllipsoidEstimateNode>
        ellipsoid_estimates_copy;
    for (const auto &ellipsoid_est : this->ellipsoid_estimates_) {
      ellipsoid_estimates_copy[ellipsoid_est.first] =
          ellipsoid_est.second.makeDeepCopy();
    }
    copy->setEllipsoidEstimatePtrs(ellipsoid_estimates_copy);
    // ObjectAndReprojectionFeaturePoseGraph::ellipsoid_estimates_ =
    //     ellipsoid_estimates_copy;

    std::unordered_map<FrameId, RobotPoseNode> robot_poses_copy;
    for (const auto &robot_pose_est : this->robot_poses_) {
      robot_poses_copy[robot_pose_est.first] =
          robot_pose_est.second.makeDeepCopy();
    }
    copy->setRobotPosePtrs(robot_poses_copy);
    // ObjectAndReprojectionFeaturePoseGraph::robot_poses_ = robot_poses_copy;

    std::unordered_map<FeatureId, VisualFeatureNode> feature_positions_copy;
    for (const auto &feature_pos_est : this->feature_positions_) {
      feature_positions_copy[feature_pos_est.first] =
          feature_pos_est.second.makeDeepCopy();
    }
    copy->setFeaturePositionPtrs(feature_positions_copy);
    // ObjectAndReprojectionFeaturePoseGraph::feature_positions_ =
    //     feature_positions_copy;
    return copy;
  }

  void initializeFromState(
      const ObjectAndReprojectionFeaturePoseGraphState &pose_graph_state) {
    ReprojectionLowLevelFeaturePoseGraph::initializeFromState(
        pose_graph_state.reprojection_low_level_feature_pose_graph_state_);
    ObjAndLowLevelFeaturePoseGraph<ReprojectionErrorFactor>::
        initializeFromState(pose_graph_state.obj_only_pose_graph_state_);
  }

  void getState(ObjectAndReprojectionFeaturePoseGraphState &pose_graph_state) {
    ReprojectionLowLevelFeaturePoseGraph::getState(
        pose_graph_state.reprojection_low_level_feature_pose_graph_state_);
    ObjAndLowLevelFeaturePoseGraph<ReprojectionErrorFactor>::getState(
        pose_graph_state.obj_only_pose_graph_state_);
  }

  static std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph>
  createObjectAndReprojectionFeaturePoseGraphFromState(
      const ObjectAndReprojectionFeaturePoseGraphState &pose_graph_state,
      const std::function<
          bool(const std::unordered_set<ObjectId> &,
               util::BoostHashMap<std::pair<FactorType, FeatureFactorId>,
                                  std::unordered_set<ObjectId>> &)>
          &long_term_map_factor_provider) {
    ObjectAndReprojectionFeaturePoseGraph orig_pose_graph(
        long_term_map_factor_provider);
    orig_pose_graph.initializeFromState(pose_graph_state);

    // Make a pointer with the shallow-copy constructor because dumb c++ won't
    // allow shared_ptrs with private/protected constructors
    return std::make_shared<ObjectAndReprojectionFeaturePoseGraph>(
        orig_pose_graph);
  }

 protected:
  ObjectAndReprojectionFeaturePoseGraph(
      const std::function<
          bool(const std::unordered_set<ObjectId> &,
               util::BoostHashMap<std::pair<FactorType, FeatureFactorId>,
                                  std::unordered_set<ObjectId>> &)>
          &long_term_map_factor_provider)
      : ReprojectionLowLevelFeaturePoseGraph(),
        LowLevelFeaturePoseGraph<ReprojectionErrorFactor>(),
        ObjAndLowLevelFeaturePoseGraph<ReprojectionErrorFactor>(
            long_term_map_factor_provider) {}

 private:
};

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_OBJECT_POSE_GRAPH_H
