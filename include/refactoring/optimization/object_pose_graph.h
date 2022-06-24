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

struct EllipsoidEstimateNode {
  RawEllipsoidPtr<double> ellipsoid_;

  EllipsoidEstimateNode()
      : ellipsoid_(std::make_shared<RawEllipsoid<double>>()) {}

  EllipsoidEstimateNode(const RawPose3d<double> &pose,
                        const ObjectDim<double> &dimensions)
      : EllipsoidEstimateNode() {
    updateEllipsoidParams(pose, dimensions);
  }

  EllipsoidEstimateNode(const RawEllipsoid<double> &raw_ellipsoid_data)
      : ellipsoid_(std::make_shared<RawEllipsoid<double>>(raw_ellipsoid_data)) {
  }

  void updatePoseData(const RawPose3d<double> &pose) {
    // TODO verify that this is changing the contents of the pointer
    //  (with a copy) and not the pointer
    ellipsoid_->topRows(6) = pose;
  }

  void updateDimensionData(const ObjectDim<double> &dim) {
    // TODO verify that this is changing the contents of the pointer (with a
    //  copy) and not the pointer
    ellipsoid_->bottomRows(3) = dim;
  }

  void updateEllipsoidParams(const RawPose3d<double> &pose,
                             const ObjectDim<double> &dim) {
    updatePoseData(pose);
    updateDimensionData(dim);
  }

  void updateEllipsoidParams(const RawEllipsoid<double> &ellipsoid) {
    ellipsoid_->topRows(9) = ellipsoid;
  }

  void updatePoseData(const RawPose3dPtr<double> &pose_ptr) {
    updatePoseData(*pose_ptr);
  }

  void updateEllipsoidParams(const RawEllipsoidPtr<double> &ellipsoid_ptr) {
    updateEllipsoidParams(*ellipsoid_ptr);
  }
};

struct ObjectObservationFactor {
  FrameId frame_id_;
  CameraId camera_id_;
  ObjectId object_id_;

  BbCorners<double> bounding_box_corners_;

  Covariance<double, 4> bounding_box_corners_covariance_;

  // TODO should semantic class go in here?
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
          &camera_intrinsics_by_camera)
      : LowLevelFeaturePoseGraph<VisualFeatureFactorType>(
            camera_extrinsics_by_camera, camera_intrinsics_by_camera),
        mean_and_cov_by_semantic_class_(mean_and_cov_by_semantic_class),
        max_object_id_(0) {}

  virtual ~ObjAndLowLevelFeaturePoseGraph() = default;
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

    ellipsoid_estimates_[new_object_id] = new_node;
    semantic_class_for_object_[new_object_id] = semantic_class;
    object_only_factors_by_object_[new_object_id] = {};
    observation_factors_by_object_[new_object_id] = {};

    addShapeDimPriorBasedOnSemanticClass(new_object_id);

    return new_object_id;
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
                                      double **ellipsoid_ptr) {
    if (ellipsoid_estimates_.find(object_id) == ellipsoid_estimates_.end()) {
      return false;
    }

    *ellipsoid_ptr = ellipsoid_estimates_[object_id].ellipsoid_->data();
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
      util::BoostHashSet<std::pair<FactorType, FeatureFactorId>>
          &matching_factors) {
    for (const ObjectId &object_id : objects) {
      if (object_only_factors_by_object_.find(object_id) !=
          object_only_factors_by_object_.end()) {
        util::BoostHashSet<std::pair<FactorType, FeatureFactorId>>
            factors_for_obj = object_only_factors_by_object_[object_id];
        matching_factors.insert(factors_for_obj.begin(), factors_for_obj.end());
      }
    }
    // TODO add pom someday
  }

  virtual void getObjectEstimates(
      std::unordered_map<ObjectId, RawEllipsoid<double>> &object_estimates)
      const {
    for (const auto &pg_obj_est : ellipsoid_estimates_) {
      object_estimates[pg_obj_est.first] = *(pg_obj_est.second.ellipsoid_);
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

 protected:
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

  std::unordered_map<std::string,
                     std::pair<ObjectDim<double>, Covariance<double, 3>>>
      mean_and_cov_by_semantic_class_;
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
          &camera_intrinsics_by_camera)
      : ReprojectionLowLevelFeaturePoseGraph(camera_extrinsics_by_camera,
                                             camera_intrinsics_by_camera),
        LowLevelFeaturePoseGraph<ReprojectionErrorFactor>(
            camera_extrinsics_by_camera, camera_intrinsics_by_camera),
        ObjAndLowLevelFeaturePoseGraph<ReprojectionErrorFactor>(
            mean_and_cov_by_semantic_class,
            camera_extrinsics_by_camera,
            camera_intrinsics_by_camera) {}
  virtual ~ObjectAndReprojectionFeaturePoseGraph() = default;

 protected:
 private:
};

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_OBJECT_POSE_GRAPH_H
