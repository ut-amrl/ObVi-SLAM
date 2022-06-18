//
// Created by amanda on 6/9/22.
//

#ifndef UT_VSLAM_OBJECT_POSE_GRAPH_H
#define UT_VSLAM_OBJECT_POSE_GRAPH_H

#include <base_lib/basic_utils.h>
#include <refactoring/low_level_feature_pose_graph.h>
#include <refactoring/vslam_basic_types_refactor.h>
#include <refactoring/vslam_obj_opt_types_refactor.h>
#include <refactoring/vslam_types_refactor.h>

namespace vslam_types_refactor {

static const FactorType kObjectObservationFactorTypeId = 2;
static const FactorType kShapeDimPriorFactorTypeId = 3;

struct EllipsoidEstimateNode {
  vslam_types_refactor::RawEllipsoidPtr<double> ellipsoid_;

  EllipsoidEstimateNode() = default;

  EllipsoidEstimateNode(const RawPose3d<double> &pose,
                        const ObjectDim<double> &dimensions) {
    updateEllipsoidParams(pose, dimensions);
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
  ObjectId addNewEllipsoid(const ObjectDim<double> &object_dim,
                           const RawPose3d<double> &object_pose,
                           const std::string &semantic_class) {
    EllipsoidEstimateNode new_node(object_pose, object_dim);
    return addNewEllipsoid(new_node, semantic_class);
  }

  ObjectId addNewEllipsoid(const EllipsoidEstimateNode &new_node,
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

  void updateEllipsoid(const ObjectId &object_id,
                       const EllipsoidEstimateNode &ellipsoid_node) {
    // TODO need to make sure nothing records the pointers of the old one -- if
    // so, need to copy the data into the existing one
    ellipsoid_estimates_[object_id].updateEllipsoidParams(
        ellipsoid_node.ellipsoid_);
  }

  FeatureFactorId addShapeDimPriorBasedOnSemanticClass(
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

  FeatureFactorId addObjectObservation(
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

  std::vector<std::pair<FactorType, FeatureFactorId>> pruneFrame(
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
      removed_factor_ids.emplace_back(
          std::make_pair(kObjectObservationFactorTypeId, feature_factor_id));
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

  bool getObjectParamPointers(const ObjectId &object_id,
                              double **ellipsoid_ptr) {
    if (ellipsoid_estimates_.find(object_id) == ellipsoid_estimates_.end()) {
      return false;
    }

    *ellipsoid_ptr = ellipsoid_estimates_[object_id].ellipsoid_->data();
    return true;
  }

  bool getLastObservedFrameForObject(const ObjectId &object_id,
                                     FrameId &frame_id) {
    if (last_observed_frame_by_object_.find(object_id) !=
        last_observed_frame_by_object_.end()) {
      frame_id = last_observed_frame_by_object_[object_id];
      return true;
    }
    return false;
  }

  bool getFirstObservedFrameForObject(const ObjectId &object_id,
                                      FrameId &frame_id) {
    if (first_observed_frame_by_object_.find(object_id) !=
        first_observed_frame_by_object_.end()) {
      frame_id = first_observed_frame_by_object_[object_id];
      return true;
    }
    return false;
  }

  void getObjectsViewedBetweenFramesInclusive(
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

  void getObservationFactorsBetweenFrameIdsInclusive(
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

  void getOnlyObjectFactorsForObjects(
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
    : public ObjAndLowLevelFeaturePoseGraph<ReprojectionErrorFactor>,
      public ReprojectionLowLevelFeaturePoseGraph {
 public:
 protected:
 private:
};
}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_OBJECT_POSE_GRAPH_H
