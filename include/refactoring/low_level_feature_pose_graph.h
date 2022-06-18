//
// Created by amanda on 6/9/22.
//

#ifndef UT_VSLAM_LOW_LEVEL_FEATURE_POSE_GRAPH_H
#define UT_VSLAM_LOW_LEVEL_FEATURE_POSE_GRAPH_H

#include <refactoring/vslam_basic_types_refactor.h>
#include <refactoring/vslam_types_refactor.h>
#include <unordered_set>

namespace vslam_types_refactor {

struct RobotPoseNode {
  RawPose3dPtr<double> pose_;
};

struct VisualFeatureNode {
  Position3dPtr<double> position_;
};

struct PairwiseFeatureFactor {
  FrameId frame_id_1_;
  FrameId frame_id_2_;

  CameraId camera_id_1_;
  CameraId camera_id_2_;

  PixelCoord<double> feature_pos_1_;
  PixelCoord<double> feature_pos_2_;

  double epipolar_error_std_dev_;
};

struct ReprojectionErrorFactor {
  FrameId frame_id_;
  CameraId camera_id_;
  PixelCoord<double> feature_pos_;
  double reprojection_error_std_dev_;
};

typedef uint64_t FeatureFactorId;
typedef uint8_t FactorType;

static const FactorType kReprojectionErrorFactorTypeId = 0;
static const FactorType kPairwiseErrorFactorTypeId = 1;

template <typename VisualFeatureFactorType>
class LowLevelFeaturePoseGraph {
 public:
  bool getExtrinsicsForCamera(const CameraId &camera_id,
                              CameraExtrinsics<double> &extrinsics_return) {
    if (camera_extrinsics_by_camera_.find(camera_id) !=
        camera_extrinsics_by_camera_.end()) {
      extrinsics_return = camera_extrinsics_by_camera_[camera_id];
      return true;
    }
    return false;
  }

  bool getIntrinsicsForCamera(const CameraId &camera_id,
                              CameraIntrinsicsMat<double> &intrinsics_return) {
    if (camera_instrinsics_by_camera_.find(camera_id) !=
        camera_instrinsics_by_camera_.end()) {
      intrinsics_return = camera_instrinsics_by_camera_[camera_id];
      return true;
    }
    return false;
  }

  virtual bool getFeaturePointers(const FeatureId &feature_id,
                                  double **feature_ptr) {
    return false;
  }

  virtual bool getPosePointers(const FrameId &frame_id, double **pose_ptr) {
    if (robot_poses_.find(frame_id) != robot_poses_.end()) {
      *pose_ptr = robot_poses_[frame_id].pose_->data();
      return true;
    }
    return false;
  }

  bool getLastObservedFrameForFeature(const FeatureId &feature_id,
                                      FrameId &frame_id) {
    if (last_observed_frame_by_feature_.find(feature_id) !=
        last_observed_frame_by_feature_.end()) {
      frame_id = last_observed_frame_by_feature_[feature_id];
      return true;
    }
    return false;
  }

  bool getFirstObservedFrameForFeature(const FeatureId &feature_id,
                                      FrameId &frame_id) {
    if (first_observed_frame_by_feature_.find(feature_id) !=
        first_observed_frame_by_feature_.end()) {
      frame_id = first_observed_frame_by_feature_[feature_id];
      return true;
    }
    return false;
  }

  void getFeaturesViewedBetweenFramesInclusive(
      const FrameId &min_frame_id,
      const FrameId &max_frame_id,
      std::unordered_set<FeatureId> &matching_features) {
    for (const auto &object_id_and_most_recent_frame :
        last_observed_frame_by_feature_) {
      if (object_id_and_most_recent_frame.second >= min_frame_id) {
        if (first_observed_frame_by_feature_.find(
            object_id_and_most_recent_frame.first) !=
            first_observed_frame_by_feature_.end()) {
          if (first_observed_frame_by_feature_[object_id_and_most_recent_frame
              .first] <= max_frame_id) {
            matching_features.insert(object_id_and_most_recent_frame.first);
          }
        }
      }
    }
  }

  std::unordered_set<FrameId> getFrameIds() {
    std::unordered_set<FrameId> frames;
    for (const auto &frame_and_pose : robot_poses_) {
      frames.insert(frame_and_pose.first);
    }
    return frames;
  }

  virtual void getVisualFeatureFactorIdsBetweenFrameIdsInclusive(
      const FrameId &min_frame_id,
      const FrameId &max_frame_id,
      std::vector<std::pair<FactorType, FeatureFactorId>> &matching_factors) {
    for (const auto &frame_and_matching_factors :
         visual_feature_factors_by_frame_) {
      if ((frame_and_matching_factors.first >= min_frame_id) &&
          (frame_and_matching_factors.first <= max_frame_id)) {
        matching_factors.insert(matching_factors.end(),
                                frame_and_matching_factors.second.begin(),
                                frame_and_matching_factors.second.end());
      }
    }
  }

 protected:
  FrameId min_frame_id_;
  FrameId max_frame_id_;

  FeatureFactorId min_feature_factor_id_;
  FeatureFactorId max_feature_factor_id_;

  std::unordered_map<FrameId, RobotPoseNode> robot_poses_;

  std::unordered_map<FrameId,
                     std::vector<std::pair<FactorType, FeatureFactorId>>>
      visual_feature_factors_by_frame_;

  std::unordered_map<FeatureFactorId, VisualFeatureFactorType> factors_;

  std::unordered_map<FeatureId, FrameId> last_observed_frame_by_feature_;

  std::unordered_map<FeatureId, FrameId> first_observed_frame_by_feature_;

  /**
   * Extrinsics for each camera.
   */
  std::unordered_map<CameraId, CameraExtrinsics<double>>
      camera_extrinsics_by_camera_;

  /**
   * Intrinsics for each camera.
   */
  std::unordered_map<CameraId, CameraIntrinsicsMat<double>>
      camera_instrinsics_by_camera_;
};

class ReprojectionLowLevelFeaturePoseGraph
    : public virtual LowLevelFeaturePoseGraph<ReprojectionErrorFactor> {
 public:
  virtual bool getFeaturePointers(const FeatureId &feature_id,
                                  double **feature_ptr) override {
    if (feature_positions_.find(feature_id) == feature_positions_.end()) {
      return false;
    }

    *feature_ptr = feature_positions_[feature_id].position_->data();
    return true;
  }

 protected:
  FeatureId min_feature_id_;
  FeatureId max_feature_id_;
  std::unordered_map<FeatureId, VisualFeatureNode> feature_positions_;
};

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_LOW_LEVEL_FEATURE_POSE_GRAPH_H
