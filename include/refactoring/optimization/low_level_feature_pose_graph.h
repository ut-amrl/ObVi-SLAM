//
// Created by amanda on 6/9/22.
//

#ifndef UT_VSLAM_LOW_LEVEL_FEATURE_POSE_GRAPH_H
#define UT_VSLAM_LOW_LEVEL_FEATURE_POSE_GRAPH_H

#include <refactoring/types/vslam_basic_types_refactor.h>
#include <refactoring/types/vslam_types_conversion.h>

#include <unordered_set>

namespace vslam_types_refactor {

typedef uint64_t FeatureFactorId;
typedef uint8_t FactorType;

static const FactorType kReprojectionErrorFactorTypeId = 0;
static const FactorType kPairwiseErrorFactorTypeId = 1;

struct RobotPoseNode {
  RawPose3dPtr<double> pose_;

  RobotPoseNode()
      : pose_(std::make_shared<RawPose3d<double>>(RawPose3d<double>())) {}

  RobotPoseNode(RawPose3d<double> &pose)
      : pose_(std::make_shared<RawPose3d<double>>(pose)) {}

  RobotPoseNode makeDeepCopy() const {
    RawPose3d<double> pose_copy(*pose_);
    return RobotPoseNode(pose_copy);
  }
};

struct VisualFeatureNode {
  Position3dPtr<double> position_;

  VisualFeatureNode()
      : position_(std::make_shared<Position3d<double>>(Position3d<double>())) {}

  VisualFeatureNode(const Position3d<double> &position)
      : position_(std::make_shared<Position3d<double>>(position)) {}

  VisualFeatureNode makeDeepCopy() const {
    Position3d<double> position_copy(*position_);
    return VisualFeatureNode(position_copy);
  }
};

struct PairwiseFeatureFactor {
  FrameId frame_id_1_;
  FrameId frame_id_2_;

  CameraId camera_id_1_;
  CameraId camera_id_2_;

  PixelCoord<double> feature_pos_1_;
  PixelCoord<double> feature_pos_2_;

  FeatureId feature_id_;

  double epipolar_error_std_dev_;

  FactorType getFactorType() const { return kPairwiseErrorFactorTypeId; }

  std::vector<FrameId> getOrderedFrameIds() const {
    if (frame_id_1_ < frame_id_2_) {
      return {frame_id_1_, frame_id_2_};
    }
    return {frame_id_2_, frame_id_1_};
  }
};

struct ReprojectionErrorFactor {
  FrameId frame_id_;
  FeatureId feature_id_;
  CameraId camera_id_;
  PixelCoord<double> feature_pos_;
  double reprojection_error_std_dev_;

  FactorType getFactorType() const { return kReprojectionErrorFactorTypeId; }

  std::vector<FrameId> getOrderedFrameIds() const { return {frame_id_}; }
};

template <typename VisualFeatureFactorType>
class LowLevelFeaturePoseGraph {
 public:
  LowLevelFeaturePoseGraph(
      const std::unordered_map<CameraId, CameraExtrinsics<double>>
          &camera_extrinsics_by_camera,
      const std::unordered_map<CameraId, CameraIntrinsicsMat<double>>
          &camera_intrinsics_by_camera)
      : camera_extrinsics_by_camera_(camera_extrinsics_by_camera),
        camera_intrinsics_by_camera_(camera_intrinsics_by_camera),
        min_frame_id_(std::numeric_limits<FrameId>::max()),
        max_frame_id_(std::numeric_limits<FrameId>::min()) {}

  virtual ~LowLevelFeaturePoseGraph() = default;
  virtual bool getExtrinsicsForCamera(
      const CameraId &camera_id, CameraExtrinsics<double> &extrinsics_return) {
    if (camera_extrinsics_by_camera_.find(camera_id) !=
        camera_extrinsics_by_camera_.end()) {
      extrinsics_return = camera_extrinsics_by_camera_[camera_id];
      return true;
    }
    return false;
  }

  virtual bool getIntrinsicsForCamera(
      const CameraId &camera_id,
      CameraIntrinsicsMat<double> &intrinsics_return) {
    if (camera_intrinsics_by_camera_.find(camera_id) !=
        camera_intrinsics_by_camera_.end()) {
      intrinsics_return = camera_intrinsics_by_camera_[camera_id];
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

  virtual bool getLastObservedFrameForFeature(const FeatureId &feature_id,
                                              FrameId &frame_id) {
    if (last_observed_frame_by_feature_.find(feature_id) !=
        last_observed_frame_by_feature_.end()) {
      frame_id = last_observed_frame_by_feature_[feature_id];
      return true;
    }
    return false;
  }

  virtual bool getFirstObservedFrameForFeature(const FeatureId &feature_id,
                                               FrameId &frame_id) {
    if (first_observed_frame_by_feature_.find(feature_id) !=
        first_observed_frame_by_feature_.end()) {
      frame_id = first_observed_frame_by_feature_[feature_id];
      return true;
    }
    return false;
  }

  virtual void getFeaturesViewedBetweenFramesInclusive(
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

  virtual std::unordered_set<FrameId> getFrameIds() {
    std::unordered_set<FrameId> frames;
    for (const auto &frame_and_pose : robot_poses_) {
      frames.insert(frame_and_pose.first);
    }
    return frames;
  }

  virtual void getVisualFeatureFactorIdsBetweenFrameIdsInclusive(
      const FrameId &min_frame_id,
      const FrameId &max_frame_id,
      util::BoostHashSet<std::pair<FactorType, FeatureFactorId>> &matching_factors) {
    for (const auto &frame_and_matching_factors :
         visual_feature_factors_by_frame_) {
      if ((frame_and_matching_factors.first >= min_frame_id) &&
          (frame_and_matching_factors.first <= max_frame_id)) {
        matching_factors.insert(
            frame_and_matching_factors.second.begin(),
            frame_and_matching_factors.second.end());
      }
    }
  }

  virtual void getRobotPoseEstimates(
      std::unordered_map<FrameId, RawPose3d<double>> &robot_pose_estimates)
      const {
    for (const auto &pg_pose_est : robot_poses_) {
      robot_pose_estimates[pg_pose_est.first] = *(pg_pose_est.second.pose_);
    }
  }

  virtual bool getVisualFactor(
      const FeatureFactorId &factor_id,
      VisualFeatureFactorType &visual_feature_factor) const {
    if (factors_.find(factor_id) == factors_.end()) {
      return false;
    }
    visual_feature_factor = factors_.at(factor_id);
    return true;
  }

  virtual void addFrame(const FrameId &frame_id,
                        const Pose3D<double> &initial_pose_estimate) {
    if (frame_id < min_frame_id_) {
      min_frame_id_ = frame_id;
    }
    if (frame_id > max_frame_id_) {
      max_frame_id_ = frame_id;
    }

    visual_feature_factors_by_frame_[frame_id] = {};
    RawPose3d<double> raw_pose = convertPoseToArray(initial_pose_estimate);
    robot_poses_[frame_id] = RobotPoseNode(raw_pose);
  }

  virtual FeatureFactorId addVisualFactor(
      const VisualFeatureFactorType &factor) {
    FeatureFactorId factor_id = max_feature_factor_id_ + 1;
    max_feature_factor_id_ = factor_id;

    std::vector<FrameId> ordered_frame_ids = factor.getOrderedFrameIds();

    for (const FrameId &frame_id : ordered_frame_ids) {
      visual_feature_factors_by_frame_[frame_id].emplace_back(
          std::make_pair(factor.getFactorType(), factor_id));
    }

    factors_[factor_id] = factor;
    FeatureId feature_id = factor.feature_id_;

    FrameId smallest_frame_id = ordered_frame_ids.front();
    FrameId largest_frame_id = ordered_frame_ids.back();
    if ((last_observed_frame_by_feature_.find(feature_id) ==
         last_observed_frame_by_feature_.end()) ||
        (last_observed_frame_by_feature_.at(feature_id) < largest_frame_id)) {
      last_observed_frame_by_feature_[feature_id] = largest_frame_id;
    }
    if ((first_observed_frame_by_feature_.find(feature_id) ==
         first_observed_frame_by_feature_.end()) ||
        (first_observed_frame_by_feature_.at(feature_id) > smallest_frame_id)) {
      first_observed_frame_by_feature_[feature_id] = smallest_frame_id;
    }
    return factor_id;
  }

  virtual std::optional<RawPose3d<double>> getRobotPose(
      const FrameId &frame_id) const {
    if (robot_poses_.find(frame_id) == robot_poses_.end()) {
      return {};
    }
    RawPose3d<double> pose(*(robot_poses_.at(frame_id).pose_));
    return pose;
  }

  std::pair<FrameId, FrameId> getMinMaxFrameId() const {
    return std::make_pair(min_frame_id_, max_frame_id_);
  }

  virtual void getVisualFeatureEstimates(
      std::unordered_map<FeatureId, Position3d<double>>
  &visual_feature_estimates) const {

  }

 protected:
  /**
   * Extrinsics for each camera.
   */
  std::unordered_map<CameraId, CameraExtrinsics<double>>
      camera_extrinsics_by_camera_;

  /**
   * Intrinsics for each camera.
   */
  std::unordered_map<CameraId, CameraIntrinsicsMat<double>>
      camera_intrinsics_by_camera_;
  FrameId min_frame_id_;
  FrameId max_frame_id_;

  FeatureFactorId min_feature_factor_id_;
  FeatureFactorId max_feature_factor_id_;

  std::unordered_map<FrameId, RobotPoseNode> robot_poses_;

  // For factors involving multiple frames, the value will be stored for each
  // associated frame id
  std::unordered_map<FrameId,
                     std::vector<std::pair<FactorType, FeatureFactorId>>>
      visual_feature_factors_by_frame_;

  std::unordered_map<FeatureFactorId, VisualFeatureFactorType> factors_;

  std::unordered_map<FeatureId, FrameId> last_observed_frame_by_feature_;

  std::unordered_map<FeatureId, FrameId> first_observed_frame_by_feature_;
};

class ReprojectionLowLevelFeaturePoseGraph
    : public virtual LowLevelFeaturePoseGraph<ReprojectionErrorFactor> {
 public:
  ReprojectionLowLevelFeaturePoseGraph(
      const std::unordered_map<CameraId, CameraExtrinsics<double>>
          &camera_extrinsics_by_camera,
      const std::unordered_map<CameraId, CameraIntrinsicsMat<double>>
          &camera_intrinsics_by_camera)
      : LowLevelFeaturePoseGraph<ReprojectionErrorFactor>(
            camera_extrinsics_by_camera, camera_intrinsics_by_camera) {}
  virtual ~ReprojectionLowLevelFeaturePoseGraph() = default;
  virtual bool getFeaturePointers(const FeatureId &feature_id,
                                  double **feature_ptr) override {
    if (feature_positions_.find(feature_id) == feature_positions_.end()) {
      return false;
    }

    *feature_ptr = feature_positions_[feature_id].position_->data();
    return true;
  }

  virtual void getVisualFeatureEstimates(
      std::unordered_map<FeatureId, Position3d<double>>
          &visual_feature_estimates) const override {
    for (const auto &pg_feat_est : feature_positions_) {
      visual_feature_estimates[pg_feat_est.first] =
          *(pg_feat_est.second.position_);
    }
  }

  void addFeature(const FeatureId &feature_id,
                  const Position3d<double> &feature_position) {
    // TODO should we check if a feature with this id already exists?
    feature_positions_[feature_id] = VisualFeatureNode(feature_position);
  }

 protected:
  FeatureId min_feature_id_;
  FeatureId max_feature_id_;
  std::unordered_map<FeatureId, VisualFeatureNode> feature_positions_;
};

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_LOW_LEVEL_FEATURE_POSE_GRAPH_H
