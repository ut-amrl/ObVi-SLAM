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
static const FactorType kPairwiseRobotPoseFactorTypeId = 5;
static const FactorType kParamPriorFrameFactorTypeId = 6;
static const FactorType kParamPriorFeatFactorTypeId = 7;
static const FactorType kParamPriorObjFactorTypeId = 8;

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

  void updateRobotPoseParams(const RawPose3d<double> &pose) {
    pose_->topRows(6) = pose;
  }

  void updateRobotPoseParams(const RawPose3dPtr<double> &pose_ptr) {
    updateRobotPoseParams(*pose_ptr);
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

  void updateVisualPositionParams(const Position3d<double> &position) {
    position_->topRows(3) = position;
  }

  void updateVisualPositionParams(const Position3dPtr<double> &position_ptr) {
    updateVisualPositionParams(*position_ptr);
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

  ReprojectionErrorFactor() = default;
  ReprojectionErrorFactor(const FrameId &frame_id,
                          const FeatureId &feature_id,
                          const CameraId &camera_id,
                          const PixelCoord<double> &feature_pos,
                          const double &reprojection_error_std_dev)
      : frame_id_(frame_id),
        feature_id_(feature_id),
        camera_id_(camera_id),
        feature_pos_(feature_pos),
        reprojection_error_std_dev_(reprojection_error_std_dev) {}

  FactorType getFactorType() const { return kReprojectionErrorFactorTypeId; }

  std::vector<FrameId> getOrderedFrameIds() const { return {frame_id_}; }

  bool operator==(const ReprojectionErrorFactor &rhs) const {
    return (frame_id_ == rhs.frame_id_) && (feature_id_ == rhs.feature_id_) &&
           (camera_id_ == rhs.camera_id_) &&
           (feature_pos_.x() == rhs.feature_pos_.x()) &&
           (feature_pos_.y() == rhs.feature_pos_.y()) &&
           (reprojection_error_std_dev_ == rhs.reprojection_error_std_dev_);
  }

  bool shouldBeTheSame(const ReprojectionErrorFactor &rhs) const {
    return (frame_id_ == rhs.frame_id_) && (feature_id_ == rhs.feature_id_) &&
           (camera_id_ == rhs.camera_id_);
  }
};

struct RelPoseFactor {
  FrameId frame_id_1_;
  FrameId frame_id_2_;
  Pose3D<double> measured_pose_deviation_;
  Covariance<double, 6> pose_deviation_cov_;

  RelPoseFactor() {}

  RelPoseFactor(const FrameId &frame_id_1,
                const FrameId &frame_id_2,
                const Pose3D<double> &measured_pose_deviation,
                const Covariance<double, 6> &pose_deviation_cov)
      : frame_id_1_(frame_id_1),
        frame_id_2_(frame_id_2),
        measured_pose_deviation_(measured_pose_deviation),
        pose_deviation_cov_(pose_deviation_cov) {}

  FactorType getFactorType() const { return kPairwiseRobotPoseFactorTypeId; }

  std::vector<FrameId> getOrderedFrameIds() const {
    if (frame_id_1_ < frame_id_2_) {
      return {frame_id_1_, frame_id_2_};
    }
    return {frame_id_2_, frame_id_1_};
  }

  bool operator==(const RelPoseFactor &rhs) const {
    return (frame_id_1_ == rhs.frame_id_1_) &&
           (frame_id_2_ == rhs.frame_id_2_) &&
           (measured_pose_deviation_ == rhs.measured_pose_deviation_) &&
           (pose_deviation_cov_ == rhs.pose_deviation_cov_);
  }
};

template <typename VisualFeatureFactorType>
struct LowLevelFeaturePoseGraphState {
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
  FactorType visual_factor_type_;
  FrameId min_frame_id_;
  FrameId max_frame_id_;

  FeatureFactorId max_feature_factor_id_;

  FeatureFactorId max_pose_factor_id_;

  std::unordered_map<FrameId, RawPose3d<double>> robot_poses_;

  std::unordered_map<FrameId,
                     util::BoostHashSet<std::pair<FactorType, FeatureFactorId>>>
      pose_factors_by_frame_;

  // For factors involving multiple frames, the value will be stored for each
  // associated frame id
  std::unordered_map<FrameId,
                     std::vector<std::pair<FactorType, FeatureFactorId>>>
      visual_feature_factors_by_frame_;

  std::unordered_map<FeatureId,
                     util::BoostHashSet<std::pair<FactorType, FeatureFactorId>>>
      visual_factors_by_feature_;

  // TODO takes in generic pose factors
  std::unordered_map<FeatureFactorId, RelPoseFactor> pose_factors_;

  std::unordered_map<FeatureFactorId, VisualFeatureFactorType> factors_;

  std::unordered_map<FeatureId, FrameId> last_observed_frame_by_feature_;

  std::unordered_map<FeatureId, FrameId> first_observed_frame_by_feature_;

  bool operator==(
      const LowLevelFeaturePoseGraphState<VisualFeatureFactorType> &rhs) const {
    return (camera_extrinsics_by_camera_ == rhs.camera_extrinsics_by_camera_) &&
           (camera_intrinsics_by_camera_ == rhs.camera_intrinsics_by_camera_) &&
           (visual_factor_type_ == rhs.visual_factor_type_) &&
           (min_frame_id_ == rhs.min_frame_id_) &&
           (max_frame_id_ == rhs.max_frame_id_) &&
           (max_feature_factor_id_ == rhs.max_feature_factor_id_) &&
           (max_pose_factor_id_ == rhs.max_pose_factor_id_) &&
           (robot_poses_ == rhs.robot_poses_) &&
           (pose_factors_by_frame_ == rhs.pose_factors_by_frame_) &&
           (visual_feature_factors_by_frame_ ==
            rhs.visual_feature_factors_by_frame_) &&
           (visual_factors_by_feature_ == rhs.visual_factors_by_feature_) &&
           (pose_factors_ == rhs.pose_factors_) && (factors_ == rhs.factors_) &&
           (last_observed_frame_by_feature_ ==
            rhs.last_observed_frame_by_feature_) &&
           (first_observed_frame_by_feature_ ==
            rhs.first_observed_frame_by_feature_);
  }
};

struct ReprojectionLowLevelFeaturePoseGraphState {
  LowLevelFeaturePoseGraphState<ReprojectionErrorFactor> low_level_pg_state_;
  FeatureId min_feature_id_;
  FeatureId max_feature_id_;
  std::unordered_map<FeatureId, Position3d<double>> feature_positions_;

  bool operator==(const ReprojectionLowLevelFeaturePoseGraphState &rhs) const {
    return (low_level_pg_state_ == rhs.low_level_pg_state_) &&
           (min_feature_id_ == rhs.min_feature_id_) &&
           (max_feature_id_ == rhs.max_feature_id_) &&
           (feature_positions_ == rhs.feature_positions_);
  }
};

template <typename VisualFeatureFactorType>
class LowLevelFeaturePoseGraph {
 public:
  LowLevelFeaturePoseGraph(
      const std::unordered_map<CameraId, CameraExtrinsics<double>>
          &camera_extrinsics_by_camera,
      const std::unordered_map<CameraId, CameraIntrinsicsMat<double>>
          &camera_intrinsics_by_camera,
      const FactorType &visual_factor_type)
      : camera_extrinsics_by_camera_(camera_extrinsics_by_camera),
        camera_intrinsics_by_camera_(camera_intrinsics_by_camera),
        visual_factor_type_(visual_factor_type),
        min_frame_id_(std::numeric_limits<FrameId>::max()),
        max_frame_id_(std::numeric_limits<FrameId>::min()),
        max_feature_factor_id_(0),
        max_pose_factor_id_(0) {}

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
      util::BoostHashSet<std::pair<FactorType, FeatureFactorId>>
          &matching_factors) {
    for (const auto &frame_and_matching_factors :
         visual_feature_factors_by_frame_) {
      if ((frame_and_matching_factors.first >= min_frame_id) &&
          (frame_and_matching_factors.first <= max_frame_id)) {
        matching_factors.insert(frame_and_matching_factors.second.begin(),
                                frame_and_matching_factors.second.end());
      }
    }
  }

  /**
   * @brief Get the Pose Factor Ids By Frame Id object
   *
   * @param frame_id         [in] query frame id
   * @param min_frame_id     [in] min frame id of the current time window
   * @param max_frame_id     [in] max frame id of the current time window
   * @param matching_factors [out]
   */
  virtual void getPoseFactorInfoByFrameId(
      const FrameId &frame_id,
      const FrameId &min_frame_id,
      const FrameId &max_frame_id,
      util::BoostHashSet<std::pair<FactorType, FeatureFactorId>>
          &matching_factors) {
    for (const auto &factor_type_and_id : pose_factors_by_frame_.at(frame_id)) {
      RelPoseFactor factor = pose_factors_.at(factor_type_and_id.second);
      if (factor.frame_id_1_ >= min_frame_id &&
          factor.frame_id_2_ <= max_frame_id) {
        matching_factors.insert(factor_type_and_id);
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

  virtual bool getPoseFactor(const FeatureFactorId &factor_id,
                             RelPoseFactor &pose_factor) const {
    if (pose_factors_.find(factor_id) == pose_factors_.end()) {
      return false;
    }
    pose_factor = pose_factors_.at(factor_id);
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

    std::pair<FactorType, FeatureFactorId> factor_id_pair =
        std::make_pair(factor.getFactorType(), factor_id);
    for (const FrameId &frame_id : ordered_frame_ids) {
      visual_feature_factors_by_frame_[frame_id].emplace_back(factor_id_pair);
    }

    FeatureId feature_id = factor.feature_id_;
    visual_factors_by_feature_[feature_id].insert(factor_id_pair);

    factors_[factor_id] = factor;

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

  virtual FeatureFactorId addPoseFactor(const RelPoseFactor &factor) {
    const FeatureFactorId factor_id = max_pose_factor_id_ + 1;
    max_pose_factor_id_ = factor_id;

    std::vector<FrameId> ordered_frame_ids = factor.getOrderedFrameIds();
    std::pair<FactorType, FeatureFactorId> factor_id_pair =
        std::make_pair(factor.getFactorType(), factor_id);
    for (const FrameId &frame_id : ordered_frame_ids) {
      pose_factors_by_frame_[frame_id].insert(factor_id_pair);
    }
    pose_factors_[factor_id] = factor;
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
          &visual_feature_estimates) const {}

  bool getFactorsForFeature(
      const FeatureId &feat_id,
      util::BoostHashSet<std::pair<FactorType, FeatureFactorId>> &factors) {
    if (visual_factors_by_feature_.find(feat_id) ==
        visual_factors_by_feature_.end()) {
      return false;
    }

    factors = visual_factors_by_feature_.at(feat_id);
    return true;
  }

  bool getFeatureIdForObservationFactor(
      const std::pair<vslam_types_refactor::FactorType,
                      vslam_types_refactor::FeatureFactorId> &factor_info,
      vslam_types_refactor::FeatureId &feature_id) const {
    if (factor_info.first != visual_factor_type_) {
      LOG(WARNING) << "Visual factor type for this pose graph "
                   << visual_factor_type_ << " didn't match the requested one "
                   << factor_info.first;
      return false;
    }
    if (factors_.find(factor_info.second) == factors_.end()) {
      return false;
    }
    feature_id = factors_.at(factor_info.second).feature_id_;
    return true;
  };

  void setRobotPosePtrs(
      const std::unordered_map<FrameId, RobotPoseNode> &robot_poses) {
    robot_poses_ = robot_poses;
  }

  void getRobotPosePtrs(
      std::unordered_map<FrameId, RobotPoseNode> &robot_poses) const {
    robot_poses = robot_poses_;
  }

  void initializeFromState(
      const LowLevelFeaturePoseGraphState<VisualFeatureFactorType>
          &pose_graph_state) {
    camera_extrinsics_by_camera_ =
        pose_graph_state.camera_extrinsics_by_camera_;
    camera_intrinsics_by_camera_ =
        pose_graph_state.camera_intrinsics_by_camera_;
    visual_factor_type_ = pose_graph_state.visual_factor_type_;
    min_frame_id_ = pose_graph_state.min_frame_id_;
    max_frame_id_ = pose_graph_state.max_frame_id_;
    max_feature_factor_id_ = pose_graph_state.max_feature_factor_id_;
    max_pose_factor_id_ = pose_graph_state.max_pose_factor_id_;
    pose_factors_by_frame_ = pose_graph_state.pose_factors_by_frame_;
    visual_feature_factors_by_frame_ =
        pose_graph_state.visual_feature_factors_by_frame_;
    visual_factors_by_feature_ = pose_graph_state.visual_factors_by_feature_;
    pose_factors_ = pose_graph_state.pose_factors_;
    factors_ = pose_graph_state.factors_;
    last_observed_frame_by_feature_ =
        pose_graph_state.last_observed_frame_by_feature_;
    first_observed_frame_by_feature_ =
        pose_graph_state.first_observed_frame_by_feature_;

    for (const auto &robot_pose : pose_graph_state.robot_poses_) {
      RawPose3d<double> pose(robot_pose.second);
      robot_poses_[robot_pose.first] = RobotPoseNode(pose);
    }
  }

  void getState(LowLevelFeaturePoseGraphState<VisualFeatureFactorType>
                    &pose_graph_state) {
    pose_graph_state.camera_extrinsics_by_camera_ =
        camera_extrinsics_by_camera_;
    pose_graph_state.camera_intrinsics_by_camera_ =
        camera_intrinsics_by_camera_;
    pose_graph_state.visual_factor_type_ = visual_factor_type_;
    pose_graph_state.min_frame_id_ = min_frame_id_;
    pose_graph_state.max_frame_id_ = max_frame_id_;
    pose_graph_state.max_feature_factor_id_ = max_feature_factor_id_;
    pose_graph_state.max_pose_factor_id_ = max_pose_factor_id_;
    pose_graph_state.pose_factors_by_frame_ = pose_factors_by_frame_;
    pose_graph_state.visual_feature_factors_by_frame_ =
        visual_feature_factors_by_frame_;
    pose_graph_state.visual_factors_by_feature_ = visual_factors_by_feature_;
    pose_graph_state.pose_factors_ = pose_factors_;
    pose_graph_state.factors_ = factors_;
    pose_graph_state.last_observed_frame_by_feature_ =
        last_observed_frame_by_feature_;
    pose_graph_state.first_observed_frame_by_feature_ =
        first_observed_frame_by_feature_;

    for (const auto &robot_pose : robot_poses_) {
      pose_graph_state.robot_poses_[robot_pose.first] =
          RawPose3d<double>(*(robot_pose.second.pose_));
    }
  }

 protected:
  LowLevelFeaturePoseGraph<VisualFeatureFactorType>() = default;

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
  FactorType visual_factor_type_;
  FrameId min_frame_id_;
  FrameId max_frame_id_;

  FeatureFactorId max_feature_factor_id_;

  FeatureFactorId max_pose_factor_id_;

  std::unordered_map<FrameId, RobotPoseNode> robot_poses_;

  std::unordered_map<FrameId,
                     util::BoostHashSet<std::pair<FactorType, FeatureFactorId>>>
      pose_factors_by_frame_;

  // For factors involving multiple frames, the value will be stored for each
  // associated frame id
  std::unordered_map<FrameId,
                     std::vector<std::pair<FactorType, FeatureFactorId>>>
      visual_feature_factors_by_frame_;

  std::unordered_map<FeatureId,
                     util::BoostHashSet<std::pair<FactorType, FeatureFactorId>>>
      visual_factors_by_feature_;

  // TODO takes in generic pose factors
  std::unordered_map<FeatureFactorId, RelPoseFactor> pose_factors_;

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
            camera_extrinsics_by_camera,
            camera_intrinsics_by_camera,
            kReprojectionErrorFactorTypeId) {}
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
    visual_factors_by_feature_[feature_id] = {};
  }

  void setFeaturePositionPtrs(
      const std::unordered_map<FeatureId, VisualFeatureNode>
          &feature_positions) {
    feature_positions_ = feature_positions;
  }

  void getFeaturePositionPtrs(std::unordered_map<FeatureId, VisualFeatureNode>
                                  &feature_positions) const {
    feature_positions = feature_positions_;
  }

  void getState(ReprojectionLowLevelFeaturePoseGraphState &pose_graph_state) {
    LowLevelFeaturePoseGraph<ReprojectionErrorFactor>::getState(
        pose_graph_state.low_level_pg_state_);
    pose_graph_state.min_feature_id_ = min_feature_id_;
    pose_graph_state.max_feature_id_ = max_feature_id_;
    for (const auto &feature_pos : feature_positions_) {
      pose_graph_state.feature_positions_[feature_pos.first] =
          Position3d<double>(*(feature_pos.second.position_));
    }
  }

  void initializeFromState(
      const ReprojectionLowLevelFeaturePoseGraphState &pose_graph_state) {
    LowLevelFeaturePoseGraph<ReprojectionErrorFactor>::initializeFromState(
        pose_graph_state.low_level_pg_state_);
    min_feature_id_ = pose_graph_state.min_feature_id_;
    max_feature_id_ = pose_graph_state.max_feature_id_;
    for (const auto &feature_pos : pose_graph_state.feature_positions_) {
      feature_positions_[feature_pos.first] =
          VisualFeatureNode(feature_pos.second);
    }
  }

 protected:
  ReprojectionLowLevelFeaturePoseGraph()
      : LowLevelFeaturePoseGraph<ReprojectionErrorFactor>() {}

  FeatureId min_feature_id_;
  FeatureId max_feature_id_;
  std::unordered_map<FeatureId, VisualFeatureNode> feature_positions_;
};

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_LOW_LEVEL_FEATURE_POSE_GRAPH_H
