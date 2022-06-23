//
// Created by amanda on 6/17/22.
//

#ifndef UT_VSLAM_OFFLINE_PROBLEM_DATA_H
#define UT_VSLAM_OFFLINE_PROBLEM_DATA_H

#include <refactoring/types/vslam_obj_opt_types_refactor.h>

namespace vslam_types_refactor {

FrameId getMaxFrame(const std::unordered_map<FrameId, Pose3D<double>>& poses) {
  CHECK(!poses.empty());
  FrameId max_frame_id = 0;
  for (const auto& frame_and_pose : poses) {
    max_frame_id = std::max(max_frame_id, frame_and_pose.first);
  }
  return max_frame_id;
}

struct VisionFeature {
  FrameId frame_id_;

  /**
   * Camera pixel location of feature by the camera id that captured them.
   *
   * There must be an entry here for the primary camera id.
   */
  std::unordered_map<CameraId, PixelCoord<double>> pixel_by_camera_id;
  /**
   * Id of the primary camera. This one should be used if only one camera is
   * used.
   */
  CameraId primary_camera_id;

  /**
   * Convenience constructor: initialize everything.
   */
  VisionFeature(const FrameId& frame_id,
                const std::unordered_map<CameraId, PixelCoord<double>>&
                    pixel_by_camera_id,
                const CameraId& primary_camera_id)
      : frame_id_(frame_id),
        pixel_by_camera_id(pixel_by_camera_id),
        primary_camera_id(primary_camera_id) {}
};

struct VisionFeatureTrack {
  FeatureId feature_id_;

  // Assumes this is sorted by frame id
  std::vector<VisionFeature> feature_observations_;

  /**
   * Default constructor: do nothing.
   */
  VisionFeatureTrack() {}
  /**
   * Convenience constructor: initialize everything.
   */
  VisionFeatureTrack(const FeatureId& feature_id,
                     const std::vector<VisionFeature>& feature_observations =
                         std::vector<VisionFeature>())
      : feature_id_(feature_id), feature_observations_(feature_observations){};
};

/**
 * Structured vision feature track. In addition to a track feature_idx there is
 * a 3D point associated with it as well.
 */

struct StructuredVisionFeatureTrack {
  /**
   * 3D coordinate of the feature tracked by the feature track.
   */
  Position3d<double> feature_pos_;
  /**
   * Image feature track - same as the structureless feature track.
   * */
  VisionFeatureTrack feature_track;
  /**
   * Default constructor: Do nothing.
   */
  StructuredVisionFeatureTrack() {}
  /**
   * Convenience constructor: initialize everything.
   */
  StructuredVisionFeatureTrack(const Position3d<double>& feature_pos,
                               const VisionFeatureTrack& feature_track)
      : feature_pos_(feature_pos), feature_track(feature_track){};
};

template <typename FeatureTrackType>
class AbstractOfflineProblemData {
 public:
  AbstractOfflineProblemData(
      const std::unordered_map<CameraId, CameraIntrinsicsMat<double>>&
          camera_intrinsics_by_camera,
      const std::unordered_map<CameraId, CameraExtrinsics<double>>&
          camera_extrinsics_by_camera,
      const std::unordered_map<FeatureId, FeatureTrackType>& visual_features,
      const std::unordered_map<FrameId, Pose3D<double>>& robot_poses,
      const std::unordered_map<
          std::string,
          std::pair<ObjectDim<double>, Covariance<double, 3>>>&
          mean_and_cov_by_semantic_class)
      : camera_intrinsics_by_camera_(camera_intrinsics_by_camera),
        camera_extrinsics_by_camera_(camera_extrinsics_by_camera),
        visual_features_(visual_features),
        robot_poses_(robot_poses),
        mean_and_cov_by_semantic_class_(mean_and_cov_by_semantic_class),
        max_frame_id_(getMaxFrame(robot_poses)) {}

  virtual ~AbstractOfflineProblemData() = default;
  virtual FrameId getMaxFrameId() const { return max_frame_id_; }

  virtual std::unordered_map<CameraId, CameraIntrinsicsMat<double>>
  getCameraIntrinsicsByCamera() const {
    return camera_intrinsics_by_camera_;
  }

  virtual std::unordered_map<CameraId, CameraExtrinsics<double>>
  getCameraExtrinsicsByCamera() const {
    return camera_extrinsics_by_camera_;
  }

  virtual std::unordered_map<
      std::string,
      std::pair<ObjectDim<double>, Covariance<double, 3>>>
  getObjDimMeanAndCovByClass() const {
    return mean_and_cov_by_semantic_class_;
  }

  virtual std::unordered_map<FrameId, Pose3D<double>> getRobotPoseEstimates()
      const {
    return robot_poses_;
  }

  virtual bool getRobotPoseEstimateForFrame(const FrameId& frame_id,
                                            Pose3D<double>& est_out) const {
    if (robot_poses_.find(frame_id) == robot_poses_.end()) {
      return false;
    }
    est_out = robot_poses_.at(frame_id);
    return true;
  }

  // TODO figure out some intermediate storage so that we can store this at
  // least sort of by frame id
  virtual std::unordered_map<FeatureId, FeatureTrackType> getVisualFeatures()
      const {
    return visual_features_;
  };

  virtual std::unordered_map<
      FrameId,
      std::unordered_map<CameraId, std::vector<RawBoundingBox>>>
  getBoundingBoxes() const = 0;

 protected:
  std::unordered_map<CameraId, CameraIntrinsicsMat<double>>
      camera_intrinsics_by_camera_;
  std::unordered_map<CameraId, CameraExtrinsics<double>>
      camera_extrinsics_by_camera_;
  std::unordered_map<FeatureId, FeatureTrackType> visual_features_;
  std::unordered_map<FrameId, Pose3D<double>> robot_poses_;
  std::unordered_map<std::string,
                     std::pair<ObjectDim<double>, Covariance<double, 3>>>
      mean_and_cov_by_semantic_class_;
  FrameId max_frame_id_;
};

// Keeping bounding box type abstracted because we may want to store some
// appearance information with each bounding box
// Keeping object estimate abstracted in case we want to try different (ex.
// cuboid) object models with different parameters.
template <typename FeatureTrackType, typename ObjectEstimateType>
class AssociatedBoundingBoxOfflineProblemData
    : public AbstractOfflineProblemData<FeatureTrackType> {
 public:
  AssociatedBoundingBoxOfflineProblemData(
      const std::unordered_map<CameraId, CameraIntrinsicsMat<double>>&
          camera_intrinsics_by_camera,
      const std::unordered_map<CameraId, CameraExtrinsics<double>>&
          camera_extrinsics_by_camera,
      const std::unordered_map<FeatureId, FeatureTrackType>& visual_features,
      const std::unordered_map<FrameId, Pose3D<double>>& robot_poses,
      const std::unordered_map<
          std::string,
          std::pair<ObjectDim<double>, Covariance<double, 3>>>&
          mean_and_cov_by_semantic_class,
      const std::unordered_map<
          FrameId,
          std::unordered_map<CameraId,
                             std::vector<std::pair<ObjectId, RawBoundingBox>>>>&
          bounding_boxes,
      const std::unordered_map<ObjectId, ObjectEstimateType>& object_estimates)
      : AbstractOfflineProblemData<FeatureTrackType>(
            camera_intrinsics_by_camera,
            camera_extrinsics_by_camera,
            visual_features,
            robot_poses,
            mean_and_cov_by_semantic_class),
        bounding_boxes_(bounding_boxes),
        object_estimates_(object_estimates) {}
  virtual std::unordered_map<
      FrameId,
      std::unordered_map<CameraId, std::vector<RawBoundingBox>>>
  getBoundingBoxes() const override {
    std::unordered_map<
        FrameId,
        std::unordered_map<CameraId, std::vector<RawBoundingBox>>>
        bbs;
    for (const auto& frame_and_bbs : bounding_boxes_) {
      std::unordered_map<CameraId, std::vector<RawBoundingBox>> bbs_for_frame;
      for (const auto& camera_and_bbs : frame_and_bbs.second) {
        std::vector<RawBoundingBox> bbs_for_cam;
        for (const std::pair<ObjectId, RawBoundingBox>& bb :
             camera_and_bbs.second) {
          bbs_for_cam.emplace_back(bb.second);
        }
        bbs_for_frame[camera_and_bbs.first] = bbs_for_cam;
      }
      bbs[frame_and_bbs.first] = bbs_for_frame;
    }
    return bbs;
  };

 protected:
  // Bounding boxes (by object)
  std::unordered_map<
      FrameId,
      std::unordered_map<CameraId,
                         std::vector<std::pair<ObjectId, RawBoundingBox>>>>
      bounding_boxes_;

  // (Optional) object estimates
  std::unordered_map<ObjectId, ObjectEstimateType> object_estimates_;
};

// Keeping bounding box type abstracted because we may want to store some
// appearance information with each bounding box
template <typename FeatureTrackType, typename ImageType>
class UnassociatedBoundingBoxOfflineProblemData
    : public AbstractOfflineProblemData<FeatureTrackType> {
 public:
  UnassociatedBoundingBoxOfflineProblemData(
      const std::unordered_map<CameraId, CameraIntrinsicsMat<double>>&
          camera_intrinsics_by_camera,
      const std::unordered_map<CameraId, CameraExtrinsics<double>>&
          camera_extrinsics_by_camera,
      const std::unordered_map<FeatureId, FeatureTrackType>& visual_features,
      const std::unordered_map<FrameId, Pose3D<double>>& robot_poses,
      const std::unordered_map<
          std::string,
          std::pair<ObjectDim<double>, Covariance<double, 3>>>&
          mean_and_cov_by_semantic_class,
      const std::unordered_map<
          FrameId,
          std::unordered_map<CameraId, std::vector<RawBoundingBox>>>&
          bounding_boxes,
      const std::unordered_map<FrameId,
                               std::unordered_map<CameraId, ImageType>>& images)
      : AbstractOfflineProblemData<FeatureTrackType>(
            camera_intrinsics_by_camera,
            camera_extrinsics_by_camera,
            visual_features,
            robot_poses,
            mean_and_cov_by_semantic_class),
        bounding_boxes_(bounding_boxes),
        images_(images) {}

  virtual std::unordered_map<
      FrameId,
      std::unordered_map<CameraId, std::vector<RawBoundingBox>>>
  getBoundingBoxes() const override {
    return bounding_boxes_;
  }

  virtual ImageType getImageForFrameAndCamera(const FrameId& frame,
                                              const CameraId& camera) const {
    return (images_.at(frame)).at(camera);
  }

 protected:
  // Need bounding boxes
  std::unordered_map<FrameId,
                     std::unordered_map<CameraId, std::vector<RawBoundingBox>>>
      bounding_boxes_;

  // Need images (by frame id and camera id)
  std::unordered_map<FrameId, std::unordered_map<CameraId, ImageType>> images_;
};

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_OFFLINE_PROBLEM_DATA_H
