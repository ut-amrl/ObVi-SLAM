#ifndef __VSLAM_TYPES_H__
#define __VSLAM_TYPES_H__

#include <assert.h>

#include <iostream>
#include <unordered_map>
#include <utility>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

namespace vslam_types {

typedef uint64_t CameraId;
typedef uint64_t FrameId;
typedef uint64_t EllipsoidId;

/**
 * A structureless vision feature describes the association between a
 * measurement (pixel), an identified feature (feature_idx), and the camera it
 * was captured by (frame_idx).
 */
struct VisionFeature {
  /**
   * Index of this feature - same as the index of the feature track of which
   * this is a part - stored here for redundancy.
   */
  uint64_t feature_idx;
  /**
   * Index of the frame/camera/robot_pose this feature was acquired at.
   */
  FrameId frame_idx;
  /**
   * Camera pixel location of feature by the camera id that captured them.
   *
   * There must be an entry here for the primary camera id.
   */
  std::unordered_map<CameraId, Eigen::Vector2f> pixel_by_camera_id;
  /**
   * Id of the primary camera. This one should be used if only one camera is
   * used.
   */
  CameraId primary_camera_id;
  /**
   * Default constructor: do nothing.
   */
  VisionFeature(){};
  /**
   * Convenience constructor: initialize everything.
   */
  VisionFeature(
      const uint64_t& feature_idx,
      const FrameId & frame_idx,
      const std::unordered_map<CameraId, Eigen::Vector2f>& pixel_by_camera_id,
      const CameraId& primary_camera_id)
      : feature_idx(feature_idx),
        frame_idx(frame_idx),
        pixel_by_camera_id(pixel_by_camera_id),
        primary_camera_id(primary_camera_id) {}
  /**
   * Convenience override of ostream.
   */
  friend std::ostream& operator<<(std::ostream& o, const VisionFeature& f) {
    o << "feature_idx: " << f.feature_idx << "\tframe_idx: " << f.frame_idx
      << "\tPrimary camera id: " << f.primary_camera_id
      << "\tPixels by camera id: ";
    for (const auto& pixel_camera_id_pair : f.pixel_by_camera_id) {
      o << "("
        << "Camera id:" << pixel_camera_id_pair.first
        << ", pixel: " << pixel_camera_id_pair.second.x() << ", "
        << pixel_camera_id_pair.second.y() << "),";
    }
    return o;
  }
  /**
   * Override of operator<() - compares two features based on the
   * frame_idx - assumes that frame_idxs can be sorted to be sequential and
   * always increasing- this overload allows us to sort feature tracks using
   * std::sort
   */
  friend bool operator<(const VisionFeature& lhs, const VisionFeature& rhs) {
    // Assert that features being compared are in same track - comparing
    // features across tracks in this manner doesn't have a very intuitive
    // meaning
    assert(lhs.feature_idx == rhs.feature_idx);
    return lhs.frame_idx < rhs.frame_idx;
  }
};

/**
 * Structureless vision feature track.
 */
struct VisionFeatureTrack {
  /**
   * Index of this feature track - should never be changed once created - each
   * member feature in the track will also have this redundantly.
   */
  uint64_t feature_idx;
  /**
   * The track of feature matches.
   */
  std::vector<VisionFeature> track;
  /**
   * Default constructor: do nothing.
   */
  VisionFeatureTrack() {}
  /**
   * Convenience constructor: initialize everything.
   */
  VisionFeatureTrack(
      const uint64_t& feature_idx,
      const std::vector<VisionFeature>& track = std::vector<VisionFeature>())
      : feature_idx(feature_idx), track(track){};
  /**
   *
   */
  // Convenience constructor: initialize with new seed feature
  VisionFeatureTrack(const VisionFeature& feature)
      : feature_idx(feature.feature_idx) {
    track.push_back(feature);
  }
};

/**
 * Structured vision feature track. In addition to a track feature_idx there is
 * a 3D point associated with it as well.
 */

struct StructuredVisionFeatureTrack {
  /**
   * 3D coordinate of the feature tracked by the feature track.
   */
  Eigen::Vector3d point;
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
  StructuredVisionFeatureTrack(const Eigen::Vector3d& point,
                               const VisionFeatureTrack& feature_track)
      : point(point), feature_track(feature_track){};
};

/**
 * The robot pose at which the frame was acquired in. A robot to camera
 * describes the static transformation from the robot base link to camera frame
 */
struct RobotPose {
  /**
   * Index of the robot pose/frame - each feature capture at each time step has
   * the frame_idx of the robot pose it was captured at.
   */
  FrameId frame_idx;
  /**
   * Robot location.
   */
  Eigen::Vector3f loc;
  /**
   * Robot angle: rotates points from robot frame to global.
   */
  Eigen::AngleAxisf angle;
  /**
   * Default constructor:
   */
  RobotPose(){};
  /**
   * Convenience constructor: initialize everything.
   */
  RobotPose(const FrameId& frame_idx,
            const Eigen::Vector3f& loc,
            const Eigen::AngleAxisf& angle)
      : frame_idx(frame_idx), loc(loc), angle(angle) {}
  /**
   * Return a transform from the robot to the world frame for this pose.
   */
  Eigen::Affine3f RobotToWorldTF() const {
    return (Eigen::Translation3f(loc) * angle);
  }
  /**
   * Return a transform from the world to the robot frame for this pose.
   */
  Eigen::Affine3f WorldToRobotTF() const {
    return ((Eigen::Translation3f(loc) * angle).inverse());
  }
  /**
   * Convenience override of ostream.
   */
  friend std::ostream& operator<<(std::ostream& o,
                                  const vslam_types::RobotPose& p) {
    o << "frame_idx: " << p.frame_idx << "\tloc: " << p.loc.x() << " "
      << p.loc.y() << " " << p.loc.z() << "\tangle"
      << " " << p.angle.angle() << " " << p.angle.axis().x() << " "
      << p.angle.axis().y() << " " << p.angle.axis().z();
    return o;
  }
};

/**
 * Ellipsoid estimate data structure. This one should not be used during
 * optimization because it is harder to update.
 */
struct EllipsoidEstimate {
//  /**
//   * Index of the ellipsoid. Indices should be consecutive and index should
//   * match index of the ellipsoid in the list.
//   */
//  uint64_t ellipsoid_idx;
  /**
   * Location of the ellipsoid's center.
   */
  Eigen::Vector3f loc;
  /**
   * Ellipsoid orientation: rotates points from ellipsoid frame to global.
   */
  Eigen::AngleAxisf orientation;
  /**
   * Dimension of the ellipsoid along each of the major axes.
   */
  Eigen::Vector3f ellipsoid_dim;
//  /**
//   * Semantic class of the ellipsoid.
//   *
//   * TODO should this be an index instead of a string? That's more efficient,
//   * but we don't have a great idea of the classes at the moment.
//   */
//  std::string semantic_class;

  /**
   * Convenience constructor: initialize everything.
   */
  EllipsoidEstimate(const Eigen::Vector3f& loc,
                    const Eigen::AngleAxisf& orientation,
                    const Eigen::Vector3f& ellipsoid_dim
//                    ,
//                    const std::string& semantic_class
  )
//                    ,
//                    const uint64_t& ellipsoid_idx)
      : loc(loc),
        orientation(orientation),
        ellipsoid_dim(ellipsoid_dim)
//        ,
//        semantic_class(semantic_class)
//                          ,
//        ellipsoid_idx(ellipsoid_idx)
          {}
};

/**
 * SLAM Node that can be updated during optimization.
 */
struct SLAMNode {
  /**
   * Node index.
   */
  uint64_t node_idx;

  /**
   * 6DOF parameters: tx, ty, tx, angle_x, angle_y, angle_z. Note that angle_*
   * are the coordinates in scaled angle-axis form.
   */
  double pose[6];

  /**
   * Default constructor.
   */
  SLAMNode() = default;

  /**
   * Constructor that takes in index, translation, and rotation details for the
   * node.
   *
   * @param index           Index of the node.
   * @param pose_transl     Translation of the node.
   * @param pose_rot        Rotation of the node.
   */
  SLAMNode(const uint64_t& index,
           const Eigen::Vector3f& pose_transl,
           const Eigen::AngleAxisf& pose_rot) {
    node_idx = index;
    pose[0] = pose_transl.x();
    pose[1] = pose_transl.y();
    pose[2] = pose_transl.z();
    pose[3] = pose_rot.axis().x() * pose_rot.angle();
    pose[4] = pose_rot.axis().y() * pose_rot.angle();
    pose[5] = pose_rot.axis().z() * pose_rot.angle();
  }
};

/**
 * Ellispoid Estimate Node that can be updated during optimization.
 */
struct EllipsoidEstimateNode {
  /**
   * Index of the ellipsoid. Indices should be consecutive and index should
   * match index of the ellipsoid in the list..
   */
  EllipsoidId ellipsoid_idx;

  /**
   * 9DOF parameters: tx, ty, tx, angle_x, angle_y, angle_z, dim_x, dim_y,
   * dim_z. Note that angle_* are the coordinates in scaled angle-axis form.
   */
  double pose[9];

//  /**
//   * TODO should this be here? It's needed during optimization to determine
//   * which POM to use (since there's a different one per semantic class), but
//   * isn't changed during optimization
//   */
//  std::string semantic_class;

  /**
   * Default constructor.
   *
   * TODO do we need the default constructor or should we just remove it?
   */
  EllipsoidEstimateNode() = default;

  /**
   * Constructor that takes in translation, rotation, dimension, semantic class
   * and ellipsoid index details for the node.
   *
   * @param pose_transl     Translation of the node.
   * @param pose_rot        Rotation of the node.
   * @param ellipsoid_dim   Dimension of the ellipsoid (x length, y length,
   *                        z length).
   * @param semantic_class  Semantic class for the ellipsoid.
   * @param ellipsoid_idx   Ellipsoid index.
   */
  EllipsoidEstimateNode(const Eigen::Vector3f& pose_transl,
                        const Eigen::AngleAxisf& pose_rot,
                        const Eigen::Vector3f& ellipsoid_dim,
//                        const std::string& semantic_class,
                        const EllipsoidId& ellipsoid_idx)
      :
//        semantic_class(semantic_class),
        ellipsoid_idx(ellipsoid_idx) {
    pose[0] = pose_transl.x();
    pose[1] = pose_transl.y();
    pose[2] = pose_transl.z();
    pose[3] = pose_rot.axis().x() * pose_rot.angle();
    pose[4] = pose_rot.axis().y() * pose_rot.angle();
    pose[5] = pose_rot.axis().z() * pose_rot.angle();
    pose[6] = ellipsoid_dim.x();
    pose[7] = ellipsoid_dim.y();
    pose[8] = ellipsoid_dim.z();
  }
};

/**
 * Data structure representing the bounding box detection of an object in an
 * image.
 */
struct RawImageBoundingBoxDetection {
  /**
   * Pixel coordinates of the two opposite corners that define the bounding box
   * of an object within an image. The first of the pair should have the smaller
   * x and y values.
   */
  std::pair<Eigen::Vector2f, Eigen::Vector2f> pixel_corner_locations;

  /**
   * Semantic class of the detected bounding box
   *
   * TODO should this instead be an index? Should it store a set of
   * possible semantic classes with their likelihood?
   */
  std::string semantic_class;

  /**
   * Index of the frame/camera/robot_pose this bounding box was acquired at.
   */
  FrameId frame_idx;

  /**
   * Id of the camera that captured this boundign box.
   */
  CameraId camera_id;
};

template <typename AppearanceInfo>
struct BoundingBoxDetectionWithAppearanceInfo {
  AppearanceInfo appearance_info;

  RawImageBoundingBoxDetection bounding_box;
};

template <typename AppearanceInfo>
struct AssociatedBoundingBoxDetectionWithAppearanceInfo {
  /**
   * Index of the ellipsoid that this bounding box corresponds to.
   */
  EllipsoidId ellipsoid_idx;

  /**
   * Bounding box detection with appearance info.
   */
  BoundingBoxDetectionWithAppearanceInfo<AppearanceInfo> bounding_box_detection;
};

/**
 * Pinhole camera intrinsics parameters.
 */
struct CameraIntrinsics {
  /**
   * Camera matrix.
   */
  Eigen::Matrix3f camera_mat;

  uint32_t image_width;

  uint32_t image_height;
};

/**
 * The camera extrinsics consists of the coordinate transform from the
 * camera to the robot pose. That is, it consists of the translation and
 * rotation that takes a point from the camera frame to the robot frame.
 * In other words, provides the location of the camera in the robot frame.
 */
struct CameraExtrinsics {
  /**
   * 3D vector of translation.
   */
  Eigen::Vector3f translation;

  /**
   * Rotation Quaternion form.
   */
  Eigen::Quaternionf rotation;
};

/**
 * A UT SLAM problem templated by feature track type (stuctureless or
 * structured)
 */
template <typename FeatureTrackType>
struct UTSLAMProblem {
  /**
   * Unordered map representing the database of tracks - indexed by
   * track/feature ID.
   */
  std::unordered_map<uint64_t, FeatureTrackType> tracks;
  /**
   * TODO: I think it'd be good to either convert this to a map or make sure
   *  the poses are stored in order of their indices so we can just do
   *  robot_poses[index] to get the pose
   * Robot/frame poses of the entire trajectory
   */
  std::vector<RobotPose> robot_poses;
  /**
   * Extrinsics for each camera.
   */
  std::unordered_map<CameraId, CameraExtrinsics> camera_extrinsics_by_camera;
  /**
   * Intrinsics for each camera.
   */
  std::unordered_map<CameraId, CameraIntrinsics> camera_intrinsics_by_camera;
  /**
   * Default constructor: do nothing.
   */
  UTSLAMProblem() {}
  /**
   * Convenience constructor: initialize everything.
   */
  UTSLAMProblem(std::unordered_map<uint64_t, FeatureTrackType> const& tracks,
                const std::vector<RobotPose>& robot_poses,
                const std::unordered_map<CameraId, CameraExtrinsics>&
                    camera_extrinsics_by_camera,
                const std::unordered_map<CameraId, CameraIntrinsics>&
                    camera_intrinsics_by_camera)
      : tracks(tracks),
        robot_poses(robot_poses),
        camera_extrinsics_by_camera(camera_extrinsics_by_camera),
        camera_intrinsics_by_camera(camera_intrinsics_by_camera) {}
};

template <typename AppearanceInfo>
struct ObjectInfo {

  /**
   * Ellipsoid estimate. If we don't have enough information to initialize it,
   */
  std::optional<EllipsoidEstimate> ellipsoid_estimate;

  // TODO should this be here or in the ellispoid estimate?
  /**
   * Semantic class of the ellipsoid.
   *
   * TODO should this be an index instead of a string? That's more efficient,
   * but we don't have a great idea of the classes at the moment.
   */
  std::string semantic_class;

  std::unordered_map<FrameId, std::unordered_map<CameraId, AssociatedBoundingBoxDetectionWithAppearanceInfo<AppearanceInfo>>> bounding_box_detections;
};

/**
 * Extension of the SLAM problem that also approximates ellipsoids for objects
 * in the scene based on bounding box detections of objects in images.
 *
 * @tparam FeatureTrackType Type of the feature track to use in performing
 * low-level visual SLAM.
 */
template <typename FeatureTrackType, typename AppearanceInfo>
struct UTObjectSLAMProblem : public UTSLAMProblem<FeatureTrackType> {
//  /**
//   * List of object detections (as bounding boxes).
//   *
//   * TODO do we want to store these by their frame index?
//   */
//  std::vector<ObjectImageBoundingBoxDetection> bounding_boxes;
//
//  /**
//   * Estimates of the ellipsoids for objects in the scene. Order should match
//   * the ids for each ellipsoid.
//   */
//  std::vector<EllipsoidEstimate> ellipsoid_estimates;

  std::unordered_map<EllipsoidId, ObjectInfo<AppearanceInfo>> object_info_;

  // TODO do we need a data structure in here for the class-specific dimension
  //  priors?
  // TODO do we need a data structure in here for covariances for various terms
  // TODO should the SLAM problem contain a list of the useful semantic classes
  //  or should we assume that the detections have already been filtered to
  //  contain only the relevant classes.
  // TODO Should we require initial estimates for the ellipsoids or create the
  // estimates on the fly?

  /**
   * Default constructor: do nothing.
   */
  UTObjectSLAMProblem() : UTSLAMProblem<FeatureTrackType>() {}
  /**
   * Convenience constructor: initialize everything.
   *
   * TODO do we want to get initial estimates for the ellipsoid or should we
   * initialize online?
   */
  UTObjectSLAMProblem(
      std::unordered_map<uint64_t, FeatureTrackType> const& tracks,
      const std::vector<RobotPose>& robot_poses,
      const std::unordered_map<EllipsoidId, ObjectInfo<AppearanceInfo>>& object_info)
      : UTSLAMProblem<FeatureTrackType>(tracks, robot_poses),
        object_info_(object_info) {}
};

}  // namespace vslam_types

#endif  // __VSLAM_TYPES_H__