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

/**
 * Threshold for a small angle when creating a axis-angle representation.
 */
const double kSmallAngleThreshold = 1e-8;

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
  uint64_t frame_idx;
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
      const uint64_t& frame_idx,
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
  uint64_t frame_idx;
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
  RobotPose(const uint64_t& frame_idx,
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
  /**
   * Semantic class of the ellipsoid.
   *
   * TODO should this be an index instead of a string? That's more efficient,
   * but we don't have a great idea of the classes at the moment.
   */
  std::string semantic_class;

  /**
   * Convenience constructor: initialize everything.
   */
  EllipsoidEstimate(const Eigen::Vector3f& loc,
                    const Eigen::AngleAxisf& orientation,
                    const Eigen::Vector3f& ellipsoid_dim,
                    const std::string& semantic_class)
      : loc(loc),
        orientation(orientation),
        ellipsoid_dim(ellipsoid_dim),
        semantic_class(semantic_class) {}
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
  uint64_t ellipsoid_idx;

  /**
   * 9DOF parameters: tx, ty, tx, angle_x, angle_y, angle_z, dim_x, dim_y,
   * dim_z. Note that angle_* are the coordinates in scaled angle-axis form.
   */
  double pose[9];

  /**
   * TODO should this be here? It's needed during optimization to determine
   * which POM to use (since there's a different one per semantic class), but
   * isn't changed during optimization
   */
  std::string semantic_class;

  /**
   * Default constructor.
   *
   * TODO do we need the default constructor or should we just remove it?
   */
  EllipsoidEstimateNode() = default;

  /**
   * Constructor that takes in translation, rotation, and dimension details for
   * the node.
   *
   * @param pose_transl     Translation of the node.
   * @param pose_rot        Rotation of the node.
   * @param ellipsoid_dim   Dimension of the ellipsoid (x length, y length,
   *                        z length).
   */
  EllipsoidEstimateNode(const Eigen::Vector3f& pose_transl,
                        const Eigen::AngleAxisf& pose_rot,
                        const Eigen::Vector3f& ellipsoid_dim,
                        const std::string& semantic_class)
      : semantic_class(semantic_class) {
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
struct ObjectImageBoundingBoxDetection {
  /**
   * Index of the ellipsoid that this bounding box corresponds to.
   */
  uint64_t ellipsoid_idx;

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
  uint64_t frame_idx;

  /**
   * Id of the camera that captured this boundign box.
   */
  CameraId camera_id;
};

/**
 * Pinhole camera intrinsics parameters.
 */
struct CameraIntrinsics {
  /**
   * Camera matrix.
   */
  Eigen::Matrix3f camera_mat;
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
  std::unordered_map<CameraId, CameraIntrinsics> camera_instrinsics_by_camera;
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
        camera_instrinsics_by_camera(camera_intrinsics_by_camera) {}
};

/**
 * Extension of the SLAM problem that also approximates ellipsoids for objects
 * in the scene based on bounding box detections of objects in images.
 *
 * @tparam FeatureTrackType Type of the feature track to use in performing
 * low-level visual SLAM.
 */
template <typename FeatureTrackType>
struct UTObjectSLAMProblem : public UTSLAMProblem<FeatureTrackType> {
  /**
   * List of object detections (as bounding boxes).
   *
   * TODO do we want to store these by their frame index?
   */
  std::vector<ObjectImageBoundingBoxDetection> bounding_boxes;

  /**
   * Estimates of the ellipsoids for objects in the scene. Order should match
   * the ids for each ellipsoid.
   */
  std::vector<EllipsoidEstimate> ellipsoid_estimates;

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
      const std::vector<EllipsoidEstimate>& ellipsoid_estimates)
      : UTSLAMProblem<FeatureTrackType>(tracks, robot_poses),
        ellipsoid_estimates(ellipsoid_estimates) {}
};

// TODO  Move these functions to a utility file

/**
 * Convert from a vector that stores the axis-angle representation (with
 * angle as the magnitude of the vector) to the Eigen AxisAngle
 * representation.
 *
 * @tparam T                Type of each field.
 * @param axis_angle_vec    Vector encoding the axis of rotation (as the
 *                          direction) and the angle as the magnitude of the
 *                          vector.
 *
 * @return Eigen AxisAngle representation for the rotation.
 */
template <typename T>
Eigen::AngleAxis<T> VectorToAxisAngle(
    const Eigen::Matrix<T, 3, 1> axis_angle_vec) {
  const T rotation_angle = axis_angle_vec.norm();
  if (rotation_angle > kSmallAngleThreshold) {
    return Eigen::AngleAxis<T>(rotation_angle, axis_angle_vec / rotation_angle);
  } else {
    return Eigen::AngleAxis<T>(T(0), Eigen::Matrix<T, 3, 1>{T(1), T(0), T(0)});
  }
}

/**
 * Create an Eigen Affine transform from the rotation and translation.
 *
 * @tparam T            Type to use in the matrix.
 * @param rotation      Three entry array containing the axis-angle form of
 * the rotation. Magnitude gives the angle of the rotation and the direction
 * gives the axis of rotation.
 * @param translation   Three entry array containing the translation.
 *
 * @return Eigen Affine transform for the rotation and translation.
 */
template <typename T>
Eigen::Transform<T, 3, Eigen::Affine> PoseArrayToAffine(const T* rotation,
                                                        const T* translation) {
  const Eigen::Matrix<T, 3, 1> rotation_axis(
      rotation[0], rotation[1], rotation[2]);
  const T rotation_angle = rotation_axis.norm();

  Eigen::AngleAxis<T> rotation_aa;
  if (rotation_angle < T(kSmallAngleThreshold)) {
    rotation_aa =
        Eigen::AngleAxis<T>(T(0), Eigen::Matrix<T, 3, 1>(T(0), T(0), T(1)));
  } else {
    rotation_aa = VectorToAxisAngle(rotation_axis);
  }

  const Eigen::Translation<T, 3> translation_tf(
      translation[0], translation[1], translation[2]);
  const Eigen::Transform<T, 3, Eigen::Affine> transform =
      translation_tf * rotation_aa;
  return transform;
}
}  // namespace vslam_types

#endif  // __VSLAM_TYPES_H__