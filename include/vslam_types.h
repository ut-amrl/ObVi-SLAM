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
   * Camera pixel location of feature.
   */
  Eigen::Vector2f pixel;
  /**
   * Default constructor: do nothing.
   */
  VisionFeature(){};
  /**
   * Convenience constructor: initialize everything.
   */
  VisionFeature(const uint64_t& feature_idx,
                const uint64_t& frame_idx,
                const Eigen::Vector2f& pixel)
      : feature_idx(feature_idx), frame_idx(frame_idx), pixel(pixel) {}
  /**
   * Convenience override of ostream.
   */
  friend std::ostream& operator<<(std::ostream& o, const VisionFeature& f) {
    o << "feature_idx: " << f.feature_idx << "\tframe_idx: " << f.frame_idx
      << "\tpixel: " << f.pixel.x() << " " << f.pixel.y();
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
    return lhs.frame_idx < rhs.frame_idx ? true : false;
  }
};

/**
 * Represents an IMU or odometry, or fused IMU-odometry observation.
 */
struct OdometryFactor {
  /**
   * ID of first pose.
   */
  uint64_t pose_i;

  /**
   * ID of second pose.
   */
  uint64_t pose_j;

  /**
   * Translation to go from pose i to pose j.
   */
  Eigen::Vector3f translation;

  /**
   * Rotation to go from pose i to pose j.
   */
  Eigen::Quaternionf rotation;

  /**
   * Sqrt information matrix (square root of inverse covariance). First 3
   * entries are translation, second 3 are rotation.
   */
  Eigen::Matrix<double, 6, 6> sqrt_information;

  /**
   * Default constructor: do nothing.
   */
  OdometryFactor() {}

  /**
   * Convenience constructor: initialize everything.
   *
   * @param pose_i              Id of the first pose.
   * @param pose_j              Id of the second pose.
   * @param translation         Measured translation from pose i to pose j.
   * @param rotation            Measured rotation from pose i to pose j.
   * @param sqrt_information    Sqrt information matrix (square root of inverse
   *                            covariance). First 3 entries are translation,
   *                            second 3 are rotation.
   */
  OdometryFactor(const uint64_t& pose_i,
                 const uint64_t& pose_j,
                 const Eigen::Vector3f& translation,
                 const Eigen::Quaternionf& rotation,
                 const Eigen::Matrix<double, 6, 6>& sqrt_information)
      : pose_i(pose_i),
        pose_j(pose_j),
        translation(translation),
        rotation(rotation) {}
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
   * Odometry factors.
   */
  std::vector<OdometryFactor> odom_factors;

  /**
   * Default constructor: do nothing.
   */
  UTSLAMProblem() {}
  /**
   * Convenience constructor: initialize everything.
   */
  UTSLAMProblem(std::unordered_map<uint64_t, FeatureTrackType> const& tracks,
                const std::vector<RobotPose>& robot_poses)
      : tracks(tracks), robot_poses(robot_poses) {}
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