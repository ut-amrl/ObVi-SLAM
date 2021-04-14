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

struct VisionFeature {
  // Index of this feature - same as the index of the feature track of which
  // this is a part - stored here for redundancy
  uint64_t feature_idx;
  // Index of the frame this feature was acquired in
  uint64_t frame_idx;
  // Camera pixel location of feature.
  Eigen::Vector2f pixel;
  // Default constructor: do nothing.
  VisionFeature(){};
  // Convenience constructor: initialize everything.
  VisionFeature(uint64_t const& feature_idx,
                uint64_t const& frame_idx,
                Eigen::Vector2f const& pixel)
      : feature_idx(feature_idx), frame_idx(frame_idx), pixel(pixel) {}
  // Convenience override of ostream
  friend std::ostream& operator<<(std::ostream& o, VisionFeature const& f) {
    o << "feature_idx: " << f.feature_idx << "\tframe_idx: " << f.frame_idx
      << "\tpixel: " << f.pixel.x() << " " << f.pixel.y();
    return o;
  }
  // Override of less than operator - compares two features based on the
  // frame_idx - assumes that frame_idxs are sequential and always increasing-
  // this overload allows us to sort feature tracks
  friend bool operator<(VisionFeature const& lhs, VisionFeature const& rhs) {
    // Assert that features being compared are in same track - comparing
    // features across tracks in this manner doesn't have a very intuitive
    // meaning
    assert(lhs.feature_idx == rhs.feature_idx);
    return lhs.frame_idx < rhs.frame_idx ? true : false;
  }
};

struct VisionFeatureTrack {
  // Index of this feature track - should never be changed once created - each
  // member feature in the track will also have this redundantly
  uint64_t feature_idx;
  // The track of feature matches - private - only allow controlled feature
  // addition and no feature subtraction
  std::vector<VisionFeature> track;
  // Default constructor: do nothing.
  VisionFeatureTrack() {}
  // Convenience constructor: initialize everything.
  VisionFeatureTrack(
      uint64_t const& feature_idx,
      std::vector<VisionFeature> const& track = std::vector<VisionFeature>())
      : feature_idx(feature_idx), track(track){};
  // Convenience constructor: initialize with new seed feature
  VisionFeatureTrack(VisionFeature const& feature)
      : feature_idx(feature.feature_idx) {
    track.push_back(feature);
  }
};

struct RobotPose {
  // Index of the frame this feature was acquired in
  uint64_t frame_idx;
  // Frame/robot location
  Eigen::Vector3f loc;
  // Frame/robot angle: rotates points from robot frame to global.
  Eigen::AngleAxisf angle;
  // Default constructor: initialize frame_idx
  RobotPose(){};
  // Convenience constructor: initialize everything.
  RobotPose(uint64_t const& frame_idx,
            Eigen::Vector3f const& loc,
            Eigen::AngleAxisf const& angle)
      : frame_idx(frame_idx), loc(loc), angle(angle) {}

  // Return a transform from the robot to the world frame for this pose.
  Eigen::Affine3f RobotToWorldTF() const {
    return (Eigen::Translation3f(loc) * angle);
  }
  // Return a transform from the world to the robot frame for this pose.
  Eigen::Affine3f WorldToRobotTF() const {
    return ((Eigen::Translation3f(loc) * angle).inverse());
  }
  // Convenience override of ostream
  friend std::ostream& operator<<(std::ostream& o,
                                  vslam_types::RobotPose const& p) {
    o << "frame_idx: " << p.frame_idx << "\tloc: " << p.loc.x() << " "
      << p.loc.y() << " " << p.loc.z() << "\tangle"
      << " " << p.angle.angle() << " " << p.angle.axis().x() << " "
      << p.angle.axis().y() << " " << p.angle.axis().z();
    return o;
  }
};

struct UTSLAMProblem {
  // Unordered map representing the database of tracks
  std::unordered_map<uint64_t, VisionFeatureTrack> tracks;

  // TODO: I think it'd be good to either convert this to a map or make sure
  //  the poses are stored in order of their indices so we can just do
  //  robot_poses[index] to get the pose
  // Robot/frame poses of the entire trajectory
  std::vector<RobotPose> robot_poses;
  // Default constructor: do nothing.
  UTSLAMProblem() {}
  // Convenience constructor: initialize everything.
  UTSLAMProblem(std::unordered_map<uint64_t, VisionFeatureTrack> const& tracks,
                std::vector<RobotPose> const& robot_poses)
      : tracks(tracks), robot_poses(robot_poses) {}
};

//
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
 * Convert from a vector that stores the axis-angle representation (with
 * angle as the magnitude of the vector) to the Eigen AxisAngle representation.
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
  return Eigen::AngleAxis<T>(rotation_angle, axis_angle_vec / rotation_angle);
}

/**
 * Create an Eigen Affine transform from the rotation and translation.
 *
 * @tparam T            Type to use in the matrix.
 * @param rotation      Three entry array containing the axis-angle form of the
 *                      rotation. Magnitude gives the angle of the rotation and
 *                      the direction gives the axis of rotation.
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