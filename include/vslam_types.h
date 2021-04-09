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
  // Robot/frame poses of the entire trajectory
  std::vector<RobotPose> robot_poses;
  // Default constructor: do nothing.
  UTSLAMProblem() {}
  // Convenience constructor: initialize everything.
  UTSLAMProblem(std::unordered_map<uint64_t, VisionFeatureTrack> cosnt& tracks,
                std::vector<RobotPose> const& robot_poses)
      : tracks(tracks), robot_poses(robot_poses) {}
};
}  // namespace vslam_types

#endif  // __VSLAM_TYPES_H__