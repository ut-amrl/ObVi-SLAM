#ifndef __VSLAM_TYPES_H__
#define __VSLAM_TYPES_H__

#include <assert.h>

#include <iostream>
#include <utility>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

namespace vslam_types {

// Templated for the type of descriptor used
template <typename T>
struct VisionFeature {
  // Index of this feature - same as the index of the feature track of which
  // this is a part - stored here for redundancy
  uint64_t feature_idx;
  // Index of the frame this feature was acquired in
  uint64_t frame_idx;
  // The descriptor of the feature from the frame it was imaged in - the
  // descriptor will potentially change slightly over the course of the
  // feature track
  T descriptor;
  // Camera pixel location of feature.
  Eigen::Vector2f pixel;
  // Estimated 3d location in the camera frame - not used directly in
  // structureless slam but left here for completeness and later use
  Eigen::Vector3f point3d;
  // Default constructor: do nothing.
  VisionFeature();
  // Convenience constructor: initialize everything.
  VisionFeature(uint64_t const feature_idx,
                uint64_t const frame_idx,
                T const descriptor,
                Eigen::Vector2f const& pixel,
                Eigen::Vector3f const& point3d = Eigen::Vector3f())
      : feature_idx(feature_idx),
        frame_idx(frame_idx),
        descriptor(descriptor),
        pixel(pixel),
        point3d(point3d) {}
  // Convenience override of ostream - Each descriptor will need to have its
  // own overridden operator
  friend std::ostream& operator<<(std::ostream& o, const VisionFeature<T>& f) {
    o << "feature_idx: " << f.feature_idx << "\tframe_idx: " << f.frame_idx
      << "\tdescriptor: " << f.descriptor << "\tpixel: " << f.pixel.x() << " "
      << f.pixel.y() << "\tpoint3d: " << f.point3d.x() << " " << f.point3d.y()
      << " " << f.point3d.z() << std::endl;
    return o;
  }
  // Override of less than operator - compares two features based on the
  // frame_idx - assumes that frame_idxs are sequential and always increasing-
  // this overload allows us to sort feature tracks
  friend bool operator<(VisionFeature<T> const& lhs,
                        VisionFeature<T> const& rhs) {
    // Assert that features being compared are in same track - comparing
    // features across tracks in this manner doesn't have a very intuitive
    // meaning
    assert(lhs.feature_idx == rhs.feature_idx);
    return lhs.frame_idx < rhs.frame_idx ? true : false;
  }
};

// Templated for the type of descriptor used
template <typename T>
struct VisionFeatureTrack {
  // Index of this feature track - should never be changed once created - each
  // member feature in the track will also have this redundantly
  uint64_t feature_idx;

  // The track of feature matches - private - only allow controlled feature
  // addition and no feature subtraction
  std::vector<VisionFeature<T>> track;
  // Sort feature track so frame_idxs are in ascending order
  void sort() {
    std::sort(track.begin(), track.end());
    return;
  }
  // Default constructor: removed - must construct feature track with const
  // feature_idx
  VisionFeatureTrack(uint64_t const feature_idx) : feature_idx(feature_idx) {}
  // Convenience constructor: initialize everything.
  VisionFeatureTrack(
      uint64_t const feature_idx,
      std::vector<VisionFeature<T>> track = std::vector<VisionFeature<T>>())
      : feature_idx(feature_idx), track(track){};
  // Convenience constructor: initialize with new seed feature
  VisionFeatureTrack(VisionFeature<T> feature)
      : feature_idx(feature.feature_idx) {
    track.push_back(feature);
  }
};

// Templated for the type of descriptor used
template <typename T>
struct TrackDatabase {
  // All frame IDs - each tracks individual frame IDs will be a subset of
  // these
  std::vector<uint64_t> frame_idxs;
  // All feature tracks
  std::vector<VisionFeatureTrack<T>> feature_tracks;
  // Add feature to track database
  void addFeature(VisionFeature<T> feature) {
    // If feature track exists add the feature
    for (auto& ft : feature_tracks) {
      if (ft.feature_idx == feature.feature_idx) {
        ft.track.push_back(feature);
        return;  // Exit early if we already found the track we were looking
                 // for
      }
    }

    // If feature track doesn't exist seed a new one with the feature
    feature_tracks.push_back(VisionFeatureTrack<int>(feature));

    return;
  }
  // Default constructor: do nothing.
  TrackDatabase(){};
  // Convenience constructor: initialize everything.
  TrackDatabase(std::vector<uint64_t> frame_idxs,
                std::vector<VisionFeatureTrack<T>> feature_tracks)
      : frame_idxs(frame_idxs), feature_tracks(feature_tracks){};
};

struct RobotPose {
  // Index of the frame this feature was acquired in
  uint64_t const frame_idx;
  // Frame/robot location
  Eigen::Vector3f loc;
  // Frame/robot angle: rotates points from robot frame to global.
  Eigen::AngleAxisf angle;
  // Default constructor: initialize frame_idx
  RobotPose(uint64_t const frame_idx) : frame_idx(frame_idx){};
  // Convenience constructor: initialize everything.
  RobotPose(uint64_t const frame_idx,
            const Eigen::Vector3f& loc,
            const Eigen::AngleAxisf& angle)
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
                                  const vslam_types::RobotPose& p) {
    o << "frame_idx: " << p.frame_idx << "\tloc: " << p.loc.x() << " "
      << p.loc.y() << " " << p.loc.z() << "\tangle"
      << " " << p.angle.angle() << " " << p.angle.axis().x() << " "
      << p.angle.axis().y() << " " << p.angle.axis().z() << std::endl;
    return o;
  }
};

// Templated for the type of descriptor used
template <typename T>
struct UTSLAMProblem {
  TrackDatabase<T> track_database;

  std::vector<RobotPose> robot_poses;

  // Default constructor: do nothing.
  UTSLAMProblem() {}
  // Convenience constructor: initialize everything.
  UTSLAMProblem(TrackDatabase<T> track_database,
                std::vector<RobotPose> robot_poses)
      : track_database(track_database), robot_poses(robot_poses) {}
};
}  // namespace vslam_types

#endif  // __VSLAM_TYPES_H__