#ifndef __VSLAM_TYPES_H__
#define __VSLAM_TYPES_H__

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
  uint64_t const feature_idx;
  // Index of the frame this feature was acquired in
  uint64_t const frame_idx;
  // The descriptor of the feature from the frame it was imaged in - the
  // descriptor will potentially change slightly over the course of the feature
  // track
  T const descriptor;
  // Camera pixel location of feature.
  Eigen::Vector2f const pixel;
  // Estimated 3d location in the camera frame - not used directly in
  // structureless slam but left here for completeness and later use - the only
  // non const member
  Eigen::Vector3f point3d;
  // Default constructor: removed - should only be constructed when all parts
  // are available - allows us to enforce all parts of the feature being const -
  // except for the point3d location which we may use later/optimize
  VisionFeature() = delete;
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
  // Convenience override of ostream - Each descriptor will need to have its own
  // overridden operator
  friend std::ostream& operator<<(std::ostream& o,
                                  const vslam_types::VisionFeature<T>& f) {
    o << "feature_idx: " << f.feature_idx << "frame_idx: " << f.frame_idx
      << "\tdescriptor: " << f.descriptor << "\tpixel: " << f.pixel.x() << " "
      << f.pixel.y() << "\tpoint3d: " << f.point3d.x() << " " << f.point3d.y()
      << " " << f.point3d.z() << std::endl;
    return o;
  }
};

// Templated for the type of descriptor used
template <typename T>
struct VisionFeatureTrack {
  // Index of this feature track - should never be changed once created - each
  // member feature will also have this redundantly
  uint64_t const feature_idx;

 private:
  // The track of feature matches - private - only allow controlled feature
  // addition and no feature subtraction
  std::vector<VisionFeature<T>> track;

 public:
  // Add feature to end of feature track
  void addFeature(VisionFeature<T> feature) {
    track.push_back(feature);
    return;
  };
  // Return reference to the feature track vector that doesn't allow you to edit
  // the feature track
  std::vector<VisionFeature<T>> const& getTrack() const { return track; }
  // Default constructor: removed - must construct feature track with const
  // feature_idx
  VisionFeatureTrack(uint64_t const feature_idx) : feature_idx(feature_idx) {}
  // Convenience constructor: initialize everything.
  VisionFeatureTrack(
      uint64_t const feature_idx,
      std::vector<VisionFeature<T>> track = std::vector<VisionFeature<T>>())
      : feature_idx(feature_idx), track(track){};
};

// Templated for the type of descriptor used
template <typename T>
struct TrackDatabase {
  // All frame IDs - each tracks individual frame IDs will be a subset of these
  std::vector<uint64_t> frame_idxs;
  // All feature tracks
  std::vector<VisionFeatureTrack<T>> feature_tracks;
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
}  // namespace vslam_types

#endif  // __VSLAM_TYPES_H__