#ifndef __VSLAM_TYPES_H__
#define __VSLAM_TYPES_H__

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
  VisionFeature(const uint64_t feature_idx,
                const T descriptor,
                const Eigen::Vector2f& pixel,
                const Eigen::Vector3f& point3d = Eigen::Vector3f())
      : feature_idx(feature_idx),
        descriptor(descriptor),
        pixel(pixel),
        point3d(point3d) {}
  // Convenience override of ostream - Each descriptor will need to have its own
  // overridden operator
  friend std::ostream& operator<<(std::ostream& o,
                                  const vslam_types::VisionFeature<T>& f) {
    o << "feature_idx: " << f.feature_idx << "\tdescriptor: " << f.descriptor
      << "\tpixel: " << f.pixel.x() << " " << f.pixel.y()
      << "\tpoint3d: " << f.point3d.x() << " " << f.point3d.y() << " "
      << f.point3d.z() << std::endl;
    return o;
  }
};

// Templated for the type of descriptor used
template <typename T>
struct VisionFeatureTrack {
  // Index of this feature track - should never be changed once created
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
  VisionFeatureTrack() = delete;
  // Convenience constructor: initialize everything.
  VisionFeatureTrack(
      uint64_t const feature_idx,
      std::vector<VisionFeature<T>> track = std::vector<VisionFeature<T>>())
      : feature_idx(feature_idx), track(track){};
};

}  // namespace vslam_types

#endif  // __VSLAM_TYPES_H__