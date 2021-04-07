#ifndef __VSLAM_TYPES_H__
#define __VSLAM_TYPES_H__

#include <utility>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

namespace vslam_types {

struct VisionFeature {
  // Index of this feature in the features vector for the node, stored here
  // for redundancy.
  uint64_t feature_idx;
  // Camera pixel location of feature.
  Eigen::Vector2f pixel;
  // Estimated 3d location in the camera frame.
  Eigen::Vector3f point3d;
  // Default constructor: do nothing.
  VisionFeature() {}
  // Convenience constructor: initialize everything.
  VisionFeature(uint64_t idx,
                const Eigen::Vector2f& p,
                const Eigen::Vector3f& point3d)
      : feature_idx(idx), pixel(p), point3d(point3d) {}
};

struct FeatureMatch {
  // Feature ID from the initial pose.
  uint64_t feature_idx_initial;
  // Feature ID from current pose.
  uint64_t feature_idx_current;
  // Default constructor: do nothing.
  FeatureMatch() {}
  // Convenience constructor: initialize everything.
  FeatureMatch(uint64_t fid_initial, uint64_t fid_current)
      : feature_idx_initial(fid_initial), feature_idx_current(fid_current) {}
};

struct VisionFactor {
  // ID of the pose where the features were *first* observed.
  uint64_t pose_idx_initial;
  // ID of second pose.
  uint64_t pose_idx_current;
  // Pair of feature ID from first pose, and feature ID from second pose,
  // and feature ID from initial pose.
  std::vector<FeatureMatch> feature_matches;
  // Default constructor: do nothing.
  VisionFactor() {}
  // Convenience constructor: initialize everything.
  VisionFactor(uint64_t pose_initial,
               uint64_t pose_current,
               const std::vector<vslam_types::FeatureMatch>& feature_matches)
      : pose_idx_initial(pose_initial),
        pose_idx_current(pose_current),
        feature_matches(feature_matches) {}
};

}  // namespace vslam_types

#endif  // __VSLAM_TYPES_H__