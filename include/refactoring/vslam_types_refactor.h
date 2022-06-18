//
// Created by amanda on 6/9/22.
//

#ifndef UT_VSLAM_VSLAM_TYPES_REFACTOR_H
#define UT_VSLAM_VSLAM_TYPES_REFACTOR_H

#include <refactoring/vslam_basic_types_refactor.h>
#include <refactoring/vslam_types_conversion.h>

namespace vslam_types_refactor {

typedef uint64_t CameraId;
typedef uint64_t FrameId;
typedef uint64_t FeatureId;

template <typename NumType>
using CameraExtrinsics = Pose3D<NumType>;

///**
// * A structureless vision feature describes the association between a
// * measurement (pixel), an identified feature (feature_idx), and the camera it
// * was captured by (frame_idx).
// */
//struct VisionFeature {
//  //  /**
//  //   * Index of the frame/camera/robot_pose this feature was acquired at.
//  //   */
//  //  FrameId frame_idx_;
//
//  /**
//   * Camera pixel location of feature by the camera id that captured them.
//   *
//   * There must be an entry here for the primary camera id.
//   */
//  std::unordered_map<CameraId, PixelCoord<float>> pixel_by_camera_id;
//
//  /**
//   * Id of the primary camera. This one should be used if only one camera is
//   * used.
//   */
//  CameraId primary_camera_id;
//
//  /**
//   * Default constructor: do nothing.
//   */
//  VisionFeature(){};
//
//  /**
//   * Convenience constructor: initialize everything.
//   */
//  VisionFeature(
//      //      const FrameId& frame_idx,
//      const std::unordered_map<CameraId, PixelCoord<float>>& pixel_by_camera_id,
//      const CameraId& primary_camera_id)
//      :  //        frame_idx(frame_idx),
//        pixel_by_camera_id(pixel_by_camera_id),
//        primary_camera_id(primary_camera_id) {}
//  /**
//   * Convenience override of ostream.
//   */
//  friend std::ostream& operator<<(std::ostream& o, const VisionFeature& f) {
//    o
//        //        << "frame_idx: " << f.frame_idx
//        << "\tPrimary camera id: " << f.primary_camera_id
//        << "\tPixels by camera id: ";
//    for (const auto& pixel_camera_id_pair : f.pixel_by_camera_id) {
//      o << "("
//        << "Camera id:" << pixel_camera_id_pair.first
//        << ", pixel: " << pixel_camera_id_pair.second.x() << ", "
//        << pixel_camera_id_pair.second.y() << "),";
//    }
//    return o;
//  }
//  //  /**
//  //   * Override of operator<() - compares two features based on the
//  //   * frame_idx - assumes that frame_idxs can be sorted to be sequential and
//  //   * always increasing- this overload allows us to sort feature tracks using
//  //   * std::sort
//  //   */
//  //  friend bool operator<(const VisionFeature& lhs, const VisionFeature& rhs)
//  //  {
//  //    // Assert that features being compared are in same track - comparing
//  //    // features across tracks in this manner doesn't have a very intuitive
//  //    // meaning
//  //    return lhs.frame_idx < rhs.frame_idx;
//  //  }
//};
//
///**
// * Structureless vision feature track.
// */
//struct VisionFeatureTrack {
//  //  /**
//  //   * Index of this feature track - should never be changed once created -
//  //   each
//  //   * member feature in the track will also have this redundantly.
//  //   */
//  //  FeatureId feature_idx;
//  /**
//   * The track of feature matches.
//   */
//  std::unordered_map<FrameId, VisionFeature> track_;
//
//  //  /**
//  //   * Default constructor: do nothing.
//  //   */
//  //  VisionFeatureTrack() {}
//  /**
//   * Convenience constructor: initialize everything.
//   */
//  VisionFeatureTrack(
//      //      const uint64_t& feature_idx,
//      const std::unordered_map<FrameId, VisionFeature>& track =
//          std::unordered_map<FrameId, VisionFeature>())
//      :  //        feature_idx(feature_idx),
//        track_(track){};
//};
//
///**
// * Structured vision feature track. In addition to a track feature_idx there is
// * a 3D point associated with it as well.
// */
//
//struct StructuredVisionFeatureTrack {
//  /**
//   * 3D coordinate of the feature tracked by the feature track.
//   */
//  Position3d<double> point_est_;
//
//  /**
//   * Image feature track - same as the structureless feature track.
//   * */
//  VisionFeatureTrack feature_track_;
//  /**
//   * Default constructor: Do nothing.
//   */
//  StructuredVisionFeatureTrack() {}
//  /**
//   * Convenience constructor: initialize everything.
//   */
//  StructuredVisionFeatureTrack(const Eigen::Vector3d& point,
//                               const VisionFeatureTrack& feature_track)
//      : point_est_(point), feature_track_(feature_track){};
//};
//
//class RobotPose {
// public:
//  RobotPose() = default;
//  RobotPose(const Pose3D<double>& pose)
//      : slam_node_(convertPoseToArray(pose)) {}
//
//  Position3d<double> getPosition() {}
//
//  Orientation3D<double> getOrientation() {}
//
//  Pose3D<double> getPose() { return convertToPose3D(slam_node_); }
//
// private:
//  RawPose3d<double> slam_node_;
//};
//
//template <typename FeatureTrackType>
//class LowLevelFeatureSLAMProblemInfo {
// public:
// protected:
//  std::unordered_map<FrameId, RobotPose>& robot_poses_;
//
//  /**
//   * Extrinsics for each camera.
//   */
//  std::unordered_map<CameraId, CameraExtrinsics<double>>
//      camera_extrinsics_by_camera_;
//
//  /**
//   * Intrinsics for each camera.
//   */
//  std::unordered_map<CameraId, CameraIntrinsicsMat<double>>
//      camera_instrinsics_by_camera_;
//
//  std::unordered_map<FeatureId, FeatureTrackType> tracks_;
//};

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_VSLAM_TYPES_REFACTOR_H
