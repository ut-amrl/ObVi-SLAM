//
// Created by amanda on 6/9/22.
//

#ifndef UT_VSLAM_VSLAM_OBJ_OPT_TYPES_REFACTOR_H
#define UT_VSLAM_VSLAM_OBJ_OPT_TYPES_REFACTOR_H

#include <refactoring/types/vslam_basic_types_refactor.h>
#include <refactoring/types/vslam_types_conversion.h>

#include <eigen3/Eigen/Dense>

namespace vslam_types_refactor {

#ifdef CONSTRAIN_ELLIPSOID_ORIENTATION
const static int kEllipsoidPoseParameterizationSize = 4;
#else
const static int kEllipsoidPoseParameterizationSize = 6;
#endif
const static int kEllipsoidParamterizationSize =
    kEllipsoidPoseParameterizationSize + 3;

typedef uint64_t ObjectId;

// std::pair<Eigen::Vector2f, Eigen::Vector2f>
template <typename NumType>
using BbCornerPair = std::pair<PixelCoord<NumType>, PixelCoord<NumType>>;

template <typename NumType>
using BbCorners = Eigen::Matrix<NumType, 4, 1>;

template <typename NumType>
using ObjectDim = Eigen::Matrix<NumType, 3, 1>;

template <typename NumType>
using RawEllipsoid = Eigen::Matrix<NumType, kEllipsoidParamterizationSize, 1>;

template <typename NumType>
using RawEllipsoidPtr = std::shared_ptr<RawEllipsoid<NumType>>;

template <typename NumType>
#ifdef CONSTRAIN_ELLIPSOID_ORIENTATION
struct EllipsoidState {
  Pose3DYawOnly<NumType> pose_;
  ObjectDim<NumType> dimensions_;

  EllipsoidState() = default;

  EllipsoidState(const Pose3DYawOnly<NumType> &pose,
                 const ObjectDim<double> &dimensions)
      : pose_(pose), dimensions_(dimensions) {}
};
#else
struct EllipsoidState {
  Pose3D<NumType> pose_;
  ObjectDim<NumType> dimensions_;

  EllipsoidState() = default;

  EllipsoidState(const Pose3D<double> &pose,
                 const ObjectDim<double> &dimensions)
      : pose_(pose), dimensions_(dimensions) {}
};
#endif

template <typename NumType>
#ifdef CONSTRAIN_ELLIPSOID_ORIENTATION
struct FullDOFEllipsoidState {
  Pose3D<NumType> pose_;
  ObjectDim<NumType> dimensions_;

  FullDOFEllipsoidState() = default;

  FullDOFEllipsoidState(const Pose3D<NumType> &pose,
                        const ObjectDim<double> &dimensions)
      : pose_(pose), dimensions_(dimensions) {}
};
#else
using FullDOFEllipsoidState = EllipsoidState<NumType>;
#endif

template <typename NumType>
using FullDOFRawEllipsoid = Eigen::Matrix<NumType, 9, 1>;

struct RawBoundingBox {
  /**
   * Pixel coordinates of the two opposite corners that define the bounding box
   * of an object within an image. The first of the pair should have the smaller
   * x and y values.
   */
  BbCornerPair<double> pixel_corner_locations_;

  /**
   * Semantic class of the detected bounding box
   *
   * TODO should this instead be an index? Should it store a set of
   * possible semantic classes with their likelihood?
   */
  std::string semantic_class_;

  double detection_confidence_;
};

struct RawBoundingBoxObservation {
  RawBoundingBox bb_detection_;
  /**
   * Index of the frame/camera/robot_pose this bounding box was acquired at.
   */
  FrameId frame_idx_;

  /**
   * Id of the camera that captured this bounding box.
   */
  CameraId camera_id_;
};

template <typename NumType>
void convertToEllipsoidState(const RawEllipsoid<NumType> &raw_ellipsoid,
                             EllipsoidState<NumType> &ellipsoid) {
#ifdef CONSTRAIN_ELLIPSOID_ORIENTATION
  RawPose3dYawOnly<NumType> pose =
      raw_ellipsoid.topRows(kEllipsoidPoseParameterizationSize);
  convertToYawOnlyPose3D(pose, ellipsoid.pose_);
#else
  RawPose3d<NumType> pose =
      raw_ellipsoid.topRows(kEllipsoidPoseParameterizationSize);
  convertToPose3D(pose, ellipsoid.pose_);
#endif
  ellipsoid.dimensions_ = raw_ellipsoid.bottomRows(3);
}

template <typename NumType>
EllipsoidState<NumType> convertToEllipsoidState(
    const RawEllipsoid<NumType> &raw_state) {
  EllipsoidState<NumType> state;
  convertToEllipsoidState(raw_state, state);
  return state;
}

template <typename NumType>
void convertToRawEllipsoid(const EllipsoidState<NumType> &ellipsoid_state,
                           RawEllipsoid<NumType> &raw_ellipsoid) {
#ifdef CONSTRAIN_ELLIPSOID_ORIENTATION
  raw_ellipsoid.topRows(kEllipsoidPoseParameterizationSize) =
      convertYawOnlyPoseToArray(ellipsoid_state.pose_);
#else
  raw_ellipsoid.topRows(kEllipsoidPoseParameterizationSize) =
      convertPoseToArray(ellipsoid_state.pose_);
#endif
  raw_ellipsoid.bottomRows(3) = ellipsoid_state.dimensions_;
}

template <typename NumType>
RawEllipsoid<NumType> convertToRawEllipsoid(
    const EllipsoidState<NumType> &ellipsoid_state) {
  RawEllipsoid<NumType> raw_ellipsoid;
  convertToRawEllipsoid(ellipsoid_state, raw_ellipsoid);
  return raw_ellipsoid;
}

template <typename NumType>
void extractPosition(const RawEllipsoid<NumType> &raw_obj_state,
                     Position3d<NumType> &position) {
  position(0) = raw_obj_state[0];
  position(1) = raw_obj_state[1];
  position(2) = raw_obj_state[2];
}

template <typename NumType>
Position3d<NumType> extractPosition(
    const RawEllipsoid<NumType> &raw_obj_state) {
  Position3d<NumType> position;
  extractPosition(raw_obj_state, position);
  return position;
}

template <typename NumType>
BbCornerPair<NumType> cornerLocationsVectorToPair(
    const BbCorners<NumType> &corner_vec) {
  return std::make_pair(PixelCoord<NumType>(corner_vec(0), corner_vec(2)),
                        PixelCoord<NumType>(corner_vec(1), corner_vec(3)));
}

template <typename NumType>
BbCorners<NumType> cornerLocationsPairToVector(
    const BbCornerPair<NumType> &corner_pair) {
  return BbCorners<NumType>(corner_pair.first.x(),
                            corner_pair.second.x(),
                            corner_pair.first.y(),
                            corner_pair.second.y());
}

template <typename NumType>
bool operator==(const BbCorners<NumType> &bb_1,
                const BbCorners<NumType> &bb_2) {
  return (cornerLocationsVectorToPair(bb_1) ==
          cornerLocationsVectorToPair(bb_2));
}

template <typename NumType>
std::size_t hash_value(const BbCorners<NumType> &bb) {
  boost::hash<BbCornerPair<NumType>> hasher;
  return hasher(cornerLocationsVectorToPair(bb));
}

bool operator==(const RawBoundingBox &bb_1, const RawBoundingBox &bb_2);

template <typename NumType>
std::size_t hash_value(const RawBoundingBox &bb) {
  boost::hash<std::pair<std::pair<std::string, BbCornerPair<NumType>>, double>>
      hasher;
  return hasher(std::make_pair(
      std::make_pair(bb.semantic_class_, bb.pixel_corner_locations_),
      bb.detection_confidence_));
}

}  // namespace vslam_types_refactor
// namespace vslam_types_refactor

#endif  // UT_VSLAM_VSLAM_OBJ_OPT_TYPES_REFACTOR_H
