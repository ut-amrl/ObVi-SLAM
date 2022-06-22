//
// Created by amanda on 6/9/22.
//

#ifndef UT_VSLAM_VSLAM_OBJ_OPT_TYPES_REFACTOR_H
#define UT_VSLAM_VSLAM_OBJ_OPT_TYPES_REFACTOR_H

#include <refactoring/types/vslam_basic_types_refactor.h>

#include <eigen3/Eigen/Dense>

namespace vslam_types_refactor {

typedef uint64_t ObjectId;

// std::pair<Eigen::Vector2f, Eigen::Vector2f>
template <typename NumType>
using BbCornerPair = std::pair<PixelCoord<NumType>, PixelCoord<NumType>>;

template <typename NumType>
using BbCorners = Eigen::Matrix<NumType, 4, 1>;

template <typename NumType>
using ObjectDim = Eigen::Matrix<NumType, 3, 1>;

template <typename NumType>
using RawEllipsoid = Eigen::Matrix<NumType, 9, 1>;

template <typename NumType>
using RawEllipsoidPtr = std::shared_ptr<RawEllipsoid<NumType>>;

template <typename NumType>
struct EllipsoidState {
  Pose3D<NumType> pose_;
  ObjectDim<NumType> dimensions_;
};

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
};

struct RawBoundingBoxObservation {
  RawBoundingBox bb_detection_;
  /**
   * Index of the frame/camera/robot_pose this bounding box was acquired at.
   */
  FrameId frame_idx_;

  /**
   * Id of the camera that captured this boundign box.
   */
  CameraId camera_id_;
};

template <typename NumType>
void convertToEllipsoidState(const RawEllipsoid<NumType> &raw_ellipsoid,
                             EllipsoidState<NumType> &ellipsoid) {
  RawPose3d<NumType> pose = raw_ellipsoid.topRows(6);
  ellipsoid.dimensions_ = raw_ellipsoid.bottomRows(3);
  convertToPose3D(pose, ellipsoid.pose_);
}

template <typename NumType>
EllipsoidState<NumType> convertToEllipsoidState(
    const RawEllipsoid<NumType> &raw_state) {
  EllipsoidState<NumType> state;
  convertToEllipsoidState(raw_state, state);
  return state;
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

bool operator==(const RawBoundingBox &bb_1, const RawBoundingBox &bb_2) {
  return (bb_1.semantic_class_ == bb_2.semantic_class_) &&
         (bb_1.pixel_corner_locations_ == bb_2.pixel_corner_locations_);
}

template <typename NumType>
std::size_t hash_value(const RawBoundingBox &bb) {
  boost::hash<std::pair<std::string, BbCornerPair<NumType>>> hasher;
  return hasher(std::make_pair(bb.semantic_class_, bb.pixel_corner_locations_));
}

}  // namespace vslam_types_refactor
// namespace vslam_types_refactor

#endif  // UT_VSLAM_VSLAM_OBJ_OPT_TYPES_REFACTOR_H
