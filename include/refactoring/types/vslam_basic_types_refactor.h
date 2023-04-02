//
// Created by amanda on 6/9/22.
//

#ifndef UT_VSLAM_VSLAM_OPT_TYPES_REFACTOR_H
#define UT_VSLAM_VSLAM_OPT_TYPES_REFACTOR_H

#include <boost/functional/hash.hpp>
#include <eigen3/Eigen/Dense>
#include <memory>

namespace vslam_types_refactor {

typedef uint64_t CameraId;
typedef uint64_t FrameId;
typedef uint64_t FeatureId;

template <typename NumType>
using CameraIntrinsicsMat = Eigen::Matrix<NumType, 3, 3>;

template <typename NumType>
using PixelCoord = Eigen::Matrix<NumType, 2, 1>;

template <typename NumType>
using Position3d = Eigen::Matrix<NumType, 3, 1>;

template <typename NumType>
using Position3dPtr = std::shared_ptr<Position3d<NumType>>;

template <typename NumType>
using Orientation3D = Eigen::AngleAxis<NumType>;

template <typename NumType>
using Transform6Dof = Eigen::Transform<NumType, 3, Eigen::Affine>;

template <typename NumType, int MatDim>
using Covariance = Eigen::Matrix<NumType, MatDim, MatDim>;

template <typename NumType, int MatDim>
using SqrtInfMat = Eigen::Matrix<NumType, MatDim, MatDim>;

template <typename NumType>
using RawPose3d = Eigen::Matrix<NumType, 6, 1>;

template <typename NumType>
using RawPose3dPtr = std::shared_ptr<RawPose3d<NumType>>;

template <typename NumType>
using RawPose3dYawOnly = Eigen::Matrix<NumType, 4, 1>;

template <typename NumType>
using RawPose3dYawOnlyPtr = std::shared_ptr<RawPose3dYawOnly<NumType>>;

template <typename NumType>
struct Pose3D {
  Position3d<NumType> transl_;
  Orientation3D<NumType> orientation_;

  Pose3D() = default;
  Pose3D(const Position3d<NumType> &transl,
         const Orientation3D<NumType> &orientation)
      : transl_(transl), orientation_(orientation) {}
};

template <typename NumType>
struct Pose3DYawOnly {
  Position3d<NumType> transl_;
  NumType yaw_ = NumType(0);

  Pose3DYawOnly() = default;
  Pose3DYawOnly(const Position3d<NumType> &transl, const NumType &yaw)
      : transl_(transl), yaw_(yaw) {}
};

template <typename NumType>
using CameraExtrinsics = Pose3D<NumType>;

template <typename NumType>
bool operator==(const PixelCoord<NumType> &px_1,
                const PixelCoord<NumType> &px_2) {
  return (std::make_pair(px_1(0), px_1(1)) == std::make_pair(px_2(0), px_2(1)));
}

template <typename NumType>
std::size_t hash_value(const PixelCoord<NumType> &px) {
  boost::hash<std::pair<NumType, NumType>> hasher;
  return hasher(std::make_pair(px(0), px(1)));
}

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_VSLAM_OPT_TYPES_REFACTOR_H
