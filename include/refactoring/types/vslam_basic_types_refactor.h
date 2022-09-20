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

typedef std::pair<uint32_t, uint32_t> Timestamp;

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
using Position2d = Eigen::Matrix<NumType, 2, 1>;

template <typename NumType>
using Position2dPtr = std::shared_ptr<Position2d<NumType>>;

template <typename NumType>
using Orientation2D = NumType;

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
struct Pose3D {
  Position3d<NumType> transl_;
  Orientation3D<NumType> orientation_;

  Pose3D() = default;
  Pose3D(const Position3d<NumType> &transl,
         const Orientation3D<NumType> &orientation)
      : transl_(transl), orientation_(orientation) {}
  Pose3D(const NumType& x, const NumType& y, const NumType& z,
         const NumType& qw, const NumType& qx, const NumType& qy, const NumType& qz) {
    transl_ << x, y, z;
    orientation_ = Eigen::AngleAxis<NumType>(Eigen::Quaternion<NumType>(qw, qx, qy, qz));
  }
};

template <typename NumType>
struct Pose2D {
  Position2d<NumType> transl_;
  Orientation2D<NumType> orientation_;

  Pose2D() = default;
  Pose2D(const Position2d<NumType> &transl,
         const Orientation2D<NumType> &orientation)
      : transl_(transl), orientation_(orientation) {}
};

template <typename NumType>
using CameraExtrinsics = Pose3D<NumType>;

/**
 * Pinhole camera intrinsics parameters.
 */
template <typename NumType>
struct CameraIntrinsics {
  /**
   * Camera matrix.
   */
  Eigen::Matrix3f camera_mat;

  uint32_t image_width;

  uint32_t image_height;

  CameraIntrinsics() {}
  CameraIntrinsics(const Eigen::Matrix3f& camera_mat) : camera_mat(camera_mat) {}
  CameraIntrinsics(const float fx, const float fy, const float cx, const float cy) {
      this->camera_mat = Eigen::Matrix3f::Identity();
      this->camera_mat(0,0) = fx;
      this->camera_mat(1,1) = fy;
      this->camera_mat(0,2) = cx;
      this->camera_mat(1,2) = cy;
  }
  CameraIntrinsics(const float fx, const float fy, const float cx, const float cy, 
                   const float image_width, const float image_height) {
      this->camera_mat = Eigen::Matrix3f::Identity();
      this->camera_mat(0,0) = fx;
      this->camera_mat(1,1) = fy;
      this->camera_mat(0,2) = cx;
      this->camera_mat(1,2) = cy;
      this->image_width = image_width;
      this->image_height = image_height;
  }
};

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
