//
// Created by amanda on 6/9/22.
//

#ifndef UT_VSLAM_VSLAM_OPT_TYPES_REFACTOR_H
#define UT_VSLAM_VSLAM_OPT_TYPES_REFACTOR_H

#include <eigen3/Eigen/Dense>
#include <memory>

namespace vslam_types_refactor {

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
struct Pose3D {
  Position3d<NumType> transl_;
  Orientation3D<NumType> orientation_;

  Pose3D() = default;
  Pose3D(const Position3d<NumType> &transl,
           const Orientation3D<NumType> &orientation)
      : transl_(transl), orientation_(orientation) {}
};

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_VSLAM_OPT_TYPES_REFACTOR_H
