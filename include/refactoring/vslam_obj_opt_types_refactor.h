//
// Created by amanda on 6/9/22.
//

#ifndef UT_VSLAM_VSLAM_OBJ_OPT_TYPES_REFACTOR_H
#define UT_VSLAM_VSLAM_OBJ_OPT_TYPES_REFACTOR_H

#include <refactoring/vslam_basic_types_refactor.h>

#include <eigen3/Eigen/Dense>

namespace vslam_types_refactor {

typedef uint64_t ObjectId;

//std::pair<Eigen::Vector2f, Eigen::Vector2f>
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

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_VSLAM_OBJ_OPT_TYPES_REFACTOR_H
