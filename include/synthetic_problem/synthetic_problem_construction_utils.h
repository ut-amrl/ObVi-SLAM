//
// Created by amanda on 1/23/22.
//

#ifndef UT_VSLAM_SYNTHETIC_PROBLEM_CONSTRUCTION_UTILS_H
#define UT_VSLAM_SYNTHETIC_PROBLEM_CONSTRUCTION_UTILS_H

#include <eigen3/Eigen/Dense>

namespace synthetic_problem {

/**
 * Create an AngleAxis orientation that represents the given yaw in radians.
 *
 * @param yaw Yaw in radians
 *
 * @return Rotation around z axis expressed in angle-axis form.
 */
Eigen::AngleAxisf createAngleAxisFromYaw(const float &yaw);

}  // namespace synthetic_problem

#endif  // UT_VSLAM_SYNTHETIC_PROBLEM_CONSTRUCTION_UTILS_H
