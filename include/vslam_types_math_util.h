#ifndef UT_VSLAM_SLAM_TYPES_MATH_UTIL_H
#define UT_VSLAM_SLAM_TYPES_MATH_UTIL_H

#include <glog/logging.h>
#include <vslam_types.h>

#include <unsupported/Eigen/MatrixFunctions>
#include <vector>

namespace vslam_util {

/**
 * Adjust trajectory so that the first pose is at the origin and all
 * other poses are adjusted to maintain the same transform to the first pose.
 *
 * @param original_trajectory[in]   Original trajectory.
 * @param adjusted_trajectory[out]  Trajectory adjusted to have the first pose
 *                                  at the origin.
 */
void AdjustTrajectoryToStartAtZero(
    const std::vector<vslam_types::RobotPose>& original_trajectory,
    std::vector<vslam_types::RobotPose>& adjusted_trajectory);

/**
 * Get the pose of pose 2 (provided in frame A) relative to pose 1 (also
 * provided in frame A).
 *
 * @param pose_1    Pose 1 -- pose that we want to be the frame for in the
 *                  returned pose.
 * @param pose_2    Pose 2 -- pose that we want to get location of relative to
 * pose 1.
 *
 * @return Relative transform that provides the location of pose 2 relative to
 * pose 1.
 */
vslam_types::RobotPose getPose2RelativeToPose1(
    const vslam_types::RobotPose& pose_1, const vslam_types::RobotPose& pose_2);

/**
 * Get the position of the given position (provided in frame A) relative to
 * pose 1 (also provided in frame A).
 *
 * @param pose_1    Pose 1 -- pose that we want to be the frame for in the
 *                  returned pose.
 * @param position  Position that we want to get relative to pose 1
 *
 * @return Relative position that provides the location of the given position
 * relative to pose 1.
 */
Eigen::Vector3d getPositionRelativeToPose(const vslam_types::RobotPose& pose_1,
                                          const Eigen::Vector3d& position);

/**
 * Combine the given poses. I.e. if pose 1 is in frame A and pose 2 is in the
 * frame of pose 1, then this will return pose 2 relative to frame A.
 *
 * Uses the frame id for pose 2 in constructing the new pose.
 *
 * @param pose_1    Pose 1 -- pose 2 is relative to this.
 * @param pose_2    Pose 2 -- relative to pose 1, but we want to get it in the
 * same frame as pose 1.
 *
 * @return Relative transform that provides the location of pose 2 relative to
 * the frame that pose 1 is in.
 */
vslam_types::RobotPose combinePoses(const vslam_types::RobotPose& pose_1,
                                    const vslam_types::RobotPose& pose_2);
}  // namespace vslam_util

#endif  // UT_VSLAM_SLAM_TYPES_MATH_UTIL_H