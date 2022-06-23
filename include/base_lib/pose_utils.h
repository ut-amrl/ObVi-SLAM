//
// Created by amanda on 2/7/22.
//

#ifndef UT_VSLAM_POSE_UTILS_H
#define UT_VSLAM_POSE_UTILS_H

#include <utility>
#include <base_lib/pose_reps.h>

namespace pose {

    typedef std::pair<uint32_t, uint32_t> Timestamp;

    std::ostream &operator<<(std::ostream& os, const Timestamp& timestamp) {
      os << timestamp.first << ", " << timestamp.second;
      return os;
    }

    uint64_t timestampToMillis(const Timestamp &timestamp) {
        return timestamp.first * 1000 + (timestamp.second / 1e6);
    }

    bool posesSame(const pose::Pose2d &p1, const pose::Pose2d &p2) {
        pose::Pose2d rel_pose = pose::getPoseOfObj1RelToObj2(p1, p2);
        return ((rel_pose.first.norm() == 0) && (rel_pose.second == 0));
    }

    bool posesAlmostSame(const pose::Pose2d &p1, const pose::Pose2d &p2, const double &transl_tol, const double &angle_tol) {
        pose::Pose2d rel_pose = pose::getPoseOfObj1RelToObj2(p1, p2);

        if (rel_pose.first.norm() > transl_tol) {
            return false;
        }
        return abs(rel_pose.second) <= angle_tol;
    }

    pose::Pose2d interpolatePoses(const std::pair<Timestamp, pose::Pose2d> &pose_1,
                                  const std::pair<Timestamp, pose::Pose2d> &pose_2, const Timestamp &target_time) {

        pose::Pose2d rel_pose = pose::getPoseOfObj1RelToObj2(pose_2.second, pose_1.second);

        uint64_t pose_one_millis = timestampToMillis(pose_1.first);
        uint64_t pose_two_millis = timestampToMillis(pose_2.first);
        uint64_t target_millis = timestampToMillis(target_time);

        double fraction = ((double) (target_millis - pose_one_millis)) /
                          (pose_two_millis - pose_one_millis);

        pose::Pose2d rel_pose_interpolated;

        rel_pose_interpolated = pose::createPose2d(rel_pose.first.x() * fraction,
                                                   rel_pose.first.y() * fraction,
                                                   rel_pose.second * fraction);
        if (abs(rel_pose.second) > 1e-10) {
            double radius = sqrt(rel_pose.first.squaredNorm() / (2 * (1 - cos(rel_pose.second))));
            double x = radius * sin(abs(rel_pose.second) * fraction);
            double y = radius - (radius * cos(rel_pose.second * fraction));
            if (rel_pose.second < 0) {
                y = -y;
            }
            if (rel_pose.first.x() < 0) {
                y = -y;
                x = -x;
            }
            rel_pose_interpolated = pose::createPose2d(x, y, fraction * rel_pose.second);
        }
        return pose::combinePoses(pose_1.second, rel_pose_interpolated);
    }
}

#endif //AUTODIFF_GP_POSE_UTILS_H
