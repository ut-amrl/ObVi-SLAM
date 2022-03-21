#ifndef UNPROJECT_H
#define UNPROJECT_H

#include <vector>
#include <utility>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <glog/logging.h>
#include "vslam_types.h"

using namespace std;
using namespace Eigen;
using namespace vslam_types;

namespace vslam_unproject {

Vector3f Unproject(const Vector2f& image_feature,
                   const CameraIntrinsics& intrinsics,
                   const CameraExtrinsics& extrinsics,
                   const RobotPose& robot_pose,
                   const float depth) {
    Vector3f point_in_cam  = intrinsics.camera_mat.inverse() * Vector3f(image_feature.x(), image_feature.y(), 1);
    point_in_cam = depth * point_in_cam;
    AngleAxisf angle(extrinsics.rotation);
    Affine3f camera_to_robot = Affine3f::Identity();
    camera_to_robot.translate(extrinsics.translation);
    camera_to_robot.rotate(extrinsics.rotation);
    camera_to_robot = camera_to_robot.inverse();
    Vector3f point_in_robot = camera_to_robot * point_in_cam;
    // Vector3f point_in_robot = PoseArrayToAffine(&angle, &extrinsics.translation).inverse() * point_in_cam;
    Vector3f point_in_world = robot_pose.RobotToWorldTF() * point_in_robot;
    if (0) {
        printf("measurement: %.2f,%.2f; depth: %.2f\n", image_feature.x(), image_feature.y(), depth);
        printf("point_in_cam: %.2f,%.2f,%.2f\n", point_in_cam.x(), point_in_cam.y(), point_in_cam.z());
        printf("point_in_robot: %.2f,%.2f,%.2f\n", point_in_robot.x(), point_in_robot.y(), point_in_robot.z());
        printf("point_in_world: %.2f,%.2f,%.2f\n", point_in_world.x(), point_in_world.y(), point_in_world.z());
        cout << endl;
    }
    return point_in_world;
}

} // namespace unproject

#endif // UNPROJECT_H