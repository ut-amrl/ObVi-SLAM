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

// extrinsics: transform from camera pose to robot pose; loc of camera in robot frame
Vector3f Unproject(const Vector2f& image_feature,
                   const CameraIntrinsics& intrinsics,
                   const CameraExtrinsics& extrinsics,
                   const RobotPose& robot_pose,
                   const float depth) {
    Vector3f point_in_cam  = intrinsics.camera_mat.inverse() * Vector3f(image_feature.x(), image_feature.y(), 1);
    AngleAxisf angle(extrinsics.rotation);
    Affine3f camera_to_robot = Affine3f::Identity();
    camera_to_robot.translate(extrinsics.translation);
    camera_to_robot.rotate(extrinsics.rotation);
    camera_to_robot = camera_to_robot.inverse();
    Vector3f point_in_robot = camera_to_robot * point_in_cam;
    // Vector3f point_in_robot = PoseArrayToAffine(&angle, &extrinsics.translation).inverse() * point_in_cam;
    Vector3f point_in_world = robot_pose.RobotToWorldTF() * point_in_world;
    printf("measurement: %.2f,%.2f\n", image_feature.x(), image_feature.y());
    printf("point_in_cam: %.2f,%.2f,%.2f\n", point_in_cam.x(), point_in_cam.y(), point_in_cam.z());
    printf("point_in_robot: %.2f,%.2f,%.2f\n", point_in_robot.x(), point_in_robot.y(), point_in_robot.z());
    cout << endl;
    return point_in_world;
}

void Unproject(const Vector2f& image_feature,
               const CameraIntrinsics& intrinsics,
               const CameraExtrinsics& extrinsics,
               const RobotPose& robot_pose,
               const float depth,
               Vector3f* point_ptr) {
    Vector3f& point = *point_ptr;
    Vector3f point_in_cam  = intrinsics.camera_mat.inverse() * Vector3f(image_feature.x(), image_feature.y(), 1);
    Affine3f camera_to_robot = Affine3f::Identity();
    camera_to_robot.translate(extrinsics.translation);
    camera_to_robot.rotate(extrinsics.rotation);
    camera_to_robot = camera_to_robot.inverse();
    Vector3f point_in_robot = camera_to_robot * point_in_cam;
    // AngleAxisf angle(extrinsics.rotation);
    // Vector3f point_in_robot = PoseArrayToAffine(const_cast<const AngleAxisf*> (&angle), &extrinsics.translation).inverse() * point_in_cam;
    Vector3f point_in_world = robot_pose.RobotToWorldTF() * point_in_world;
    point = point_in_world;
}

void Unproject(const vector<Vector2f>& image_features,
               const CameraIntrinsics& intrinsics,
               const CameraExtrinsics& extrinsics,
               const vector<RobotPose>& robot_poses,
               const vector<float>& depths,
               vector<Vector3f>* points_ptr) {
    vector<Vector3f>& points = *points_ptr;
    if (image_features.size() != robot_poses.size() 
        || image_features.size() != depths.size()
        || robot_poses.size() != depths.size()) {
        LOG(FATAL) << "Size mismatch! "
            << "image size = " << image_features.size() << ", "
            << "pose size = " << robot_poses.size() << ", "
            << "depth size = " << depths.size();
        return;
    }
    for (size_t i = 0; i < image_features.size(); ++i) {
        points[i] = Unproject(image_features[i], intrinsics, extrinsics, robot_poses[i], depths[i]);
    }
    
}


} // namespace unproject

#endif // UNPROJECT_H