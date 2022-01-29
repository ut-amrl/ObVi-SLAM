//
// Created by amanda on 1/23/22.
//

#ifndef UT_VSLAM_ROS_VISUALIZATION_H
#define UT_VSLAM_ROS_VISUALIZATION_H

#include <sensor_msgs/CameraInfo.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <vslam_types.h>
#include <vslam_util.h>
#include <sensor_msgs/Image.h>

namespace vslam_viz {

class RosVisualization {
 public:
  RosVisualization(ros::NodeHandle &node_handle, const std::string &prefix = "")
      : node_handle_(node_handle), prefix_(prefix) {}

  std::string createEllipsoidFrameId(const std::string &prefix,
                                     const uint64_t &ellipsoid_idx) {
    return prefix + "ellipsoid_" + std::to_string(ellipsoid_idx);
  }

  void visualizeEllipsoids(
      const std::vector<vslam_types::EllipsoidEstimate> &ellipsoid_estimates,
      const std::string &topic,
      const std_msgs::ColorRGBA &color) {
    if (publishers_by_topic_.find(topic) == publishers_by_topic_.end()) {
      ros::Publisher ellipsoid_pub =
          node_handle_.advertise<visualization_msgs::Marker>(
              prefix_ + topic, kEllipsoidMarkerPubQueueSize);
      publishers_by_topic_[prefix_ + topic] = ellipsoid_pub;
      ros::Duration(kSleepAfterPubCreationTime).sleep();
    }
    ros::Publisher pub = publishers_by_topic_[prefix_ + topic];
    for (const vslam_types::EllipsoidEstimate &ellipsoid :
         ellipsoid_estimates) {
      publishEllipsoid(ellipsoid, pub, color);
    }
  }

  void visualizeTrajectoryAndEllipsoidsWithTf(
      const std::vector<vslam_types::RobotPose> &trajectory,
      const std::vector<vslam_types::EllipsoidEstimate> &ellipsoids,
      const std::unordered_map<vslam_types::CameraId,
                               vslam_types::CameraExtrinsics> &extrinsics,
      const std::unordered_map<vslam_types::CameraId,
                               vslam_types::CameraIntrinsics> &intrinsics,
      const std::string &prefix = "") {
    for (const vslam_types::EllipsoidEstimate &ellipsoid_est : ellipsoids) {
      geometry_msgs::TransformStamped ellipsoid_transform;
      ellipsoid_transform.header.stamp = ros::Time::now();
      ellipsoid_transform.header.frame_id = "map";
      ellipsoid_transform.child_frame_id =
          createEllipsoidFrameId(prefix, ellipsoid_est.ellipsoid_idx);
      ellipsoid_transform.transform.translation.x = ellipsoid_est.loc.x();
      ellipsoid_transform.transform.translation.y = ellipsoid_est.loc.y();
      ellipsoid_transform.transform.translation.z = ellipsoid_est.loc.z();
      Eigen::Quaternionf ellipsoid_quat(ellipsoid_est.orientation);
      ellipsoid_transform.transform.rotation.x = ellipsoid_quat.x();
      ellipsoid_transform.transform.rotation.y = ellipsoid_quat.y();
      ellipsoid_transform.transform.rotation.z = ellipsoid_quat.z();
      ellipsoid_transform.transform.rotation.w = ellipsoid_quat.w();
      static_tf_broadcaster_.sendTransform(ellipsoid_transform);
    }

    for (const vslam_types::RobotPose &robot_pose : trajectory) {
      geometry_msgs::TransformStamped robot_transform;
      robot_transform.header.stamp = ros::Time::now();
      robot_transform.header.frame_id = "map";
      robot_transform.child_frame_id =
          prefix + "robot_" + std::to_string(robot_pose.frame_idx);
      robot_transform.transform.translation.x = robot_pose.loc.x();
      robot_transform.transform.translation.y = robot_pose.loc.y();
      robot_transform.transform.translation.z = robot_pose.loc.z();
      Eigen::Quaternionf robot_quat(robot_pose.angle);
      robot_transform.transform.rotation.x = robot_quat.x();
      robot_transform.transform.rotation.y = robot_quat.y();
      robot_transform.transform.rotation.z = robot_quat.z();
      robot_transform.transform.rotation.w = robot_quat.w();
      static_tf_broadcaster_.sendTransform(robot_transform);

      for (const auto &cam_id_and_extrinsics : extrinsics) {
        geometry_msgs::TransformStamped cam_transform;
        cam_transform.header.stamp = ros::Time::now();
        cam_transform.header.frame_id = robot_transform.child_frame_id;
        cam_transform.child_frame_id =
            robot_transform.child_frame_id + "_cam" +
            std::to_string(cam_id_and_extrinsics.first);
        cam_transform.transform.translation.x =
            cam_id_and_extrinsics.second.translation.x();
        cam_transform.transform.translation.y =
            cam_id_and_extrinsics.second.translation.y();
        cam_transform.transform.translation.z =
            cam_id_and_extrinsics.second.translation.z();
        cam_transform.transform.rotation.x =
            cam_id_and_extrinsics.second.rotation.x();
        cam_transform.transform.rotation.y =
            cam_id_and_extrinsics.second.rotation.y();
        cam_transform.transform.rotation.z =
            cam_id_and_extrinsics.second.rotation.z();
        cam_transform.transform.rotation.w =
            cam_id_and_extrinsics.second.rotation.w();
        static_tf_broadcaster_.sendTransform(cam_transform);
        if (intrinsics.find(cam_id_and_extrinsics.first) != intrinsics.end()) {
          publishCameraIntrinsics(intrinsics.at(cam_id_and_extrinsics.first),
                                  cam_transform.child_frame_id,
                                  cam_transform.child_frame_id + "/camera_info",
                                  cam_transform.child_frame_id + "/image_raw");
        }

        vslam_types::RobotPose extrinsics_pose(
            robot_pose.frame_idx,
            cam_id_and_extrinsics.second.translation,
            Eigen::AngleAxisf(cam_id_and_extrinsics.second.rotation));
        vslam_types::RobotPose cam_pose_in_world =
            vslam_util::combinePoses(robot_pose, extrinsics_pose);
        Eigen::Quaternionf cam_quat(cam_pose_in_world.angle);
        geometry_msgs::TransformStamped alt_cam_transform;
        alt_cam_transform.header.stamp = ros::Time::now();
        alt_cam_transform.header.frame_id = "map";
        alt_cam_transform.child_frame_id =
            robot_transform.child_frame_id + "_cam" +
            std::to_string(cam_id_and_extrinsics.first) + "_alt";
        alt_cam_transform.transform.translation.x = cam_pose_in_world.loc.x();
        alt_cam_transform.transform.translation.y = cam_pose_in_world.loc.y();
        alt_cam_transform.transform.translation.z = cam_pose_in_world.loc.z();
        alt_cam_transform.transform.rotation.x = cam_quat.x();
        alt_cam_transform.transform.rotation.y = cam_quat.y();
        alt_cam_transform.transform.rotation.z = cam_quat.z();
        alt_cam_transform.transform.rotation.w = cam_quat.w();

        static_tf_broadcaster_.sendTransform(alt_cam_transform);
        for (const vslam_types::EllipsoidEstimate &ellipsoid_est : ellipsoids) {
          vslam_types::RobotPose ellipsoid_pose_in_world(
              robot_pose.frame_idx,
              ellipsoid_est.loc,
              ellipsoid_est.orientation);

          vslam_types::RobotPose ellipsoid_pose_rel_cam =
              vslam_util::getPose2RelativeToPose1(cam_pose_in_world,
                                                  ellipsoid_pose_in_world);
          Eigen::Quaternionf ellipsoid_quat(ellipsoid_pose_rel_cam.angle);
          geometry_msgs::TransformStamped rel_ellipsoid_transform;
          rel_ellipsoid_transform.header.stamp = ros::Time::now();
          rel_ellipsoid_transform.header.frame_id =
              alt_cam_transform.child_frame_id;
          rel_ellipsoid_transform.child_frame_id = createEllipsoidFrameId(
              alt_cam_transform.child_frame_id, ellipsoid_est.ellipsoid_idx);
          rel_ellipsoid_transform.transform.translation.x =
              ellipsoid_pose_rel_cam.loc.x();
          rel_ellipsoid_transform.transform.translation.y =
              ellipsoid_pose_rel_cam.loc.y();
          rel_ellipsoid_transform.transform.translation.z =
              ellipsoid_pose_rel_cam.loc.z();
          rel_ellipsoid_transform.transform.rotation.x = ellipsoid_quat.x();
          rel_ellipsoid_transform.transform.rotation.y = ellipsoid_quat.y();
          rel_ellipsoid_transform.transform.rotation.z = ellipsoid_quat.z();
          rel_ellipsoid_transform.transform.rotation.w = ellipsoid_quat.w();

          static_tf_broadcaster_.sendTransform(rel_ellipsoid_transform);

          vslam_types::RobotPose cam_pose_rel_ellispoid =
              vslam_util::getPose2RelativeToPose1(ellipsoid_pose_in_world,
                                                  cam_pose_in_world);
          Eigen::Quaternionf cam_pose_rel_ellipsoid_quat(
              cam_pose_rel_ellispoid.angle);
          geometry_msgs::TransformStamped rel_cam_pose_transform;
          rel_cam_pose_transform.header.stamp = ros::Time::now();
          rel_cam_pose_transform.header.frame_id =
              createEllipsoidFrameId(prefix, ellipsoid_est.ellipsoid_idx);
          rel_cam_pose_transform.child_frame_id =
              createEllipsoidFrameId(alt_cam_transform.child_frame_id,
                                     ellipsoid_est.ellipsoid_idx) +
              "_rel_cam";
          rel_cam_pose_transform.transform.translation.x =
              cam_pose_rel_ellispoid.loc.x();
          rel_cam_pose_transform.transform.translation.y =
              cam_pose_rel_ellispoid.loc.y();
          rel_cam_pose_transform.transform.translation.z =
              cam_pose_rel_ellispoid.loc.z();
          rel_cam_pose_transform.transform.rotation.x =
              cam_pose_rel_ellipsoid_quat.x();
          rel_cam_pose_transform.transform.rotation.y =
              cam_pose_rel_ellipsoid_quat.y();
          rel_cam_pose_transform.transform.rotation.z =
              cam_pose_rel_ellipsoid_quat.z();
          rel_cam_pose_transform.transform.rotation.w =
              cam_pose_rel_ellipsoid_quat.w();

          static_tf_broadcaster_.sendTransform(rel_cam_pose_transform);
        }
      }
    }
  }

 private:
  const uint32_t kEllipsoidMarkerPubQueueSize = 100;

  const uint32_t kCameraInfoQueueSize = 100;

  const double kSleepAfterPubCreationTime = 0.5;

  const std::string kVizFrame = "map";

  tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;

  /**
   * Node handle.
   */
  ros::NodeHandle node_handle_;

  std::string prefix_;

  /**
   * Need to take care that messages published align with the message type that
   * the publisher was initialized for.
   */
  std::unordered_map<std::string, ros::Publisher> publishers_by_topic_;

  void publishMarker(visualization_msgs::Marker &marker_msg,
                     ros::Publisher &marker_pub) {
    if (marker_msg.header.frame_id.empty()) {
      marker_msg.header.frame_id = kVizFrame;
    }
    marker_msg.header.stamp = ros::Time::now();
    marker_msg.ns = "ut_vslam";
    marker_msg.action = visualization_msgs::Marker::ADD;
    marker_pub.publish(marker_msg);
  }

  void publishEllipsoid(const vslam_types::EllipsoidEstimate &ellipsoid,
                        ros::Publisher &pub,
                        const std_msgs::ColorRGBA &color) {
    visualization_msgs::Marker marker;
    marker.scale.x = ellipsoid.ellipsoid_dim.x();
    marker.scale.y = ellipsoid.ellipsoid_dim.y();
    marker.scale.z = ellipsoid.ellipsoid_dim.z();

    marker.pose.position.x = ellipsoid.loc.x();
    marker.pose.position.y = ellipsoid.loc.y();
    marker.pose.position.z = ellipsoid.loc.z();

    Eigen::Quaternionf ellipsoid_quat(ellipsoid.orientation);
    marker.pose.orientation.x = ellipsoid_quat.x();
    marker.pose.orientation.y = ellipsoid_quat.y();
    marker.pose.orientation.z = ellipsoid_quat.z();
    marker.pose.orientation.w = ellipsoid_quat.w();

    marker.id = ellipsoid.ellipsoid_idx;

    marker.type = visualization_msgs::Marker::Type::SPHERE;

    marker.color = color;

    publishMarker(marker, pub);
  }

  void publishCameraIntrinsics(const vslam_types::CameraIntrinsics &intrinsics,
                               const std::string &frame_id,
                               const std::string &topic_id,
                               const std::string &image_topic_id) {
    if (publishers_by_topic_.find(topic_id) == publishers_by_topic_.end()) {
      ros::Publisher pub = node_handle_.advertise<sensor_msgs::CameraInfo>(
          topic_id, kCameraInfoQueueSize);
      publishers_by_topic_[topic_id] = pub;
      ros::Duration(kSleepAfterPubCreationTime).sleep();
    }
    if (publishers_by_topic_.find(image_topic_id) == publishers_by_topic_.end()) {
      ros::Publisher pub = node_handle_.advertise<sensor_msgs::Image>(
          image_topic_id, kCameraInfoQueueSize);
      publishers_by_topic_[image_topic_id] = pub;
      ros::Duration(kSleepAfterPubCreationTime).sleep();
    }
    ros::Publisher intrinsics_pub = publishers_by_topic_[topic_id];
    ros::Publisher image_pub = publishers_by_topic_[image_topic_id];
    sensor_msgs::CameraInfo cam_info;
    cam_info.header.stamp = ros::Time::now();
    cam_info.header.frame_id = frame_id;
    cam_info.K[0] = intrinsics.camera_mat(0, 0);
    cam_info.K[1] = intrinsics.camera_mat(0, 1);
    cam_info.K[2] = intrinsics.camera_mat(0, 2);
    cam_info.K[3] = intrinsics.camera_mat(1, 0);
    cam_info.K[4] = intrinsics.camera_mat(1, 1);
    cam_info.K[5] = intrinsics.camera_mat(1, 2);
    cam_info.K[6] = intrinsics.camera_mat(2, 0);
    cam_info.K[7] = intrinsics.camera_mat(2, 1);
    cam_info.K[8] = intrinsics.camera_mat(2, 2);

    cam_info.width = intrinsics.camera_mat(0, 2) * 2;
    cam_info.height = intrinsics.camera_mat(1, 2) * 2;

    cam_info.P[0] = cam_info.K[0];
    cam_info.P[1] = cam_info.K[1];
    cam_info.P[2] = cam_info.K[2];
    cam_info.P[4] = cam_info.K[3];
    cam_info.P[5] = cam_info.K[4];
    cam_info.P[6] = cam_info.K[5];
    cam_info.P[7] = cam_info.K[6];
    cam_info.P[8] = cam_info.K[7];
    cam_info.P[9] = cam_info.K[8];

    cam_info.distortion_model = "plumb_bob";

    sensor_msgs::Image image;
    image.width = cam_info.width;
    image.height = cam_info.height;
    image.header.frame_id = cam_info.header.frame_id;
    image.header.stamp = cam_info.header.stamp;

    intrinsics_pub.publish(cam_info);
    image_pub.publish(image);
  }
};
}  // namespace vslam_viz

#endif  // UT_VSLAM_ROS_VISUALIZATION_H
