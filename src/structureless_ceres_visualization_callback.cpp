#include <glog/logging.h>
#include <structureless_ceres_visualization_callback.h>
#include <vslam_types.h>

#include <draw_epipolar_lines.hpp>
#include <opencv2/core/eigen.hpp>

namespace vslam_viz {

StructurelessCeresVisualizationCallback::
    StructurelessCeresVisualizationCallback(
        const vslam_types::CameraIntrinsics &intrinsics,
        const vslam_types::CameraExtrinsics &extrinsics,
        const vslam_types::UTSLAMProblem<vslam_types::VisionFeatureTrack>
            &slam_problem,
        std::vector<vslam_types::SLAMNode> *slam_nodes)
    : intrinsics_(intrinsics),
      extrinsics_(extrinsics),
      slam_problem_(slam_problem),
      slam_nodes_(slam_nodes) {}

ceres::CallbackReturnType StructurelessCeresVisualizationCallback::operator()(
    const ceres::IterationSummary &summary) {
  // DO NOT MODIFY THESE -- they are being updated during optimization
  std::vector<vslam_types::SLAMNode> nodes = *slam_nodes_;

  Eigen::Affine3d cam_to_robot_tf =
      Eigen::Translation3d(extrinsics_.translation.cast<double>()) *
      extrinsics_.rotation.cast<double>();

  // Assumes everything is zero indexed and the frames increase by 1 every time
  for (int i = 0; i < nodes.size() - 1; i++) {
    // Convert pose 1 to tf
    const vslam_types::SLAMNode node1 = nodes[i];  // i
    Eigen::Transform<double, 3, Eigen::Affine> first_robot_pose_in_world =
        vslam_types::PoseArrayToAffine<double>(&(node1.pose[3]),
                                               &(node1.pose[0]));

    // Convert pose 2 to tf
    const vslam_types::SLAMNode node2 = nodes[i + 1];  // i+1
    Eigen::Transform<double, 3, Eigen::Affine> second_robot_pose_in_world =
        vslam_types::PoseArrayToAffine<double>(&(node2.pose[3]),
                                               &(node2.pose[0]));

    Eigen::Transform<double, 3, Eigen::Affine> cam_1_to_cam_2_mat =
        cam_to_robot_tf.inverse() * first_robot_pose_in_world.inverse() *
        second_robot_pose_in_world * cam_to_robot_tf;

    // Extract Tx and R from cam_1_to_cam_2_mat
    Eigen::Matrix<double, 3, 1> t_vec = cam_1_to_cam_2_mat.translation();
    Eigen::Matrix<double, 3, 3> t_cross;  // skew symmetric
    t_cross << (0.0), -t_vec(2), t_vec(1), t_vec(2), (0.0), -t_vec(0),
        -t_vec(1), t_vec(0), (0.0);
    Eigen::Matrix<double, 3, 3> rotation = cam_1_to_cam_2_mat.linear();

    Eigen::Matrix<double, 3, 3> essential_mat = t_cross * rotation;

    // Convert to fundamental matrix with K
    Eigen::Matrix<double, 3, 3> fundamental_mat =
        intrinsics_.camera_mat.inverse().inverse().cast<double>() *
        essential_mat * intrinsics_.camera_mat.inverse().cast<double>();

    // Get cam 1 and cam 2 points
    std::vector<cv::Point_<double>> cam_1_points;
    std::vector<cv::Point_<double>> cam_2_points;

    for (const auto &ft : slam_problem_.tracks) {
      for (const auto feature : ft.second.track) {
        if (feature.frame_idx == i) {
          // In first frame
          cv::Point_<float> point1(feature.pixel.x(), feature.pixel.y());
          cam_1_points.push_back(point1);
        } else if (feature.frame_idx == i + 1) {
          // In second frame
          cv::Point_<float> point2(feature.pixel.x(), feature.pixel.y());
          cam_2_points.push_back(point2);
        }
      }
    }

    // Make empty mats to represent the image
    cv::Mat cam1_pic(1024, 1224, CV_8U, cv::Scalar(100));
    cv::Mat cam2_pic(1024, 1224, CV_8U, cv::Scalar(100));
    // Convert F to cv format
    cv::Matx<double, 3, 3> F_cv;
    cv::eigen2cv(fundamental_mat, F_cv);
    // Draw image w/ epipolar lines
    drawEpipolarLines(
        "epipolar error", F_cv, cam1_pic, cam2_pic, cam_1_points, cam_2_points);
  }

  return ceres::SOLVER_CONTINUE;
}

}  // namespace vslam_viz