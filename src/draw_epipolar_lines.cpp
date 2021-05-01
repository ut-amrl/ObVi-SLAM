#include <draw_epipolar_lines.h>

#include <opencv2/core/eigen.hpp>

namespace vslam_viz {

void VisualizeEpipolarError(
    const Eigen::Affine3d &cam_to_robot_tf,
    const vslam_types::CameraIntrinsics &intrinsics,
    const vslam_types::UTSLAMProblem<vslam_types::VisionFeatureTrack>
        &slam_problem,
    const std::vector<vslam_types::SLAMNode> &nodes) {
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
        intrinsics.camera_mat.transpose().inverse().cast<double>() *
        essential_mat * intrinsics.camera_mat.inverse().cast<double>();

    // Get cam 1 and cam 2 points
    std::vector<cv::Point_<double>> cam_1_points;
    std::vector<cv::Point_<double>> cam_2_points;

    for (const auto &ft : slam_problem.tracks) {
      for (int j = 0; j < ft.second.track.size(); ++j) {
        // This conditional ensures that we only include points that have
        // matches across consecutive frame pairs
        if (ft.second.track[j].frame_idx == i &&
            ft.second.track[j + 1].frame_idx == i + 1) {
          // In first frame
          cv::Point_<float> point1(ft.second.track[j].pixel.x(),
                                   ft.second.track[j].pixel.y());
          cam_1_points.push_back(point1);
          // In second frame
          cv::Point_<float> point2(ft.second.track[j + 1].pixel.x(),
                                   ft.second.track[j + 1].pixel.y());
          cam_2_points.push_back(point2);
        }
      }
    }

    // Make empty mats to represent the image
    cv::Mat cam1_pic(480, 640, CV_8U, cv::Scalar(100));
    cv::Mat cam2_pic(480, 640, CV_8U, cv::Scalar(100));
    // Convert F to cv format
    cv::Matx<double, 3, 3> F_cv;
    cv::eigen2cv(fundamental_mat, F_cv);
    // Draw image w/ epipolar lines
    // Swapped points because of non comutativity of x`Ex
    DrawEpipolarLines(
        "Frame", F_cv, cam2_pic, cam1_pic, cam_2_points, cam_1_points);
  }
}

}  // namespace vslam_viz