#ifndef UT_VSLAM_DRAW_EPIPOLAR_LINES_H
#define UT_VSLAM_DRAW_EPIPOLAR_LINES_H

// https://hasper.info/opencv-draw-epipolar-lines/

#include <vslam_types.h>

#include <cmath>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace vslam_viz {

/**
 * calculate the perpendicular distance between a point and a line in 2D
 *
 * @tparam T    Point and line data type
 * @param point A point in 2D in the same plane as the line
 * @param line  A line in 2D
 *
 * @return      Perpendicular distance between a point and line in 2D
 */
template <typename T>
static float distancePointLine(const cv::Point_<T> point,
                               const cv::Vec<T, 3> &line) {
  // Line is given as a*x + b*y + c = 0
  return std::fabs(line(0) * point.x + line(1) * point.y + line(2)) /
         std::sqrt(line(0) * line(0) + line(1) * line(1));
}

/**
 * Compute and draw the epipolar lines in two images
 * associated to each other by a fundamental matrix
 *
 * @tparam T1                 Type of the fundamental matrix
 * @tparam T2                 Type of the image points
 * @param title               Title of the window to display
 * @param F                   Fundamental matrix
 * @param img1                First image
 * @param img2                Second image
 * @param points1             Set of points in the first image
 * @param points2             Set of points in the second image matching to the
 *                            first set
 * @param inlierDistance      Points with a distance to the epipolar lines
 *                            higher than this are not displayed. If it is
 *                            negative, all points are displayed
 **/
template <typename T1, typename T2>
static void DrawEpipolarLines(const std::string &title,
                              const cv::Matx<T1, 3, 3> F,
                              const cv::Mat &img1,
                              const cv::Mat &img2,
                              const std::vector<cv::Point_<T2>> points1,
                              const std::vector<cv::Point_<T2>> points2,
                              const float inlierDistance = -1) {
  CV_Assert(img1.size() == img2.size() && img1.type() == img2.type());
  cv::Mat outImg(img1.rows, img1.cols * 2, CV_8UC3);
  cv::Rect rect1(0, 0, img1.cols, img1.rows);
  cv::Rect rect2(img1.cols, 0, img1.cols, img1.rows);

  /*
   * Allow color drawing
   */
  if (img1.type() == CV_8U) {
    cv::cvtColor(img1, outImg(rect1), cv::COLOR_BGR2RGB);
    cv::cvtColor(img2, outImg(rect2), cv::COLOR_BGR2RGB);
  } else {
    img1.copyTo(outImg(rect1));
    img2.copyTo(outImg(rect2));
  }

  std::vector<cv::Vec<T2, 3>> epilines1, epilines2;
  cv::computeCorrespondEpilines(points1, 1, F, epilines1);  // Index starts with
                                                            // 1
  cv::computeCorrespondEpilines(points2, 2, F, epilines2);

  CV_Assert(points1.size() == points2.size() &&
            points2.size() == epilines1.size() &&
            epilines1.size() == epilines2.size());

  cv::RNG rng(0);
  for (size_t i = 0; i < points1.size(); i++) {
    const float point1_err = distancePointLine(points1[i], epilines2[i]);
    const float point2_err = distancePointLine(points2[i], epilines1[i]);
    if (inlierDistance > 0) {
      if (point1_err > inlierDistance || point2_err > inlierDistance) {
        // The point match is no inlier
        continue;
      }
    }
    /*
     * Epipolar lines of the 1st point set are drawn in the 2nd image and
     * vice-versa
     */
    cv::Scalar color(rng(256), rng(256), rng(256));

    cv::line(outImg(rect2),
             cv::Point(0, -epilines1[i][2] / epilines1[i][1]),
             cv::Point(img1.cols,
                       -(epilines1[i][2] + epilines1[i][0] * img1.cols) /
                           epilines1[i][1]),
             color,
             std::ceil(point1_err));
    cv::circle(outImg(rect1), points1[i], 3, color, -1, cv::LINE_AA);

    cv::line(outImg(rect1),
             cv::Point(0, -epilines2[i][2] / epilines2[i][1]),
             cv::Point(img2.cols,
                       -(epilines2[i][2] + epilines2[i][0] * img2.cols) /
                           epilines2[i][1]),
             color,
             std::ceil(point2_err));
    cv::circle(outImg(rect2), points2[i], 3, color, -1, cv::LINE_AA);
  }

  cv::namedWindow(title, cv::WINDOW_NORMAL);
  cv::imshow(title, outImg);
  cv::waitKey(10);
}

template <typename FeatureTrackType>
void VisualizeEpipolarError(
    const Eigen::Affine3d &cam_to_robot_tf,
    const vslam_types::CameraIntrinsics &intrinsics,
    const vslam_types::UTSLAMProblem<FeatureTrackType> &slam_problem,
    const std::vector<vslam_types::SLAMNode> &nodes,
    std::function<std::vector<vslam_types::VisionFeature>(
        const FeatureTrackType &)> feature_retriever) {
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
      std::vector<vslam_types::VisionFeature> features =
          feature_retriever(ft.second);
      for (int j = 0; j < features.size(); ++j) {
        // This conditional ensures that we only include points that have
        // matches across consecutive frame pairs
        if (features[j].frame_idx == i && features[j + 1].frame_idx == i + 1) {
          // In first frame
          cv::Point_<float> point1(features[j].pixel.x(),
                                   features[j].pixel.y());
          cam_1_points.push_back(point1);
          // In second frame
          cv::Point_<float> point2(features[j + 1].pixel.x(),
                                   features[j + 1].pixel.y());
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

#endif  // UT_VSLAM_DRAW_EPIPOLAR_LINES_H