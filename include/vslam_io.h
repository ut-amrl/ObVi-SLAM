#ifndef __VSLAM_IO_H__
#define __VSLAM_IO_H__

#include <glog/logging.h>
#include <vslam_types.h>

namespace vslam_io {

/**
 * Loads a unstructured slam problem from file. This function loads an
 * unstructured slam problem for a directory containing data files. Each data
 * file  is an integer frame ID on line one, the second line is the frame
 * position in the global frame (x, y, z, qx, qy, qz, qw). All other lines are
 * images keypoints (ID, x_pixel, y_pixel)
 *
 * @param data_path[in]     Absolute or relative path to the data directory
 * @param prob[out]         The vslam_types::UTSLAMProblem<>() that the data
 *                          will be loaded into - this essentially consists of
 *                          robot/frame poses and feature tracks
 *@param camera_mat[out]    Camera intrinsic calibration matrix
 */
void LoadStructurelessUTSLAMProblem(
    const std::string& data_path,
    vslam_types::UTSLAMProblem<vslam_types::VisionFeatureTrack>& prob,
    Eigen::Matrix3f& camera_mat);

void LoadStructurelessUTSLAMProblemMicrosoft(
    const std::string& data_path,
    vslam_types::UTSLAMProblem<vslam_types::VisionFeatureTrack>& prob,
    Eigen::Matrix3f& camera_mat);

void LoadStructurelessUTSLAMProblemTartan(
    const std::string& data_path,
    vslam_types::UTSLAMProblem<vslam_types::VisionFeatureTrack>& prob,
    Eigen::Matrix3f& camera_mat);

/**
 * Loads a structured slam problem from file. This function loads an
 * structured slam problem for a directory containing data files. Each data
 * file  is an integer frame ID on line one, the second line is the frame
 * position in the global frame (x, y, z, qx, qy, qz, qw). All other lines are
 * images keypoints (ID, x_pixel, y_pixel). The feature file contains features
 *IDs and the 3D location of the feature in the world frame (ID, x, y, z)
 *
 * @param data_path[in]     Absolute or relative path to the data directory
 * @param prob[out]         The vslam_types::UTSLAMProblem<>() that the data
 *                          will be loaded into - this essentially consists of
 *                          robot/frame poses, vision feature tracks and 3D
 *                          feature points
 *@param camera_mat[out]    Camera intrinsic calibration matrix
 */
void LoadStructuredUTSLAMProblem(
    const std::string& data_path,
    vslam_types::UTSLAMProblem<vslam_types::StructuredVisionFeatureTrack>& prob,
    Eigen::Matrix3f& camera_mat);

/**
 * Loads a camera calibration into the camera calibration intrinsic K
 * matrix
 *
 * @param calibration_path[in]  Full path to the calibration file
 * @param camera_mat[out]       Camera intrinsic calibration matrix K
 *
 */
void LoadCameraCalibration(const std::string& calibration_path,
                           Eigen::Matrix3f& camera_mat);

/**
 * Set the robot poses in the SLAM problem.
 *
 * @tparam FeatureTrackType Type of the feature track in the SLAM problem.
 * @param poses_by_id[in]   Map of robot poses by the frame id.
 * @param prob[out]         SLAM problem to update.
 */
template <typename FeatureTrackType>
void SetRobotPosesInSlamProblem(
    const std::unordered_map<uint64_t, vslam_types::RobotPose>& poses_by_id,
    vslam_types::UTSLAMProblem<FeatureTrackType>& prob) {
  for (uint64_t frame_num = 0; frame_num < poses_by_id.size(); frame_num++) {
    if (poses_by_id.find(frame_num) == poses_by_id.end()) {
      LOG(ERROR) << "No pose found for frame num (after subtracting 1) "
                 << frame_num;
      return;
    }
    prob.robot_poses.emplace_back(poses_by_id.at(frame_num));
  }
}

/**
 * Read the features from a file stream.
 *
 * Assumes that the line containing the pose has already been read from the
 * stream.
 *
 * @param data_file_stream[in]          Data file stream to read features from.
 * @param frame_id[in]                  Frame id that the features are for.
 * @param feature_track_retriever[out]  Function that retrieves the feature
 *                                      track to update based on the feature id
 *                                      read in.
 */
void ReadFeaturesFromFile(
    std::ifstream& data_file_stream,
    const uint64_t& frame_id,
    std::function<vslam_types::VisionFeatureTrack*(const uint64_t&)>
        feature_track_retriever);

/**
 * Read the estimated robot pose and features from a file stream containing
 * information for a single frame.
 *
 * @param data_file_stream[in]          Data file stream to read robot pose and
 *                                      features from.
 * @param poses_by_id[in]               Structure to add the robot pose by the
 *                                      frame id to.
 * @param feature_track_retriever[out]  Function that retrieves the feature
 *                                      track to update based on the feature id
 *                                      read in.
 */
void ReadRobotPoseAndFeaturesFromFile(
    std::ifstream& data_file_stream,
    std::unordered_map<uint64_t, vslam_types::RobotPose>& poses_by_id,
    std::function<vslam_types::VisionFeatureTrack*(const uint64_t&)>
        feature_track_retriever);

/**
 * Extract all of the data from the files with the detected features and
 * estimated robot poses for each frame.
 *
 * @param dataset_path[in]              Path to the dataset directory.
 * @param feature_track_retriever[out]  Function that retrieves the feature
 *                                      track to update based on the feature id
 *                                      read in.
 * @param poses_by_id[out]              Map of initial robot pose estimates by
 *                                      frame id.
 *
 * @return True if the if reading succeeded, false if it failed.
 */
bool ReadAllFeatureFiles(
    const std::string& dataset_path,
    std::function<vslam_types::VisionFeatureTrack*(const uint64_t&)>
        feature_track_retriever,
    std::unordered_map<uint64_t, vslam_types::RobotPose>& poses_by_id);

/**
 * Read the initial feature estimates from a file.
 *
 * @param features_file[in]             File containing initial estimates for
 *                                      feature locations.
 * @param feature_estimates_by_id[out]  Estimates for feature locations by the
 *                                      feature id.
 *
 * @return True if the reading succeeded, false if it failed.
 */
bool LoadInitialFeatureEstimates(
    const std::string& features_file,
    std::unordered_map<uint64_t, Eigen::Vector3d>& feature_estimates_by_id);

}  // namespace vslam_io

#endif  // __VSLAM_IO_H__