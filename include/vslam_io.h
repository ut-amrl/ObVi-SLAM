#ifndef __VSLAM_IO_H__
#define __VSLAM_IO_H__

#include "vslam_types.h"

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
}  // namespace vslam_io

#endif  // __VSLAM_IO_H__