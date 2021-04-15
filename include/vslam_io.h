#ifndef __VSLAM_IO_H__
#define __VSLAM_IO_H__

#include "vslam_types.h"

namespace vslam_io {

/**
 * \brief   Loads a unstuctured slam problem from file.
 *
 * \details This function loads an unstructured slam problem for a directory
 * containing data files. The first data file is am integer frame ID, the second
 * line is the frame position in the global frame (x, y, z, qx, qy, qz, qw). All
 * other lines are images keypoints (ID, x_pixel, y_pixel)
 *
 * \note    There should be no non data .txt files in the data directory -
 * potentially the system could confuse these for a valid data file
 *
 * \param[in]     data_path    Absolute or relative path to the data directory
 * \param[in,out] prob    The vslam_types::UTSLAMProblem() that the data will be
 * loaded into - this essentially consists of robot/frame poses and feature
 * tracks
 *
 * \return        void
 */
void LoadStructurelessUTSLAMProblem(
    std::string const& data_path,
    vslam_types::UTSLAMProblem<vslam_types::VisionFeatureTrack>& prob);
}  // namespace vslam_io

#endif  // __VSLAM_IO_H__