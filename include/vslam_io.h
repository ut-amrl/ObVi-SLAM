#ifndef __VSLAM_IO_H__
#define __VSLAM_IO_H__

#include "vslam_types.h"

namespace vslam_io {

// Loads UTSLAM problem from file - defaults in place holder int descriptor type
// because descriptors are not relevant for the solver
void LoadUTSLAMProblem(const std::string data_path,
                       vslam_types::UTSLAMProblem<int>* const prob_ptr);
}  // namespace vslam_io

#endif  // __VSLAM_IO_H__