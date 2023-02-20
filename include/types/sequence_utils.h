//
// Created by amanda on 2/19/23.
//

#ifndef UT_VSLAM_SEQUENCE_UTILS_H
#define UT_VSLAM_SEQUENCE_UTILS_H
#include <string>
#include <vector>
namespace vslam_types_refactor {
struct SequenceInfo {
  std::string sequence_id_;
  std::vector<std::string> bag_base_names_;
};
}  // namespace vslam_types_refactor
#endif  // UT_VSLAM_SEQUENCE_UTILS_H
