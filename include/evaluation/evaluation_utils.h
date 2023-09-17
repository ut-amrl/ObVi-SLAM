//
// Created by amanda on 9/16/23.
//

#ifndef UT_VSLAM_EVALUATION_UTILS_H
#define UT_VSLAM_EVALUATION_UTILS_H

#include <vector>

namespace vslam_types_refactor {

struct MetricsDistributionStatistics {
  int num_vals_ = 0;
  double average_ = -1;
  double std_dev_ = -1;
  double median_ = -1;
  double min_ = -1;
  double max_ = -1;
  double lower_quartile_ = -1;
  double upper_quartile_ = -1;
  double rmse_ = -1;
  double squared_err_std_dev_ = -1;

  std::vector<double> errors_;
};

double computeMedian(const std::vector<double> &sorted_dataset,
                     const int &min_range_idx = -1,
                     const int &max_range_idx = -1);

MetricsDistributionStatistics computeMetricsDistributionStatistics(
    const std::vector<double> &error_vals);

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_EVALUATION_UTILS_H
