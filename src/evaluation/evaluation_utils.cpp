//
// Created by amanda on 9/16/23.
//

#include <evaluation/evaluation_utils.h>

#include <algorithm>
#include <cmath>
namespace vslam_types_refactor {

double computeMedian(const std::vector<double> &sorted_dataset,
                     const int &min_range_idx,
                     const int &max_range_idx) {
  if (sorted_dataset.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  int min_idx = min_range_idx;
  int max_idx = max_range_idx;
  if (min_idx < 0) {
    min_idx = 0;
  }
  if (max_idx < 0) {
    max_idx = sorted_dataset.size() - 1;
  }

  if (((max_idx - min_idx + 1) % 2) == 1) {
    // Odd number of data points, take middle of range
    int average_idx = (max_idx + min_idx) / 2.0;
    return sorted_dataset[average_idx];
  } else {
    // Even number of data points, take average of middle 2 values
    int second_idx = (max_idx + min_idx + 1) / 2;
    int first_idx = second_idx - 1;
    return (sorted_dataset[first_idx] + sorted_dataset[second_idx]) / 2.0;
  }
}

MetricsDistributionStatistics computeMetricsDistributionStatistics(
    const std::vector<double> &error_vals) {
  MetricsDistributionStatistics distribution_stats;
  if (error_vals.empty()) {
    return distribution_stats;
  }
  int num_vals = error_vals.size();
  distribution_stats.num_vals_ = num_vals;
  distribution_stats.errors_ = error_vals;

  std::vector<double> squared_errs;

  double rmse = 0;
  double average = 0;
  double min = std::numeric_limits<double>::infinity();
  double max = -std::numeric_limits<double>::infinity();
  for (const double &error : error_vals) {
    average += error;

    double squared_err = pow(error, 2);
    squared_errs.emplace_back(squared_err);
    rmse += squared_err;

    min = std::min(error, min);
    max = std::max(error, max);
  }

  distribution_stats.min_ = min;
  distribution_stats.max_ = max;

  rmse = sqrt(rmse / num_vals);
  average = average / num_vals;

  distribution_stats.rmse_ = rmse;
  distribution_stats.average_ = average;

  std::vector<double> sorted_errors = error_vals;
  std::sort(sorted_errors.begin(), sorted_errors.end());

  distribution_stats.median_ = computeMedian(sorted_errors);
  if ((sorted_errors.size() % 2) == 1) {
    // List is odd length -- split list excluding median to compute quartiles
    int median_idx = sorted_errors.size() / 2;
    distribution_stats.lower_quartile_ =
        computeMedian(sorted_errors, 0, median_idx - 1);
    distribution_stats.upper_quartile_ =
        computeMedian(sorted_errors, median_idx + 1, sorted_errors.size() - 1);
  } else {
    // List is even length -- first half gives first quartile, second half gives
    // second quartile
    int start_of_second_half_idx = sorted_errors.size() / 2;
    distribution_stats.lower_quartile_ =
        computeMedian(sorted_errors, 0, start_of_second_half_idx - 1);
    distribution_stats.upper_quartile_ = computeMedian(
        sorted_errors, start_of_second_half_idx, sorted_errors.size() - 1);
  }

  double std_dev = 0;
  double squared_err_std_dev = 0;

  for (size_t err_idx = 0; err_idx < num_vals; err_idx++) {
    double err = error_vals[err_idx];
    double squared_err = squared_errs[err_idx];

    std_dev += pow(err - average, 2);
    squared_err_std_dev += pow(squared_err - rmse, 2);
  }

  std_dev = sqrt(std_dev / num_vals);
  squared_err_std_dev = sqrt(squared_err_std_dev / num_vals);
  distribution_stats.std_dev_ = std_dev;
  distribution_stats.squared_err_std_dev_ = squared_err_std_dev;

  return distribution_stats;
}
}  // namespace vslam_types_refactor
