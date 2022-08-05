//
// Created by amanda on 8/3/22.
//

#ifndef UT_VSLAM_LONG_TERM_MAP_EXTRACTION_H
#define UT_VSLAM_LONG_TERM_MAP_EXTRACTION_H

#include <ceres/ceres.h>
#include <refactoring/long_term_map/long_term_map.h>
#include <refactoring/optimization/object_pose_graph.h>

namespace vslam_types_refactor {

/**
 * Parameters used in pairwise covariance extraction process.
 */
class PairwiseCovarianceExtractorParams {
 public:
  /**
   * See Ceres covariance documentation.
   */
  int num_threads_ = 1;

  /**
   * See Ceres covariance documentation.
   */
  ceres::CovarianceAlgorithmType covariance_estimation_algorithm_type_ =
      ceres::CovarianceAlgorithmType::SPARSE_QR;
};

/**
 * Class for extracting the pairwise covariance long-term map from the
 * optimization problem/pose graph.
 */
class PairwiseCovarianceLongTermMapExtractor {
 public:
  /**
   * Create the long term map extractor.
   *
   * @param covariance_extractor_params Parameters to be used during covariance
   * extraction.
   */
  PairwiseCovarianceLongTermMapExtractor(
      const PairwiseCovarianceExtractorParams &covariance_extractor_params)
      : covariance_extractor_params_(covariance_extractor_params) {}
  ~PairwiseCovarianceLongTermMapExtractor() = default;

  /**
   * Extract the long term map.
   *
   * @param pose_graph[in]      Pose graph from which to extract information
   *                            needed for long-term map.
   * @param problem[in]         The ceres problem from which to extract the
   *                            covariance. Assumes that the problem was left
   *                            in a complete state (not missing variables that
   *                            should be included in the long-term map or
   *                            factored into the covariance extraction.
   * @param long_term_map[out]  Long term map to update with information.
   *
   * @return True if the long term map extraction was successful. False if
   * something failed. If returns false, may or may not have updated the
   * long-term map object.
   */
  bool extractLongTermMap(
      const std::shared_ptr<const ObjectAndReprojectionFeaturePoseGraph>
          &pose_graph,
      ceres::Problem *problem,
      PairwiseCovarianceLongTermMap &long_term_map);

 private:
  /**
   * Covariance extractor params.
   */
  PairwiseCovarianceExtractorParams covariance_extractor_params_;
};

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_LONG_TERM_MAP_EXTRACTION_H
