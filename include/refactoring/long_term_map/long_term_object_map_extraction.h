//
// Created by amanda on 8/3/22.
//

#ifndef UT_VSLAM_LONG_TERM_OBJECT_MAP_EXTRACTION_H
#define UT_VSLAM_LONG_TERM_OBJECT_MAP_EXTRACTION_H

#include <ceres/ceres.h>
#include <refactoring/long_term_map/long_term_object_map.h>
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
template <typename FrontEndObjMapData>
class PairwiseCovarianceLongTermObjectMapExtractor {
 public:
  /**
   * Create the long term map extractor.
   *
   * @param covariance_extractor_params Parameters to be used during covariance
   * extraction.
   */
  PairwiseCovarianceLongTermObjectMapExtractor(
      const PairwiseCovarianceExtractorParams &covariance_extractor_params)
      : covariance_extractor_params_(covariance_extractor_params) {}

  ~PairwiseCovarianceLongTermObjectMapExtractor() = default;

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
  bool extractLongTermObjectMap(
      const std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph> &pose_graph,
      ceres::Problem *problem,
      const std::function<bool(FrontEndObjMapData &)>
          front_end_map_data_extractor,
      PairwiseCovarianceLongTermObjectMap<FrontEndObjMapData>
          &long_term_obj_map) {
    EllipsoidResults ellipsoid_results;
    extractEllipsoidEstimates(pose_graph, ellipsoid_results);
    long_term_obj_map.setEllipsoidResults(ellipsoid_results);

    ceres::Covariance::Options covariance_options;
    covariance_options.num_threads = covariance_extractor_params_.num_threads_;
    covariance_options.algorithm_type =
        covariance_extractor_params_.covariance_estimation_algorithm_type_;

    ceres::Covariance covariance_extractor(covariance_options);
    std::vector<ObjectId> object_ids;
    for (const auto &object_id_est_pair : ellipsoid_results.ellipsoids_) {
      object_ids.emplace_back(object_id_est_pair.first);
    }

    // Sort object ids
    std::sort(object_ids.begin(), object_ids.end());

    std::vector<std::pair<const double *, const double *>> covariance_blocks;
    for (size_t obj_1_idx = 0; obj_1_idx < object_ids.size(); obj_1_idx++) {
      ObjectId obj_1 = object_ids[obj_1_idx];
      double *obj_1_ptr;
      pose_graph->getObjectParamPointers(obj_1, &obj_1_ptr);
      for (size_t obj_2_idx = obj_1_idx + 1; obj_2_idx < object_ids.size();
           obj_2_idx++) {
        ObjectId obj_2 = object_ids[obj_2_idx];
        double *obj_2_ptr;
        pose_graph->getObjectParamPointers(obj_2, &obj_2_ptr);
        covariance_blocks.emplace_back(std::make_pair(obj_1_ptr, obj_2_ptr));
      }
    }

    bool covariance_compute_result =
        covariance_extractor.Compute(covariance_blocks, problem);

    if (!covariance_compute_result) {
      LOG(ERROR) << "Covariance computation failed";
      return false;
    }

    util::BoostHashMap<std::pair<ObjectId, ObjectId>,
                       Eigen::Matrix<double, 9, 9>>
        pairwise_ellipsoid_covariances;
    for (size_t obj_1_idx = 0; obj_1_idx < object_ids.size(); obj_1_idx++) {
      ObjectId obj_1 = object_ids[obj_1_idx];
      double *obj_1_ptr;
      pose_graph->getObjectParamPointers(obj_1, &obj_1_ptr);
      for (size_t obj_2_idx = obj_1_idx + 1; obj_2_idx < object_ids.size();
           obj_2_idx++) {
        ObjectId obj_2 = object_ids[obj_2_idx];
        double *obj_2_ptr;
        pose_graph->getObjectParamPointers(obj_2, &obj_2_ptr);
        Eigen::Matrix<double, 9, 9> cov_result;
        bool success = covariance_extractor.GetCovarianceBlock(
            obj_1_ptr, obj_2_ptr, cov_result.data());
        if (!success) {
          LOG(ERROR) << "Failed to get the covariance block for objects "
                     << obj_1 << " and " << obj_2;
          return false;
        }
        std::pair<ObjectId, ObjectId> object_pair =
            std::make_pair(obj_1, obj_2);
        pairwise_ellipsoid_covariances[object_pair] = cov_result;
      }
    }

    FrontEndObjMapData front_end_map_data;
    if (!front_end_map_data_extractor(front_end_map_data)) {
      LOG(ERROR) << "Could not extract the front end data required for the "
                    "long-term map";
      return false;
    }
    long_term_obj_map.setFrontEndObjMapData(front_end_map_data);

    long_term_obj_map.setPairwiseEllipsoidCovariance(
        pairwise_ellipsoid_covariances);
    return true;
  }

 private:
  /**
   * Covariance extractor params.
   */
  PairwiseCovarianceExtractorParams covariance_extractor_params_;
};

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_LONG_TERM_OBJECT_MAP_EXTRACTION_H
