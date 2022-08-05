//
// Created by amanda on 8/4/22.
//

#include <refactoring/long_term_map/long_term_map_extraction.h>
#include <refactoring/output_problem_data_extraction.h>

namespace vslam_types_refactor {
bool PairwiseCovarianceLongTermMapExtractor::extractLongTermMap(
    const std::shared_ptr<const ObjectAndReprojectionFeaturePoseGraph>
        &pose_graph,
    ceres::Problem *problem,
    PairwiseCovarianceLongTermMap &long_term_map) {
  EllipsoidResults ellipsoid_results;
  extractEllipsoidEstimates(pose_graph, ellipsoid_results);
  long_term_map.setEllipsoidResults(ellipsoid_results);

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
      Eigen::Matrix<double, 9, 9>> pairwise_ellipsoid_covariances;
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
      bool success = covariance_extractor.GetCovarianceBlock(obj_1_ptr, obj_2_ptr, cov_result.data());
      if (!success) {
        LOG(ERROR) << "Failed to get the covariance block for objects " << obj_1 << " and " << obj_2;
        return false;
      }
      std::pair<ObjectId, ObjectId> object_pair = std::make_pair(obj_1, obj_2);
      pairwise_ellipsoid_covariances[object_pair] = cov_result;
    }
  }

  long_term_map.setPairwiseEllipsoidCovariance(pairwise_ellipsoid_covariances);
  return true;
}

}  // namespace vslam_types_refactor