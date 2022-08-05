//
// Created by amanda on 8/4/22.
//

#include <refactoring/long_term_map/long_term_map.h>

namespace vslam_types_refactor {
void AbsLongTermMap::setEllipsoidResults(
    const EllipsoidResults &ellipsoid_results) {
  ellipsoids_ = ellipsoid_results;
}

void PairwiseCovarianceLongTermMap::setPairwiseEllipsoidCovariance(
    util::BoostHashMap<std::pair<ObjectId, ObjectId>,
                       Eigen::Matrix<double, 9, 9>>
        &pairwise_ellipsoid_covariances) {
  pairwise_ellipsoid_covariances_ = pairwise_ellipsoid_covariances;
}
}  // namespace vslam_types_refactor