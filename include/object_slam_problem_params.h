#ifndef UT_VSLAM_OBJECT_SLAM_PROBLEM_PARAMS_H
#define UT_VSLAM_OBJECT_SLAM_PROBLEM_PARAMS_H

namespace vslam_solver {

/**
 * Contains parameters related to the semantic shape prior factor for
 * ellipsoids.
 *
 * This will contain default values that can be overwritten before actually
 * constructing the SLAM solver.
 */
struct SemanticShapePriorParams {
  /**
   * Mean and covariance for the size of an ellipsoid by the semantic class of
   * the ellipsoid. This must contain an entry for every ellipsoid considered.
   *
   * TODO Make the code tolerant of missing ellipsoids (skip the factor and log
   * instead of failing).
   */
  std::unordered_map<std::string, std::pair<Eigen::Vector3f, Eigen::Matrix3f>>
      mean_and_cov_by_semantic_class;

  /**
   * Huber loss parameter to use for the residuals for the semantic shape
   * priors.
   */
  double huber_loss_param = 1.0;
};

/**
 * Contains parameters related to the constraints based on the detected
 * bounding box of an ellipsoid.
 *
 * This will contain default values that can be overwritten before actually
 * constructing the SLAM solver.
 */
struct EllipsoidBoundingBoxConstraintParams {
  /**
   * Covariance for bounding box measurements.
   */
  Eigen::Matrix4f bounding_box_covariance;

  /**
   * Huber loss parameter to use for the residuals for the bounding box factors.
   */
  double huber_loss_param = 1.0;
};

/**
 * Contains parameters related to solving the object SLAM problem that are
 * not specific to any particular instantiation of the SLAM problem.
 *
 * This will contain default values that can be overwritten before actually
 * constructing the SLAM solver.
 */
struct ObjectSlamProblemParams {
  /**
   * Parameters related to the semantic shape prior.
   */
  SemanticShapePriorParams semantic_shape_prior_params;

  /**
   * Parameters related to bounding box observation constraints.
   */
  EllipsoidBoundingBoxConstraintParams ellipsoid_bounding_box_constraint_params;
};
}  // namespace vslam_solver

#endif  // UT_VSLAM_OBJECT_SLAM_PROBLEM_PARAMS_H
