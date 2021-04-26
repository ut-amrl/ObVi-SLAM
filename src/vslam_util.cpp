#include <vslam_util.h>

namespace vslam_util {

/**
 * Calculate the essential matrix given a relative transform
 *
 * @param rel_tf[in]    Relative transfrom between two poses
 *
 * @return  Essential matrix corresponding to the realtive transform
 */
template <typename T>
void CalcEssentialMatrix(Eigen::Transform<T, 3, Eigen::Affine> const& rel_tf,
                         Eigen::Matrix<T, 3, 3>* const essential_mat) {
  // Extract Tx and R rel_tf (rel_tf aka "cam 1 to cam 2 tf")
  Eigen::Matrix<T, 3, 1> t_vec = rel_tf.translation();
  Eigen::Matrix<T, 3, 3> t_cross;  // skew symmetric
  t_cross << T(0.0), -t_vec(2), t_vec(1), t_vec(2), T(0.0), -t_vec(0),
      -t_vec(1), t_vec(0), T(0.0);
  Eigen::Matrix<T, 3, 3> rotation = rel_tf.linear();

  *essential_mat = t_cross * rotation;

  return;
}

}  // namespace vslam_util