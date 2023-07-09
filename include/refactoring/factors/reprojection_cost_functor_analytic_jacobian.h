#ifndef UT_VSLAM_REFACTORING_REPROJECTION_COST_FUNCTOR_ANALYTIC_JACOBIAN_H
#define UT_VSLAM_REFACTORING_REPROJECTION_COST_FUNCTOR_ANALYTIC_JACOBIAN_H

#include <analysis/cumulative_timer_constants.h>
#include <analysis/cumulative_timer_factory.h>
#include <ceres/autodiff_cost_function.h>
#include <refactoring/types/vslam_basic_types_refactor.h>
#include <refactoring/types/vslam_math_util.h>

#include <eigen3/Eigen/Dense>

namespace vslam_types_refactor {

/**
 * Cost functor that adds a residual for Gaussian-distributed reprojection
 * error.
 */
class ReprojectionCostFunctorAnalyticJacobian
    : public ceres::SizedCostFunction<2, 6, 3> {
 public:
  /**
   * Constructor.
   *
   * @param image_feature               Pixel location of the feature in image
   * @param intrinsics                  Camera intrinsics.
   * @param extrinsics                  Camera extrinsics (pose of the camera
   *                                    relative to the robot).
   * @param reprojection_error_std_dev  Standard deviation of the reprojection
   *                                    error
   */
  ReprojectionCostFunctorAnalyticJacobian(
      const vslam_types_refactor::PixelCoord<double> &image_feature,
      const vslam_types_refactor::CameraIntrinsicsMat<double> &intrinsics,
      const vslam_types_refactor::CameraExtrinsics<double> &extrinsics,
      const double &reprojection_error_std_dev);

  virtual ~ReprojectionCostFunctorAnalyticJacobian() = default;

  virtual bool Evaluate(double const *const *parameters,
                        double *residuals,
                        double **jacobians) const {
//#ifdef RUN_TIMERS
//    std::shared_ptr<CumulativeFunctionTimer::Invocation> invoc;
//    if (jacobians != nullptr) {
//      invoc = std::make_shared<CumulativeFunctionTimer::Invocation>(
//          CumulativeTimerFactory::getInstance()
//              .getOrCreateFunctionTimer(
//                  kTimerNameFactorAnalyticalReprojectionCostFunctorJacobian)
//              .get());
//    } else {
//      invoc = std::make_shared<CumulativeFunctionTimer::Invocation>(
//          CumulativeTimerFactory::getInstance()
//              .getOrCreateFunctionTimer(
//                  kTimerNameFactorAnalyticalReprojectionCostFunctorDouble)
//              .get());
//    }
//#endif

    const double *robot_pose_block = parameters[0];
    const double *point_block = parameters[1];

    // Intermediate terms (294)
    const double _tmp0 = std::pow(robot_pose_block[5], 2);
    const double _tmp1 = std::pow(robot_pose_block[3], 2);
    const double _tmp2 = std::pow(robot_pose_block[4], 2);
    const double _tmp3 = _tmp0 + _tmp1 + _tmp2 + kEpsilon;
    const double _tmp4 = std::sqrt(_tmp3);
    const double _tmp5 = (1.0 / 2) * _tmp4;
    const double _tmp6 = std::sin(_tmp5);
    const double _tmp7 = _tmp6 / _tmp4;
    const double _tmp8 = cam_rel_bl_vec_[1] * _tmp7;
    const double _tmp9 = _tmp8 * robot_pose_block[3];
    const double _tmp10 = cam_rel_bl_vec_[3] * _tmp7;
    const double _tmp11 = _tmp10 * robot_pose_block[5];
    const double _tmp12 = cam_rel_bl_vec_[0] * _tmp7;
    const double _tmp13 = _tmp12 * robot_pose_block[4];
    const double _tmp14 = std::cos(_tmp5);
    const double _tmp15 = cam_rel_bl_vec_[2] * _tmp14;
    const double _tmp16 = _tmp11 - _tmp13 + _tmp15 + _tmp9;
    const double _tmp17 = _tmp8 * robot_pose_block[4];
    const double _tmp18 = _tmp12 * robot_pose_block[3];
    const double _tmp19 = cam_rel_bl_vec_[2] * _tmp7;
    const double _tmp20 = _tmp19 * robot_pose_block[5];
    const double _tmp21 = cam_rel_bl_vec_[3] * _tmp14;
    const double _tmp22 = -_tmp17 - _tmp18 - _tmp20 + _tmp21;
    const double _tmp23 = 2 * _tmp22;
    const double _tmp24 = _tmp16 * _tmp23;
    const double _tmp25 = _tmp8 * robot_pose_block[5];
    const double _tmp26 = _tmp10 * robot_pose_block[3];
    const double _tmp27 = _tmp19 * robot_pose_block[4];
    const double _tmp28 = cam_rel_bl_vec_[0] * _tmp14;
    const double _tmp29 = -_tmp25 + _tmp26 + _tmp27 + _tmp28;
    const double _tmp30 = _tmp10 * robot_pose_block[4];
    const double _tmp31 = _tmp12 * robot_pose_block[5];
    const double _tmp32 = _tmp19 * robot_pose_block[3];
    const double _tmp33 = cam_rel_bl_vec_[1] * _tmp14;
    const double _tmp34 = _tmp30 + _tmp31 - _tmp32 + _tmp33;
    const double _tmp35 = 2 * _tmp34;
    const double _tmp36 = _tmp29 * _tmp35;
    const double _tmp37 = _tmp24 + _tmp36;
    const double _tmp38 = _tmp22 * _tmp35;
    const double _tmp39 = -_tmp38;
    const double _tmp40 = 2 * _tmp16;
    const double _tmp41 = _tmp29 * _tmp40;
    const double _tmp42 = _tmp39 + _tmp41;
    const double _tmp43 = 2 * std::pow(_tmp16, 2);
    const double _tmp44 = -_tmp43;
    const double _tmp45 = 2 * std::pow(_tmp34, 2);
    const double _tmp46 = 1 - _tmp45;
    const double _tmp47 = _tmp44 + _tmp46;
    const double _tmp48 = std::pow(_tmp6, 2);
    const double _tmp49 = 1.0 / (_tmp3);
    const double _tmp50 = _tmp48 * _tmp49;
    const double _tmp51 = _tmp50 * robot_pose_block[4];
    const double _tmp52 = _tmp51 * robot_pose_block[3];
    const double _tmp53 = 2 * _tmp52;
    const double _tmp54 = 2 * _tmp14;
    const double _tmp55 = _tmp54 * _tmp7;
    const double _tmp56 = _tmp55 * robot_pose_block[5];
    const double _tmp57 = robot_pose_block[3] * robot_pose_block[5];
    const double _tmp58 = _tmp50 * _tmp57;
    const double _tmp59 = 2 * _tmp58;
    const double _tmp60 = _tmp55 * robot_pose_block[4];
    const double _tmp61 = _tmp0 * _tmp49;
    const double _tmp62 = _tmp48 * _tmp61;
    const double _tmp63 = -2 * _tmp62;
    const double _tmp64 = _tmp2 * _tmp50;
    const double _tmp65 = 1 - 2 * _tmp64;
    const double _tmp66 = cam_rel_bl_vec_[4] * (_tmp63 + _tmp65) +
                          cam_rel_bl_vec_[5] * (_tmp53 - _tmp56) +
                          cam_rel_bl_vec_[6] * (_tmp59 + _tmp60) +
                          robot_pose_block[0];
    const double _tmp67 = _tmp51 * robot_pose_block[5];
    const double _tmp68 = 2 * _tmp67;
    const double _tmp69 = _tmp55 * robot_pose_block[3];
    const double _tmp70 = _tmp1 * _tmp50;
    const double _tmp71 = -2 * _tmp70;
    const double _tmp72 = cam_rel_bl_vec_[4] * (_tmp53 + _tmp56) +
                          cam_rel_bl_vec_[5] * (_tmp63 + _tmp71 + 1) +
                          cam_rel_bl_vec_[6] * (_tmp68 - _tmp69) +
                          robot_pose_block[1];
    const double _tmp73 = cam_rel_bl_vec_[4] * (_tmp59 - _tmp60) +
                          cam_rel_bl_vec_[5] * (_tmp68 + _tmp69) +
                          cam_rel_bl_vec_[6] * (_tmp65 + _tmp71) +
                          robot_pose_block[2];
    const double _tmp74 = -_tmp37 * _tmp72 + _tmp37 * point_block[1] -
                          _tmp42 * _tmp73 + _tmp42 * point_block[2] -
                          _tmp47 * _tmp66 + _tmp47 * point_block[0];
    const double _tmp75 = _tmp23 * _tmp29;
    const double _tmp76 = -_tmp75;
    const double _tmp77 = _tmp16 * _tmp35;
    const double _tmp78 = _tmp76 + _tmp77;
    const double _tmp79 = _tmp38 + _tmp41;
    const double _tmp80 = 2 * std::pow(_tmp29, 2);
    const double _tmp81 = -_tmp80;
    const double _tmp82 = _tmp46 + _tmp81;
    const double _tmp83 = -_tmp66 * _tmp79 - _tmp72 * _tmp78 - _tmp73 * _tmp82 +
                          _tmp78 * point_block[1] + _tmp79 * point_block[0] +
                          _tmp82 * point_block[2];
    const double _tmp84 = std::max<double>(_tmp83, kEpsilon);
    const double _tmp85 = 1.0 / (_tmp84);
    const double _tmp86 = -_tmp24;
    const double _tmp87 = _tmp36 + _tmp86;
    const double _tmp88 = _tmp75 + _tmp77;
    const double _tmp89 = _tmp44 + _tmp81 + 1;
    const double _tmp90 = -_tmp66 * _tmp87 - _tmp72 * _tmp89 - _tmp73 * _tmp88 +
                          _tmp87 * point_block[0] + _tmp88 * point_block[2] +
                          _tmp89 * point_block[1];

    residuals[0] =
        rectified_error_multiplier_x_ * (_tmp74 * _tmp85 - rect_feature_x_);
    residuals[1] =
        rectified_error_multiplier_y_ * (_tmp85 * _tmp90 - rect_feature_y_);

    if (jacobians == nullptr) {
      return true;
    }

    const double _tmp91 = _tmp6 / (_tmp3 * std::sqrt(_tmp3));
    const double _tmp92 = _tmp1 * _tmp91;
    const double _tmp93 = -_tmp19;
    const double _tmp94 = (1.0 / 2) * _tmp49;
    const double _tmp95 = _tmp15 * _tmp94;
    const double _tmp96 = _tmp28 * _tmp94;
    const double _tmp97 = _tmp57 * _tmp96;
    const double _tmp98 = _tmp57 * _tmp91;
    const double _tmp99 = cam_rel_bl_vec_[0] * _tmp98;
    const double _tmp100 = _tmp21 * _tmp94;
    const double _tmp101 = robot_pose_block[3] * robot_pose_block[4];
    const double _tmp102 = _tmp91 * robot_pose_block[4];
    const double _tmp103 = _tmp102 * robot_pose_block[3];
    const double _tmp104 = -cam_rel_bl_vec_[3] * _tmp103 + _tmp100 * _tmp101;
    const double _tmp105 = cam_rel_bl_vec_[2] * _tmp92 - _tmp1 * _tmp95 +
                           _tmp104 - 1.0 / 2 * _tmp9 + _tmp93 + _tmp97 - _tmp99;
    const double _tmp106 = 4 * _tmp34;
    const double _tmp107 = -_tmp105 * _tmp106;
    const double _tmp108 = _tmp101 * _tmp95;
    const double _tmp109 = _tmp33 * _tmp94;
    const double _tmp110 = _tmp109 * _tmp57;
    const double _tmp111 = cam_rel_bl_vec_[1] * _tmp98;
    const double _tmp112 = cam_rel_bl_vec_[2] * _tmp103;
    const double _tmp113 = -cam_rel_bl_vec_[3] * _tmp92 + _tmp1 * _tmp100 +
                           _tmp10 + _tmp108 - _tmp110 + _tmp111 - _tmp112 -
                           1.0 / 2 * _tmp18;
    const double _tmp114 = 4 * _tmp29;
    const double _tmp115 = -_tmp113 * _tmp114;
    const double _tmp116 = _tmp107 + _tmp115;
    const double _tmp117 = -_tmp12;
    const double _tmp118 = _tmp101 * _tmp109;
    const double _tmp119 = cam_rel_bl_vec_[1] * _tmp103;
    const double _tmp120 = cam_rel_bl_vec_[2] * _tmp98 - _tmp57 * _tmp95;
    const double _tmp121 = cam_rel_bl_vec_[0] * _tmp92 - _tmp1 * _tmp96 +
                           _tmp117 - _tmp118 + _tmp119 + _tmp120 -
                           1.0 / 2 * _tmp26;
    const double _tmp122 = _tmp121 * _tmp35;
    const double _tmp123 = _tmp105 * _tmp23;
    const double _tmp124 = cam_rel_bl_vec_[0] * _tmp103 - _tmp101 * _tmp96;
    const double _tmp125 = -cam_rel_bl_vec_[3] * _tmp98 + _tmp100 * _tmp57;
    const double _tmp126 = -cam_rel_bl_vec_[1] * _tmp92 + _tmp1 * _tmp109 +
                           _tmp124 + _tmp125 - 1.0 / 2 * _tmp32 + _tmp8;
    const double _tmp127 = 2 * _tmp29;
    const double _tmp128 = _tmp113 * _tmp40 + _tmp126 * _tmp127;
    const double _tmp129 = _tmp122 + _tmp123 + _tmp128;
    const double _tmp130 = _tmp103 * _tmp54;
    const double _tmp131 = std::pow(_tmp14, 2);
    const double _tmp132 = _tmp131 * _tmp49;
    const double _tmp133 = _tmp132 * robot_pose_block[4];
    const double _tmp134 = _tmp133 * robot_pose_block[3];
    const double _tmp135 = -_tmp130 + _tmp134 - _tmp52;
    const double _tmp136 = _tmp50 * robot_pose_block[5];
    const double _tmp137 = 2 * _tmp136;
    const double _tmp138 = _tmp54 * _tmp92;
    const double _tmp139 = _tmp138 * robot_pose_block[5];
    const double _tmp140 = 4 * _tmp48 / std::pow(_tmp3, 2);
    const double _tmp141 = _tmp140 * robot_pose_block[5];
    const double _tmp142 = _tmp1 * _tmp141;
    const double _tmp143 = _tmp137 + _tmp139 - _tmp142;
    const double _tmp144 = _tmp138 * robot_pose_block[4];
    const double _tmp145 = _tmp140 * robot_pose_block[4];
    const double _tmp146 = _tmp1 * _tmp145;
    const double _tmp147 = 2 * _tmp51;
    const double _tmp148 = _tmp144 - _tmp146 + _tmp147;
    const double _tmp149 = _tmp54 * _tmp91;
    const double _tmp150 = _tmp149 * _tmp57;
    const double _tmp151 = _tmp132 * _tmp57;
    const double _tmp152 = _tmp150 - _tmp151 + _tmp58;
    const double _tmp153 = _tmp0 * _tmp91;
    const double _tmp154 = _tmp153 * _tmp54;
    const double _tmp155 = _tmp154 * robot_pose_block[3];
    const double _tmp156 = _tmp0 * _tmp140;
    const double _tmp157 = _tmp156 * robot_pose_block[3];
    const double _tmp158 = -_tmp155 + _tmp157;
    const double _tmp159 = _tmp2 * _tmp91;
    const double _tmp160 = _tmp159 * _tmp54;
    const double _tmp161 = _tmp160 * robot_pose_block[3];
    const double _tmp162 = _tmp140 * _tmp2 * robot_pose_block[3];
    const double _tmp163 = -_tmp161 + _tmp162;
    const double _tmp164 = cam_rel_bl_vec_[4] * (_tmp158 + _tmp163) +
                           cam_rel_bl_vec_[5] * (_tmp148 + _tmp152) +
                           cam_rel_bl_vec_[6] * (_tmp135 + _tmp143);
    const double _tmp165 = [&]() {
      const double base = robot_pose_block[3];
      return base * base * base;
    }();
    const double _tmp166 = _tmp50 * robot_pose_block[3];
    const double _tmp167 = _tmp140 * _tmp165 - _tmp149 * _tmp165 - 4 * _tmp166;
    const double _tmp168 = _tmp130 - _tmp134 + _tmp52;
    const double _tmp169 = _tmp1 * _tmp132;
    const double _tmp170 = _tmp102 * _tmp54 * _tmp57 - _tmp145 * _tmp57;
    const double _tmp171 = _tmp170 + _tmp55;
    const double _tmp172 =
        cam_rel_bl_vec_[4] * (_tmp143 + _tmp168) +
        cam_rel_bl_vec_[5] * (-_tmp138 + _tmp169 + _tmp171 - _tmp70) +
        cam_rel_bl_vec_[6] * (_tmp163 + _tmp167);
    const double _tmp173 = _tmp113 * _tmp23;
    const double _tmp174 = _tmp121 * _tmp127;
    const double _tmp175 = _tmp105 * _tmp40 + _tmp126 * _tmp35;
    const double _tmp176 = -_tmp173 - _tmp174 + _tmp175;
    const double _tmp177 = -_tmp150 + _tmp151 - _tmp58;
    const double _tmp178 = _tmp170 - _tmp55;
    const double _tmp179 =
        cam_rel_bl_vec_[4] * (_tmp148 + _tmp177) +
        cam_rel_bl_vec_[5] * (_tmp158 + _tmp167) +
        cam_rel_bl_vec_[6] * (_tmp138 - _tmp169 + _tmp178 + _tmp70);
    const double _tmp180 =
        -_tmp116 * _tmp73 + _tmp116 * point_block[2] - _tmp129 * _tmp66 +
        _tmp129 * point_block[0] - _tmp164 * _tmp79 - _tmp172 * _tmp82 -
        _tmp176 * _tmp72 + _tmp176 * point_block[1] - _tmp179 * _tmp78;
    const double _tmp181 =
        (1.0 / 2) *
        ((((_tmp83 - kEpsilon) > 0) - ((_tmp83 - kEpsilon) < 0)) + 1) /
        std::pow(_tmp84, 2);
    const double _tmp182 = _tmp181 * _tmp74;
    const double _tmp183 = _tmp121 * _tmp40;
    const double _tmp184 = _tmp126 * _tmp23;
    const double _tmp185 = _tmp105 * _tmp127 + _tmp113 * _tmp35;
    const double _tmp186 = _tmp183 + _tmp184 + _tmp185;
    const double _tmp187 = -_tmp122 - _tmp123 + _tmp128;
    const double _tmp188 = 4 * _tmp16;
    const double _tmp189 = -_tmp126 * _tmp188;
    const double _tmp190 = _tmp107 + _tmp189;
    const double _tmp191 = _tmp181 * _tmp90;
    const double _tmp192 = -_tmp183 - _tmp184 + _tmp185;
    const double _tmp193 = _tmp115 + _tmp189;
    const double _tmp194 = _tmp173 + _tmp174 + _tmp175;
    const double _tmp195 = _tmp154 * robot_pose_block[4];
    const double _tmp196 = _tmp156 * robot_pose_block[4];
    const double _tmp197 = -_tmp195 + _tmp196;
    const double _tmp198 = [&]() {
      const double base = robot_pose_block[4];
      return base * base * base;
    }();
    const double _tmp199 = _tmp140 * _tmp198 - _tmp149 * _tmp198 - 4 * _tmp51;
    const double _tmp200 = 2 * _tmp166;
    const double _tmp201 = _tmp161 - _tmp162 + _tmp200;
    const double _tmp202 = _tmp102 * robot_pose_block[5];
    const double _tmp203 = _tmp202 * _tmp54;
    const double _tmp204 = _tmp133 * robot_pose_block[5];
    const double _tmp205 = _tmp203 - _tmp204 + _tmp67;
    const double _tmp206 = _tmp132 * _tmp2;
    const double _tmp207 =
        cam_rel_bl_vec_[4] * (_tmp197 + _tmp199) +
        cam_rel_bl_vec_[5] * (_tmp201 + _tmp205) +
        cam_rel_bl_vec_[6] * (-_tmp160 + _tmp171 + _tmp206 - _tmp64);
    const double _tmp208 = robot_pose_block[4] * robot_pose_block[5];
    const double _tmp209 = -cam_rel_bl_vec_[3] * _tmp202 + _tmp100 * _tmp208;
    const double _tmp210 = cam_rel_bl_vec_[0] * _tmp159 + _tmp117 + _tmp118 -
                           _tmp119 - _tmp2 * _tmp96 + _tmp209 -
                           1.0 / 2 * _tmp27;
    const double _tmp211 = -_tmp188 * _tmp210;
    const double _tmp212 = _tmp208 * _tmp96;
    const double _tmp213 = cam_rel_bl_vec_[0] * _tmp202;
    const double _tmp214 = -cam_rel_bl_vec_[3] * _tmp159 + _tmp10 +
                           _tmp100 * _tmp2 - _tmp108 + _tmp112 -
                           1.0 / 2 * _tmp17 + _tmp212 - _tmp213;
    const double _tmp215 = -_tmp106 * _tmp214;
    const double _tmp216 = _tmp211 + _tmp215;
    const double _tmp217 = _tmp160 * robot_pose_block[5];
    const double _tmp218 = _tmp141 * _tmp2;
    const double _tmp219 = _tmp137 + _tmp217 - _tmp218;
    const double _tmp220 = -_tmp203 + _tmp204 - _tmp67;
    const double _tmp221 = -_tmp144 + _tmp146;
    const double _tmp222 = cam_rel_bl_vec_[4] * (_tmp201 + _tmp220) +
                           cam_rel_bl_vec_[5] * (_tmp197 + _tmp221) +
                           cam_rel_bl_vec_[6] * (_tmp168 + _tmp219);
    const double _tmp223 = _tmp214 * _tmp23;
    const double _tmp224 = -_tmp8;
    const double _tmp225 = _tmp208 * _tmp95;
    const double _tmp226 = cam_rel_bl_vec_[2] * _tmp202;
    const double _tmp227 = cam_rel_bl_vec_[1] * _tmp159 - _tmp109 * _tmp2 +
                           _tmp124 + _tmp224 - _tmp225 + _tmp226 -
                           1.0 / 2 * _tmp30;
    const double _tmp228 = _tmp227 * _tmp35;
    const double _tmp229 = cam_rel_bl_vec_[1] * _tmp202 - _tmp109 * _tmp208;
    const double _tmp230 = -cam_rel_bl_vec_[2] * _tmp159 + _tmp104 -
                           1.0 / 2 * _tmp13 + _tmp19 + _tmp2 * _tmp95 + _tmp229;
    const double _tmp231 = 2 * _tmp230;
    const double _tmp232 = _tmp127 * _tmp210 + _tmp16 * _tmp231;
    const double _tmp233 = -_tmp223 - _tmp228 + _tmp232;
    const double _tmp234 =
        cam_rel_bl_vec_[4] * (_tmp160 + _tmp178 - _tmp206 + _tmp64) +
        cam_rel_bl_vec_[5] * (_tmp135 + _tmp219) +
        cam_rel_bl_vec_[6] * (_tmp199 + _tmp221);
    const double _tmp235 = _tmp210 * _tmp23;
    const double _tmp236 = _tmp227 * _tmp40;
    const double _tmp237 = _tmp127 * _tmp214 + _tmp230 * _tmp35;
    const double _tmp238 = _tmp235 + _tmp236 + _tmp237;
    const double _tmp239 = -_tmp114 * _tmp230;
    const double _tmp240 = _tmp215 + _tmp239;
    const double _tmp241 = _tmp22 * _tmp231;
    const double _tmp242 = _tmp127 * _tmp227;
    const double _tmp243 = _tmp210 * _tmp35 + _tmp214 * _tmp40;
    const double _tmp244 = -_tmp241 - _tmp242 + _tmp243;
    const double _tmp245 = _tmp223 + _tmp228 + _tmp232;
    const double _tmp246 =
        _tmp181 * (-_tmp207 * _tmp79 - _tmp222 * _tmp78 - _tmp234 * _tmp82 -
                   _tmp240 * _tmp73 + _tmp240 * point_block[2] -
                   _tmp244 * _tmp72 + _tmp244 * point_block[1] -
                   _tmp245 * _tmp66 + _tmp245 * point_block[0]);
    const double _tmp247 = _tmp211 + _tmp239;
    const double _tmp248 = _tmp241 + _tmp242 + _tmp243;
    const double _tmp249 = -_tmp235 - _tmp236 + _tmp237;
    const double _tmp250 = (1.0 / 2) * _tmp61;
    const double _tmp251 = -cam_rel_bl_vec_[0] * _tmp153 + _tmp12 + _tmp120 +
                           _tmp209 - 1.0 / 2 * _tmp25 + _tmp250 * _tmp28;
    const double _tmp252 = -_tmp106 * _tmp251;
    const double _tmp253 = -cam_rel_bl_vec_[3] * _tmp153 + _tmp10 + _tmp110 -
                           _tmp111 - 1.0 / 2 * _tmp20 + _tmp21 * _tmp250 -
                           _tmp212 + _tmp213;
    const double _tmp254 = -_tmp188 * _tmp253;
    const double _tmp255 = _tmp252 + _tmp254;
    const double _tmp256 = _tmp23 * _tmp251;
    const double _tmp257 = cam_rel_bl_vec_[2] * _tmp153 - 1.0 / 2 * _tmp11 -
                           _tmp15 * _tmp250 + _tmp229 + _tmp93 - _tmp97 +
                           _tmp99;
    const double _tmp258 = _tmp257 * _tmp35;
    const double _tmp259 = cam_rel_bl_vec_[1] * _tmp153 + _tmp125 + _tmp224 +
                           _tmp225 - _tmp226 - _tmp250 * _tmp33 -
                           1.0 / 2 * _tmp31;
    const double _tmp260 = _tmp127 * _tmp253 + _tmp259 * _tmp40;
    const double _tmp261 = -_tmp256 - _tmp258 + _tmp260;
    const double _tmp262 = -_tmp217 + _tmp218;
    const double _tmp263 = [&]() {
      const double base = robot_pose_block[5];
      return base * base * base;
    }();
    const double _tmp264 = -4 * _tmp136 + _tmp140 * _tmp263 - _tmp149 * _tmp263;
    const double _tmp265 = _tmp155 - _tmp157 + _tmp200;
    const double _tmp266 = _tmp131 * _tmp61;
    const double _tmp267 =
        cam_rel_bl_vec_[4] * (_tmp262 + _tmp264) +
        cam_rel_bl_vec_[5] * (_tmp154 + _tmp178 - _tmp266 + _tmp62) +
        cam_rel_bl_vec_[6] * (_tmp220 + _tmp265);
    const double _tmp268 = -_tmp139 + _tmp142;
    const double _tmp269 = _tmp147 + _tmp195 - _tmp196;
    const double _tmp270 =
        cam_rel_bl_vec_[4] * (-_tmp154 + _tmp171 + _tmp266 - _tmp62) +
        cam_rel_bl_vec_[5] * (_tmp264 + _tmp268) +
        cam_rel_bl_vec_[6] * (_tmp152 + _tmp269);
    const double _tmp271 = cam_rel_bl_vec_[4] * (_tmp205 + _tmp265) +
                           cam_rel_bl_vec_[5] * (_tmp177 + _tmp269) +
                           cam_rel_bl_vec_[6] * (_tmp262 + _tmp268);
    const double _tmp272 = _tmp257 * _tmp40;
    const double _tmp273 = _tmp23 * _tmp253;
    const double _tmp274 = _tmp127 * _tmp251 + _tmp259 * _tmp35;
    const double _tmp275 = _tmp272 + _tmp273 + _tmp274;
    const double _tmp276 = _tmp256 + _tmp258 + _tmp260;
    const double _tmp277 = _tmp127 * _tmp257;
    const double _tmp278 = _tmp23 * _tmp259;
    const double _tmp279 = _tmp251 * _tmp40 + _tmp253 * _tmp35;
    const double _tmp280 = -_tmp277 - _tmp278 + _tmp279;
    const double _tmp281 = -_tmp114 * _tmp259;
    const double _tmp282 = _tmp252 + _tmp281;
    const double _tmp283 =
        -_tmp267 * _tmp79 - _tmp270 * _tmp78 - _tmp271 * _tmp82 -
        _tmp276 * _tmp66 + _tmp276 * point_block[0] - _tmp280 * _tmp72 +
        _tmp280 * point_block[1] - _tmp282 * _tmp73 + _tmp282 * point_block[2];
    const double _tmp284 = _tmp277 + _tmp278 + _tmp279;
    const double _tmp285 = _tmp254 + _tmp281;
    const double _tmp286 = -_tmp272 - _tmp273 + _tmp274;
    const double _tmp287 = _tmp45 - 1;
    const double _tmp288 = -_tmp41;
    const double _tmp289 = _tmp288 + _tmp39;
    const double _tmp290 = -_tmp36;
    const double _tmp291 = -_tmp77;
    const double _tmp292 = _tmp291 + _tmp75;
    const double _tmp293 = _tmp287 + _tmp80;

    double *robot_pose_jacobian = jacobians[0];
    double *point_jacobian = jacobians[1];

    if (robot_pose_jacobian != nullptr) {
      // Res 0, transl 0
      robot_pose_jacobian[0] =
          rectified_error_multiplier_x_ *
          (-_tmp182 * _tmp289 + _tmp85 * (_tmp287 + _tmp43));

      // Res 0, transl 1
      robot_pose_jacobian[1] =
          rectified_error_multiplier_x_ *
          (-_tmp182 * _tmp292 + _tmp85 * (_tmp290 + _tmp86));

      // Res 0, transl 2
      robot_pose_jacobian[2] =
          rectified_error_multiplier_x_ *
          (-_tmp182 * _tmp293 + _tmp85 * (_tmp288 + _tmp38));

      // Res 0, ang 0
      robot_pose_jacobian[3] =
          rectified_error_multiplier_x_ *
          (-_tmp180 * _tmp182 +
           _tmp85 * (-_tmp164 * _tmp47 - _tmp172 * _tmp42 - _tmp179 * _tmp37 -
                     _tmp186 * _tmp72 + _tmp186 * point_block[1] -
                     _tmp187 * _tmp73 + _tmp187 * point_block[2] -
                     _tmp190 * _tmp66 + _tmp190 * point_block[0]));

      // Res 0, ang 1
      robot_pose_jacobian[4] =
          rectified_error_multiplier_x_ *
          (-_tmp246 * _tmp74 +
           _tmp85 *
               (-_tmp207 * _tmp47 - _tmp216 * _tmp66 +
                _tmp216 * point_block[0] - _tmp222 * _tmp37 - _tmp233 * _tmp73 +
                _tmp233 * point_block[2] - _tmp234 * _tmp42 - _tmp238 * _tmp72 +
                _tmp238 * point_block[1]));

      // Res 0, ang 2
      robot_pose_jacobian[5] =
          rectified_error_multiplier_x_ *
          (-_tmp182 * _tmp283 +
           _tmp85 * (-_tmp255 * _tmp66 + _tmp255 * point_block[0] -
                     _tmp261 * _tmp73 + _tmp261 * point_block[2] -
                     _tmp267 * _tmp47 - _tmp270 * _tmp37 - _tmp271 * _tmp42 -
                     _tmp275 * _tmp72 + _tmp275 * point_block[1]));

      // Res 1, transl 0
      robot_pose_jacobian[6] =
          rectified_error_multiplier_y_ *
          (-_tmp191 * _tmp289 + _tmp85 * (_tmp24 + _tmp290));

      // Res 1, transl 1
      robot_pose_jacobian[7] =
          rectified_error_multiplier_y_ *
          (-_tmp191 * _tmp292 + _tmp85 * (_tmp43 + _tmp80 - 1));

      // Res 1, transl 2
      robot_pose_jacobian[8] =
          rectified_error_multiplier_y_ *
          (-_tmp191 * _tmp293 + _tmp85 * (_tmp291 + _tmp76));

      // Res 1, ang 0
      robot_pose_jacobian[9] =
          rectified_error_multiplier_y_ *
          (-_tmp180 * _tmp191 +
           _tmp85 * (-_tmp164 * _tmp87 - _tmp172 * _tmp88 - _tmp179 * _tmp89 -
                     _tmp192 * _tmp66 + _tmp192 * point_block[0] -
                     _tmp193 * _tmp72 + _tmp193 * point_block[1] -
                     _tmp194 * _tmp73 + _tmp194 * point_block[2]));
      // Res 1, ang 1
      robot_pose_jacobian[10] =
          rectified_error_multiplier_y_ *
          (-_tmp246 * _tmp90 +
           _tmp85 * (-_tmp207 * _tmp87 - _tmp222 * _tmp89 - _tmp234 * _tmp88 -
                     _tmp247 * _tmp72 + _tmp247 * point_block[1] -
                     _tmp248 * _tmp73 + _tmp248 * point_block[2] -
                     _tmp249 * _tmp66 + _tmp249 * point_block[0]));
      // Res 1, ang 2
      robot_pose_jacobian[11] =
          rectified_error_multiplier_y_ *
          (-_tmp191 * _tmp283 +
           _tmp85 * (-_tmp267 * _tmp87 - _tmp270 * _tmp89 - _tmp271 * _tmp88 -
                     _tmp284 * _tmp73 + _tmp284 * point_block[2] -
                     _tmp285 * _tmp72 + _tmp285 * point_block[1] -
                     _tmp286 * _tmp66 + _tmp286 * point_block[0]));
    }

    if (point_jacobian != nullptr) {
      // Jacobian is 2x3

      // 0, 0
      point_jacobian[0] =
          rectified_error_multiplier_x_ * (-_tmp182 * _tmp79 + _tmp47 * _tmp85);

      // 0, 1
      point_jacobian[1] =
          rectified_error_multiplier_x_ * (-_tmp182 * _tmp78 + _tmp37 * _tmp85);

      // 0, 2
      point_jacobian[2] =
          rectified_error_multiplier_x_ * (-_tmp182 * _tmp82 + _tmp42 * _tmp85);

      // 1, 0
      point_jacobian[3] =
          rectified_error_multiplier_y_ * (-_tmp191 * _tmp79 + _tmp85 * _tmp87);

      // 1, 1
      point_jacobian[4] =
          rectified_error_multiplier_y_ * (-_tmp191 * _tmp78 + _tmp85 * _tmp89);

      // 1, 2
      point_jacobian[5] =
          rectified_error_multiplier_y_ * (-_tmp191 * _tmp82 + _tmp85 * _tmp88);
    }

    return true;
  }

  /**
   * Create the autodiff cost function with this cost functor.
   *
   * @param intrinsics                  Camera intrinsics.
   * @param extrinsics                  Camera extrinsics (provide camera pose
   *                                    relative to robot).
   * @param feature_pixel               Pixel location of the feature.
   * @param reprojection_error_std_dev  Standard deviation of the reprojection
   *                                    error (assuming this is normally
   *                                    distributed).
   *
   * @return Ceres cost function.
   */
  static ReprojectionCostFunctorAnalyticJacobian *create(
      const vslam_types_refactor::CameraIntrinsicsMat<double> &intrinsics,
      const vslam_types_refactor::CameraExtrinsics<double> &extrinsics,
      const vslam_types_refactor::PixelCoord<double> &feature_pixel,
      const double &reprojection_error_std_dev) {
    return new ReprojectionCostFunctorAnalyticJacobian(
        feature_pixel, intrinsics, extrinsics, reprojection_error_std_dev);
  }

 private:
  const double kEpsilon = 1e-15;
  double rect_feature_x_;

  double rect_feature_y_;

  double rectified_error_multiplier_x_;

  double rectified_error_multiplier_y_;

  Eigen::Matrix<double, 7, 1> cam_rel_bl_vec_;
};
}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_REFACTORING_REPROJECTION_COST_FUNCTOR_ANALYTIC_JACOBIAN_H
