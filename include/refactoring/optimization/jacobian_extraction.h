//
// Created by amanda on 1/27/23.
//

#ifndef UT_VSLAM_JACOBIAN_EXTRACTION_H
#define UT_VSLAM_JACOBIAN_EXTRACTION_H

#include <ceres/ceres.h>
#include <ceres/problem.h>
#include <file_io/cv_file_storage/generic_factor_info_file_storage_io.h>
#include <file_io/file_access_utils.h>
#include <refactoring/factors/generic_factor_info.h>

namespace vslam_types_refactor {

std::pair<std::pair<std::vector<int>, std::vector<int>>, std::vector<int>>
writeJacobianToFile(const ceres::CRSMatrix &crs_matrix,
                    const std::string &jacobian_file);

void writeJacobianMatlabFormatToFile(const ceres::CRSMatrix &crs_matrix,
                                     const std::string &jacobian_file);

void generateGenericFactorInfoForFactor(
    const std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph> &pose_graph,
    const FactorType &factor_type,
    const FeatureFactorId &feature_factor_id,
    const std::function<bool(const FactorType &,
                             const FeatureFactorId &,
                             ObjectId &)> &long_term_map_obj_retriever,
    const std::unordered_set<FrameId> &added_frames,
    const std::unordered_set<ObjectId> &added_objects,
    const std::unordered_set<FeatureId> &added_features,
    GenericFactorInfo &generic_factor_info,
    std::vector<ParameterBlockInfo> &new_param_block_info);

void displayInfoForSmallJacobian(const ceres::CRSMatrix &jacobian_mat);

void validateZeroColumnEntries(
    const std::pair<std::pair<std::vector<int>, std::vector<int>>,
                    std::vector<int>> &all_zero_columns,
    const std::vector<ParameterBlockInfo> &ordered_parameter_block_infos,

    const std::vector<ceres::ResidualBlockId> &residual_block_ids,
    const std::vector<GenericFactorInfo> &generic_factor_infos,
    const std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph> &pose_graph,
    ceres::Problem &problem_for_ltm);

void outputJacobianInfo(
    const std::string &jacobian_output_dir,
    const std::unordered_map<ceres::ResidualBlockId,
                             std::pair<vslam_types_refactor::FactorType,
                                       vslam_types_refactor::FeatureFactorId>>
        &residual_info,
    const std::unordered_map<ceres::ResidualBlockId, double>
        &block_ids_and_residuals,
    const std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph> &pose_graph,
    const std::function<bool(const FactorType &,
                             const FeatureFactorId &,
                             ObjectId &)> &long_term_map_obj_retriever,
    ceres::Problem &problem_for_ltm);
}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_JACOBIAN_EXTRACTION_H
