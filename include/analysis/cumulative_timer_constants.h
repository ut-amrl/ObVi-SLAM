//
// Created by amanda on 6/14/23.
//

#ifndef UT_VSLAM_CUMULATIVE_TIMER_CONSTANTS_H
#define UT_VSLAM_CUMULATIVE_TIMER_CONSTANTS_H

#include <string>

namespace vslam_types_refactor {
const std::string kTimerNameFullTrajectoryExecution =
    "full_trajectory_execution";
const std::string kTimerNameVisFunction = "visualization_top_level";
const std::string kTimerNameVisualFrontendFunction =
    "visual_frontend_top_level";
const std::string kTimerNameResidualCreator = "residual_creator_top";
const std::string kTimerNameBbContextRetriever = "bb_context_retriever";
const std::string kTimerNameBbQuerier = "bb_querier";
const std::string kTimerNameFromYoloBbQuerier = "from_yolo_bb_querier";
const std::string kTimerNameFrameDataAdderTopLevel =
    "frame_data_adder_top_level";
const std::string kTimerNameOptimizationIteration = "optimization_iteration";
const std::string kTimerNameGlobalBundleAdjustment = "global_bundle_adjustment";
const std::string kTimerNameMapMergeGba = "map_merge_global_bundle_adjustment";
const std::string kTimerNameObjOnlyPgoFullProcess = "obj_only_pgo_full_process";
const std::string kTimerNameMapMergeObjOnlyPgoFullProcess =
    "map_merge_obj_only_pgo_full_process";
const std::string kTimerNameObjOnlyPgoLocalTrackBuild =
    "obj_only_pgo_local_track_build";
const std::string kTimerNameMapMergeObjOnlyPgoLocalTrackBuild =
    "map_merge_obj_only_pgo_local_track_build";
const std::string kTimerNameObjOnlyPgoLocalTrackSolve =
    "obj_only_pgo_local_track_solve";
const std::string kTimerNameMapMergeObjOnlyPgoLocalTrackSolve =
    "map_merge_obj_only_pgo_local_track_solve";
const std::string kTimerNameObjOnlyPgoBuildPgo = "obj_only_pgo_build_pgo";
const std::string kTimerNameMapMergeObjOnlyPgoBuildPgo =
    "map_merge_obj_only_pgo_build_pgo";
const std::string kTimerNameObjOnlyPgoSolvePgo = "obj_only_pgo_solve_pgo";
const std::string kTimerNameMapMergeObjOnlyPgoSolvePgo =
    "map_merge_obj_only_pgo_solve_pgo";
const std::string kTimerNameObjOnlyPgoManualFeatAdjust =
    "obj_only_pgo_manual_feat_adjust";
const std::string kTimerNameMapMergeObjOnlyPgoManualFeatAdjust =
    "map_merge_obj_only_pgo_manual_feat_adjust";
const std::string kTimerNameObjOnlyPgoOptFeatAdjustBuild =
    "obj_only_pgo_opt_feat_adjust_build";
const std::string kTimerNameMapMergeObjOnlyPgoOptFeatAdjustBuild =
    "map_merge_obj_only_pgo_opt_feat_adjust_build";
const std::string kTimerNameObjOnlyPgoOptFeatAdjustSolve =
    "obj_only_pgo_opt_feat_adjust_solve";
const std::string kTimerNameMapMergeObjOnlyPgoOptFeatAdjustSolve =
    "map_merge_obj_only_pgo_opt_feat_adjust_solve";

const std::string kTimerNameLocalBundleAdjustment = "local_bundle_adjustment";
const std::string kTimerNameConsecutivePosesStable = "consecutive_pose_stable";
const std::string kTimerNamePhaseOneLbaBuildOpt = "phase_one_lba_build_opt";
const std::string kTimerNamePhaseOneGbaBuildOpt = "phase_one_gba_build_opt";
const std::string kTimerNameMapMergePhaseOneGbaBuildOpt =
    "map_merge_phase_one_gba_build_opt";
const std::string kTimerNamePhaseOneLbaSolveOpt = "phase_one_lba_solve_opt";
const std::string kTimerNamePhaseOneGbaSolveOpt = "phase_one_gba_solve_opt";
const std::string kTimerNameMapMergePhaseOneGbaSolveOpt =
    "map_merge_phase_one_gba_solve_opt";

const std::string kTimerNamePhaseTwoLbaBuildOpt = "phase_two_lba_build_opt";
const std::string kTimerNamePhaseTwoGbaBuildOpt = "phase_two_gba_build_opt";
const std::string kTimerNameMapMergePhaseTwoGbaBuildOpt =
    "map_merge_phase_two_gba_build_opt";
const std::string kTimerNamePhaseTwoLbaSolveOpt = "phase_two_lba_solve_opt";
const std::string kTimerNamePhaseTwoGbaSolveOpt = "phase_two_gba_solve_opt";
const std::string kTimerNameMapMergePhaseTwoGbaSolveOpt =
    "map_merge_phase_two_gba_solve_opt";

const std::string kTimerNamePostOptResidualCompute =
    "post_opt_residual_compute";
const std::string kTimerNameTwoPhaseOptOutlierIdentification =
    "two_phase_opt_outlier_identification";

// Bounding box front-end
const std::string kTimerNameBbFrontEndAddBbObs = "bb_front_end_add_bb_obs";
const std::string kTimerNameBbFrontEndMergePending =
    "bb_front_end_merge_pending";
const std::string kTimerNameBbFrontEndAddObservationForObj =
    "bb_front_end_add_observation_for_obj";
const std::string kTimerNameAbsBbFrontEndGetBbAssignments =
    "abs_bb_front_end_get_bb_assignments";
const std::string kTimerNameBbFrontEndHelpersIdentifyMergeCandidates =
    "bb_front_end_helpers_identify_merge_candidates";
const std::string kTimerNameBbFrontEndHelpersGreedilyAssign =
    "bb_front_end_helpers_greedily_assign";
const std::string kTimerNameBbFrontEndHelpersRemoveStale =
    "bb_front_end_helpers_remove_stale";
const std::string kTimerNameBbFrontEndHelpersGenSingleViewEst =
    "bb_front_end_helpers_gen_single_view_est";
const std::string kTimerNameFeatBasedBbFrontEndFilterBbs =
    "feat_based_bb_front_end_filter_bbs";
const std::string kTimerNameFeatBasedBbFrontEndGenSingleBbContextInfo =
    "feat_based_bb_front_end_gen_single_bb_context_info";
const std::string kTimerNameFeatBasedBbFrontEndCreateObjInfoFromSingle =
    "feat_based_bb_front_end_create_obj_info_from_single";
const std::string kTimerNameFeatBasedBbFrontEndMergeObjAssocInfo =
    "feat_based_bb_front_end_merge_obj_assoc_info";
const std::string kTimerNameFeatBasedBbFrontEndMergeSingleBbContext =
    "feat_based_bb_front_end_merge_single_bb_context";
const std::string kTimerNameFeatBasedBbFrontEndIdentifyCandidateMatches =
    "feat_based_bb_front_end_identify_candidate_matches";
const std::string kTimerNameFeatBasedBbFrontEndPruneCandidateMatches =
    "feat_based_bb_front_end_prune_candidate_matches";
const std::string kTimerNameFeatBasedBbFrontEndScoreCandidateMatch =
    "feat_based_bb_front_end_score_candidate_match";
const std::string kTimerNameFeatBasedBbFrontEndCleanUpBbAssocRound =
    "feat_based_bb_front_end_clean_up_bb_assoc_round";
const std::string kTimerNameFeatBasedBbFrontEndSetupInitialEstimateGeneration =
    "feat_based_bb_front_end_setup_initial_estimate_generation";
const std::string kTimerNameFeatBasedBbFrontEndTryInitializeEllipsoid =
    "feat_based_bb_front_end_try_initialize_ellipsoid";
const std::string kTimerNameFeatBasedBbFrontEndMergeExistingPendingObjects =
    "feat_based_bb_front_end_merge_existing_pending_objects";
const std::string kTimerNameFeatBasedBbFrontEndSearchForObjectMerges =
    "feat_based_bb_front_end_search_for_object_merges";
const std::string kTimerNameFeatBasedBbFrontEndGetMaxFeatureIntersection =
    "feat_based_bb_front_end_get_max_feature_intersection";

// Factors
const std::string kTimerNameFactorShapePriorDouble =
    "factor_shape_prior_double";
const std::string kTimerNameFactorShapePriorJacobian =
    "factor_shape_prior_jacobian";
const std::string kTimerNameFactorReprojectionCostFunctorDouble =
    "factor_reprojection_cost_functor_double";
const std::string kTimerNameFactorReprojectionCostFunctorJacobian =
    "factor_reprojection_cost_functor_jacobian";
const std::string kTimerNameFactorAnalyticalReprojectionCostFunctorDouble =
    "factor_analytical_reprojection_cost_functor_double";
const std::string kTimerNameFactorAnalyticalReprojectionCostFunctorJacobian =
    "factor_analytical_reprojection_cost_functor_jacobian";

const std::string kTimerNameFactorBoundingBoxDouble =
    "factor_bounding_box_double";
const std::string kTimerNameFactorBoundingBoxJacobian =
    "factor_bounding_box_jacobian";
const std::string kTimerNameFactorIndependentObjectMapDouble =
    "factor_independent_object_map_double";
const std::string kTimerNameFactorIndependentObjectMapJacobian =
    "factor_independent_object_map_jacobian";
const std::string kTimerNameFactorRelativePoseDouble =
    "factor_relative_pose_double";
const std::string kTimerNameFactorRelativePoseJacobian =
    "factor_relative_pose_jacobian";
const std::string kTimerNameFactorParamPriorDouble =
    "factor_param_prior_double";
const std::string kTimerNameFactorParamPriorJacobian =
    "factor_param_prior_jacobian";

// Pose graph frame data adder
const std::string kTimerNamePgFrameDataAdderAddFrameDataAssociatedBoundingBox =
    "pg_frame_data_adder_add_frame_data_associated_bounding_box";
const std::string kTimerNamePgFrameDataAdderAddVisualFeatureFactors =
    "pg_frame_data_adder_add_visual_feature_factors";
const std::string kTimerNamePgFrameDataAdderAddConsecutiveRelativePoseFactors =
    "pg_frame_data_adder_add_relative_pose_factors";

// Residual creator
const std::string kTimerNameResidualCreatorObject = "residual_creator_object";
const std::string kTimerNameResidualCreatorShapeDim =
    "residual_creator_shape_dim";
const std::string kTimerNameResidualCreatorReprojection =
    "residual_creator_reprojection";
const std::string kTimerNameResidualCreatorRelPose =
    "residual_creator_rel_pose";
const std::string kTimerNameResidualLongTermMap =
    "residual_creator_long_term_map";

// Optimizer
const std::string kTimerNameOptimizerBuildPgo = "optimizer_build_pgo";
const std::string kTimerNameOptimizerSolveOptimization = "optimizer_solve_opt";
const std::string kTimerNameOptimizerApplyMinObsReq = "optimizer_min_obs";
const std::string kTimerNameOptimizerExtractObservationFactors =
    "optimizer_extract_observation_factors";
const std::string kTimerNameOptimizerSetVariability =
    "optimizer_set_variability";
const std::string kTimerNameOptimizerRemoveParamWithIdentifiers =
    "optimizer_remove_param_with_identifiers";
const std::string kTimerNameOptimizerAddParamBlocksWithIdentifiers =
    "optimizer_add_param_blocks_with_identifiers";
const std::string kTimerNameOptimizerAddOrRefreshResidualBlocks =
    "optimizer_add_or_refresh_residual_blocks";
const std::string kTimerNameOptimizerRemoveUnnecessaryResidualBlocks =
    "optimizer_remove_unnecessary_residual_blocks";

// Pose graph
const std::string kTimerNamePoseGraphGetObjectsWithSemanticClass =
    "pose_graph_get_objects_with_semantic_class";
const std::string kTimerNamePoseGraphGetObservationFactorsForObj =
    "pose_graph_get_observation_factors_for_obj";

// Math utils
const std::string kTimerNameMathUtilGetPose2Rel1 = "math_util_get_pose_2_rel_1";
const std::string kTimerNameMathUtilCombinePoses = "math_util_combine_poses";
const std::string kTimerNameMathUtilGetWorldFramePos =
    "math_util_get_world_frame_pos";
const std::string kTimerNameMathUtilGetProjectedPixelCoord =
    "math_util_get_projected_pixel_coord";

const std::string kTimerNameMathUtilGetCornerLocationsVector =
    "math_util_get_corner_locations_vector";
const std::string kTimerNameMathUtilGetProjectedPixelLocation =
    "math_util_get_projected_pixel_location";

const std::string kTimerNamePostSessionMapMerge = "post_session_map_merge";

const std::string kTimerNameLongTermMapExtraction = "long_term_map_extraction";
}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_CUMULATIVE_TIMER_CONSTANTS_H
