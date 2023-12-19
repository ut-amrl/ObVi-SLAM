import os
import argparse


class PlottingConfigGeneratorConstants:
    kConfigBaseNameArg = "config_file_base_name"
    kAblationsArg = "ablations"
    kRootDir = "./metrics_plotting_configs/"
    kObjRelativePath = kRootDir + "object_metrics/"
    kSavePathRelativePath = kRootDir + "savepaths/"
    kComparisonsTrajData = "OA-SLAM,/home/amanda/rosbags/ellipsoid_slam/eer_jackal/oa_slam_out/evaluation_2023_07_v1_0821_ellipsoids_fixed/metrics.json\nORB-SLAM3,/home/amanda/rosbags/ellipsoid_slam/eer_jackal/orb_slam_3_out/orb_final_probably/metrics.json\nDROID-SLAM,/home/amanda/rosbags/ellipsoid_slam/eer_jackal/droid_slam_out/evaluation_2023_07_v1/metrics.json"
    kComparisonsObjData = "OA-SLAM,/home/amanda/rosbags/ellipsoid_slam/eer_jackal/oa_slam_out/evaluation_2023_07_v1_0821_ellipsoids_fixed/object_metrics.json"
    kComparisonsSuffix = "_compare.csv"
    kAblationsSuffix = "_ablations.csv"
    kSavePathTrajPrefix = "save_path_traj_"
    kSavePathObjPrefix = "save_path_obj_"
    kObViSLAMApproachName = "ObVi-SLAM"
    kObViSLAMResultsDir = "/home/amanda/rosbags/ellipsoid_slam/eer_jackal/ut_vslam_results/evaluation_2023_07_v1/"
    kTrajMetricsFileName = "metrics.json"
    kObjMetricsFileName = "object_metrics.json"
    kFiguresRootDir = "/home/amanda/rosbags/ellipsoid_slam/eer_jackal/paperstuff/figures/"
    kSavePathComparisionPrefix = "comparisons_"
    kSavePathAblationsPrefix = "ablations_"
    kNoShapePriorPrefix = "no_shape_prior_"
    kNoLtmPrefix = "no_ltm_"
    kNoVisFeatsPrefix = "no_vis_feats_"
    kNoShapePriorApproachName = kObViSLAMApproachName + "-S"
    kNoVisFeatsApproachName = kObViSLAMApproachName + "-VF"
    kNoLtmApproachName = kObViSLAMApproachName + "-LTM"
    kCsvExtension = ".csv"

    kPosDevAvgKey = "pos_dev_avg"
    kPosDevAvgSuffix = "pos_dev_avg_line_std_dev.svg"
    kPosDevMedKey = "pos_dev_med"
    kPosDevMedSuffix = "pos_dev_med_quartiles.svg"
    kAvgIouKey = "avg_ious"
    kAvgIouSuffix = "avg_ious_line_std_dev.svg"
    kMedIouKey = "median_ious"
    kMedIouSuffix = "median_ious_line.svg"
    kObjRecallKey = "missed_gts"
    kObjRecallSuffix = "obj_recall_line.svg"
    kObjRatioKey = "obj_ratio"
    kObjRatioSuffix = "obj_ratio_line.svg"

    kTranslCdfKey = "transl_cdf"
    kTranslCdfSuffix = "transl_cdf.svg"
    kOrientCdfKey = "orient_cdf"
    kOrientCdfSuffix = "orient_cdf.svg"
    kTranslAteKey = "transl_ate"
    kTranslAteSuffix = "transl_ate.svg"
    kOrientAteKey = "orient_ate"
    kOrientAteSuffix = "orient_ate.svg"

def generateMetricsPlottingConfig(forAblations, configBaseName):
    print("Here")

    if (forAblations):
        print("For ablations")
        trajPlottingConfigFileName = PlottingConfigGeneratorConstants.kRootDir + configBaseName + \
                                     PlottingConfigGeneratorConstants.kAblationsSuffix
        objPlottingConfigFileName = PlottingConfigGeneratorConstants.kObjRelativePath + configBaseName + \
                                    PlottingConfigGeneratorConstants.kAblationsSuffix

        print("Traj plotting config name " + trajPlottingConfigFileName)
        print("Obj plotting config name " + objPlottingConfigFileName)

        if (not os.path.exists(trajPlottingConfigFileName)):
            with open(trajPlottingConfigFileName, "w") as trajPlottingConfigFile:
                obViSLAMEntry = PlottingConfigGeneratorConstants.kObViSLAMApproachName + "," + \
                                PlottingConfigGeneratorConstants.kObViSLAMResultsDir + configBaseName + "/" + \
                                PlottingConfigGeneratorConstants.kTrajMetricsFileName + "\n"
                noShapePriorEntry = PlottingConfigGeneratorConstants.kNoShapePriorApproachName + "," + \
                                    PlottingConfigGeneratorConstants.kObViSLAMResultsDir + \
                                    PlottingConfigGeneratorConstants.kNoShapePriorPrefix + configBaseName + "/" + \
                                    PlottingConfigGeneratorConstants.kTrajMetricsFileName + "\n"
                noVisFeatsEntry = PlottingConfigGeneratorConstants.kNoVisFeatsApproachName + "," + \
                                  PlottingConfigGeneratorConstants.kObViSLAMResultsDir + \
                                  PlottingConfigGeneratorConstants.kNoVisFeatsPrefix + configBaseName + "/" + \
                                  PlottingConfigGeneratorConstants.kTrajMetricsFileName + "\n"
                noLtmEntry = PlottingConfigGeneratorConstants.kNoLtmApproachName + "," + \
                             PlottingConfigGeneratorConstants.kObViSLAMResultsDir + \
                             PlottingConfigGeneratorConstants.kNoLtmPrefix + configBaseName + "/" + \
                             PlottingConfigGeneratorConstants.kTrajMetricsFileName
                trajPlottingConfigFile.write(obViSLAMEntry)
                trajPlottingConfigFile.write(noShapePriorEntry)
                trajPlottingConfigFile.write(noVisFeatsEntry)
                trajPlottingConfigFile.write(noLtmEntry)

        if (not os.path.exists(objPlottingConfigFileName)):
            with open(objPlottingConfigFileName, "w") as objPlottingConfigFile:
                obViSLAMEntry = PlottingConfigGeneratorConstants.kObViSLAMApproachName + "," + \
                                PlottingConfigGeneratorConstants.kObViSLAMResultsDir + configBaseName + "/" + \
                                PlottingConfigGeneratorConstants.kObjMetricsFileName + "\n"
                noShapePriorEntry = PlottingConfigGeneratorConstants.kNoShapePriorApproachName + "," + \
                                    PlottingConfigGeneratorConstants.kObViSLAMResultsDir + \
                                    PlottingConfigGeneratorConstants.kNoShapePriorPrefix + configBaseName + "/" + \
                                    PlottingConfigGeneratorConstants.kObjMetricsFileName + "\n"
                noVisFeatsEntry = PlottingConfigGeneratorConstants.kNoVisFeatsApproachName + "," + \
                                  PlottingConfigGeneratorConstants.kObViSLAMResultsDir + \
                                  PlottingConfigGeneratorConstants.kNoVisFeatsPrefix + configBaseName + "/" + \
                                  PlottingConfigGeneratorConstants.kObjMetricsFileName + "\n"
                noLtmEntry = PlottingConfigGeneratorConstants.kNoLtmApproachName + "," + \
                             PlottingConfigGeneratorConstants.kObViSLAMResultsDir + \
                             PlottingConfigGeneratorConstants.kNoLtmPrefix + configBaseName + "/" + \
                             PlottingConfigGeneratorConstants.kObjMetricsFileName
                objPlottingConfigFile.write(obViSLAMEntry)
                objPlottingConfigFile.write(noShapePriorEntry)
                objPlottingConfigFile.write(noVisFeatsEntry)
                objPlottingConfigFile.write(noLtmEntry)
    else:
        print("Not for ablations")
        trajPlottingConfigFileName = PlottingConfigGeneratorConstants.kRootDir + configBaseName + \
                                     PlottingConfigGeneratorConstants.kComparisonsSuffix
        objPlottingConfigFileName = PlottingConfigGeneratorConstants.kObjRelativePath + configBaseName +\
                                    PlottingConfigGeneratorConstants.kComparisonsSuffix

        print("Traj plotting config name " + trajPlottingConfigFileName)
        print("Obj plotting config name " + objPlottingConfigFileName)

        if (not os.path.exists(trajPlottingConfigFileName)):
            with open(trajPlottingConfigFileName, "w") as trajPlottingConfigFile:
                obViSLAMEntry = PlottingConfigGeneratorConstants.kObViSLAMApproachName + "," + \
                                PlottingConfigGeneratorConstants.kObViSLAMResultsDir + configBaseName + "/" +\
                                PlottingConfigGeneratorConstants.kTrajMetricsFileName + "\n"
                trajPlottingConfigFile.write(obViSLAMEntry)
                trajPlottingConfigFile.write(PlottingConfigGeneratorConstants.kComparisonsTrajData)

        if (not os.path.exists(objPlottingConfigFileName)):
            with open(objPlottingConfigFileName, "w") as objPlottingConfigFile:
                obViSLAMEntry = PlottingConfigGeneratorConstants.kObViSLAMApproachName + "," + \
                                PlottingConfigGeneratorConstants.kObViSLAMResultsDir + configBaseName + "/" + \
                                PlottingConfigGeneratorConstants.kObjMetricsFileName + "\n"
                objPlottingConfigFile.write(obViSLAMEntry)
                objPlottingConfigFile.write(PlottingConfigGeneratorConstants.kComparisonsObjData)

    figOutDir = PlottingConfigGeneratorConstants.kFiguresRootDir + configBaseName
    if (not os.path.exists(figOutDir)):
        os.system("mkdir -p " + figOutDir)

    figOutDir += "/"

    prefixForPath = PlottingConfigGeneratorConstants.kSavePathComparisionPrefix
    if (forAblations):
        prefixForPath = PlottingConfigGeneratorConstants.kSavePathAblationsPrefix

    objSavePathFileName = PlottingConfigGeneratorConstants.kSavePathRelativePath + prefixForPath + \
                          PlottingConfigGeneratorConstants.kSavePathObjPrefix + configBaseName + \
                          PlottingConfigGeneratorConstants.kCsvExtension
    trajSavePathFileName = PlottingConfigGeneratorConstants.kSavePathRelativePath + prefixForPath + \
                           PlottingConfigGeneratorConstants.kSavePathTrajPrefix + configBaseName + \
                           PlottingConfigGeneratorConstants.kCsvExtension

    if (not os.path.exists(objSavePathFileName)):
        with open(objSavePathFileName, "w") as objSavePathFile:
            posDevAvgOutFile = figOutDir + prefixForPath + PlottingConfigGeneratorConstants.kPosDevAvgSuffix
            posDevMedOutFile = figOutDir + prefixForPath + PlottingConfigGeneratorConstants.kPosDevMedSuffix
            avgIouOutFile = figOutDir + prefixForPath + PlottingConfigGeneratorConstants.kAvgIouSuffix
            medIouOutFile = figOutDir + prefixForPath + PlottingConfigGeneratorConstants.kMedIouSuffix
            objRecallOutFile = figOutDir + prefixForPath + PlottingConfigGeneratorConstants.kObjRecallSuffix
            objRatioOutFile = figOutDir + prefixForPath + PlottingConfigGeneratorConstants.kObjRatioSuffix

            objSavePathFile.write(PlottingConfigGeneratorConstants.kPosDevAvgKey + "," + posDevAvgOutFile + "\n")
            objSavePathFile.write(PlottingConfigGeneratorConstants.kPosDevMedKey + "," + posDevMedOutFile + "\n")
            objSavePathFile.write(PlottingConfigGeneratorConstants.kAvgIouKey + "," + avgIouOutFile + "\n")
            objSavePathFile.write(PlottingConfigGeneratorConstants.kMedIouKey + "," + medIouOutFile + "\n")
            objSavePathFile.write(PlottingConfigGeneratorConstants.kObjRecallKey + "," + objRecallOutFile + "\n")
            objSavePathFile.write(PlottingConfigGeneratorConstants.kObjRatioKey + "," + objRatioOutFile)

    if (not os.path.exists(trajSavePathFileName)):
        with open(trajSavePathFileName, "w") as trajSavePathFile:
            translCdfOutFile = figOutDir + prefixForPath + PlottingConfigGeneratorConstants.kTranslCdfSuffix
            orientCdfOutFile = figOutDir + prefixForPath + PlottingConfigGeneratorConstants.kOrientCdfSuffix
            translAteOutFile = figOutDir + prefixForPath + PlottingConfigGeneratorConstants.kTranslAteSuffix
            orientAteOutFile = figOutDir + prefixForPath + PlottingConfigGeneratorConstants.kOrientAteSuffix

            trajSavePathFile.write(PlottingConfigGeneratorConstants.kTranslCdfKey + "," + translCdfOutFile + "\n")
            trajSavePathFile.write(PlottingConfigGeneratorConstants.kOrientCdfKey + "," + orientCdfOutFile + "\n")
            trajSavePathFile.write(PlottingConfigGeneratorConstants.kTranslAteKey + "," + translAteOutFile + "\n")
            trajSavePathFile.write(PlottingConfigGeneratorConstants.kOrientAteKey + "," + orientAteOutFile)


def argParse():
    parser = argparse.ArgumentParser(description="Generate metrics plotting configs")
    parser.add_argument("--" + PlottingConfigGeneratorConstants.kConfigBaseNameArg,
                        required=True,
                        help="config file base name")
    parser.add_argument("--" + PlottingConfigGeneratorConstants.kAblationsArg,
                        default=False,
                        action='store_true',
                        help="for comparisons (no if for ablations)")
    parser.add_argument('--no-' + PlottingConfigGeneratorConstants.kAblationsArg,
                        dest=PlottingConfigGeneratorConstants.kAblationsArg, action='store_false',
                        help="for ablations")

    args_dict = vars(parser.parse_args())
    forAblations = args_dict[PlottingConfigGeneratorConstants.kAblationsArg]
    configBaseName = args_dict[PlottingConfigGeneratorConstants.kConfigBaseNameArg]

    return forAblations, configBaseName


if __name__ == "__main__":
    forAblations, configBaseName = argParse()
    generateMetricsPlottingConfig(forAblations, configBaseName)
