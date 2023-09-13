import argparse

from approach_metrics import *


def runPlotter(approaches_and_metrics_file_name, error_types_and_savepaths_file_name=None):
    metricsFilesInfo = readApproachesAndMetricsFile(approaches_and_metrics_file_name)

    missedGtObjs = {}
    objectsPerGtObj = {}

    avgPosDeviations = {}
    avgIous = {}
    medianPosDeviations = {}
    medianIous = {}

    for approachName, metricsFile in metricsFilesInfo.approachNameAndMetricsFileInfo.items():
        print("Reading results for " + approachName)
        approachMetrics = readObjectsMetricsFile(metricsFile)

        missedGtObjsPerTraj = []
        objectsPerGtObjPerTraj = []

        avgPosDeviationsPerTraj = []
        avgIousPerTraj = []
        medianPosDeviationsPerTraj = []
        medianIousPerTraj = []

        for indiv_traj_metric_set in approachMetrics.indiv_trajectory_object_metrics:
            missedGtObjsPerTraj.append(indiv_traj_metric_set.missed_gt_objs)
            objectsPerGtObjPerTraj.append(indiv_traj_metric_set.objects_per_gt_obj)
            avgPosDeviationsPerTraj.append(indiv_traj_metric_set.average_pos_deviation)
            avgIousPerTraj.append(indiv_traj_metric_set.avg_iou)
            medianPosDeviationsPerTraj.append(indiv_traj_metric_set.median_pos_deviation)
            medianIousPerTraj.append(indiv_traj_metric_set.median_iou)

        missedGtObjs[approachName] = missedGtObjsPerTraj
        objectsPerGtObj[approachName] = objectsPerGtObjPerTraj
        avgPosDeviations[approachName] = avgPosDeviationsPerTraj
        avgIous[approachName] = avgIousPerTraj
        medianPosDeviations[approachName] = medianPosDeviationsPerTraj
        medianIous[approachName] = medianIousPerTraj

    errorTypesAndSavepaths = readErrTypesAndSavepathsFile(error_types_and_savepaths_file_name)

    # comparison
    # posDev_y_lims = None
    # dev_height_ratios = None
    # posDevLegendLoc="upper left"

    # Ablations
    # posDev_y_lims=[(0, 7), (20, 80), (1500, 2700), (2700, 9200), (9200, 2600000)]
    # dev_height_ratios=[1, 1, 1, 1, 3]
    # posDevLegendLoc="upper left"

    # Ablations v2
    # data clumps: 0-12.5
    # 18-24
    # 74
    #
    # 1702.0294601450921, 2234.7263255833327, 2291.6132405210865, 2443.0238495021567, 2609.7453652838276, 3133.219807222736, 3218.3879357744145, 3862.4604199141168, 3965.0710297364967, 4033.1715693161177, 4642.179776890731, 4768.872521695702, 4875.233192844391, 5256.993691265597, 5404.876131796244, 5851.665448928461, 7102.182963215413, 9117.014865759034,\
    # 34289.87480059462, 56979.70088890067, 135887.58326993897, 320073.27785990667, 461991.27553753025, 628519.3234581841, 1201421.3476381698, 2576100.332544633
    posDev_y_lims=[(0, 12.5), (15, 24), (70, 78), (1600, 10000),(33000, 35000), (50000, 700000), (1150000, 2600000)]

    allDevs = []
    for approachLabel, dev_list in avgPosDeviations.items():
        allDevs.extend(dev_list)

    sorted(allDevs)
    allDevs.sort()
    print(allDevs)

    # dev_height_ratios=[1 for i in posDev_y_lims]
    # posDev_y_lims=[(min(allDevs)/2.5, max(allDevs)*100)]
    posDev_y_lims=[(min(allDevs)/2, max(allDevs)*10)]
    dev_height_ratios=None
    posDevLegendLoc="upper right"

    print(avgPosDeviations)
    maxes = {approachLabel:max(dev_list) for approachLabel, dev_list in avgPosDeviations.items()}
    print(maxes)
    avgDevLegendNcol=2
    devScaleType="log"
    # devScaleType=None
    plotRMSEs(metricsFilesInfo.primaryApproachName, avgPosDeviations, kAveragePositionDeviationsErrorType,
              ylims=posDev_y_lims, legend_loc=posDevLegendLoc, savepath=None, height_ratios=dev_height_ratios, yscaleType=devScaleType, legend_ncol=avgDevLegendNcol)

    # Comparisions
    # medianDevYLims = None
    # med_dev_height_ratios=None
    # medianDevLegendLoc="upper left"

    # Ablations
    medianDevYLims = None
    med_dev_height_ratios=None
    medianDevLegendLoc="upper right"

    plotRMSEs(metricsFilesInfo.primaryApproachName, medianPosDeviations, kMedianPositionDeviationsErrorType,
              ylims=medianDevYLims, legend_loc=medianDevLegendLoc, savepath=None, height_ratios=med_dev_height_ratios)

    # comparison
    # iou_y_lims = []
    # avgIouLegendLoc = "upper center"
    # avgIouLegendNCol = 2


    # ablations
    iou_y_lims=[(0, 0.15)]
    avgIouLegendLoc="upper right"
    avgIouLegendNCol = 2

    plotRMSEs(metricsFilesInfo.primaryApproachName, avgIous, kAverageIousErrorType, ylims=iou_y_lims,
              legend_loc=avgIouLegendLoc, savepath=None,  legend_ncol=avgIouLegendNCol)

    # Comparison
    # missedGtsYLims = None
    # missedGtsLegendLoc="upper center"
    # missedGtsLegendNcol =2

    # Ablation
    missedGtsYLims = [(0, 70)]
    missedGtsLegendLoc="upper center"
    missedGtsLegendNcol =2


    plotRMSEs(metricsFilesInfo.primaryApproachName, missedGtObjs, kMissedGtsErrorType, ylims=missedGtsYLims,
              legend_loc=missedGtsLegendLoc, savepath=None,  legend_ncol=missedGtsLegendNcol)

    # Comparison
    # objsPerGTYLims = None
    # objsPerGTLegendLoc="upper center"
    # objsPerGTLegendNcol =2

    # Ablation
    objsPerGTYLims = None
    objsPerGTLegendLoc="lower center"
    objsPerGTLegendNcol =2


    plotRMSEs(metricsFilesInfo.primaryApproachName, objectsPerGtObj, kObjRatioErrorType, ylims=objsPerGTYLims,
              legend_loc=objsPerGTLegendLoc, savepath=None,  legend_ncol=objsPerGTLegendNcol)

    plt.show()


def parseArgs():
    parser = argparse.ArgumentParser(description='Plot object metrics results.')
    parser.add_argument('--approaches_and_metrics_file_name', required=True, default="")
    parser.add_argument('--error_types_and_savepaths_file_name', required=False, default="")

    args = parser.parse_args()
    return args


if __name__ == "__main__":
    cmdLineArgs = parseArgs()
    approaches_and_metrics_file_name = cmdLineArgs.approaches_and_metrics_file_name
    error_types_and_savepaths_file_name = cmdLineArgs.error_types_and_savepaths_file_name
    runPlotter(approaches_and_metrics_file_name, error_types_and_savepaths_file_name)
