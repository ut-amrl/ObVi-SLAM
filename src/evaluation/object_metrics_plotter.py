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
    posDev_y_lims = None
    dev_height_ratios = None

    # Ablations
    # posDev_y_lims=[(0, 7), (20, 80), (1500, 2700), (2700, 9200), (9200, 2600000)]
    # dev_height_ratios=[1, 1, 1, 1, 3]
    print(avgPosDeviations)
    maxes = {approachLabel:max(dev_list) for approachLabel, dev_list in avgPosDeviations.items()}
    print(maxes)

    plotRMSEs(metricsFilesInfo.primaryApproachName, avgPosDeviations, kAveragePositionDeviationsErrorType,
              ylims=posDev_y_lims, legend_loc="upper left", savepath=None, height_ratios=dev_height_ratios)


    plotRMSEs(metricsFilesInfo.primaryApproachName, medianPosDeviations, kMedianPositionDeviationsErrorType,
              ylims=posDev_y_lims, legend_loc="upper left", savepath=None, height_ratios=dev_height_ratios)

    # comparison
    iou_y_lims = []

    # ablations
    # iou_y_lims=[(0, 0.16)]
    plotRMSEs(metricsFilesInfo.primaryApproachName, avgIous, kAverageIousErrorType, ylims=iou_y_lims,
              legend_loc="upper center", savepath=None,  legend_ncol=2)

    plotRMSEs(metricsFilesInfo.primaryApproachName, missedGtObjs, kMissedGtsErrorType, ylims=iou_y_lims,
              legend_loc="upper center", savepath=None,  legend_ncol=2)

    plotRMSEs(metricsFilesInfo.primaryApproachName, objectsPerGtObj, kObjRatioErrorType, ylims=iou_y_lims,
              legend_loc="upper center", savepath=None,  legend_ncol=2)

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
