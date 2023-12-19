import argparse
import rospy
import math
import csv
import sys
import warnings
import os

import matplotlib.pyplot as plt
from brokenaxes import brokenaxes

import numpy as np
import pandas as pd

from approach_metrics import *


def plotTranslationConsistency(primaryApproachName, translationConsistency, savepath=None):
    # # For orb, oa, droid and ours
    # xlims = [(0, 5.2)]
    # width_ratios = [1, 1]
    #
    # # ablations
    # xlims = [(0, 6.5)]
    # width_ratios = [1, 1]

    xlims=None
    width_ratios=None

    #update_rev_lower_conv_thresholds_manual_feat_true_single_phase
    xlims = [(0, 8)]
    width_ratios=[1, 1]

    plotCDF(primaryApproachName, translationConsistency,
            "CDF of Position Deviation from Waypoint Estimate Centroid", "Meters from Centroid", 1,
            savepath=savepath, xlims=xlims, width_ratios=width_ratios)


def plotOrientationConsistency(primaryApproachName, orientationConsistency, savepath=None):
    orientationConsistencyDeg = {approachName: np.degrees(approachResults) for approachName, approachResults in
                                 orientationConsistency.items()}

    # # For orb, prev oa and ours
    # xlims = [(0, 10)]
    # width_ratios = [1, 1]
    #
    # # ablations
    # xlims = [(0, 13)]
    # width_ratios = [1, 1]
    #
    # xlims = None
    # width_ratios = None

    #update_rev_lower_conv_thresholds_manual_feat_true_single_phase
    xlims = [(0, 7)]
    width_ratios=[1, 1]

    plotCDF(primaryApproachName, orientationConsistencyDeg,
            "CDF of Orientation Estimate Deviation from Mean Waypoint Orientation",
            "Degrees from Mean Orientation", 2, savepath=savepath, xlims=xlims, width_ratios=width_ratios)


def generateSingleTable(primaryApproachName, perTrajAvgResults, perTrajStdDevs, overallAvgs, overallStdDevs, decimalPlaces):
    numTrajs = len(perTrajAvgResults[primaryApproachName])

    colHeaders = [str(i) for i in range(1, numTrajs + 1)]
    colHeaders.append("Overall")

    rowHeaders = list(perTrajAvgResults.keys())

    boldMask = [[False for _ in range(numTrajs + 1)] for row in rowHeaders]
    italicMask = [[False for _ in range(numTrajs + 1)] for row in rowHeaders]
    tableValues = [["-" for _ in range(numTrajs + 1)] for row in rowHeaders]

    for trajIdx in range(numTrajs):
        minForTraj = float('inf')
        secondMinForTraj = float('inf')
        for approachIdx in range(len(rowHeaders)):
            approachName = rowHeaders[approachIdx]
            if (perTrajAvgResults[approachName][trajIdx] < minForTraj):
                secondMinForTraj = minForTraj
                minForTraj = perTrajAvgResults[approachName][trajIdx]
            elif (perTrajAvgResults[approachName][trajIdx] < secondMinForTraj):
                secondMinForTraj = perTrajAvgResults[approachName][trajIdx]
            # minForTraj = min(minForTraj, perTrajAvgResults[approachName][trajIdx])
            # tableValues[approachIdx][trajIdx] = str(round(perTrajAvgResults[approachName][trajIdx], decimalPlaces)) + " $\\pm$ " + str(round(
            #     perTrajStdDevs[approachName][trajIdx], decimalPlaces))
            tableValues[approachIdx][trajIdx] = str(round(perTrajAvgResults[approachName][trajIdx], decimalPlaces))
        for approachIdx in range(len(rowHeaders)):
            approachName = rowHeaders[approachIdx]
            if (perTrajAvgResults[approachName][trajIdx] == minForTraj):
                boldMask[approachIdx][trajIdx] = True
        boldCountForTraj = 0
        for approachIdx in range(len(rowHeaders)):
            if (boldMask[approachIdx][trajIdx]):
                boldCountForTraj += 1
        if (boldCountForTraj <= 1):
            for approachIdx in range(len(rowHeaders)):
                approachName = rowHeaders[approachIdx]
                if (perTrajAvgResults[approachName][trajIdx] == secondMinForTraj):
                    italicMask[approachIdx][trajIdx] = True

    minForOverall = float('inf')
    secondMinForOverall = float('inf')
    for approachIdx in range(len(rowHeaders)):
        approachName = rowHeaders[approachIdx]
        if (overallAvgs[approachName] < minForOverall):
            secondMinForOverall = minForOverall
            minForOverall = overallAvgs[approachName]
        elif (overallAvgs[approachName] < secondMinForOverall):
            secondMinForOverall = overallAvgs[approachName]
        # tableValues[approachIdx][numTrajs] = str(round(overallAvgs[approachName], decimalPlaces)) + " $\\pm$ " + str(round(
        #     overallStdDevs[approachName], decimalPlaces))
        tableValues[approachIdx][numTrajs] = str(round(overallAvgs[approachName], decimalPlaces))
    for approachIdx in range(len(rowHeaders)):
        approachName = rowHeaders[approachIdx]
        if (overallAvgs[approachName] == minForOverall):
            boldMask[approachIdx][numTrajs] = True
    boldCountForAvg = 0
    for approachIdx in range(len(rowHeaders)):
        if (boldMask[approachIdx][numTrajs]):
            boldCountForAvg += 1
    if (boldCountForAvg <= 1):
        for approachIdx in range(len(rowHeaders)):
            approachName = rowHeaders[approachIdx]
            if (overallAvgs[approachName] == secondMinForOverall):
                italicMask[approachIdx][numTrajs] = True


    # return generateLatexTable(colHeaders, rowHeaders, tableValues, boldMask)

    flippedTableValues = [["" for _ in rowHeaders] for _ in range(numTrajs + 1)]
    flippedBoldMask = [[False for _ in rowHeaders] for _ in range(numTrajs + 1)]
    flippedItalicMask = [[False for _ in rowHeaders] for _ in range(numTrajs + 1)]

    for colIdx in range(numTrajs + 1):
        for rowIdx in range(len(rowHeaders)):
            flippedTableValues[colIdx][rowIdx] = tableValues[rowIdx][colIdx]
            flippedBoldMask[colIdx][rowIdx] = boldMask[rowIdx][colIdx]
            flippedItalicMask[colIdx][rowIdx] = italicMask[rowIdx][colIdx]

    return generateLatexTable(rowHeaders, colHeaders, flippedTableValues, flippedBoldMask, flippedItalicMask)


def generateATETables(primaryApproachName,
                      averageTranslAtes, averageRotAtes, translAtesByTrajectory, rotAtesByTrajectory,
                      translStdDevsByTrajectory, rotStdDevsByTrajectory, overallTranslStdDevs, overallRotStdDevs):
    print("Translation ATE table")
    print(generateSingleTable(primaryApproachName, translAtesByTrajectory, translStdDevsByTrajectory, averageTranslAtes,
                              overallTranslStdDevs, 2))
    print("Rotation ATE table")
    print(generateSingleTable(primaryApproachName, rotAtesByTrajectory, rotStdDevsByTrajectory, averageRotAtes,
                              overallRotStdDevs, 1))


def runPlotter(approaches_and_metrics_file_name, error_types_and_savepaths_file_name=None):
    plt.rcParams.update({
        "font.family": "serif",  # use serif/main font for text elements
        # "text.usetex": True,     # use inline math for ticks
        "pgf.rcfonts": False  # don't setup fonts from rc parameters
    })

    metricsFilesInfo = readApproachesAndMetricsFile(approaches_and_metrics_file_name)
    translationConsistency = {}
    orientationConsistency = {}
    averageTranslAtes = {}
    averageRotAtes = {}
    translAtesByTrajectory = {}
    rotAtesByTrajectory = {}

    translStdDevsByTrajectory = {}
    rotStdDevsByTrajectory = {}

    overallTranslStdDevs = {}
    overallRotStdDevs = {}

    for approachName, metricsFile in metricsFilesInfo.approachNameAndMetricsFileInfo.items():
        print("Reading results for " + approachName)
        approachMetrics = readMetricsFile(metricsFile)
        translationDeviations = approachMetrics.sequence_metrics.all_translation_deviations
        rotationDeviations = approachMetrics.sequence_metrics.all_rotation_deviations

        translationConsistency[approachName] = translationDeviations
        orientationConsistency[approachName] = rotationDeviations
        averageTranslAtes[approachName] = approachMetrics.sequence_metrics.ate_results.rmse_transl_err
        averageRotAtes[approachName] = np.degrees(approachMetrics.sequence_metrics.ate_results.rmse_rot_err)
        overallTranslStdDevs[approachName] = approachMetrics.sequence_metrics.ate_results.transl_stats.stdDev
        overallRotStdDevs[approachName] = np.degrees(approachMetrics.sequence_metrics.ate_results.rot_stats.stdDev)

        translAtePerTraj = []
        rotAtePerTraj = []
        translStdDevPerTraj = []
        rotStdDevPerTraj = []
        for indiv_traj_metric_set in approachMetrics.indiv_trajectory_metrics:
            translAtePerTraj.append(indiv_traj_metric_set.ate_results.rmse_transl_err)
            rotAtePerTraj.append(np.degrees(indiv_traj_metric_set.ate_results.rmse_rot_err))
            translStdDevPerTraj.append(indiv_traj_metric_set.ate_results.transl_stats.stdDev)
            rotStdDevPerTraj.append(np.degrees(indiv_traj_metric_set.ate_results.rot_stats.stdDev))
        translAtesByTrajectory[approachName] = translAtePerTraj
        rotAtesByTrajectory[approachName] = rotAtePerTraj
        translStdDevsByTrajectory[approachName] = translStdDevPerTraj
        rotStdDevsByTrajectory[approachName] = rotStdDevPerTraj

    errorTypesAndSavepaths = readErrTypesAndSavepathsFile(error_types_and_savepaths_file_name)

    generateATETables(metricsFilesInfo.primaryApproachName,
                      averageTranslAtes, averageRotAtes, translAtesByTrajectory, rotAtesByTrajectory,
                      translStdDevsByTrajectory, rotStdDevsByTrajectory, overallTranslStdDevs, overallRotStdDevs)

    plotTranslationConsistency(metricsFilesInfo.primaryApproachName, translationConsistency,
                               errorTypesAndSavepaths.get(kCDFTranslErrorType))
    plotOrientationConsistency(metricsFilesInfo.primaryApproachName, orientationConsistency,
                               errorTypesAndSavepaths.get(kCDFOrientErrorType))

    # For orb, prev oa and ours
    # transl_y_lims = [(0, 3), (3.5, 8), (20, 22.5)]
    # transl_height_ratios = [1, 1, 2]

    # ablations
    # transl_y_lims = [(0, 4.5), (8, 45), (22500, 24900)]
    # transl_height_ratios = [1, 2, 2.5]
    # transl_legend_ncol = 2
    # transl_legend_loc = "upper right"

    # New comparison results
    # transl_legend_ncol=1
    # transl_y_lims=None
    # transl_height_ratios = None
    # transl_legend_loc="upper left"

    # New comparison results v2
    # transl_legend_ncol = 1
    # transl_y_lims = [(0, 5.8), (6, 12)]
    # transl_height_ratios = [1, 2]
    transl_legend_loc = "upper left"
    transl_height_ratios=None
    transl_legend_ncol=2
    transl_y_lims=None

    plotRMSEs(metricsFilesInfo.primaryApproachName, translAtesByTrajectory, kATETranslErrorType, ylims=transl_y_lims,
              legend_loc=transl_legend_loc, savepath=errorTypesAndSavepaths.get(kATETranslErrorType),
              height_ratios=transl_height_ratios, legend_ncol=transl_legend_ncol, scatter=False)
    # orient_y_lims=[(0, 10)]
    # orient_y_lims=[(0, 30), (64, 71)]

    # For orb, prev oa and ours
    # orient_y_lims = [(0, 10), (10, 30), (43, 72.5)]
    # orient_height_ratios = [1, 1, 2]

    # ablations

    # orient_y_lims=[(0, 10), (10, 24), (41.5, 250)]
    # orient_height_ratios=[1.5, 1, 3]
    # orient_y_lims = [(0, 13), (18, 25), (40, 130), (150, 180)]
    # orient_height_ratios = [1, 1, 1, 4]
    # orient_legend_ncol = 2
    # orient_legend_loc = "center left"

    # orient_y_lims = [(0, 15), (40, 73), (83, 185)]
    # orient_height_ratios = [1, 1, 4]
    # orient_legend_ncol=2

    # New comparison results

    # orient_y_lims = [(0, 9), (10, 24), (40, 60)]
    # orient_height_ratios = [1, 1, 2]
    # orient_legend_loc = "upper left"
    # orient_legend_ncol = 1

    # orient_y_lims = [(0, 9), (10, 24), (40, 60)]
    # orient_height_ratios = [1, 1, 2]
    orient_y_lims=None
    orient_height_ratios=None
    orient_legend_loc = "upper left"
    orient_legend_ncol = 1

    # # New comparison results v2 (slides version)
    # orient_y_lims=[(0, 3.7), (4.1, 8),  (9.5, 24), (40, 60)]
    # orient_height_ratios=[1, 1, 1, 1.5]
    # orient_legend_loc="upper left"
    # orient_legend_ncol=1

    print(rotAtesByTrajectory)
    plotRMSEs(metricsFilesInfo.primaryApproachName, rotAtesByTrajectory, kATEOrientErrorType, ylims=orient_y_lims,
              legend_loc=orient_legend_loc, savepath=errorTypesAndSavepaths.get(kATEOrientErrorType),
              height_ratios=orient_height_ratios, legend_ncol=orient_legend_ncol, scatter=False)

    plt.show()


def parseArgs():
    parser = argparse.ArgumentParser(description='Plot consistency results.')
    parser.add_argument('--approaches_and_metrics_file_name', required=True, default="")
    parser.add_argument('--error_types_and_savepaths_file_name', required=False, default="")

    args = parser.parse_args()
    return args


if __name__ == "__main__":
    cmdLineArgs = parseArgs()
    approaches_and_metrics_file_name = cmdLineArgs.approaches_and_metrics_file_name
    error_types_and_savepaths_file_name = cmdLineArgs.error_types_and_savepaths_file_name
    runPlotter(approaches_and_metrics_file_name, error_types_and_savepaths_file_name)
