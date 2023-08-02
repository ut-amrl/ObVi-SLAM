import argparse
import rospy
import math
import csv
import sys
import warnings
import os

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from approach_metrics import *

kMaxXAxisBoundsMultiplier = 1.2
kCDFTranslErrorType = "transl_cdf"
kCDFOrientErrorType = "orient_cdf"


class MetricsFilesInfo:
    def __init__(self, approachNameAndMetricsFileInfo, primaryApproachName):
        self.primaryApproachName = primaryApproachName
        self.approachNameAndMetricsFileInfo = approachNameAndMetricsFileInfo


def readTranslationAndOrientationConsistencyFromFile(metrics_file):
    approachMetrics = readMetricsFile(metrics_file)
    return (approachMetrics.sequence_metrics.all_translation_deviations,
            approachMetrics.sequence_metrics.all_rotation_deviations)


def readApproachesAndMetricsFile(approaches_and_metrics_file_name):
    primaryApproachName = None
    approachesAndMetricsFiles = {}
    with open(approaches_and_metrics_file_name) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        line_count = 0
        for row in csv_reader:
            if line_count == 0:
                primaryApproachName = row[0]
            approachesAndMetricsFiles[row[0]] = row[1].strip()
            line_count += 1
    return MetricsFilesInfo(primaryApproachName=primaryApproachName,
                            approachNameAndMetricsFileInfo=approachesAndMetricsFiles)

def readErrTypesAndSavepathsFile(filepath):
    errTypesAndSavepaths = {}
    if not os.path.exists(filepath):
        warnings.warn("Specified filepath " + filepath + " doesn't exist; Not saveing files")
        errTypesAndSavepaths[kCDFTranslErrorType] = None
        errTypesAndSavepaths[kCDFOrientErrorType] = None
        return errTypesAndSavepaths
    df = pd.read_csv(filepath, delimiter=",", header=None)
    errorTypeConstants = set([kCDFTranslErrorType, kCDFOrientErrorType])
    if (df.iloc[0,0] not in errorTypeConstants) or (df.iloc[1,0] not in errorTypeConstants):
        warnings.warn("Found invalide error type. Not saving files for those entries")
        # handle the file if there're unexpected entries
        errTypesAndSavepaths[kCDFTranslErrorType] = None
        errTypesAndSavepaths[kCDFOrientErrorType] = None
        if df.iloc[0,0] in errorTypeConstants:
            errTypesAndSavepaths[df.iloc[0,0]] = df.iloc[0,1]
        if df.iloc[1,0] in errorTypeConstants:
            errTypesAndSavepaths[df.iloc[1,0]] = df.iloc[1,1]
    else:
        # handle the file if there aren't unexpected entries
        errTypesAndSavepaths[df.iloc[0,0]] = df.iloc[0,1]
        errTypesAndSavepaths[df.iloc[1,0]] = df.iloc[1,1]
    return errTypesAndSavepaths

def getCDFData(dataset, num_bins):
    # getting data of the histogram
    # non_inf_dataset = [data_entry if (data_entry != float('inf')) else 1e10 for data_entry in dataset]
    infs_removed_dataset = [data_entry for data_entry in dataset  if (data_entry != float('inf'))]
    count, bins_count = np.histogram(infs_removed_dataset, bins=num_bins)

    # finding the PDF of the histogram using count values

    pdf = count / sum(count)

    # using numpy np.cumsum to calculate the CDF
    # We can also find using the PDF values by looping and adding
    cdf = np.cumsum(pdf)
    cdf = np.insert(cdf, 0, 0)

    max_val = np.amax(infs_removed_dataset)
    print("Max")
    print(max_val)
    print(bins_count)

    return (cdf * (len(infs_removed_dataset) / len(dataset)), bins_count , max_val)


def plotCDF(primaryApproachName, approach_results, title, x_label, fig_num, bins=1000, savepath=None):
    plt.figure(fig_num)
    comparison_approach_summary_max = 0
    comparison_approach_summary_min_max = None

    alternate_line_styles = ['dotted', 'dashdot', 'dashed', (0, (3, 1, 1, 1)), (0, (3, 1, 1, 1, 1, 1))]
    alternate_line_style_index = 0
    primary_approach_max = None
    # getting data of the histogram
    for approach_label, comparison_dataset in approach_results.items():
        print("Working on " + approach_label)
        approach_cdf, bins_count, comparison_approach_max = getCDFData(comparison_dataset, bins)
        if (approach_label is not primaryApproachName):
            if (comparison_approach_summary_min_max is None):
                comparison_approach_summary_min_max = comparison_approach_max
            else:
                comparison_approach_summary_min_max = min(comparison_approach_max, comparison_approach_summary_min_max)
            comparison_approach_summary_max = max(comparison_approach_max, comparison_approach_summary_max)
            line_style = alternate_line_styles[alternate_line_style_index]
            alternate_line_style_index += 1
        else:
            primary_approach_max = comparison_approach_max
            line_style = 'solid'
        plt.plot(bins_count, approach_cdf, linestyle=line_style,
                 label=approach_label)

    if (len(approach_results) >= 2):
        plt.xlim(0, max(primary_approach_max, comparison_approach_summary_min_max))
        # if (primary_approach_max > comparison_approach_summary_max):
        # x_lim = primary_approach_max
        # else:
        #     x_lim = min(primary_approach_max * kMaxXAxisBoundsMultiplier, comparison_approach_summary_max)
        # plt.xlim(0, x_lim)
        plt.legend(prop={'size': 'small'})
    plt.ylim(0, 1)
    plt.title(title)
    plt.xlabel(x_label)
    plt.ylabel("Proportion of data")
    plt.grid(alpha=0.4)

    if savepath:
        print("Saving plot to " + savepath)
        plt.savefig(savepath)
    else:
        plt.show()


def plotTranslationConsistency(primaryApproachName, translationConsistency, savepath=None):
    plotCDF(primaryApproachName, translationConsistency,
            "CDF of Position Deviation from Waypoint Estimate Centroid", "Meters from Respective Centroid", 1, savepath=savepath)


def plotOrientationConsistency(primaryApproachName, orientationConsistency, savepath=None):
    orientationConsistencyDeg = {approachName: np.degrees(approachResults) for approachName, approachResults in
                                 orientationConsistency.items()}
    plotCDF(primaryApproachName, orientationConsistencyDeg,
            "CDF of Orientation Estimate Deviation from Mean Waypoint Orientation",
            "Degrees from Mean Waypoint Orientation", 2, savepath=savepath)


def runPlotter(approaches_and_metrics_file_name, error_types_and_savepaths_file_name=None):
    metricsFilesInfo = readApproachesAndMetricsFile(approaches_and_metrics_file_name)
    translationConsistency = {}
    orientationConsistency = {}
    averageAtes = {}
    atesByTrajectory = {}

    for approachName, metricsFile in metricsFilesInfo.approachNameAndMetricsFileInfo.items():
        print("Reading results for " + approachName)
        translationDeviations, rotationDeviations = readTranslationAndOrientationConsistencyFromFile(metricsFile)
        translationConsistency[approachName] = translationDeviations
        orientationConsistency[approachName] = rotationDeviations

    errorTypesAndSavepaths = readErrTypesAndSavepathsFile(error_types_and_savepaths_file_name)

    plotTranslationConsistency(metricsFilesInfo.primaryApproachName, translationConsistency, errorTypesAndSavepaths[kCDFTranslErrorType])
    plotOrientationConsistency(metricsFilesInfo.primaryApproachName, orientationConsistency, errorTypesAndSavepaths[kCDFOrientErrorType])

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
