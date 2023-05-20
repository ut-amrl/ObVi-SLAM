import argparse
import rospy
import math
import csv
import sys

import matplotlib.pyplot as plt
import numpy as np

from approach_metrics import *

kMaxXAxisBoundsMultiplier = 1.2


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


def getCDFData(dataset, num_bins):
    # getting data of the histogram
    count, bins_count = np.histogram(dataset, bins=num_bins)

    # finding the PDF of the histogram using count values

    pdf = count / sum(count)

    # using numpy np.cumsum to calculate the CDF
    # We can also find using the PDF values by looping and adding
    cdf = np.cumsum(pdf)
    cdf = np.insert(cdf, 0, 0)

    max_val = np.amax(dataset)

    return (cdf, bins_count, max_val)


def plotCDF(primaryApproachName, approach_results, title, x_label, fig_num, bins=40, savepath=None):
    plt.figure(fig_num)
    comparison_approach_summary_max = 0

    alternate_line_styles = ['dotted', 'dashdot', 'dashed', (0, (3, 1, 1, 1)), (0, (3, 1, 1, 1, 1, 1))]
    alternate_line_style_index = 0
    primary_approach_max = None
    # getting data of the histogram
    for approach_label, comparison_dataset in approach_results.items():
        approach_cdf, bins_count, comparison_approach_max = getCDFData(comparison_dataset, bins)
        if (approach_label is not primaryApproachName):
            comparison_approach_summary_max = max(comparison_approach_max, comparison_approach_summary_max)
            line_style = alternate_line_styles[alternate_line_style_index]
            alternate_line_style_index += 1
        else:
            primary_approach_max = comparison_approach_max
            line_style = 'solid'
        plt.plot(bins_count, approach_cdf, linestyle=line_style,
                 label=approach_label)

    if (len(approach_results) != 1):
        # if (primary_approach_max > comparison_approach_summary_max):
        x_lim = primary_approach_max
        # else:
        #     x_lim = min(primary_approach_max * kMaxXAxisBoundsMultiplier, comparison_approach_summary_max)
        plt.xlim(0, x_lim)
        plt.legend(prop={'size': 'small'})
    plt.ylim(0, 1)
    plt.title(title)
    plt.xlabel(x_label)
    plt.ylabel("Proportion of data")
    plt.grid(alpha=0.4)
    if savepath:
        plt.savefig(savepath)

def plotTranslationConsistency(primaryApproachName, translationConsistency):
    plotCDF(primaryApproachName, translationConsistency,
            "CDF of Position Deviation from Waypoint Estimate Centroid", "Meters from Respective Centroid", 1, savepath="transl_cdf.png")


def plotOrientationConsistency(primaryApproachName, orientationConsistency):
    orientationConsistencyDeg = {approachName: np.degrees(approachResults) for approachName, approachResults in
                                 orientationConsistency.items()}
    plotCDF(primaryApproachName, orientationConsistencyDeg,
            "CDF of Orientation Estimate Deviation from Mean Waypoint Orientation",
            "Degrees from Mean Waypoint Orientation", 2, savepath="orient_cdf.png")


def runPlotter(approaches_and_metrics_file_name):
    metricsFilesInfo = readApproachesAndMetricsFile(approaches_and_metrics_file_name)
    translationConsistency = {}
    orientationConsistency = {}

    for approachName, metricsFile in metricsFilesInfo.approachNameAndMetricsFileInfo.items():
        translationDeviations, rotationDeviations = readTranslationAndOrientationConsistencyFromFile(metricsFile)
        translationConsistency[approachName] = translationDeviations
        orientationConsistency[approachName] = rotationDeviations

    plotTranslationConsistency(metricsFilesInfo.primaryApproachName, translationConsistency)
    plotOrientationConsistency(metricsFilesInfo.primaryApproachName, orientationConsistency)

    plt.show()


def parseArgs():
    parser = argparse.ArgumentParser(description='Plot consistency results.')
    parser.add_argument('--approaches_and_metrics_file_name', required=True, default="")

    args = parser.parse_args()
    return args


if __name__ == "__main__":
    cmdLineArgs = parseArgs()
    approaches_and_metrics_file_name = cmdLineArgs.approaches_and_metrics_file_name
    runPlotter(approaches_and_metrics_file_name)
