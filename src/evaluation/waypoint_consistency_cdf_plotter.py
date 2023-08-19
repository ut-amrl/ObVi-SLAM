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
    plotCDF(primaryApproachName, translationConsistency,
            "CDF of Position Deviation from Waypoint Estimate Centroid", "Meters from Respective Centroid", 1,
            savepath=savepath)


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
    averageTranslAtes = {}
    averageRotAtes = {}
    translAtesByTrajectory = {}
    rotAtesByTrajectory = {}

    for approachName, metricsFile in metricsFilesInfo.approachNameAndMetricsFileInfo.items():
        print("Reading results for " + approachName)
        approachMetrics = readMetricsFile(metricsFile)
        translationDeviations = approachMetrics.sequence_metrics.all_translation_deviations
        rotationDeviations = approachMetrics.sequence_metrics.all_rotation_deviations

        translationConsistency[approachName] = translationDeviations
        orientationConsistency[approachName] = rotationDeviations
        averageTranslAtes[approachName] = approachMetrics.sequence_metrics.ate_results.rmse_transl_err
        averageRotAtes[approachName] = np.degrees(approachMetrics.sequence_metrics.ate_results.rmse_rot_err)

        translAtePerTraj = []
        rotAtePerTraj = []
        for indiv_traj_metric_set in approachMetrics.indiv_trajectory_metrics:
            translAtePerTraj.append(indiv_traj_metric_set.ate_results.rmse_transl_err)
            rotAtePerTraj.append(np.degrees(indiv_traj_metric_set.ate_results.rmse_rot_err))
        translAtesByTrajectory[approachName] = translAtePerTraj
        rotAtesByTrajectory[approachName] = rotAtePerTraj

    errorTypesAndSavepaths = readErrTypesAndSavepathsFile(error_types_and_savepaths_file_name)

    plotTranslationConsistency(metricsFilesInfo.primaryApproachName, translationConsistency,
                               errorTypesAndSavepaths[kCDFTranslErrorType])
    plotOrientationConsistency(metricsFilesInfo.primaryApproachName, orientationConsistency,
                               errorTypesAndSavepaths[kCDFOrientErrorType])

    transl_y_lims = []
    # transl_y_lims=[(0, 6.0), (19.5, 22)]
    plotRMSEs(metricsFilesInfo.primaryApproachName, translAtesByTrajectory, kATETranslErrorType, ylims=transl_y_lims,
              legend_loc="upper left", savepath=None)
    # orient_y_lims=[(0, 10)]
    # orient_y_lims=[(0, 30), (64, 71)]
    orient_y_lims = []
    plotRMSEs(metricsFilesInfo.primaryApproachName, rotAtesByTrajectory, kATEOrientErrorType, ylims=orient_y_lims,
              legend_loc="upper left", savepath=None)

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
