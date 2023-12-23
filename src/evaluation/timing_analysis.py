import argparse

import os
import subprocess
import signal
import argparse
import time
from cmd_line_arg_utils import *
from file_structure_utils import *
from trajectory_interpolation import *
import csv
from cmd_line_arg_utils import *
from file_structure_utils import *
from trajectory_sequence import *

import matplotlib.pyplot as plt
import json
import numpy as np
import os.path


class VisualizationTimerConfig:

    def __init__(self, sequenceFilesDirectory, resultsRootDirectory, configFileBaseName, sequenceFileBaseName):
        self.sequenceFilesDirectory = sequenceFilesDirectory
        self.resultsRootDirectory = resultsRootDirectory
        self.configFileBaseName = configFileBaseName
        self.sequenceFileBaseName = sequenceFileBaseName


def getNameAvgTimeAndInvocationsFromFileLine(line_from_file):
    words = line_from_file.split()
    preceding_timer_name_word = "for"

    timerName = 0
    timerAvgTime = 0
    timerInvocations = 0

    words_after_for = 0
    found_for = False
    for indiv_word in words:
        if (found_for):
            words_after_for += 1
            if (words_after_for == 1):
                timerName = indiv_word
            elif (words_after_for == 7):
                timerAvgTime = float(indiv_word)
            elif (words_after_for == 11):
                timerInvocations = int(indiv_word)
        else:
            if indiv_word == preceding_timer_name_word:
                found_for = True

    return (timerName, timerAvgTime, timerInvocations)


def getTimerDataFromLogFile(log_file_name):
    query_string = "Run-time stats for"
    log_file_obj = open(log_file_name, 'r')

    timerInfo = []

    for file_line in log_file_obj.readlines():
        if (query_string in file_line):
            timerInfo.append(getNameAvgTimeAndInvocationsFromFileLine(file_line))

    return timerInfo


def getTotalTimeForTimerData(timerData):
    totalTimeData = {}
    for timerEntry in timerData:
        totalTime = timerEntry[1] * timerEntry[2]
        totalTimeData[timerEntry[0]] = totalTime
    return totalTimeData


def getTotalTimeForBag(bagId):
    bagTimesInMs = {
        "_2023_05_11_18_35_54": 249820,
        "_2023_05_12_13_15_27": 235590,
        "_2023_06_30_11_27_42": 323560,
        "_2023_06_22_22_12_29": 206420,
        "_2023_05_13_21_51_39": 211270,
        "_2023_06_23_10_22_43": 300150,
        "_2023_06_28_11_02_23": 214010,
        "_2023_06_21_10_32_23": 247510,
        "_2023_05_17_12_13_10": 258590,
        "_2023_05_13_19_03_07": 274400,
        "_2023_05_12_10_50_32": 266070,
        "_2023_06_26_11_08_53": 247540,
        "_2023_05_16_15_02_33": 225500,
        "_2023_06_22_21_51_46": 192140,
        "_2023_06_27_21_52_02": 242220,
        "_2023_06_27_21_36_30": 307310
    }

    return bagTimesInMs[bagId]


def getNumberOfFramesAndCumulativeTimeMap(resultsDir, sequenceName, configName, bagName):
    logName = FileStructureUtils.ensureDirectoryEndsWithSlash(
        resultsDir) + sequenceName + "/" + configName + "/" + bagName + "/logs/offline_object_visual_slam_main.INFO"

    if (not os.path.exists(logName)):
        print("Log file " + logName + " does not exist")
        exit(1)

    rawTimerData = getTimerDataFromLogFile(logName)
    numberOfFrames = 0
    for timerEntry in rawTimerData:
        if (timerEntry[0] == "visual_frontend_top_level"):
            numberOfFrames = timerEntry[2]
            break

    cumulativeTimerData = getTotalTimeForTimerData(rawTimerData)

    return cumulativeTimerData, numberOfFrames


class TimerNames:
    # ONLINE STUFF

    kVisualFrontendTopLevel = "visual_frontend_top_level"
    kBbFrontEndAddBbObs = "bb_front_end_add_bb_obs"

    # outlier_identification
    kPhaseOneLbaBuildOpt = "phase_one_lba_build_opt"
    kPhaseOneLbaSolveOpt = "phase_one_lba_solve_opt"
    kLbaPoseGraphCopy = "lba_pose_graph_copy"
    kPostOptResidualComputeOnline = "post_opt_residual_compute_online"
    kTwoPhaseOptOutlierIdentificationOnline = "two_phase_opt_outlier_identification_online"

    # Local optimization (lba_phase_two)
    kPhaseTwoLbaBuildOpt = "phase_two_lba_build_opt"
    kPhaseTwoLbaSolveOpt = "phase_two_lba_solve_opt"

    # pre_pgo_local_track
    kObjOnlyPgoLocalTrackBuild = "obj_only_pgo_local_track_build"
    kObjOnlyPgoLocalTrackSolve = "obj_only_pgo_local_track_solve"

    # obj_only_pgo"
    kObjOnlyPgoBuildPgo = "obj_only_pgo_build_pgo"
    kObjOnlyPgoSolvePgo = "obj_only_pgo_solve_pgo"

    # post_pgo_vf_adjustment
    kObjOnlyPgoManualFeatAdjust = "obj_only_pgo_manual_feat_adjust"
    kObjOnlyPgoOptFeatAdjustBuild = "obj_only_pgo_opt_feat_adjust_build"
    kObjOnlyPgoOptFeatAdjustSolve = "obj_only_pgo_opt_feat_adjust_solve"

    # All other
    kOnlinePlusVis = "online_plus_vis"
    # Minus
    kVisualizationTopLevelOnline = "visualization_top_level_online"
    kIterationLoggerWrite = "iteration_logger_write"
    kLbaOptLoggerWrite = "lba_opt_logger_write"
    # Plus all above

    # OFFLINE STUFF
    kPostSessionMapMerge = "post_session_map_merge"
    kLongTermMapExtraction = "long_term_map_extraction"

    # All other
    kOfflinePlusVis = "offline_plus_vis"
    # Minus
    kVisualizationTopLevelOffline = "visualization_top_level_offline"
    # Plus all above


class TimerPlotName:
    kFrontEndVF = "Front-End, Visual Features"
    kFrontEndObjects = "Front-End, Objects"
    kOutlierId = "Outlier Rejection"
    kLocalOptimization = "Local Optimization"
    kGlobalAdjustmentTracking = "Pre-Global Tracking"
    kGlobalOptimization = "Global Optimization"
    kPostGlobalAdjustment = "Post-Global Adjustment"
    kOnlineOther = "Other"

    kMapMergingAndRefinement = "Map Merge/Refinement"
    kLongTermMapExtraction = "Long-Term Map Extraction"
    kOfflineOther = "Other"


def getCumulativeTimeForEntriesOfInterest(allTimeDataMap):
    onlineTimingMap = {}

    onlineTimingMap[TimerPlotName.kFrontEndVF] = allTimeDataMap.get(TimerNames.kVisualFrontendTopLevel, 0)
    onlineTimingMap[TimerPlotName.kFrontEndObjects] = allTimeDataMap.get(TimerNames.kBbFrontEndAddBbObs, 0)

    onlineTimingMap[TimerPlotName.kOutlierId] = allTimeDataMap.get(TimerNames.kPhaseOneLbaBuildOpt, 0) + \
                                                allTimeDataMap.get(TimerNames.kPhaseOneLbaSolveOpt, 0) + \
                                                allTimeDataMap.get(TimerNames.kLbaPoseGraphCopy, 0) + \
                                                allTimeDataMap.get(TimerNames.kPostOptResidualComputeOnline) + \
                                                allTimeDataMap.get(TimerNames.kTwoPhaseOptOutlierIdentificationOnline)

    onlineTimingMap[TimerPlotName.kLocalOptimization] = allTimeDataMap.get(TimerNames.kPhaseTwoLbaBuildOpt, 0) + \
                                                        allTimeDataMap.get(TimerNames.kPhaseTwoLbaSolveOpt, 0)

    onlineTimingMap[TimerPlotName.kGlobalAdjustmentTracking] = \
        allTimeDataMap.get(TimerNames.kObjOnlyPgoLocalTrackBuild, 0) + \
        allTimeDataMap.get(TimerNames.kObjOnlyPgoLocalTrackSolve, 0)

    onlineTimingMap[TimerPlotName.kGlobalOptimization] = allTimeDataMap.get(TimerNames.kObjOnlyPgoBuildPgo, 0) + \
                                                         allTimeDataMap.get(TimerNames.kObjOnlyPgoSolvePgo, 0)

    onlineTimingMap[TimerPlotName.kPostGlobalAdjustment] = \
        allTimeDataMap.get(TimerNames.kObjOnlyPgoManualFeatAdjust, 0) + \
        allTimeDataMap.get(TimerNames.kObjOnlyPgoOptFeatAdjustBuild, 0) + \
        allTimeDataMap.get(TimerNames.kObjOnlyPgoOptFeatAdjustSolve, 0)

    totalAccountedForOnlineTime = 0
    for timeEntry in onlineTimingMap.values():
        totalAccountedForOnlineTime += timeEntry

    onlineTimingMap[TimerPlotName.kOnlineOther] = allTimeDataMap.get(TimerNames.kOnlinePlusVis, 0) - (
            totalAccountedForOnlineTime + allTimeDataMap.get(TimerNames.kVisualizationTopLevelOnline, 0) +
            allTimeDataMap.get(TimerNames.kIterationLoggerWrite, 0) + allTimeDataMap.get(TimerNames.kLbaOptLoggerWrite,
                                                                                         0)
    )

    offlineTimingMap = {}

    offlineTimingMap[TimerPlotName.kMapMergingAndRefinement] = allTimeDataMap.get(TimerNames.kPostSessionMapMerge, 0)
    offlineTimingMap[TimerPlotName.kLongTermMapExtraction] = allTimeDataMap.get(TimerNames.kLongTermMapExtraction, 0)

    totalAccountedForOfflineTime = 0
    for timeEntry in offlineTimingMap.values():
        totalAccountedForOfflineTime += timeEntry

    offlineTimingMap[TimerPlotName.kOfflineOther] = allTimeDataMap.get(TimerNames.kOfflinePlusVis, 0) - (
            totalAccountedForOnlineTime + allTimeDataMap.get(TimerNames.kVisualizationTopLevelOffline, 0)
    )

    return onlineTimingMap, offlineTimingMap


def runTimerVisualization(vizConfig):
    # Read sequence file

    rosbagsSequence = readTrajectorySequence(vizConfig.sequenceFilesDirectory, vizConfig.sequenceFileBaseName)

    # Aggregate online data
    aggregateOnlineData = {}
    aggregateOfflineData = {}
    aggregateFrameCount = 0
    aggregateBagTimeSec = 0

    onlineTimingMapByTraj = []
    offlineTimingMapByTraj = []
    targetRTs = []

    for idx, bagName in enumerate(rosbagsSequence):
        # Get bag time
        bagTime = getTotalTimeForBag(bagName)

        bagIdentifier = str(idx) + "_" + bagName

        allTimeDataMap, frameCount = getNumberOfFramesAndCumulativeTimeMap(vizConfig.resultsRootDirectory,
                                                                           vizConfig.sequenceFileBaseName,
                                                                           vizConfig.configFileBaseName, bagIdentifier)
        onlineTimingMap, offlineTimingMap = getCumulativeTimeForEntriesOfInterest(allTimeDataMap)

        aggregateFrameCount += frameCount
        aggregateBagTimeSec += bagTime

        targetRT = bagTime / frameCount
        targetRTs.append(targetRT)

        onlineTimingMapPerFrame = {timerName: (timerVal / frameCount) for timerName, timerVal in
                                   onlineTimingMap.items()}

        onlineTimingMapByTraj.append(onlineTimingMapPerFrame)
        offlineTimingMapByTraj.append(offlineTimingMap)

        # TODO visualize

        for timerName, timerVal in onlineTimingMap.items():
            if timerName not in aggregateOnlineData:
                aggregateOnlineData[timerName] = 0
            aggregateOnlineData[timerName] += timerVal

        for timerName, timerVal in offlineTimingMap.items():
            if timerName not in aggregateOnlineData:
                aggregateOfflineData[timerName] = 0
            aggregateOfflineData[timerName] += timerVal

    # TODO plot online
    # 16 stacked bars with 17th Avg
    aggregateOnlineByFrame = {timerName: (timerVal / aggregateFrameCount) for timerName, timerVal in
                              aggregateOnlineData.items()}
    # line before 17
    averageTargetRT = aggregateBagTimeSec / aggregateFrameCount
    # piece wise function for target RTS

    # TODO plot offline
    # 16 stacked bars with 17th Avg
    # line before 17


def argParse():
    parser = argparse.ArgumentParser(description="Run timing sequence analysis")
    parser.add_argument(
        CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.trajectorySequenceFileDirectoryBaseArgName),
        required=True,
        help=CmdLineArgConstants.trajectorySequenceFileDirectoryHelp)
    parser.add_argument(CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.resultsRootDirectoryBaseArgName),
                        required=True,
                        help=CmdLineArgConstants.resultsRootDirectoryHelp)

    parser.add_argument(CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.configFileBaseNameBaseArgName),
                        required=True,
                        help=CmdLineArgConstants.configFileBaseNameHelp)
    parser.add_argument(CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.sequenceFileBaseNameBaseArgName),
                        required=True,
                        help=CmdLineArgConstants.sequenceFileBaseNameHelp)
    args_dict = vars(parser.parse_args())

    return VisualizationTimerConfig(
        sequenceFilesDirectory=args_dict[CmdLineArgConstants.trajectorySequenceFileDirectoryBaseArgName],
        resultsRootDirectory=args_dict[CmdLineArgConstants.resultsRootDirectoryBaseArgName],
        configFileBaseName=args_dict[CmdLineArgConstants.configFileBaseNameBaseArgName],
        sequenceFileBaseName=args_dict[CmdLineArgConstants.sequenceFileBaseNameBaseArgName])


if __name__ == "__main__":
    executionConfig = argParse()
    runTimerVisualization(executionConfig)
