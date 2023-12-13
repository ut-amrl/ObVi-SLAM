import os
import argparse
from cmd_line_arg_utils import *
from file_structure_utils import *
from trajectory_sequence import *
import csv
from math import sqrt
import matplotlib.pyplot as plt



class IterationPlotterConstants:
    iterationLogFilePrefix = "ceres_iterations_"

    iterationFileTypes = ["gba_phase_1", "vf_adjust", "gba_phase_2", "lba_phase_1",
                          "lba_phase_2", "pending_obj_est", "pgo", "pre_pgo_track"]

    iterationFileTypesToMerge = {"gba_phase_1": "lba_phase_1", "gba_phase_2": "lba_phase_2"}


class IterationPlotterConfig:
    def __init__(self,
                 sequence_dir,
                 sequence_file_base_name,
                 results_for_approach_root,
                 config_base_name):
        self.sequence_dir = sequence_dir
        self.sequence_file_base_name = sequence_file_base_name
        self.results_for_approach_root = results_for_approach_root
        self.config_base_name = config_base_name


def readOptimizationIterationInfosFromFile(logFile):
    # print("New file " + logFile)
    costInfosInLogFile = []
    extraNormalizedCostInfosInLogFile = []
    paramChangeInLogFile = []
    iterValsInLogFile = []
    with open(logFile) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        lastIdentifier = None

        costDataForCurrIdentifier = []
        averageStepChangeForCurrIdentifier = []
        iterValsForCurrIdentifier = []
        firstRow = True
        for row in csv_reader:
            if (firstRow):
                firstRow = False
                continue
            trimmedRow = [rowEntry.strip() for rowEntry in row]
            # print(trimmedRow)
            iterRoundIdentifier = trimmedRow[0]
            iterationNum = int(trimmedRow[1])
            costTotal = float(trimmedRow[2])
            stepNorm = float(trimmedRow[4])
            stepNormPerParam = float(trimmedRow[5])
            # print(stepNorm)
            # if (stepNormPerParam > 5000):
            #     print("Step norm per param " + str(stepNormPerParam) + " " + str(stepNorm))

            if ((stepNorm != 0) and (stepNormPerParam != 0)):
                numParams = round(stepNorm / stepNormPerParam)
                # print(numParams)
                averageStepChange = sqrt((stepNorm**2) / numParams)
            else:
                averageStepChange = 0
            if ((lastIdentifier is not None) and (iterationNum == 0)):

                maxCost = max(costDataForCurrIdentifier)
                minCost = min(costDataForCurrIdentifier)
                if (maxCost != 0):
                    # print(logFile)
                    # print(iterRoundIdentifier)
                    # print("max cost " + str(maxCost))
                    # print("min cost " + str(minCost))
                    normalizedCost = [costVal / maxCost for costVal in costDataForCurrIdentifier]
                    costInfosInLogFile.append(normalizedCost)
                    if (maxCost == minCost):
                        extraNormalizedCost = [(costVal - minCost)  for costVal in costDataForCurrIdentifier]
                    else:
                        extraNormalizedCost = [((costVal - minCost) / (maxCost - minCost)) for costVal in costDataForCurrIdentifier]
                    extraNormalizedCostInfosInLogFile.append(extraNormalizedCost)
                else:
                    costInfosInLogFile.append(costDataForCurrIdentifier)
                    extraNormalizedCostInfosInLogFile.append(costDataForCurrIdentifier)
                paramChangeInLogFile.append(averageStepChangeForCurrIdentifier)
                iterValsInLogFile.append(iterValsForCurrIdentifier)
                hasStepSizeOne = all(x2 - x1 == 1 for x1, x2 in zip(iterValsForCurrIdentifier[:-1], iterValsForCurrIdentifier[1:]))
                if (not hasStepSizeOne):
                    print("Not incremental")
                    print(iterValsForCurrIdentifier)
                    exit(1)

                # if (iterValsForCurrIdentifier != range(0, len(costDataForCurrIdentifier))):
                #     print("iters not incremental  in log file " + logFile)
                #     print(iterValsForCurrIdentifier)
                # print(iterValsForCurrIdentifier)
                # input("")

                costDataForCurrIdentifier = []
                averageStepChangeForCurrIdentifier = []
                iterValsForCurrIdentifier = []

            lastIdentifier = iterRoundIdentifier
            costDataForCurrIdentifier.append(costTotal)
            averageStepChangeForCurrIdentifier.append(averageStepChange)
            iterValsForCurrIdentifier.append(iterationNum)

        if (len(iterValsForCurrIdentifier) > 0):
            maxCost = max(costDataForCurrIdentifier)
            minCost = min(costDataForCurrIdentifier)
            if (maxCost != 0):
                normalizedCost = [costVal / maxCost for costVal in costDataForCurrIdentifier]
                costInfosInLogFile.append(normalizedCost)
                if (maxCost == minCost):
                    extraNormalizedCost = [(costVal - minCost)  for costVal in costDataForCurrIdentifier]
                else:
                    extraNormalizedCost = [((costVal - minCost) / (maxCost - minCost)) for costVal in costDataForCurrIdentifier]
                extraNormalizedCostInfosInLogFile.append(extraNormalizedCost)
            else:
                costInfosInLogFile.append(costDataForCurrIdentifier)
                extraNormalizedCostInfosInLogFile.append(costDataForCurrIdentifier)
            paramChangeInLogFile.append(averageStepChangeForCurrIdentifier)
            iterValsInLogFile.append(iterValsForCurrIdentifier)
            hasStepSizeOne = all(x2 - x1 == 1 for x1, x2 in zip(iterValsForCurrIdentifier[:-1], iterValsForCurrIdentifier[1:]))
            if (not hasStepSizeOne):
                print("Not incremental in loop")
                print(iterValsForCurrIdentifier)
                exit(1)


    return (costInfosInLogFile, paramChangeInLogFile, iterValsInLogFile, extraNormalizedCostInfosInLogFile)


def plotCostIterationData(fileType, itersInOptimization, costInfos, prefix=""):
    plt.figure()

    for idx in range(len(itersInOptimization)):
        # print(range(0, len(costInfos[idx])))
        hasStepSizeOne = all(x2 - x1 == 1 for x1, x2 in zip(itersInOptimization[idx][:-1], itersInOptimization[idx][1:]))

        if (not hasStepSizeOne):
            print("iters not incremental")
            print(itersInOptimization[idx])
        plt.plot(itersInOptimization[idx], costInfos[idx])

    plt.title(prefix + "Normalized costs for " + fileType)
    plt.show(block=False)


def plotParamChangeIterationData(fileType, itersInOptimization, paramChangeInfos):
    plt.figure()

    for idx in range(len(itersInOptimization)):
        plt.plot(itersInOptimization[idx], paramChangeInfos[idx])

    currYMin, currYMax = plt.ylim()
    newYMax = min(currYMax, 5000)
    plt.ylim(currYMin, newYMax)

    plt.title("Param change for " + fileType)
    plt.show(block=False)


def plotIterationInfoForFileType(fileType, logFilesForType):
    costInfosForOptimizations = []
    extraNormalizedCostInfosForOptimizations = []
    normalizedParamChangeByNumParamsForOptimizations = []
    itersInOptimization = []

    for logFile in logFilesForType:
        costInfosInLogFile, paramChangeInLogFile, iterValsInLogFile, extraNormalizedCostInfosInLogFile = readOptimizationIterationInfosFromFile(logFile)
        costInfosForOptimizations.extend(costInfosInLogFile)
        normalizedParamChangeByNumParamsForOptimizations.extend(paramChangeInLogFile)
        itersInOptimization.extend(iterValsInLogFile)
        extraNormalizedCostInfosForOptimizations.extend(extraNormalizedCostInfosInLogFile)

    plotCostIterationData(fileType, itersInOptimization, costInfosForOptimizations)
    plotCostIterationData(fileType, itersInOptimization, extraNormalizedCostInfosForOptimizations, "Extra ")
    plotParamChangeIterationData(fileType, itersInOptimization, normalizedParamChangeByNumParamsForOptimizations)


def plotIterationInfo(iterationInfoConfig):
    rosbagsSequence = readTrajectorySequence(iterationInfoConfig.sequence_dir,
                                             iterationInfoConfig.sequence_file_base_name)

    dirForResults = FileStructureUtils.ensureDirectoryEndsWithSlash(
        iterationInfoConfig.results_for_approach_root) + \
                    iterationInfoConfig.sequence_file_base_name + "/" + \
                    iterationInfoConfig.config_base_name + "/"

    logFilesForType = {}

    for idx, bagName in enumerate(rosbagsSequence):
        bagInSeqSubdir = str(idx) + "_" + bagName
        dirForLogs = dirForResults + bagInSeqSubdir + "/" + FileStructureConstants.logsRootDirBaseName + "/"

        for fileType in IterationPlotterConstants.iterationFileTypes:
            logFileForType = dirForLogs + IterationPlotterConstants.iterationLogFilePrefix + fileType + FileStructureConstants.csvExtension
            fileTypeToAddTo = fileType

            if (fileType in IterationPlotterConstants.iterationFileTypesToMerge):
                fileTypeToAddTo = IterationPlotterConstants.iterationFileTypesToMerge[fileType]
            if (fileTypeToAddTo not in logFilesForType):
                logFilesForType[fileTypeToAddTo] = []
            logFilesForType[fileTypeToAddTo].append(logFileForType)

    # print("Log files for type")
    # print(logFilesForType)
    for fileType, logFiles in logFilesForType.items():
        # print(fileType)
        plotIterationInfoForFileType(fileType, logFiles)
    plt.show()
    input("Here!")


def iterationPlotterArgParse():
    parser = argparse.ArgumentParser(description="Plot data for each iteration")
    parser.add_argument(CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.resultsRootDirectoryBaseArgName),
                        required=True,
                        help=CmdLineArgConstants.resultsRootDirectoryHelp)
    parser.add_argument(
        CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.trajectorySequenceFileDirectoryBaseArgName),
        required=True,
        help=CmdLineArgConstants.trajectorySequenceFileDirectoryHelp)
    parser.add_argument(CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.sequenceFileBaseNameBaseArgName),
                        required=True,
                        help=CmdLineArgConstants.sequenceFileBaseNameHelp)
    parser.add_argument(CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.configFileBaseNameBaseArgName),
                        required=True,
                        help=CmdLineArgConstants.configFileBaseNameHelp)

    args_dict = vars(parser.parse_args())
    return IterationPlotterConfig(sequence_dir=args_dict[
        CmdLineArgConstants.trajectorySequenceFileDirectoryBaseArgName],
                                  sequence_file_base_name=args_dict[
                                      CmdLineArgConstants.sequenceFileBaseNameBaseArgName],
                                  results_for_approach_root=args_dict[
                                      CmdLineArgConstants.resultsRootDirectoryBaseArgName],
                                  config_base_name=args_dict[CmdLineArgConstants.configFileBaseNameBaseArgName])


if __name__ == "__main__":
    iterationInfoConfig = iterationPlotterArgParse()
    plotIterationInfo(iterationInfoConfig)
