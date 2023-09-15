import json
import csv
import rospy
import math
import csv
import sys
import warnings
import os

import matplotlib as mpl
# mpl.use("pgf")

import matplotlib.pyplot as plt

from custom_json_file_parsing import *

from brokenaxes import brokenaxes

import numpy as np
import pandas as pd
from matplotlib.ticker import MultipleLocator


kFigSize=(6.5,4)
kCDFFigSize=(6.5,4)
kFigBottomSpacing=0.15

kMaxXAxisBoundsMultiplier = 1.2
kCDFTranslErrorType = "transl_cdf"
kCDFOrientErrorType = "orient_cdf"
kATETranslErrorType = "transl_ate"
kATEOrientErrorType = "orient_ate"
kAveragePositionDeviationsErrorType = "pos_dev_avg"
kMedianPositionDeviationsErrorType = "pos_dev_med"
kAverageIousErrorType = "avg ious"
kMissedGtsErrorType = "missed_gts"
kObjRatioErrorType = "obj_ratio"

kATETranslErrorYLabel = "RMSE (m)"
kATEOrientErrorYLabel = "RMSE (deg)"

kAveragePositionDeviationsYLabel = "Average Deviation (m)"
kAverageIousYLabel = "Average IoU"
kMedianDevYLabel = "Median Deviation (m)"
kMissedGtsYLabel = "Object Recall"
kObjsPerGtsYLabel = "Estimated Objs Per GT Obj"

# TODO make this part of config
kTotalObjects = 72

kATEErrorYLabelDict = {
    kATETranslErrorType: kATETranslErrorYLabel, \
    kATEOrientErrorType: kATEOrientErrorYLabel, \
    kAveragePositionDeviationsErrorType: kAveragePositionDeviationsYLabel, \
    kAverageIousErrorType: kAverageIousYLabel, \
    kMedianPositionDeviationsErrorType: kMedianDevYLabel, \
    kMissedGtsErrorType: kMissedGtsYLabel, \
    kObjRatioErrorType: kObjsPerGtsYLabel
}

kObViSLAMApproachName = "ObVi-SLAM"
kORBSLAM3ApproachName = "ORB-SLAM3"
kOASLAMApproachName = "OA-SLAM"
# kGTApproachName = "Pseudo-Groundtruth"
kGTApproachName = "ObVi-SLAM_vis_only"
kDROIDApproachName = "DROID-SLAM"
kAblationNoShapePriorName = "ObVi-SLAM-S"
kAblationNoVisFeatName = "ObVi-SLAM-VF"
kAblationNoLtmName = "ObVi-SLAM-LTM"

kApproachNames = set([
    kObViSLAMApproachName, \
    kORBSLAM3ApproachName, \
    kOASLAMApproachName, \
    kGTApproachName, \
    kDROIDApproachName, \
    kAblationNoShapePriorName, \
    kAblationNoVisFeatName, \
    kAblationNoLtmName
])

kObViSLAMColor = "tab:blue"
kORBSLAM3Color = "tab:orange"
kOASLAMColor = "tab:green"
kGTColor = "tab:red"
kDROIDColor = "tab:purple"
kAblationNoShapePriorColor = kORBSLAM3Color
kAblationNoVisFeatColor = kOASLAMColor
kAblationNoLtmColor = kDROIDColor

kApproachColorDict = {
    kObViSLAMApproachName: kObViSLAMColor, \
    kORBSLAM3ApproachName: kORBSLAM3Color, \
    kOASLAMApproachName: kOASLAMColor, \
    kGTApproachName: kGTColor, \
    kDROIDApproachName: kDROIDColor, \
    kAblationNoShapePriorName: kAblationNoShapePriorColor, \
    kAblationNoVisFeatName: kAblationNoVisFeatColor, \
    kAblationNoLtmName: kAblationNoLtmColor
}

kObViSLAMLineStyle = "solid"
kORBSLAM3LineStyle = "dotted"
kOASLAMLineStyle = "dashdot"
kGTLineStyle = "dashed"
kDROIDLineStyle = (0, (3, 5, 1, 5, 1, 5))
kAblationNoShapePriorLineStyle = kORBSLAM3LineStyle
kAblationNoVisFeatLineStyle = kOASLAMLineStyle
kAblationNoLtmLineStyle = kDROIDLineStyle

kApproachLineStyleDict = {
    kObViSLAMApproachName: kObViSLAMLineStyle, \
    kORBSLAM3ApproachName: kORBSLAM3LineStyle, \
    kOASLAMApproachName: kOASLAMLineStyle, \
    kGTApproachName: kGTLineStyle, \
    kDROIDApproachName: kDROIDLineStyle, \
    kAblationNoShapePriorName: kAblationNoShapePriorLineStyle, \
    kAblationNoVisFeatName: kAblationNoVisFeatLineStyle, \
    kAblationNoLtmName: kAblationNoLtmLineStyle
}

kObViSLAMLinewidth = 3
kORBSLAM3Linewidth = 4
kOASLAMLinewidth = 2
kGTLinewidth = 3
kDROIDLinewidth = 3
kAblationNoShapePriorLinewidth = kORBSLAM3Linewidth
kAblationNoVisFeatLinewidth = kOASLAMLinewidth
kAblationNoLtmLinewitdh = kDROIDLinewidth

kApproachLinewidthDict = {
    kObViSLAMApproachName: kObViSLAMLinewidth, \
    kORBSLAM3ApproachName: kORBSLAM3Linewidth, \
    kOASLAMApproachName: kOASLAMLinewidth, \
    kGTApproachName: kGTLinewidth, \
    kDROIDApproachName: kDROIDLinewidth, \
    kAblationNoShapePriorName: kAblationNoShapePriorLinewidth, \
    kAblationNoVisFeatName: kAblationNoVisFeatLinewidth, \
    kAblationNoLtmName: kAblationNoLtmLinewitdh
}

kObViSLAMMarker = "X"
kORBSLAM3Marker = "o"
kOASLAMMarker = "P"
kGTApproachMarker = "v"
kDROIDApproachMarker = "d"
kAblationNoShapePriorMarker = kORBSLAM3Marker
kAblationNoVisFeatMarker = kOASLAMMarker
kAblationNoLtmMarker = kDROIDApproachMarker
kApproachMarkerDict = {
    kObViSLAMApproachName: kObViSLAMMarker, \
    kORBSLAM3ApproachName: kORBSLAM3Marker, \
    kOASLAMApproachName: kOASLAMMarker, \
    kGTApproachName: kGTApproachMarker, \
    kDROIDApproachName: kDROIDApproachMarker,
    kAblationNoShapePriorName: kAblationNoShapePriorMarker, \
    kAblationNoVisFeatName: kAblationNoVisFeatMarker, \
    kAblationNoLtmName: kAblationNoLtmMarker
}

kObViSLAMMakerSize = 100
kORBSLAM3MMakerSize = 100
kOASLAMMarkerSize = 100
kGTApproachMarkerSize = 100
kDROIDApproachMarkerSize = 100,
kAblationNoShapePriorMarkerSize = 100
kAblationNoVisFeatMarkerSize = 100
kAblationNoLtmMarkerSize = 100

kAxisFontsize = 16
kGridAlpha = .4

kApproachMarkerSizeDict = {
    kObViSLAMApproachName: kObViSLAMMakerSize, \
    kORBSLAM3ApproachName: kORBSLAM3MMakerSize, \
    kOASLAMApproachName: kOASLAMMarkerSize, \
    kGTApproachName: kGTApproachMarkerSize, \
    kDROIDApproachName: kDROIDApproachMarkerSize, \
    kAblationNoShapePriorName: kAblationNoShapePriorMarkerSize, \
    kAblationNoVisFeatName: kAblationNoVisFeatMarkerSize, \
    kAblationNoLtmName: kAblationNoLtmMarkerSize
}


class MetricsFilesInfo:
    def __init__(self, approachNameAndMetricsFileInfo, primaryApproachName):
        self.primaryApproachName = primaryApproachName
        self.approachNameAndMetricsFileInfo = approachNameAndMetricsFileInfo


# def readTranslationAndOrientationConsistencyFromFile(metrics_file):
#     approachMetrics = readMetricsFile(metrics_file)
#     return (approachMetrics.sequence_metrics.all_translation_deviations,
#             approachMetrics.sequence_metrics.all_rotation_deviations)


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
    if os.path.exists(filepath):
        with open(filepath) as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            for row in csv_reader:
                errTypesAndSavepaths[row[0]] = row[1].strip()
    # if not os.path.exists(filepath):
    #     warnings.warn("Specified filepath " + filepath + " doesn't exist; Not saveing files")
    #     errTypesAndSavepaths[kCDFTranslErrorType] = None
    #     errTypesAndSavepaths[kCDFOrientErrorType] = None
    #     return errTypesAndSavepaths
    # df = pd.read_csv(filepath, delimiter=",", header=None)
    # print(df)
    # errorTypeConstants = set([kCDFTranslErrorType, kCDFOrientErrorType])
    # if (df.iloc[0, 0] not in errorTypeConstants) or (df.iloc[1, 0] not in errorTypeConstants):
    #     warnings.warn("Found invalide error type. Not saving files for those entries")
    #     # handle the file if there're unexpected entries
    #     errTypesAndSavepaths[kCDFTranslErrorType] = None
    #     errTypesAndSavepaths[kCDFOrientErrorType] = None
    #     if df.iloc[0, 0] in errorTypeConstants:
    #         errTypesAndSavepaths[df.iloc[0, 0]] = df.iloc[0, 1]
    #     if df.iloc[1, 0] in errorTypeConstants:
    #         errTypesAndSavepaths[df.iloc[1, 0]] = df.iloc[1, 1]
    # else:
    #     # handle the file if there aren't unexpected entries
    #     errTypesAndSavepaths[df.iloc[0, 0]] = df.iloc[0, 1]
    #     errTypesAndSavepaths[df.iloc[1, 0]] = df.iloc[1, 1]
    return errTypesAndSavepaths

def set_size(width_pt, fraction=1, subplots=(1, 1)):
    """Set figure dimensions to sit nicely in our document.

    Parameters
    ----------
    width_pt: float
            Document width in points
    fraction: float, optional
            Fraction of the width which you wish the figure to occupy
    subplots: array-like, optional
            The number of rows and columns of subplots.
    Returns
    -------
    fig_dim: tuple
            Dimensions of figure in inches
    """
    # Width of figure (in pts)
    fig_width_pt = width_pt * fraction
    # Convert from pt to inches
    inches_per_pt = 1 / 72.27

    # Golden ratio to set aesthetic figure height
    golden_ratio = (5**.5 - 1) / 2

    # Figure width in inches
    fig_width_in = fig_width_pt * inches_per_pt
    # Figure height in inches
    fig_height_in = fig_width_in * golden_ratio * (subplots[0] / subplots[1])

    return (fig_width_in, fig_height_in)



def getCDFData(dataset, num_bins):
    # getting data of the histogram
    # non_inf_dataset = [data_entry if (data_entry != float('inf')) else 1e10 for data_entry in dataset]
    infs_removed_dataset = [data_entry for data_entry in dataset if (data_entry != float('inf'))]
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

    return (cdf * (len(infs_removed_dataset) / len(dataset)), bins_count, max_val)
    # return (cdf , bins_count , max_val)


def plotRMSEs(primaryApproachName, errs_dict, err_type, ylims=[], legend_loc="upper left", savepath=None, height_ratios=None, legend_ncol=1, yscaleType=None, scatter=True):
    fig = plt.figure(figsize=kFigSize)
    # fig = plt.figure(figsize=set_size(505, 0.2))

    if (ylims == None) or (len(ylims) == 0):
        non_inf_max = 0
        for approach_name, errs in errs_dict.items():
            for err_val in errs:
                if (err_val != float('inf')):
                    non_inf_max = max(err_val, non_inf_max)
        ylims = [(0, non_inf_max * 1.05)]

    bax = brokenaxes(ylims=ylims, height_ratios=height_ratios, yscale=yscaleType)
    alternate_line_styles = ['dotted', 'dashdot', 'dashed', (0, (3, 1, 1, 1)), (0, (3, 1, 1, 1, 1, 1))]
    alternate_line_style_index = 0

    if (kORBSLAM3ApproachName in errs_dict):
        orb_split_idx = 6
        orb_split_display = orb_split_idx + 1  # because 1 indexed vs 0
        bax.axvline(x=orb_split_display + 0.5, color='purple', ls='--', lw=0.5)

    for approach_name, errs in errs_dict.items():
        # if approach_name not in kApproachNames:
        #     warnings.warn("Undefined approach name " + approach_name + ". Skip plotting trajectory...")
        #     continue
        xx = np.arange(len(errs)) + 1
        zorder =1
        if (approach_name == primaryApproachName):
            zorder=2
        # if (approach_name == kOASLAMApproachName):
        #     bax.scatter(xx, errs, label=approach_name, \
        #                 # plt.scatter(xx, errs, label=approach_name, \
        #                 color='#2ca02c', \
        #                 zorder=zorder,
        #                 marker=kApproachMarkerDict[approach_name], \
        #                 s=kApproachMarkerSizeDict[approach_name])
        # else:
        if (approach_name is not primaryApproachName):
            # if (comparison_approach_summary_min_max is None):
            #     comparison_approach_summary_min_max = comparison_approach_max
            # else:
            #     comparison_approach_summary_min_max = min(comparison_approach_max, comparison_approach_summary_min_max)
            # comparison_approach_summary_max = max(comparison_approach_max, comparison_approach_summary_max)
            line_style = alternate_line_styles[alternate_line_style_index]
            alternate_line_style_index += 1
        else:
            line_style = 'solid'
        if (scatter):
            bax.scatter(xx, errs, label=approach_name, \
                        # plt.scatter(xx, errs, label=approach_name, \
                        # color=kApproachColorDict[approach_name], \
                        zorder=zorder,
                        marker=kApproachMarkerDict[approach_name], \
                        s=kApproachMarkerSizeDict[approach_name])
        else:
            bax.plot(xx, errs, linestyle=line_style, zorder=zorder,
                     label=approach_name, linewidth=3)
    # bax.set_xlabel("Trajectory Number", fontsize=kAxisFontsize)
    # bax.set_ylabel(kATEErrorYLabelDict[err_type], fontsize=kAxisFontsize)
    # bax.legend(loc=legend_loc, ncol=legend_ncol, fontsize=kAxisFontsize)

    # Use this one for trajectory ATEs and object position devs?
    # bax.legend(loc="upper center", ncol=2, fontsize=kAxisFontsize, bbox_to_anchor=(0.5, 1.3))

    bax.legend(loc="upper center", ncol=2, fontsize=kAxisFontsize)
    bax.grid(alpha=0.4)
    if (yscaleType is not None):
        bax.set_yscale(yscaleType)

    lastBaxIdx = 0
    if (len(ylims) > 1):
        for i in range(len(ylims)):
            min_tick = math.floor(ylims[i][0])
            max_tick = math.ceil(ylims[i][-1])
            tick_inc = math.ceil((max_tick - min_tick) / 3)
            bax.axs[len(ylims) - 1 - i].set_yticks(np.arange(min_tick, max_tick, tick_inc), fontsize=kAxisFontsize,
                                                   labels=[str(i) for i in np.arange(min_tick, max_tick, tick_inc)])
            # bax.axs[i].set_xticks(np.arange(1, 16))
        lastBaxIdx = len(ylims) -1
    # else:
    # bax.axs[lastBaxIdx].set_xticks(np.arange(1, 16))


    fig.subplots_adjust(bottom=kFigBottomSpacing)
    bax.set_xlabel("Trajectory Number", labelpad=30, fontsize=kAxisFontsize)
    bax.set_ylabel(kATEErrorYLabelDict[err_type], fontsize=kAxisFontsize)


    plt.tight_layout(rect=(-0.05, -0.06, 1, 1))
    for handle in bax.diag_handles:
        handle.remove()
    bax.draw_diags()

    lastBaxIdx = 0
    if (len(ylims) > 1):
        for i in range(len(ylims)):
            # bax.axs[i].set_xticks([1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16], labels=[""]*16)
            bax.axs[i].set_xticks(list(np.arange(1, 17)))
        lastBaxIdx = len(ylims) -1
        for i in range(len(ylims) - 1):
            for tick in bax.axs[i].get_xticklabels():
                tick.set_visible(False)
    # else:
    bax.axs[lastBaxIdx].set_xticks(np.arange(1, 17), fontsize=kAxisFontsize, labels=[str(i) for i in range(1, 17)])

    # for ax in bax.axs:
    #     ax.xaxis.set_major_locator(MultipleLocator(1))

    # bax.set_xlabel(x_label)

# Note: cannot use tight_layout. It'll break the brokenaxis
    if savepath:
        print("Saving figure to " + savepath)

        if ("pgf" in savepath):
            print("Saving in pgf format")
            plt.savefig(savepath, format='pgf')
        else:
            plt.savefig(savepath)
    else:
        # plt.tight_layout()
        # for handle in bax.diag_handles:
        #     handle.remove()
        # bax.draw_diags()
        plt.show()


def plotCDF(primaryApproachName, approach_results, title, x_label, fig_num, bins=1000, savepath=None,
            xlims=None, width_ratios=None):
    fig=plt.figure(fig_num, figsize=kCDFFigSize)
    # fig = plt.figure(figsize=set_size(505, 0.2))
    comparison_approach_summary_max = 0
    comparison_approach_summary_min_max = None

    alternate_line_styles = ['dotted', 'dashdot', 'dashed', (0, (3, 1, 1, 1)), (0, (3, 1, 1, 1, 1, 1))]
    alternate_line_style_index = 0
    primary_approach_max = None
    # getting data of the histogram

    all_approaches_max = 0
    approach_data_to_plot = {}
    for approach_label, comparison_dataset in approach_results.items():
        print("Working on " + approach_label)
        approach_cdf, bins_count, comparison_approach_max = getCDFData(comparison_dataset, bins)
        approach_data_to_plot[approach_label] = (bins_count, approach_cdf)
        all_approaches_max = max(all_approaches_max, comparison_approach_max)
        if (approach_label is not primaryApproachName):
            if (comparison_approach_summary_min_max is None):
                comparison_approach_summary_min_max = comparison_approach_max
            else:
                comparison_approach_summary_min_max = min(comparison_approach_max, comparison_approach_summary_min_max)
            comparison_approach_summary_max = max(comparison_approach_max, comparison_approach_summary_max)
            # line_style = alternate_line_styles[alternate_line_style_index]
            # alternate_line_style_index += 1
        else:
            primary_approach_max = comparison_approach_max
            # line_style = 'solid'
        # plt.plot(bins_count, approach_cdf, linestyle=line_style,
        #          label=approach_label)

    ylims = [(0, 1)]
    # xmax = max(primary_approach_max, comparison_approach_summary_min_max)
    xmax = max(primary_approach_max, all_approaches_max)
    if (xlims == None):
        xlims=[(0, xmax)]
    else:
        xlims.append((xlims[-1][1], xmax))
    print(xlims)

    bax = brokenaxes(xlims=xlims, ylims=ylims, width_ratios=width_ratios)
    for approach_label, comparison_data_entry in approach_data_to_plot.items():
        print("Working on " + approach_label)
        bins_count = comparison_data_entry[0]
        approach_cdf = comparison_data_entry[1]
        # approach_cdf, bins_count, comparison_approach_max = getCDFData(comparison_dataset, bins)
        if (approach_label is not primaryApproachName):
            # if (comparison_approach_summary_min_max is None):
            #     comparison_approach_summary_min_max = comparison_approach_max
            # else:
            #     comparison_approach_summary_min_max = min(comparison_approach_max, comparison_approach_summary_min_max)
            # comparison_approach_summary_max = max(comparison_approach_max, comparison_approach_summary_max)
            line_style = alternate_line_styles[alternate_line_style_index]
            alternate_line_style_index += 1
        else:
            primary_approach_max = comparison_approach_max
            line_style = 'solid'
        bax.plot(bins_count, approach_cdf, linestyle=line_style,
                 label=approach_label, linewidth=3)
        # plt.plot(bins_count, approach_cdf, linestyle=line_style,
        #          label=approach_label)

    print(primary_approach_max)

    if (len(approach_results) >= 2):
        # plt.xlim(0, max(primary_approach_max, comparison_approach_summary_min_max))
        # plt.xlim(0, primary_approach_max)
        # if (primary_approach_max > comparison_approach_summary_max):
        # x_lim = primary_approach_max
        # else:
        #     x_lim = min(primary_approach_max * kMaxXAxisBoundsMultiplier, comparison_approach_summary_max)
        # plt.xlim(0, max(primary_approach_max, comparison_approach_summary_min_max))
        # bax.legend(loc="lower right", prop={'size': 'small'}, fontsize=kAxisFontsize)
        bax.legend(loc='lower right', fontsize=kAxisFontsize)

    # plt.legend(loc="lower right", prop={'size': 'small'})
    # plt.ylim(0, 1)
    # bax.set_title(title)
    # bax.set_xlabel(x_label, fontsize=kAxisFontsize)
    # bax.set_xlabel(x_label)
    # bax.set_ylabel("Proportion of data", fontsize=kAxisFontsize)
    bax.grid(alpha=0.4)
    # plt.xlabel(x_label, fontsize=kAxisFontsize)
    # plt.xlabel(x_label, labelpad=100)
    # plt.ylabel("Proportion of data", fontsize=kAxisFontsize)
    # plt.grid(alpha=0.4)

    for i in range(len(xlims)):
        min_tick = math.floor(xlims[i][0])
        max_tick = math.ceil(xlims[i][-1])
        tick_inc = math.ceil((max_tick - min_tick) / 5)
        bax.axs[i].set_xticks(np.arange(min_tick, max_tick, tick_inc), fontsize=kAxisFontsize, labels=[str(i) for i in np.arange(min_tick, max_tick, tick_inc)])

    # fig.subplots_adjust(bottom=kFigBottomSpacing)
    # plt.title(title)
    # plt.xlabel(x_label)
    # plt.ylabel("Proportion of data")
    # plt.grid(alpha=0.4)
    # bax.set_xlabel(x_label)
    # bax.set_ylabel("Proportion of data")

    # plt.xlabel(x_label)
    # plt.ylabel("Proportion of data")

    # plt.tight_layout(pad=10)
    bax.set_xlabel(x_label, labelpad=30, fontsize=kAxisFontsize)
    bax.set_ylabel("Proportion of data", fontsize=kAxisFontsize)

    plt.tight_layout(rect=(-0.05, -0.06, 1, 1))
    for handle in bax.diag_handles:
        handle.remove()
    bax.draw_diags()
    # bax.set_xlabel(x_label)

    if savepath:
        print("Saving plot to " + savepath)

        if ("pgf" in savepath):
            print("Saving in pgf format")
            plt.savefig(savepath, format='pgf')
        else:
            plt.savefig(savepath)
    else:

        plt.show()


class MetricsFileConstants:
    metricsKey = "metrics"
    rmseTranslErrLabel = "rmse_transl_err"
    rmseRotErrLabel = "rmse_rot_err"
    validPosesUsedInScoreLabel = "valid_poses_used_in_score"
    lostPosesLabel = "lost_poses"
    waypointDeviationsLabel = "waypoint_deviations"
    allTranslationDeviationsLabel = "all_translation_deviations"
    allRotationDeviationsLabel = "all_rotation_deviations"
    ateResultsLabel = "trajectory_sequence_ate_results"
    indivTrajectoryMetricsLabel = "indiv_trajectory_metrics"
    sequenceMetricsLabel = "sequence_metrics"

    objectMetricsKey = "obj_metrics"
    indivTrajectoryObjectMetricsLabel = "indiv_trajectory_object_metrics"

    missedGtObjsKey = "missed_gt_objs"
    objectsPerGtObjKey = "objects_per_gt_obj"
    avgPosDeviationKey = "average_pos_deviation"
    avgIouKey = "avg_iou"
    medianPosDeviationKey = "median_pos_deviation"
    medianIouKey = "median_iou"


class ATEResults:
    def __init__(self, rmse_transl_err, rmse_rot_err, valid_poses_used_in_score, lost_poses):
        self.rmse_transl_err = rmse_transl_err
        self.rmse_rot_err = rmse_rot_err
        self.valid_poses_used_in_score = valid_poses_used_in_score
        self.lost_poses = lost_poses

    def __repr__(self):
        return "ATEResults: {rmse_transl_err: " + str(self.rmse_transl_err) + ", rmse_rot_err: " + str(
            self.rmse_rot_err) + ", valid_poses: " + str(self.valid_poses_used_in_score) + ", lost_poses: " + str(
            self.lost_poses) + "}"


class TrajectoryMetrics:
    def __init__(self, waypoint_deviations, all_translation_deviations, all_rotation_deviations, ate_results):
        self.waypoint_deviations = waypoint_deviations
        self.all_translation_deviations = all_translation_deviations
        self.all_rotation_deviations = all_rotation_deviations
        self.ate_results = ate_results


class FullSequenceMetrics:
    def __init__(self, indiv_trajectory_metrics, sequence_metrics):
        self.indiv_trajectory_metrics = indiv_trajectory_metrics
        self.sequence_metrics = sequence_metrics


def readATEResultsFromJsonObj(ateResultsJson):
    transl_err = ateResultsJson[MetricsFileConstants.rmseTranslErrLabel]
    rot_err = ateResultsJson[MetricsFileConstants.rmseRotErrLabel]
    if (transl_err < 0):
        transl_err = float('inf')
    if (rot_err < 0):
        rot_err = float('inf')
    ateResultsObj = ATEResults(rmse_transl_err=transl_err,
                               rmse_rot_err=rot_err,
                               valid_poses_used_in_score=ateResultsJson[
                                   MetricsFileConstants.validPosesUsedInScoreLabel],
                               lost_poses=ateResultsJson[MetricsFileConstants.lostPosesLabel])
    return ateResultsObj


def readTrajectoryMetricsFromJsonObj(metricsJsonObj):
    # TODO for now we just care about the translation and rotation deviations; need to fill in waypoint deviations
    #  and ate results parsing later
    if MetricsFileConstants.allTranslationDeviationsLabel not in metricsJsonObj:
        raise ValueError(
            "entry for " + MetricsFileConstants.allTranslationDeviationsLabel + " was not in the metrics file")
    if (not (isinstance(metricsJsonObj[MetricsFileConstants.allTranslationDeviationsLabel], list))):
        raise ValueError(
            "Metrics entry with key " + MetricsFileConstants.allTranslationDeviationsLabel + " was not a list")
    all_translation_deviations = readUtVSLAMVector(metricsJsonObj[MetricsFileConstants.allTranslationDeviationsLabel])
    all_translation_deviations = [dev if (dev >= 0) else float('inf') for dev in all_translation_deviations]

    if MetricsFileConstants.allRotationDeviationsLabel not in metricsJsonObj:
        raise ValueError(
            "entry for " + MetricsFileConstants.allRotationDeviationsLabel + " was not in the metrics file")
    if (not (isinstance(metricsJsonObj[MetricsFileConstants.allRotationDeviationsLabel], list))):
        raise ValueError(
            "Metrics entry with key " + MetricsFileConstants.allRotationDeviationsLabel + " was not a list")

    all_rotation_deviations = readUtVSLAMVector(metricsJsonObj[MetricsFileConstants.allRotationDeviationsLabel])
    all_rotation_deviations = [dev if (dev >= 0) else float('inf') for dev in all_rotation_deviations]

    if MetricsFileConstants.ateResultsLabel not in metricsJsonObj:
        raise ValueError(
            "entry for " + MetricsFileConstants.ateResultsLabel + " was not in the metrics file")
    ateResults = readATEResultsFromJsonObj(metricsJsonObj[MetricsFileConstants.ateResultsLabel])

    return TrajectoryMetrics(ate_results=ateResults, all_translation_deviations=all_translation_deviations,
                             all_rotation_deviations=all_rotation_deviations,
                             waypoint_deviations=None)  # TODO fix this one


def readMetricsFile(metricsFile):
    print("Reading " + metricsFile)
    with open(metricsFile) as metricsFileObj:
        metricsFileJson = json.load(metricsFileObj)
        if MetricsFileConstants.metricsKey not in metricsFileJson:
            raise ValueError(
                "entry for " + MetricsFileConstants.metricsKey + " was not in the metrics file")
        sequenceMetricsJson = metricsFileJson[MetricsFileConstants.metricsKey]
        if MetricsFileConstants.indivTrajectoryMetricsLabel not in sequenceMetricsJson:
            raise ValueError(
                "entry for " + MetricsFileConstants.indivTrajectoryMetricsLabel + " was not in the second level of the metrics file")
        if MetricsFileConstants.sequenceMetricsLabel not in sequenceMetricsJson:
            raise ValueError(
                "entry for " + MetricsFileConstants.sequenceMetricsLabel + " was not in the second level of the metrics file")
        if (not (isinstance(sequenceMetricsJson[MetricsFileConstants.indivTrajectoryMetricsLabel], list))):
            raise ValueError(
                "Metrics entry with key " + MetricsFileConstants.indivTrajectoryMetricsLabel + " was not a list")

        sequenceMetrics = readTrajectoryMetricsFromJsonObj(
            sequenceMetricsJson[MetricsFileConstants.sequenceMetricsLabel])
        print(sequenceMetrics.all_translation_deviations)
        print(sequenceMetrics.all_rotation_deviations)
        indivTrajectoryMetrics = [readTrajectoryMetricsFromJsonObj(indivTrajJson) for indivTrajJson in
                                  readUtVSLAMVector(
                                      sequenceMetricsJson[MetricsFileConstants.indivTrajectoryMetricsLabel])]

        return FullSequenceMetrics(sequence_metrics=sequenceMetrics,
                                   indiv_trajectory_metrics=indivTrajectoryMetrics)


class SingleTrajectoryObjectMetrics:
    def __init__(self, missed_gt_objs, objects_per_gt_obj,
                 average_pos_deviation, avg_iou,
                 median_pos_deviation, median_iou):
        self.missed_gt_objs = missed_gt_objs
        self.objects_per_gt_obj = objects_per_gt_obj
        self.average_pos_deviation = average_pos_deviation
        self.avg_iou = avg_iou
        self.median_pos_deviation = median_pos_deviation
        self.median_iou = median_iou


class FullSequenceObjectMetrics:
    def __init__(self, indiv_trajectory_object_metrics):
        self.indiv_trajectory_object_metrics = indiv_trajectory_object_metrics


def readSingleTrajectoryObjectMetricsFromJsonObj(metricsJsonObj):
    # TODO reading summary values only -- not looking at per-obj metrics now, may need to revisit this
    missed_gt_objs = metricsJsonObj[MetricsFileConstants.missedGtObjsKey]
    objects_per_gt_obj = metricsJsonObj[MetricsFileConstants.objectsPerGtObjKey]
    average_pos_deviation = metricsJsonObj[MetricsFileConstants.avgPosDeviationKey]
    avg_iou = metricsJsonObj[MetricsFileConstants.avgIouKey]
    median_pos_deviation = metricsJsonObj[MetricsFileConstants.medianPosDeviationKey]
    median_iou = metricsJsonObj[MetricsFileConstants.medianIouKey]

    return SingleTrajectoryObjectMetrics(missed_gt_objs=missed_gt_objs,
                                         objects_per_gt_obj=objects_per_gt_obj,
                                         average_pos_deviation=average_pos_deviation,
                                         avg_iou=avg_iou,
                                         median_pos_deviation=median_pos_deviation,
                                         median_iou=median_iou)


def readObjectsMetricsFile(metricsFile):
    print("Reading " + metricsFile)
    with open(metricsFile) as metricsFileObj:
        metricsFileJson = json.load(metricsFileObj)
        if MetricsFileConstants.objectMetricsKey not in metricsFileJson:
            raise ValueError(
                "entry for " + MetricsFileConstants.objectMetricsKey + " was not in the metrics file")
        sequenceMetricsJson = metricsFileJson[MetricsFileConstants.objectMetricsKey]
        if MetricsFileConstants.indivTrajectoryObjectMetricsLabel not in sequenceMetricsJson:
            raise ValueError(
                "entry for " + MetricsFileConstants.indivTrajectoryObjectMetricsLabel + " was not in the second level of the metrics file")
        if (not (isinstance(sequenceMetricsJson[MetricsFileConstants.indivTrajectoryObjectMetricsLabel], list))):
            raise ValueError(
                "Metrics entry with key " + MetricsFileConstants.indivTrajectoryObjectMetricsLabel + " was not a list")

        indivTrajectoryMetrics = [readSingleTrajectoryObjectMetricsFromJsonObj(indivTrajJson) for indivTrajJson in
                                  readUtVSLAMVector(
                                      sequenceMetricsJson[MetricsFileConstants.indivTrajectoryObjectMetricsLabel])]

        return FullSequenceObjectMetrics(indiv_trajectory_object_metrics=indivTrajectoryMetrics)
