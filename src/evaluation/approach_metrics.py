import json
from custom_json_file_parsing import *


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
    print(ateResultsJson)
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
    print(ateResultsObj)
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
