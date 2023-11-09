import json
from file_structure_utils import *
from custom_json_file_parsing import *


class SequenceFileConstants:
    sequenceInfoKey = "sequence_info"
    sequenceIdentifierKey = "seq_id"
    sequenceKey = "sequence"
    bagBaseNameKey = "bag_base_name"
    waypointFileBaseNameKey = "waypoint_file_base_name"


def generateSequenceFilePath(sequenceFilesDirectory, sequenceFileBaseName):
    return FileStructureUtils.ensureDirectoryEndsWithSlash(sequenceFilesDirectory) + \
           sequenceFileBaseName + FileStructureConstants.jsonExtension


def readTrajectorySequenceAndWaypoints(sequenceFilesDirectory, sequenceFileBaseName):
    sequenceFile = generateSequenceFilePath(sequenceFilesDirectory, sequenceFileBaseName)

    with open(sequenceFile) as sequenceFileObj:
        sequenceFileJson = json.load(sequenceFileObj)
        if SequenceFileConstants.sequenceInfoKey not in sequenceFileJson:
            raise ValueError(
                "entry for " + SequenceFileConstants.sequenceInfoKey + " was not in the sequence file")
        secondLevelSeqInfo = sequenceFileJson[SequenceFileConstants.sequenceInfoKey]
        if SequenceFileConstants.sequenceIdentifierKey not in secondLevelSeqInfo:
            raise ValueError(
                "entry for " + SequenceFileConstants.sequenceInfoKey + " was not in the sequence file second level")
        # if (secondLevelSeqInfo[SequenceFileConstants.sequenceIdentifierKey] != sequenceFileBaseName):
        #     raise ValueError(
        #         "entry for " + SequenceFileConstants.sequenceIdentifierKey + " did not match the sequence file base name")
        if (SequenceFileConstants.sequenceKey not in secondLevelSeqInfo):
            raise ValueError(
                "Sequence entry with key " + SequenceFileConstants.sequenceKey + " was missing from the sequence file")
        if (not (isinstance(secondLevelSeqInfo[SequenceFileConstants.sequenceKey], list))):
            raise ValueError("Sequence entry with key " + SequenceFileConstants.sequenceKey + " was not a list")

        sequenceEntries = []
        for individualSeqEntry in secondLevelSeqInfo[SequenceFileConstants.sequenceKey]:
            if (SequenceFileConstants.bagBaseNameKey not in individualSeqEntry):
                raise ValueError(
                    "Key " + SequenceFileConstants.bagBaseNameKey + " was missing from the sequence entry")
            bagName = individualSeqEntry[SequenceFileConstants.bagBaseNameKey]
            if (SequenceFileConstants.waypointFileBaseNameKey not in individualSeqEntry):
                raise ValueError(
                    "Key " + SequenceFileConstants.waypointFileBaseNameKey + " was missing from the sequence entry")
            waypointBaseNameEntry = individualSeqEntry[SequenceFileConstants.waypointFileBaseNameKey]
            if (CustomJsonConstants.hasValueKey not in waypointBaseNameEntry):
                raise ValueError(
                    "Key " + CustomJsonConstants.hasValueKey + " was missing from the waypoint file base name entry")
            if (waypointBaseNameEntry[CustomJsonConstants.hasValueKey] == 0):
                sequenceEntries.append((bagName, None))
            else:
                if (CustomJsonConstants.valueKey not in waypointBaseNameEntry):
                    raise ValueError(
                        "Key " + CustomJsonConstants.valueKey + " was missing from the waypoint file base name entry and there was supposed to be a value")
                sequenceEntries.append((bagName, waypointBaseNameEntry[CustomJsonConstants.valueKey]))

        return sequenceEntries


def readTrajectorySequence(sequenceFilesDirectory, sequenceFileBaseName):
    sequenceEntries = readTrajectorySequenceAndWaypoints(sequenceFilesDirectory, sequenceFileBaseName)
    return [sequenceEntry[0] for sequenceEntry in sequenceEntries]
