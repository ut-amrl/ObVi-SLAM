import json
from file_structure_utils import *

class SequenceFileConstants:
    sequenceIdentifierKey = "seq_id"
    sequenceKey = "sequence"

def generateSequenceFilePath(sequenceFilesDirectory, sequenceFileBaseName):
    return FileStructureUtils.ensureDirectoryEndsWithSlash(sequenceFilesDirectory) + \
           sequenceFileBaseName + FileStructureConstants.jsonExtension

def readTrajectorySequence(sequenceFilesDirectory, sequenceFileBaseName):
    sequenceFile = generateSequenceFilePath(sequenceFilesDirectory, sequenceFileBaseName)

    with open(sequenceFile) as sequenceFileObj:
        sequenceFileJson = json.load(sequenceFileObj)
        if SequenceFileConstants.sequenceIdentifierKey not in sequenceFileJson:
            raise ValueError(
                "entry for " + SequenceFileConstants.sequenceIdentifierKey + " was not in the sequence file")
        if (sequenceFileJson[SequenceFileConstants.sequenceIdentifierKey] != sequenceFileBaseName):
            raise ValueError(
                "entry for " + SequenceFileConstants.sequenceIdentifierKey + " did not match the sequence file base name")
        if (SequenceFileConstants.sequenceKey not in sequenceFileJson):
            raise ValueError(
                "Sequence entry with key " + SequenceFileConstants.sequenceKey + " was missing from the sequence file")
        if (not (isinstance(sequenceFileJson[SequenceFileConstants.sequenceKey], list))):
            raise ValueError("Sequence entry with key " + SequenceFileConstants.sequenceKey + " was not a list")
        return sequenceFileJson[SequenceFileConstants.sequenceKey]
