import os

class FileStructureConstants:
    intrinsicsBaseName = "camera_matrix.txt"
    extrinsicsBaseName = "extrinsics.txt"

    unsparsifiedDirectoryRootBaseName = "unsparsified_ut_vslam_in"
    sparsifiedDirectoryRootBaseName = "sparsified_ut_vslam_in"

    posesByNodeIdFileWithinSparsifiedDir = "poses/initial_robot_poses_by_node.txt"
    nodesByTimestampFileWithinSparsifiedDir = "timestamps/node_ids_and_timestamps.txt"

    jacobianDebuggingRootDirBaseName = "jacobian_debugging_out"
    ellipsoidDebuggingRootDirBaseName = "ellipsoid_debugging_out"
    utVslamOutRootDirBaseName = "ut_vslam_out"
    visualizationRosbagRootDirBaseName = "visualization_rosbags"
    logsRootDirBaseName = "logs"

    longTermMapFileBaseName = "long_term_map.json"
    visualFeatureResultsFileBaseName = "visual_feature_results.json"
    bbAssocResultsFileBaseName = "data_association_results.json"

    ellipsoidResultsFileBaseName = "ellipsoid_results.json"
    robotPoseResultsFileBaseName = "robot_pose_results.json"

    # File extensions
    jsonExtension = ".json"
    bagSuffix = ".bag"


class FileStructureUtils:
    @staticmethod
    def makeDirectory(directoryName):
        os.system("mkdir -p " + directoryName)

    @staticmethod
    def ensureDirectoryEndsWithSlash(directoryName):
        if (len(directoryName) == 0):
            raise Exception("Directory name can't be empty")
        if (directoryName[-1] == "/"):
            return directoryName
        return directoryName + "/"

    @staticmethod
    def getAndOptionallyCreateConfigSpecificResultsDirectory(resultsBaseDir, resultsTypeVariantName, sequenceBaseName,
                                                             configBaseName, dirForBagResults, create):
        directoryName = FileStructureUtils.ensureDirectoryEndsWithSlash(
            resultsBaseDir) + sequenceBaseName + "/" + \
                        configBaseName + "/" + dirForBagResults + "/" + resultsTypeVariantName + "/"
        if (create):
            FileStructureUtils.makeDirectory(directoryName)
        # TODO Should we force remove old data
        return directoryName

    @staticmethod
    def getAndOptionallyCreateUtVslamOutDirectory(resultsBaseDir, sequenceBaseName, configBaseName, dirForBagResults,
                                                  create=True):
        return FileStructureUtils.getAndOptionallyCreateConfigSpecificResultsDirectory(
            resultsBaseDir, FileStructureConstants.utVslamOutRootDirBaseName, sequenceBaseName, configBaseName,
            dirForBagResults, create)

    @staticmethod
    def getAndOptionallyCreateJacobianDebuggingDirectory(resultsBaseDir, sequenceBaseName, configBaseName,
                                                         dirForBagResults, create=True):
        return FileStructureUtils.getAndOptionallyCreateConfigSpecificResultsDirectory(
            resultsBaseDir, FileStructureConstants.jacobianDebuggingRootDirBaseName, sequenceBaseName, configBaseName,
            dirForBagResults, create)

    @staticmethod
    def getAndOptionallyCreateEllipsoidDebuggingDirectory(resultsBaseDir, sequenceBaseName, configBaseName,
                                                          dirForBagResults, create=True):
        return FileStructureUtils.getAndOptionallyCreateConfigSpecificResultsDirectory(
            resultsBaseDir, FileStructureConstants.ellipsoidDebuggingRootDirBaseName, sequenceBaseName, configBaseName,
            dirForBagResults, create)

    @staticmethod
    def getAndOptionallyCreateVisualizationRosbagDirectory(resultsBaseDir, sequenceBaseName, configBaseName,
                                                           dirForBagResults, create=True):
        return FileStructureUtils.getAndOptionallyCreateConfigSpecificResultsDirectory(
            resultsBaseDir, FileStructureConstants.visualizationRosbagRootDirBaseName, sequenceBaseName, configBaseName,
            dirForBagResults, create)

    @staticmethod
    def getAndOptionallyCreateLogsDirectory(resultsBaseDir, sequenceBaseName, configBaseName, dirForBagResults,
                                            create=True):
        return FileStructureUtils.getAndOptionallyCreateConfigSpecificResultsDirectory(
            resultsBaseDir, FileStructureConstants.logsRootDirBaseName, sequenceBaseName, configBaseName,
            dirForBagResults, create)
