import os
import subprocess
import signal
import time


def getImageStreamsByResolution():
    imagesByResDict = {}

    imagesByResDict[960] = [("/robodata/husky_logs/eer_bags/Amanda/20230222/resolution_1_scale_0_5/1a/1677097619.bag",
                             "/zed/zed_node/left/image_rect_color/compressed"),
                            ("/robodata/husky_logs/eer_bags/Amanda/20230222/resolution_1_scale_0_5/1a/1677097619.bag",
                             "/zed/zed_node/right/image_rect_color/compressed"),
                            ("/robodata/husky_logs/eer_bags/Amanda/20230222/resolution_1_scale_0_5/2a/1677097326.bag",
                             "/zed/zed_node/left/image_rect_color/compressed"),
                            ("/robodata/husky_logs/eer_bags/Amanda/20230222/resolution_1_scale_0_5/2a/1677097326.bag",
                             "/zed/zed_node/right/image_rect_color/compressed")]

    imagesByResDict[640] = [("/robodata/husky_logs/eer_bags/Amanda/20230222/resolution_2_scale_0_5/1677097901.bag",
                             "/zed/zed_node/left/image_rect_color/compressed"),
                            ("/robodata/husky_logs/eer_bags/Amanda/20230222/resolution_2_scale_0_5/1677097901.bag",
                             "/zed/zed_node/right/image_rect_color/compressed"),
                            ("/robodata/husky_logs/eer_bags/Amanda/20230222/resolution_2_scale_0_5/1677098149.bag",
                             "/zed/zed_node/left/image_rect_color/compressed"),
                            ("/robodata/husky_logs/eer_bags/Amanda/20230222/resolution_2_scale_0_5/1677098149.bag",
                             "/zed/zed_node/right/image_rect_color/compressed")]
    imagesByResDict[832] = [("/robodata/husky_logs/eer_bags/Amanda/20230222/resolution_2_scale_0_65/1677098503.bag",
                             "/zed/zed_node/left/image_rect_color/compressed"),
                            ("/robodata/husky_logs/eer_bags/Amanda/20230222/resolution_2_scale_0_65/1677098503.bag",
                             "/zed/zed_node/right/image_rect_color/compressed"),
                            ("/robodata/husky_logs/eer_bags/Amanda/20230222/resolution_2_scale_0_65/1677098819.bag",
                             "/zed/zed_node/left/image_rect_color/compressed"),
                            ("/robodata/husky_logs/eer_bags/Amanda/20230222/resolution_2_scale_0_65/1677098819.bag",
                             "/zed/zed_node/right/image_rect_color/compressed")
                            ("/robodata/husky_logs/eer_bags/Amanda/20230222/resolution_2_scale_0_65/1677099139.bag",
                             "/zed/zed_node/left/image_rect_color/compressed"),
                            ("/robodata/husky_logs/eer_bags/Amanda/20230222/resolution_2_scale_0_65/1677099139.bag",
                             "/zed/zed_node/right/image_rect_color/compressed")
                            ]

    imagesByResDict[1224] = [("/robodata/husky_logs/eer_bags/Amanda/20230222/resolution_1_scale_0_5/1a/1677097619.bag",
                              "/stereo/left/image_raw/compressed"),
                             ("/robodata/husky_logs/eer_bags/Amanda/20230222/resolution_1_scale_0_5/1a/1677097619.bag",
                              "/stereo/right/image_raw/compressed"),
                             ("/robodata/husky_logs/eer_bags/Amanda/20230222/resolution_1_scale_0_5/2a/1677097326.bag",
                              "/stereo/left/image_raw/compressed"),
                             ("/robodata/husky_logs/eer_bags/Amanda/20230222/resolution_1_scale_0_5/2a/1677097326.bag",
                              "/stereo/right/image_raw/compressed"),
                             ("/robodata/husky_logs/eer_bags/Amanda/20230222/resolution_2_scale_0_5/1677097901.bag",
                              "/stereo/left/image_raw/compressed"),
                             ("/robodata/husky_logs/eer_bags/Amanda/20230222/resolution_2_scale_0_5/1677097901.bag",
                              "/stereo/right/image_raw/compressed"),
                             ("/robodata/husky_logs/eer_bags/Amanda/20230222/resolution_2_scale_0_5/1677098149.bag",
                              "/stereo/left/image_raw/compressed"),
                             ("/robodata/husky_logs/eer_bags/Amanda/20230222/resolution_2_scale_0_5/1677098149.bag",
                              "/stereo/right/image_raw/compressed"),
                             ("/robodata/husky_logs/eer_bags/Amanda/20230222/resolution_2_scale_0_65/1677098503.bag",
                              "/stereo/left/image_raw/compressed"),
                             ("/robodata/husky_logs/eer_bags/Amanda/20230222/resolution_2_scale_0_65/1677098503.bag",
                              "/stereo/right/image_raw/compressed"),
                             ("/robodata/husky_logs/eer_bags/Amanda/20230222/resolution_2_scale_0_65/1677098819.bag",
                              "/stereo/left/image_raw/compressed"),
                             ("/robodata/husky_logs/eer_bags/Amanda/20230222/resolution_2_scale_0_65/1677098819.bag",
                              "/stereo/right/image_raw/compressed"),
                             ("/robodata/husky_logs/eer_bags/Amanda/20230222/resolution_2_scale_0_65/1677099139.bag",
                              "/stereo/left/image_raw/compressed"),
                             ("/robodata/husky_logs/eer_bags/Amanda/20230222/resolution_2_scale_0_65/1677099139.bag",
                              "/stereo/right/image_raw/compressed"),
                             ]


def startYoloForResolution(resolution):
    yoloCmdArgs = []
    yoloCmdArgs.append("python3 ")
    yoloCmdArgs.append("/home/aaadkins/workspaces/yolov5-ros/detect_ros.py")
    yoloCmdArgs.append("--weights")
    yoloCmdArgs.append("/robodata/taijing/object-slam/yolov5-models/outdoors-finial-yolov5s-1.pt")
    yoloCmdArgs.append("--img")
    yoloCmdArgs.append(str(resolution))
    yoloCmdArgs.append("--conf")
    yoloCmdArgs.append("0.01")
    return subprocess.Popen(yoloCmdArgs, preexec_fn=os.setsid)


def killYolo(processReturnInfo):
    os.killpg(os.getpgid(processReturnInfo.pid), signal.SIGTERM)


def runImageGetterForResolution(resolution, matchingBags):
    print("Getting bb counts for resolution: " + str(resolution))
    processReturnInfo = startYoloForResolution(resolution)

    time.sleep(10)

    for matchingBag in matchingBags:
        resolutionGetterCmd = "./bin/check_obj_detection_freq --rosbag_file " + matchingBag[0] + " --img_topic_name " + \
                              matchingBag[1]
        print("Running cmd '" + resolutionGetterCmd + "'")
        os.system(resolutionGetterCmd)

    os.killpg(os.getpgid(processReturnInfo.pid), signal.SIGTERM)


def getResolutionByBag():
    bagsAndTopicsByResolution = getImageStreamsByResolution()

    for resolution, matchingBags in bagsAndTopicsByResolution.items():
        runImageGetterForResolution(resolution, matchingBags)


if __name__ == "__main__":
    getResolutionByBag()
