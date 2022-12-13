import os
import sys, getopt
from collections import defaultdict
from scipy.spatial.transform import Rotation as R


# The goal of this file is to format the feature observations in a way that can be used by the object-vslam
# This generates the features and depths files
# Feature position initalization and trajectory initialization are handled in unproject main


def add_to_features_mono(features_dict, line):
    tokens = line.split()
    feature_id = int(tokens[0])
    # ss = line[len(tokens[0]):]
    # features_dict[feature_id] = ss 
    measurement_x = float(tokens[1])
    measurement_y = float(tokens[2])
    features_dict[feature_id] = (measurement_x, measurement_y)


def add_to_features_stereo(features_dict, line):
    tokens = line.split()
    feature_id = int(tokens[0])
    measurement_x1 = float(tokens[1])
    measurement_y1 = float(tokens[2])
    depth = float(tokens[3])
    if (depth < 0):  # TODO: need to figure out why it happens
        return
    measurement_x2 = float(tokens[4])
    measurement_y2 = float(tokens[5])
    features_dict[feature_id] = (depth, measurement_x1, measurement_y1, measurement_x2, measurement_y2)


def add_to_features(features_dict, line, stereo=True):
    if stereo:
        add_to_features_stereo(features_dict, line)
    else:
        add_to_features_mono(features_dict, line)


def associate_odom_TUM(dataset_path, timestamp):
    fp = open(dataset_path + "groundtruth.txt")
    lines = fp.readlines()
    for line in lines:
        if line[0] == "#":
            continue  # skip comments
        tokens = line.split()
        time = float(tokens[0])
        if time > timestamp:
            odom = ""
            for token in tokens[1:]:
                odom = odom + token + " "
            odom = odom + "\n"
            return time, odom
    return None


def associate_odom_KITTI(dataset_path, timestamp_idx, seqno):
    if seqno < 10:
        fp_odom = open(dataset_path + "poses/0" + str(seqno) + ".txt")
    else:
        fp_odom = open(dataset_path + "poses/" + str(seqno) + ".txt")
    lines = fp_odom.readlines()
    line = lines[timestamp_idx]
    tokens = line.split()
    odom = []
    for token in tokens:
        odom.append(float(token))
    rotation = R.from_matrix([[odom[0], odom[1], odom[2]],
                              [odom[4], odom[5], odom[6]],
                              [odom[8], odom[9], odom[10]]]).as_quat()
    translation = [odom[3], odom[7], odom[11]]
    odom = str(translation[0])
    for t in translation[1:]:
        odom = odom + " " + str(t)
    for r in rotation:
        odom = odom + " " + str(r)
    odom += "\n"
    fp_odom.close()
    return odom


def associate_odom(dataset_path, timestamp, which_dataset="TUM", args=[]):
    if which_dataset == "TUM":
        return associate_odom_TUM(dataset_path, timestamp)
    elif which_dataset == "KITTI":
        return associate_odom_KITTI(dataset_path, timestamp, args[0])
    else:
        return None


def merge_files(fps_list, fp_out, min_frame_id, fp_depth_out=None, dataset_path=None, timestamp=None, which_dataset=""):
    lines_list = []
    for fp in fps_list:
        lines_list.append(fp.readlines())
    frame_id = ""
    frame_pose = ""
    features_dict = dict()
    for lines in lines_list:
        frame_id = int(lines[0]) - min_frame_id
        frame_pose = lines[1]
        for line in lines[2:]:
            add_to_features(features_dict, line)
    fp_out.write(str(frame_id) + "\n")
    if fp_depth_out is not None:
        fp_depth_out.write(str(frame_id) + "\n")
    if dataset_path is not None:
        if timestamp == None:
            print("timestamp is unintialized; exiting")
            exit(1)
        if which_dataset == "TUM":
            time, frame_pose = associate_odom(dataset_path, timestamp)
        elif which_dataset == "KITTI":
            frame_pose = associate_odom(dataset_path, int(frame_id), which_dataset=which_dataset, args=[3])
        # print(time , frame_pose)
    fp_out.write(frame_pose)
    if fp_depth_out is not None:
        fp_depth_out.write(frame_pose)
    delimiter = " "
    for feature_id, measurement in features_dict.items():
        if len(measurement) == 2:
            fp_out.write(str(feature_id + 1) + delimiter + str(measurement[0]) + delimiter + str(measurement[1]) + "\n")
        else:
            # TODO hardcoded for now; need to change into more general formats
            camera1_id = 1
            camera2_id = 2
            str_out = ""
            str_out += str(feature_id + 1) + delimiter
            str_out += str(camera1_id) + delimiter + str(measurement[1]) + delimiter + str(measurement[2]) + delimiter
            str_out += str(camera2_id) + delimiter + str(measurement[3]) + delimiter + str(measurement[4]) + "\n"
            fp_out.write(str_out)
            fp_depth_out.write(str(feature_id + 1) + " " + str(measurement[0]) + "\n")
    fp_out.close()
    fp_depth_out.close()

if __name__ == '__main__':
    dataset_path = None  # if specified, associate groundtruth poses in dataset with each time frame
    input_path = None
    output_path = None
    camera_type = "stereo"
    try:
        opts, args = getopt.getopt(sys.argv[1:], "hi:o:d:t:",
                                   ["input-path=", "output-path=", "dataset-path=", "camera-type="])
    except getopt.GetoptError:
        print('test.py -i <input-path> -o <output-path> -d <dataset-path> -t <camera-type>')
        sys.exit(1)
    minFrameId = float('inf')
    for opt, arg in opts:
        if opt == '-h':
            print('test.py -i <dataset-path> -o <output-path> -d <dataset-path> -t <camera-type>')
        elif opt in ("-i", "dataset-path"):
            input_path = arg
        elif opt in ("-o", "output-path"):
            output_path = arg
        elif opt in ("-d", "dataset-path"):
            dataset_path = arg
        elif opt in ("-t", "camera-type"):
            camera_type = arg
    if output_path == None:
        print("missing dataset_path or output_path")
        exit(1)
    print("input_path",   input_path)
    print("output_path",  output_path)
    print("dataset_path", dataset_path)
    filenames_dict = defaultdict(list)
    for filename in os.listdir(input_path):
        tokens = filename.split("_")
        try:
            frame_id = int(tokens[0])
            minFrameId = min(frame_id, minFrameId)
        except ValueError as e:
            continue
        try:
            timestamp = float(tokens[2].split(".")[0])
        except ValueError as e:
            print("unexpected ValueError when parsing timestamp " + timestamp)
            exit(1)
        filenames_dict[(frame_id, timestamp)].append(filename)
    for (frame_id, timestamp), filenames in filenames_dict.items():
        modified_frame_id = frame_id - minFrameId
        fps_list = []
        for filename in filenames:
            fp = open(os.path.join(input_path, filename), "r")
            if fp.closed:
                print("cannot open file " + input_path + filename)  # FIXME
            fps_list.append(fp)
        fp_out = open(output_path + str(modified_frame_id) + ".txt", "w")
        fp_out.seek(0)
        fp_out.truncate()
        if camera_type == "stereo":
            if not os.path.exists(output_path + "depths/"):
                os.makedirs(output_path + "depths/")
            fp_depth_out = open(output_path + "depths/" + str(modified_frame_id) + ".txt", "w")
            fp_depth_out.seek(0)
            fp_depth_out.truncate()
        else:
            fp_depth_out = None
        if dataset_path == None:
            merge_files(fps_list, fp_out, minFrameId, fp_depth_out)
        else:
            merge_files(fps_list, fp_out, minFrameId, fp_depth_out, dataset_path, timestamp)
