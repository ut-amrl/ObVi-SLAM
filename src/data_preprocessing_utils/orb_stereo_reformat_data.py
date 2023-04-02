import argparse
import sys
import os
import shutil
from collections import defaultdict
from tqdm import tqdm
import warnings

parser = argparse.ArgumentParser()
parser.add_argument("-i", "--input", default=None,
                    help="input data directory (raw orb output)")
parser.add_argument("-o", "--output", default=None,
                    help="output data directory (formated vlsam input)")


def add_to_features(features_dict, line):
    tokens = line.split()
    try:
        feature_id = int(tokens[0])
        measurement_x1 = float(tokens[1])
        measurement_y1 = float(tokens[2])
        depth = float(tokens[3])
        if (depth < 0):  # TODO: fix me
            return
        measurement_x2 = float(tokens[4])
        measurement_y2 = float(tokens[5])
    except ValueError as e:
        return
    features_dict[feature_id] = (
        depth, measurement_x1, measurement_y1, measurement_x2, measurement_y2)


def parse_filenames_dict(input_path: str, output_path: str):
    input_filenames_dict = defaultdict(list)
    for filename in os.listdir(input_path):
        if not filename.endswith(".txt"):
            continue
        tokens = filename.split("_")
        try:
            frame_id = int(tokens[0])
            compared_frame_id = int(tokens[-1].split(".")[0])
        except ValueError as e:
            print("unexpected ValueError when parsing frame_id " +
                  tokens[0], file=sys.stderr)
        if (compared_frame_id != frame_id-1) and (compared_frame_id != frame_id+1):
            warnings.warn("Unexpected Filename: " + filename)
            continue
        try:
            timestamp = float(tokens[2])
        except ValueError as e:
            print("unexpected ValueError when parsing timestamp " +
                  tokens[2], file=sys.stderr)
        input_filenames_dict[(frame_id, timestamp)].append(filename)

    if os.path.exists(output_path):
        shutil.rmtree(output_path)
    if not os.path.exists(output_path):
        os.makedirs(output_path)
    output_depth_path = os.path.join(output_path, "depths")
    if not os.path.exists(output_depth_path):
        os.makedirs(output_depth_path)
    output_filepaths_dict = defaultdict(list)
    for (frame_id, timestamp), filenames in input_filenames_dict.items():
        for filename in filenames:
            output_filepaths_dict[(frame_id, timestamp)] = (os.path.join(output_path, str(
                frame_id) + ".txt"), os.path.join(output_depth_path, str(frame_id) + ".txt"))
    input_filepaths_dict = defaultdict(list)
    for (frame_id, timestamp), filenames in input_filenames_dict.items():
        for filename in filenames:
            input_filepaths_dict[(frame_id, timestamp)].append(
                os.path.join(input_path, filename))

    return input_filepaths_dict, output_filepaths_dict


def merge_files(input_filepaths_dict: dict, output_filepaths_dict: dict):
    for (frame_id, timestamp), input_filepaths in tqdm(input_filepaths_dict.items()):
        for input_filepath in input_filepaths:
            input_fp = open(input_filepath, "r")
            if input_fp.closed:
                print("cannot open file " + input_filepath, file=sys.stderr)
                exit(1)
            output_filepath, output_depth_filepath = output_filepaths_dict[(
                frame_id, timestamp)]
            output_fp = open(output_filepath, "w")
            if output_fp.closed:
                print("cannot open file " + output_filepath, file=sys.stderr)
                exit(1)
            output_depth_fp = open(output_depth_filepath, "w")
            if output_depth_fp.closed:
                print("cannot open file " +
                      output_depth_filepath, file=sys.stderr)
                exit(1)
            lines = input_fp.readlines()
            if len(lines) < 2:
                print("unexpected file length from file " +
                      input_filepath, file=sys.stderr)
                exit(1)
            frame_id_str = lines[0]
            assert frame_id_str == str(frame_id) + "\n"
            output_fp.write(frame_id_str)
            output_depth_fp.write(frame_id_str)
            frame_pose_str = lines[1]
            output_fp.write(frame_pose_str)
            output_depth_fp.write(frame_pose_str)

            features_dict = dict()
            for line in lines[2:]:
                add_to_features(features_dict, line)
            delimiter = " "
            for feature_id, measurement in features_dict.items():
                camera1_id = 1
                camera2_id = 2
                str_out = ""
                str_out += str(feature_id) + delimiter
                str_out += str(camera1_id) + delimiter + \
                    str(measurement[1]) + delimiter + \
                    str(measurement[2]) + delimiter
                str_out += str(camera2_id) + delimiter + \
                    str(measurement[3]) + delimiter + \
                    str(measurement[4]) + "\n"
                output_fp.write(str_out)
                output_depth_fp.write(
                    str(feature_id) + " " + str(measurement[0]) + "\n")
            output_fp.close()
            output_depth_fp.close()


if __name__ == '__main__':
    args = parser.parse_args()
    input_path = args.input
    output_path = args.output

    if input_path == None or output_path == None:
        print("missing input path or output path", file=sys.stderr)
        exit(1)

    input_filepaths_dict, output_filepaths_dict = parse_filenames_dict(
        input_path, output_path)
    merge_files(input_filepaths_dict, output_filepaths_dict)
