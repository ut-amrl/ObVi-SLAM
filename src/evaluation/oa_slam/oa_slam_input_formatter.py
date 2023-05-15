import pandas as pd
import json
import os
import argparse

def parse_opt():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data_path", default="", type=str)
    args = parser.parse_args()
    return args

def parse_frame_detections(lines, idx):
    frame_detections = {}
    if lines[idx].strip() != "file_name":
        raise ValueError("Expect: file_name; Got: " + lines[idx].strip())
    idx += 1
    frame_detections["file_name"] = lines[idx].strip()
    idx += 1
    if lines[idx].strip() != "detections":
        raise ValueError("Expect: detections; Got: " + lines[idx].strip())
    frame_detections["detections"] = []
    idx += 1
    while lines[idx] != "\n":
        detection = {}
        if lines[idx].strip() != "category_id":
            raise ValueError("Expect: category_id; Got: " + lines[idx].strip())
        idx += 1
        detection["category_id"] = int(lines[idx].strip())
        idx += 1
        if lines[idx].strip() != "detection_score":
            raise ValueError("Expect: detection_score; Got: " + lines[idx].strip())
        idx += 1
        detection["detection_score"] = float(lines[idx].strip())
        idx += 1
        if lines[idx].strip() != "bbox":
            raise ValueError("Expect: bbox; Got: " + lines[idx].strip())
        idx += 1
        tokens = lines[idx].strip().split(",")
        bbox = []
        for token in tokens:
            bbox.append(float(token))
        detection["bbox"] = bbox
        idx += 1
        frame_detections["detections"].append(detection)
    return frame_detections, idx


def parse_detections(args):
    all_detections = []
    if not os.path.exists(args.data_path):
        raise FileNotFoundError(args.data_path + "doesn't exist!")
    for cam_id in [1, 2]:
        detection_dir = os.path.join(args.data_path, str(cam_id))
        detection_dir = os.path.join(detection_dir, "detections")
        if not os.path.exists(detection_dir):
            raise FileNotFoundError(detection_dir + "doesn't exist!")
        detection_path = os.path.join(detection_dir, "detections.txt")
        f = open(detection_path, "r")
        if f.closed:
            raise FileNotFoundError("Cannot open file " + detection_path)
        lines = f.readlines()
        idx = 0
        while idx < len(lines):
            frame_detection, idx = parse_frame_detections(lines, idx)
            all_detections.append(frame_detection)
            idx += 1
        f.close()
        detection_path = os.path.join(detection_dir, "detections.json")
        f = open(detection_path, "w")
        if f.closed:
            raise FileNotFoundError("Cannot open file " + detection_path)
        all_detections_json = json.dumps(all_detections)
        f.write(all_detections_json)
        f.close()

if __name__ == "__main__":
    args = parse_opt()
    print(args)
    parse_detections(args)