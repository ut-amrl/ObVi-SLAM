import json
import argparse
import os

# https://github.com/ultralytics/yolov5/blob/master/data/coco.yaml
kIds2Clsnames = {0:"person",
                 14:"bird", 
                 15:"cat", 
                 16:"dog", 
                 23:"giraffe", 
                 26:"handbag", 
                 32:"sports ball", 
                 39:"bottle", 
                 41:"cup", 
                 42:"fork",
                 43:"knife",
                 44:"spoon",
                 45:"bowl",
                 47:"apple",
                 51:"carrot",
                 54:"donut",
                 56:"chair",
                 58:"potted plant",
                 60:"dinning table",
                 62:"tv",
                 64:"mouse",
                 65:"remote",
                 66:"keyboard",
                 67:"cell phone", 
                 73:"book", 
                 75: "vase",
                 76: "scissors",
                 77:"teddy bear", }
kDelimiter = ", "

def double2Ints(time: float):
    return int(time), int((time-int(time))*1e9)

def parse_args():
    parser = argparse.ArgumentParser(description="Convert object detections from OA-SLAM to ObVi-SLAM format") 
    parser.add_argument("--inpath", required=True, type=str, help="input detection file .json")
    parser.add_argument("--outpath", required=True, type=str, help="output detection file .csv")
    args = parser.parse_args()
    return args

if __name__ == "__main__":
    args = parse_args()
    ifp = open(args.inpath, "r")
    if ifp.closed:
        raise FileNotFoundError("Failed to open file " + args.inpath)
    ofp = open(args.outpath, "w")
    if ofp.closed:
        raise FileNotFoundError("Failed to open file " + args.outpath)
    data = json.load(ifp)
    ofp.write("min_pixel_x, min_pixel_y, max_pixel_x, max_pixel_y, semantic_class, seconds, nano_seconds, camera_id, detection_confidence\n")

    for fdata in data:
        seconds, nano_seconds = double2Ints(float(fdata["file_name"][:-4]))
        camera_id = 1
        for detection in fdata["detections"]:
            minx,miny,maxx,maxy = detection["bbox"][0],detection["bbox"][1],detection["bbox"][2],detection["bbox"][3]
            cls = detection["category_id"]
            if cls not in kIds2Clsnames.keys():
                print(cls)
                continue
            cls = kIds2Clsnames[cls]
            detection_confidence = detection["detection_score"]
            line = [ minx,miny,maxx,maxy,cls,seconds,nano_seconds,camera_id,detection_confidence]
            line = [str(token) for token in line]
            ofp.write(line[0])
            for token in line[1:]:
                ofp.write(kDelimiter + token)
            ofp.write("\n")

    ifp.close()
    ofp.close()