import argparse
import rosbag
import csv
from darknet_ros_msgs.msg import BoundingBoxes


class BoundingBoxWithTimestamp:

    def __init__(self, min_pixel_x, min_pixel_y, max_pixel_x, max_pixel_y, semantic_class, seconds, nano_seconds, detection_confidence, camera_id=None):
        self.min_pixel_x = min_pixel_x
        self.min_pixel_y = min_pixel_y
        self.max_pixel_x = max_pixel_x
        self.max_pixel_y = max_pixel_y
        self.semantic_class = semantic_class
        self.seconds = seconds
        self.nano_seconds = nano_seconds
        self.camera_id = camera_id
        self.detection_confidence = detection_confidence


    def getEntriesAsStringList(self):
        entries = [str(self.min_pixel_x), str(self.min_pixel_y), str(self.max_pixel_x), str(self.max_pixel_y), str(self.semantic_class), str(self.seconds), str(self.nano_seconds), str(self.detection_confidence)]
        if (self.camera_id is not None):
            entries.append(str(self.camera_id))
        return entries

    # double min_pixel_x;
# double min_pixel_y;
# double max_pixel_x;
# double max_pixel_y;
#
# /**
# * Semantic class of the detected bounding box
# *
# * TODO should this instead be an index? Should it store a set of
# * possible semantic classes with their likelihood?
# */
# std::string semantic_class;
#
# uint32_t seconds;
# uint32_t nano_seconds;
#
# /**
# * Id of the camera that captured this boundign box.
# uint64_t camera_id;
# };

def parseArgs():
    parser = argparse.ArgumentParser(description='Plot results.')
    # parser.add_argument('param_prefix', required=False, default="", help='Parameter/node prefix')
    parser.add_argument('--bag_file')
    parser.add_argument('--out_file_name')
    parser.add_argument('--yolo_conf', type=float, default='0.01')

    args = parser.parse_args()
    return args

def writeBoundingBoxes(filename, bounding_boxes):
    with open(filename, 'w') as csvfile:
        csv_writer = csv.writer(csvfile, delimiter=',',
                                quotechar='|', quoting=csv.QUOTE_MINIMAL)
        csv_writer.writerow(["min_pixel_x",
                             "min_pixel_y",
                             "max_pixel_x",
                             "max_pixel_y",
                             "semantic_class",
                             "seconds",
                             "nano_seconds",
                             "camera_id",
                             "detection_confidence"])
        for detail_entry in bounding_boxes:
            csv_writer.writerow(detail_entry.getEntriesAsStringList())

if __name__ == "__main__":

    cmdLineArgs = parseArgs()
    bag_file_name = cmdLineArgs.bag_file
    print(bag_file_name)
    out_file_name = cmdLineArgs.out_file_name
    yolo_conf = cmdLineArgs.yolo_conf
    bounding_boxes_out = []

    bag = rosbag.Bag(bag_file_name)

    msg_stamp_for_image_stamp = {}

    for topic, msg, t in bag.read_messages(topics=['/darknet_ros/bounding_boxes']):
        if (msg.image_header.stamp not in msg_stamp_for_image_stamp):
            msg_stamp_for_image_stamp[msg.image_header.stamp] = msg.header.stamp
        else:
            continue
            # print("Curr msg header vs recorded ")
            # currMsgStampForImageStamp =msg_stamp_for_image_stamp[msg.image_header.stamp]
            # print(msg.header.stamp)
            # print(currMsgStampForImageStamp)
            # if (msg.header.stamp != currMsgStampForImageStamp):
            #     print("Multiple message stamps for image stamp")
            #     exit(1)
        # print(msg)
        header = msg.image_header
        # print("Detection stamp vs image stamp")
        # print(msg.header.stamp)
        # print(header.stamp)

        if (len(msg.bounding_boxes) > 0):
            bounding_boxes = msg.bounding_boxes
            for bb in bounding_boxes:
                if (bb.probability < yolo_conf):
                    continue
                bb_out = BoundingBoxWithTimestamp(bb.xmin, bb.ymin, bb.xmax, bb.ymax, bb.Class, header.stamp.secs, header.stamp.nsecs, 0, bb.probability)
                bounding_boxes_out.append(bb_out)

    for topic, msg, t in bag.read_messages(topics=['/yolov5/bboxes']):
        if (msg.header.stamp not in msg_stamp_for_image_stamp):
            msg_stamp_for_image_stamp[msg.header.stamp] = msg.header.stamp
        else:
            continue

        header = msg.header
        # print("Detection stamp vs image stamp")
        # print(msg.header.stamp)
        # print(header.stamp)

        if (len(msg.bboxes) > 0):
            bounding_boxes = msg.bboxes
            for bb in bounding_boxes:
                if (bb.conf < yolo_conf):
                    continue
                keepBb = True
                for bb_coord in bb.xyxy:
                    if (bb_coord < 0):
                        keepBb = False
                if (not keepBb):
                    continue
                bb_out = BoundingBoxWithTimestamp(bb.xyxy[0], bb.xyxy[1], bb.xyxy[2], bb.xyxy[3], bb.label, header.stamp.secs, header.stamp.nsecs, 0, bb.conf)
                bounding_boxes_out.append(bb_out)


    bag.close()

    writeBoundingBoxes(out_file_name, bounding_boxes_out)
