import argparse
import rosbag
import csv
from darknet_ros_msgs.msg import BoundingBoxes


class BoundingBoxWithTimestamp:

    def __init__(self, min_pixel_x, min_pixel_y, max_pixel_x, max_pixel_y, semantic_class, seconds, nano_seconds, camera_id=None):
        self.min_pixel_x = min_pixel_x
        self.min_pixel_y = min_pixel_y
        self.max_pixel_x = max_pixel_x
        self.max_pixel_y = max_pixel_y
        self.semantic_class = semantic_class
        self.seconds = seconds
        self.nano_seconds = nano_seconds
        self.camera_id = camera_id

    def getEntriesAsStringList(self):
        entries = [str(self.min_pixel_x), str(self.min_pixel_y), str(self.max_pixel_x), str(self.max_pixel_y), str(self.semantic_class), str(self.seconds), str(self.nano_seconds)]
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
                             "camera_id"])
        for detail_entry in bounding_boxes:
            csv_writer.writerow(detail_entry.getEntriesAsStringList())

if __name__ == "__main__":

    cmdLineArgs = parseArgs()
    bag_file_name = cmdLineArgs.bag_file
    out_file_name = cmdLineArgs.out_file_name
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
                bb_out = BoundingBoxWithTimestamp(bb.xmin, bb.ymin, bb.xmax, bb.ymax, bb.Class, header.stamp.secs, header.stamp.nsecs)
                bounding_boxes_out.append(bb_out)
    bag.close()

    print(len(bounding_boxes_out))
    writeBoundingBoxes(out_file_name, bounding_boxes_out)
