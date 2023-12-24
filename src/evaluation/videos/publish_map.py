import rospy
import cv2
from nav_msgs.msg import OccupancyGrid
import numpy as np

kImage0 = "/home/tiejean/Desktop/EER0.png"

if __name__ == "__main__":
    pub = rospy.Publisher('/satelliteView', OccupancyGrid, queue_size=10)
    rospy.init_node('map', anonymous=True)
    rate = rospy.Rate(10)

    msg = OccupancyGrid()
    img = cv2.imread(kImage0, cv2.IMREAD_GRAYSCALE)

    msg.header.stamp = rospy.get_rostime()
    msg.header.frame_id = "map"
    msg.info.map_load_time = rospy.get_rostime()
    msg.info.resolution = .055
    msg.info.width = img.shape[1]
    msg.info.height = img.shape[0]
    msg.info.origin.position.x = -21.5
    msg.info.origin.position.y = -21
    data = img
    data = cv2.rotate(data, cv2.ROTATE_180)
    data = cv2.flip(data, 1) 
    data = ((1-data/255)*100).astype(np.uint8)
    data = data.flatten()
    msg.data = (data).tolist()

    # import pdb; pdb.set_trace()

    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()