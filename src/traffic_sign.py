#!/usr/bin/env python

import numpy as np
from scipy.ndimage import filters

import cv2
import random

import roslib
roslib.load_manifest('traffic_sign')
import rospy

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

class image_feature:
    def __init__(self):
        self.raspi_subscriber = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, self.callback)
        rospy.loginfo("Subscribed to /raspicam_node/image/compressed")

        self.image_publisher = rospy.Publisher("/traffic_sign/detected", String, queue_size=10)
        rospy.loginfo("Publishing /traffic_sign/detected")

        self.detection_publisher = rospy.Publisher("/traffic_sign/image/compressed", CompressedImage, queue_size=10)
        rospy.loginfo("Publishing /traffic_sign/image/compressed")

    def callback(self, ros_data):
        rospy.loginfo("Image received")

        #self.image_publisher(self.raspi_subscriber)
        random_number = "%d" % random.randint(1,101)
        self.detection_publisher.publish(random_number)

if __name__ == '__main__':
    image = image_feature()
    rospy.init_node('traffic_sign', log_level=rospy.DEBUG)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down traffic sign node")
    cv2.destroyAllWindows()
