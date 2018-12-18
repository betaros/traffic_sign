#!/usr/bin/env python

import numpy as np
from scipy.ndimage import filters

import cv2
import random

from matplotlib import pyplot as plt

import roslib
roslib.load_manifest('traffic_sign')
import rospy

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

class image_feature:
    def __init__(self):
        self.counter = 1;
        self.raspi_subscriber = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, self.callback)
        rospy.loginfo("Subscribed to /raspicam_node/image/compressed")

        self.detection_publisher = rospy.Publisher("/traffic_sign/detected", String, queue_size=10)
        rospy.loginfo("Publishing /traffic_sign/detected")

        self.image_publisher = rospy.Publisher("/traffic_sign/image/compressed", CompressedImage, queue_size=10)
        rospy.loginfo("Publishing /traffic_sign/image/compressed")

    def callback(self, ros_data):
        #rospy.loginfo(type(ros_data))

	    """
        Shows live images with marked detections
        """

        if self.counter%10 != 0:
            self.counter = self.counter + 1
        else:
            self.counter = 1

            np_arr = np.fromstring(ros_data.data, np.uint8)
            # img = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
            img = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)  # OpenCV >= 3.0:
            center = (205, 154)

            #if not img is None:
            #    rospy.logwarn("No image received")
            #    return

            # img = cv2.resize(img, (960, 540))
            M = cv2.getRotationMatrix2D(center, 180, 1.0)
            img = cv2.warpAffine(img, M, (410, 308))
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            biggest_value = 0
            found_msg = "nothing"

            #face_cascade = cv2.CascadeClassifier('/home/user/catkin_ws/src/traffic_sign/cascades/haarcascade_frontalface_default.xml')
            #faces = face_cascade.detectMultiScale(gray, 1.3, 5)
            #for (x, y, w, h) in faces:
            #    cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            #    if biggest_value < x*y:
            #        found_msg = "face"
            #        biggest_value = x * y
            #    y = y - 5
            #    self.write_text_on_image(img, "Faces", x, y)

            formCircleCascade = cv2.CascadeClassifier('')
            formCircle = formCircleCascade.detectMultiScale(gray, 1.3, 5)
            for (x, y, w, h) in formCircle:
                # No parking
                no_parking_cascade = cv2.CascadeClassifier(
                    '/home/user/catkin_ws/src/traffic_sign/cascades/cascade_no_parking.xml')
                no_parking = no_parking_cascade.detectMultiScale(gray, 1.3, 5)
                for (x, y, w, h) in no_parking:
                    cv2.rectangle(img, (x, y), (x + w, y + h), (255, 255, 128), 2)
                    if biggest_value < x * y:
                        found_msg = "no_parking"
                        biggest_value = x * y
                    y = y - 5
                    self.write_text_on_image(img, "no parking", x, y)

                # Entry forbidden
                entry_forbidden_cascade = cv2.CascadeClassifier(
                    '/home/user/catkin_ws/src/traffic_sign/cascades/cascade_entry_forbidden.xml')
                entry_forbidden = entry_forbidden_cascade.detectMultiScale(gray, 1.3, 5)
                for (x, y, w, h) in entry_forbidden:
                    cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 2)
                    if biggest_value < x * y:
                        found_msg = "entry_forbidden"
                        biggest_value = x * y
                    y = y - 5
                    self.write_text_on_image(img, "entry forbidden", x, y)

                # Bus stop
                bus_stop_cascade = cv2.CascadeClassifier(
                    '/home/user/catkin_ws/src/traffic_sign/cascades/cascade_bus_stop.xml')
                bus_stop = bus_stop_cascade.detectMultiScale(gray, 1.3, 5)
                for (x, y, w, h) in bus_stop:
                    cv2.rectangle(img, (x, y), (x + w, y + h), (255, 255, 0), 2)
                    if biggest_value < x * y:
                        found_msg = "bus_stop"
                        biggest_value = x * y
                    y = y - 5
                    self.write_text_on_image(img, "bus stop", x, y)

            formTriangleCascade = cv2.CascadeClassifier('')
            formTriangle = formTriangleCascade.detectMultiScale(gray, 1.3, 5)
            for (x, y, w, h) in formTriangle:
                # pedestrians
                pedestrians_cascade = cv2.CascadeClassifier(
                    '/home/user/catkin_ws/src/traffic_sign/cascades/cascade_pedestrians.xml')
                pedestrians = pedestrians_cascade.detectMultiScale(gray, 1.3, 5)
                for (x, y, w, h) in pedestrians:
                    cv2.rectangle(img, (x, y), (x + w, y + h), (255, 128, 0), 2)
                    if biggest_value < x * y:
                        found_msg = "pedestrians"
                        biggest_value = x * y
                    y = y - 5
                    self.write_text_on_image(img, "pedestrians", x, y)

                # turn right
                turn_right_cascade = cv2.CascadeClassifier(
                    '/home/user/catkin_ws/src/traffic_sign/cascades/cascade_turn_right.xml')
                turn_right = turn_right_cascade.detectMultiScale(gray, 1.3, 5)
                for (x, y, w, h) in turn_right:
                    cv2.rectangle(img, (x, y), (x + w, y + h), (128, 255, 0), 2)
                    if biggest_value < x * y:
                        found_msg = "turn_right"
                        biggest_value = x * y
                    y = y - 5
                    self.write_text_on_image(img, "turn right", x, y)

                # turn left
                turn_left_cascade = cv2.CascadeClassifier(
                    '/home/user/catkin_ws/src/traffic_sign/cascades/cascade_turn_left.xml')
                turn_left = turn_left_cascade.detectMultiScale(gray, 1.3, 5)
                for (x, y, w, h) in turn_left:
                    cv2.rectangle(img, (x, y), (x + w, y + h), (0, 128, 0), 2)
                    if biggest_value < x * y:
                        found_msg = "turn_left"
                        biggest_value = x * y
                    y = y - 5
                    self.write_text_on_image(img, "turn left", x, y)

                # warning
                warning_cascade = cv2.CascadeClassifier(
                    '/home/user/catkin_ws/src/traffic_sign/cascades/cascade_warning.xml')
                warning = warning_cascade.detectMultiScale(gray, 1.3, 5)
                for (x, y, w, h) in warning:
                    cv2.rectangle(img, (x, y), (x + w, y + h), (0, 128, 128), 2)
                    if biggest_value < x * y:
                        found_msg = "warning"
                        biggest_value = x * y
                    y = y - 5
                    self.write_text_on_image(img, "warning", x, y)

                # crossing
                crossing_cascade = cv2.CascadeClassifier(
                    '/home/user/catkin_ws/src/traffic_sign/cascades/cascade_crossing.xml')
                crossing = crossing_cascade.detectMultiScale(gray, 1.3, 5)
                for (x, y, w, h) in class_09:
                    cv2.rectangle(img, (x, y), (x + w, y + h), (255, 255, 0), 2)
                    if biggest_value < x * y:
                        found_msg = "entry_crossing"
                        biggest_value = x * y
                    y = y - 5
                    self.write_text_on_image(img, "crossing", x, y)

                # slippery
                slippery_cascade = cv2.CascadeClassifier('/home/user/catkin_ws/src/traffic_sign/cascades/cascade_slippery.xml')
                slippery = slippery_cascade.detectMultiScale(gray, 1.3, 5)
                for (x, y, w, h) in slippery:
                    cv2.rectangle(img, (x, y), (x + w, y + h), (255, 255, 0), 2)
                    if biggest_value < x * y:
                        found_msg = "entry_slippery"
                        biggest_value = x * y
                    y = y - 5
                    self.write_text_on_image(img, "slippery", x, y)

            formQuadCascade = cv2.CascadeClassifier('')
            formQuad = formQuadCascade.detectMultiScale(gray, 1.3, 5)
            for (x, y, w, h) in formQuad:
                # main road
                main_road_cascade = cv2.CascadeClassifier('/home/user/catkin_ws/src/traffic_sign/cascades/cascade_main_road.xml')
                main_road = main_road_cascade.detectMultiScale(gray, 1.3, 5)
                for (x, y, w, h) in main_road:
                    cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 255), 2)
                    if biggest_value < x * y:
                        found_msg = "main_road"
                        biggest_value = x * y
                    y = y - 5
                    self.write_text_on_image(img, "main road", x, y)

            road_closed_cascade = cv2.CascadeClassifier('/home/user/catkin_ws/src/traffic_sign/cascades/cascade_road_closed.xml')
            road_closed = road_closed_cascade.detectMultiScale(gray, 1.3, 5)
            for (x, y, w, h) in road_closed:
                cv2.rectangle(img, (x, y), (x + w, y + h), (255, 255, 0), 2)
                if biggest_value < x * y:
                    found_msg = "entry_road_closed"
                    biggest_value = x * y
                y = y - 5
                self.write_text_on_image(img, "road closed", x, y)

            rospy.loginfo(found_msg)
            
            #### Create CompressedIamge ####
            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format = "jpeg"
            msg.data = np.array(cv2.imencode('.jpg', img)[1]).tostring()

            # Publish new image
            self.image_publisher.publish(msg)
            self.detection_publisher.publish(found_msg)
            #random_number = str(random.randint(1,101))
            #self.detection_publisher.publish(random_number)

    def write_text_on_image(self, img, message, x, y):
        """
        Writes text above the recognized field

        :param img:
        :param message:
        :return:
        """

        bottom_left_corner_of_text=(x,y)
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 1
        font_color = (0, 255, 0)
        line_type = 2

        cv2.putText(img, message,
                    bottom_left_corner_of_text,
                    font,
                    font_scale,
                    font_color,
                    line_type)

if __name__ == '__main__':
    image = image_feature()
    rospy.init_node('traffic_sign', log_level=rospy.DEBUG)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down traffic sign node")
    cv2.destroyAllWindows()
