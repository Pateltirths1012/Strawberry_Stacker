#!/usr/bin/env python3

"""
This is a boiler plate script that contains an example on how to subscribe a rostopic containing camera frames 
and store it into an OpenCV image to use it further for image processing tasks.
Use this code snippet in your code or you can also continue adding your code in the same file


This python file runs a ROS-node of name marker_detection which detects a moving ArUco marker.
This node publishes and subsribes the following topics:

	Subsriptions					Publications
	/camera/camera/image_raw			/marker_info
"""
from sensor_msgs.msg import Image
from task_1.msg import Marker
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv
import numpy as np
import rospy
import cv2.aruco as aruco


class image_proc:

    # Initialise everything
    def __init__(self):
        rospy.init_node("marker_detection", anonymous=True)  # Initialise rosnode

        # Making a publisher

        self.marker_pub = rospy.Publisher("/marker_info", Marker, queue_size=1)

        # ------------------------Add other ROS Publishers here-----------------------------------------------------

        # Subscribing to /camera/camera/image_raw

        self.image_sub = rospy.Subscriber(
            "/camera/camera/image_raw", Image, self.image_callback
        )  # Subscribing to the camera topic
        # -------------------------Add other ROS Subscribers here----------------------------------------------------

        self.img = np.empty([])  # This will contain your image frame from camera
        self.bridge = CvBridge()
        self.marker_msg = (
            Marker()
        )  # This will contain the message structure of message type task_1/Marker

    # Callback function of camera topic
    def image_callback(self, data):
        # Note: Do not make this function lenghty, do all the processing outside this callback functio
        self.img = self.bridge.imgmsg_to_cv2(
            data, "bgr8"
        )  # Converting the image to OpenCV standard image

    def marker(self):
        gray = cv.cvtColor(self.img, cv.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
        parameters = aruco.DetectorParameters_create()
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        _, bw = cv.threshold(gray, 50, 255, cv.THRESH_BINARY | cv.THRESH_OTSU)
        contours, _ = cv.findContours(bw, cv.RETR_LIST, cv.CHAIN_APPROX_NONE)

        for i, c in enumerate(contours):
            rect = cv.minAreaRect(c)
            center = (float(rect[0][0]), float(rect[0][1]))
            angle = float(rect[2])

        # for i in ids:
        # 	i = int(i)
        self.marker_msg.id = ids
        self.marker_msg.x = center[0]
        self.marker_msg.y = center[1]
        self.marker_msg.yaw = angle
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            self.marker_pub.publish(self.marker_msg)
            rate.sleep()


if __name__ == "__main__":
    image_proc_obj = image_proc()
    rospy.spin()
