#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import cv2

# TODO add dependencies
from cv_bridge import CvBridge


def callback(data):
	# TODO rviz warn No image received
	cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
	cv2.imshow("camera1", cv_image)


def listener():
	rospy.init_node('listener', anonymous=True)

	bridge = CvBridge()
	rospy.Subscriber("/my_robot/camera1/image_raw", Image, callback)
	rospy.spin()


if __name__ == '__main__':
	listener()

