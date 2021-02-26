#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import cv2

# TODO add dependencies
# TODO libgcc_s.so.1 must be installed for pthread_cancel to work
# from cv_bridge import CvBridge


def callback(data):
	# cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
	# cv2.imshow("camera1", cv_image)
	rospy.loginfo("msg")


def listener():
	rospy.init_node('listener', anonymous=True)

	# bridge = CvBridge()
	rospy.Subscriber("/my_robot/camera1/image_raw", Image, callback)
	rospy.spin()


if __name__ == '__main__':
	listener()

