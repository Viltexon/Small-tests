#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import cv2

from cv_bridge import CvBridge


bridge = CvBridge()

def callback(data):
	rospy.loginfo("msg")
	cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
	cv2.imshow("camera1", cv_image)
	cv2.waitKey(3)


def listener():
	rospy.init_node('cv_test_get', anonymous=True)

	rospy.Subscriber("chatter", Image, callback)
	rospy.spin()


if __name__ == '__main__':
	listener()

