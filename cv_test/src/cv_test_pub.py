#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import cv2

from cv_bridge import CvBridge


def talker():
    pub = rospy.Publisher('chatter', Image, queue_size=10)
    rospy.init_node('cv_test_pub', anonymous=True)
    
    bridge = CvBridge()
    
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)

    while cap.isOpened():
        ret, frame = cap.read()
        if ret:

            pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))
            cv2.waitKey(3)

            if cv2.waitKey(2) & 0xFF == ord('q'):
                break
        else:
            break


if __name__ == '__main__':
	talker()

