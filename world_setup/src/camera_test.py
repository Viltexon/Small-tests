#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import Image as sensorImage
import cv2

# TODO add dependencies
# TODO libgcc_s.so.1 must be installed for pthread_cancel to work
# Fixed by Ubuntu reinstall :)
from cv_bridge import CvBridge

bridge = CvBridge()

def callback(msg):
	# cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
	# str_msg = msg.data
	# buf = np.ndarray(shape=(1, len(str_msg)), dtype=np.uint8, buffer=msg.data)

	# image = Image.fromarray(buf.astype('uint8')).convert('RGB')
	# cv_image = np.array(image) 
	# cv_image = cv_image[:, :, ::-1].copy()

	# cv_image = cv2.imdecode(buf, cv2.IMREAD_ANYCOLOR)
	# cv2.imshow("camera1", cv_image)
	# TODO Current thread is not the object's thread. Cannot move to target thread
	rospy.loginfo("msg")
	cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
	cv2.imshow("camera1", cv_image)
	cv2.waitKey(3)


def listener():
	rospy.init_node('listener', anonymous=True)

	rospy.Subscriber("/my_robot/camera1/image_raw", sensorImage, callback)
	rospy.spin()


if __name__ == '__main__':
	listener()

