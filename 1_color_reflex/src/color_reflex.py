#! /usr/bin/env python

import rospy

from std_msgs.msg import Bool
from std_msgs.msg import UInt8MultiArray


zone_color = None


def zone_color_callback(color_msg):

    global zone_color
    zone_color = color_msg.data


def start():

    rospy.init_node('color_reflex')

    rospy.Subscriber('zone_color', UInt8MultiArray, zone_color_callback)

    body_color_pub = rospy.Publisher('body_color', Bool, queue_size=1)

    while zone_color is None and not rospy.is_shutdown():
        rospy.sleep(.1)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        body_color = False

        if zone_color == b'\xff\x00\x00':   # react to red: [255, 0, 0]
            body_color = True
            rospy.loginfo("DANGER")

        body_color_pub.publish(body_color)
        rate.sleep()


if __name__ == "__main__":
    start()
