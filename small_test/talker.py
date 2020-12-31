import rospy
from std_msgs.msg import String


def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(2)   # hz
    while not rospy.is_shutdown():
        hello_str = "time: %s" % rospy.get_time()

        pub.publish(hello_str)
        rate.sleep()


if __name__ == '__main__':
    talker()

