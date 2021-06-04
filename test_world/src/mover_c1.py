#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
from nav_msgs.msg import OccupancyGrid

import math

from test_world.srv import *
from test_world.msg import *


# robot state variables
position_ = Point()
position_.z = 4
position2_ = Point()
position2_.z = 4

desired_position_ = Point()
desired_position2_ = Point()
yaw_ = 0
yaw2_ = 0
# machine state
# state for each or global state?
# state_ = 0
# state2_ = 0
# goal

desired_path = []
desired_path2 = []

# map data
map_data = []

# parameters
yaw_precision_ = math.pi / 180 # +/- 1 degree allowed
dist_precision_ = 0.8

# publishers
pub = None
pub2 = None



# def change_state(state):
#     global state_
#     state_ = state
    # rospy.loginfo('State changed to [%s]' % state_)


def clbk_odom(msg):
    global position_
    global yaw_

    # position
    position_ = msg.pose.pose.position

    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]


def clbk_odom2(msg):
    global position2_
    global yaw2_

    # position
    position2_ = msg.pose.pose.position

    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw2_ = euler[2]


def get_tr_position():
    tmp_point = Point()
    tmp_point.x = position2_.x       
    tmp_point.y = position2_.y
    tmp_point.z = 0

    return tmp_point

def get_tr_position2():
    tmp_point = Point()
    tmp_point.x = position_.x       
    tmp_point.y = position_.y
    tmp_point.z = 0

    return tmp_point

def callback_map(msg):
    global map_data
    map_data = msg.data


# def clbk_laser(msg):
#     # 720 / 5 = 144
#     if min(min(msg.ranges[288:431]), 10) < 0.5:
#         rospy.loginfo("---------------------")
#         rospy.loginfo(min(min(msg.ranges[288:431]), 10))
#         rospy.loginfo(position_)
#         rospy.loginfo(yaw_)

    # if min(min(msg.ranges[288:431]), 10) < 0.5 and tmp_yaw==yaw_ and tmp_pos==position_:


def main():
    global pub, pub2, desired_path, desired_path2, desired_position_, desired_position2_
    # global yaw_, yaw2_, yaw_precision_, dist_precision_

    rospy.init_node('mover_c1')

    sub_map = rospy.Subscriber("/map", OccupancyGrid, callback_map)

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    # sub = rospy.Subscriber('/robot/laser/scan', LaserScan, clbk_laser)

    pub2 = rospy.Publisher('/cmd_vel2', Twist, queue_size=1)
    sub_odom = rospy.Subscriber('/odom2', Odometry, clbk_odom2)
    # sub = rospy.Subscriber('/robot2/laser/scan', LaserScan, clbk_laser2)

    rate = rospy.Rate(20)


    while not map_data or position_.z==4 or position2_.z==4:
        # rospy.loginfo('kk')
        rate.sleep()


    pp_request = PathPlan2Request()

    # int8[] costmap_ros
    # int32 width
    # int32 height
    # geometry_msgs/Point[] start
    # geometry_msgs/Point[] goal 

    start = get_tr_position()
    start2 = get_tr_position2()

    pp_request.start = [start, start2] # tmp

    goal_point = Point()
    goal_point.x = -8
    goal_point.y = 13
    goal_point.z = 0
    goal_point2 = Point()
    goal_point2.x = 4.5
    goal_point2.y = 9.5
    goal_point2.z = 0
    pp_request.goal = [goal_point, goal_point2] # tmp

    pp_request.costmap_ros = map_data
    pp_request.width = 33
    pp_request.height = 33

    rospy.wait_for_service('path_plan2')
    try:
        path_plan = rospy.ServiceProxy('path_plan2', PathPlan2)
        resp = path_plan(pp_request)

        desired_path = resp.plan_array[0].path    
        rospy.loginfo(desired_path)
        desired_position_ = desired_path.pop()
        # rospy.loginfo(desired_position_)

        desired_path2 = resp.plan_array[1].path    
        rospy.loginfo(desired_path2)
        desired_position2_ = desired_path2.pop()
        # rospy.loginfo(desired_position2_)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


    # while not rospy.is_shutdown() or state_!=2:

    #     desired_yaw = math.atan2(desired_position_.y - position_.y, desired_position_.x - position_.x)

    #     err_yaw = desired_yaw - yaw_
    #     twist_msg = Twist()
    #     # twist_msg.angular.z = 1
    #     # rospy.loginfo(math.fabs(err_yaw))   
    #     if math.fabs(err_yaw) > yaw_precision_*2:

    #         if err_yaw > math.pi:
    #             err_yaw = err_yaw - 2*math.pi

    #         if err_yaw < -math.pi:
    #             err_yaw = err_yaw + 2*math.pi

    #         twist_msg.linear.x = 0
    #         # if math.fabs(err_yaw) > yaw_precision_:
    #         if err_yaw > 0:
    #             twist_msg.angular.z = -1
    #         else:
    #             twist_msg.angular.z = 1

    #     else:
    #         # rospy.loginfo("-------------------------")
    #         err_pos = math.sqrt(pow(desired_position_.y - position_.y, 2) + pow(desired_position_.x - position_.x, 2))

    #         if err_pos > dist_precision_:
    #             twist_msg.linear.x = 1
    #             twist_msg.angular.z = 0
    #         else:
    #             if desired_path:
    #                 desired_position_ = desired_path.pop()
    #                 rospy.loginfo(desired_position_)
    #             else:
    #                 # rospy.loginfo('Position error: [%s]' % err_pos)
    #                 # change_state(2)
    #                 break

    #     # rospy.loginfo(twist_msg)
    #     pub.publish(twist_msg)
    #     rate.sleep()


if __name__ == '__main__':
    main()      

