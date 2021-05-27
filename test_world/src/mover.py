#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
from nav_msgs.msg import OccupancyGrid

import math

from test_world.srv import *


# robot state variables
position_ = Point()
desired_position_ = Point()
yaw_ = 0
# machine state
state_ = 0
# goal

desired_path = []

# tmp_desired_position = Point()
# tmp_desired_position.x = -16
# tmp_desired_position.y = 16
# tmp_desired_position.z = 0
# desired_path.append(tmp_desired_position)
# tmp_desired_position = Point()
# tmp_desired_position.x = 10
# tmp_desired_position.y = 12
# tmp_desired_position.z = 0
# desired_path.append(tmp_desired_position)
# tmp_desired_position = Point()
# tmp_desired_position.x = -10
# tmp_desired_position.y = -3
# tmp_desired_position.z = 0
# desired_path.append(tmp_desired_position)


# map data
map_data = []

# parameters
yaw_precision_ = math.pi / 180 # +/- 1 degree allowed
dist_precision_ = 0.8

# publishers
pub = None



def fix_yaw(des_pos):
	global yaw_, pub, yaw_precision_, state_

	desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)

	err_yaw = desired_yaw - yaw_

	# rospy.loginfo("eee")
	# rospy.loginfo(desired_yaw)
	# rospy.loginfo(yaw_)
	if err_yaw > math.pi:
		err_yaw = err_yaw - 2*math.pi

	if err_yaw < -math.pi:
		err_yaw = err_yaw + 2*math.pi
	
	# rospy.loginfo(err_yaw)

	twist_msg = Twist()
	if math.fabs(err_yaw) > yaw_precision_:
		if err_yaw > 0:
			twist_msg.angular.z = -1
		else:
			twist_msg.angular.z = 1
	
	pub.publish(twist_msg)
	
	# state change conditions
	if math.fabs(err_yaw) <= yaw_precision_:
		# rospy.loginfo('Yaw error: [%s]' % err_yaw)
		change_state(1)


def go_straight_ahead(des_pos):
	global yaw_, pub, yaw_precision_, state_, desired_position_, desired_path
	desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
	err_yaw = desired_yaw - yaw_
	err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) + pow(des_pos.x - position_.x, 2))

	if err_pos > dist_precision_:
		twist_msg = Twist()
		twist_msg.linear.x = 1
		pub.publish(twist_msg)
	else:
		if desired_path:
			desired_position_ = desired_path.pop()
			rospy.loginfo(desired_position_)
			change_state(0)
		else:
			# rospy.loginfo('Position error: [%s]' % err_pos)
			change_state(2)

	# state change conditions
	if math.fabs(err_yaw) > yaw_precision_*10:
		# rospy.loginfo('Yaw error: [%s]' % err_yaw)
		change_state(0)


def done():
	twist_msg = Twist()
	twist_msg.linear.x = 0
	twist_msg.angular.z = 0
	pub.publish(twist_msg)



def change_state(state):
	global state_
	state_ = state
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


# to int here or to index on server?
def get_tr_position():
	tmp_point = Point()
	tmp_point.x = position_.x 		
	tmp_point.y = position_.y
	tmp_point.z = 0

	return tmp_point


def callback_map(msg):
	global map_data
	map_data = msg.data

	#map size TODO



def main():
	global pub, desired_path, desired_position_

	rospy.init_node('mover')

	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

	sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
	sub_map = rospy.Subscriber("/map", OccupancyGrid, callback_map)

	pp_request = PathPlanRequest()

	# int8[] costmap_ros
	# int32 width
	# int32 height
	# geometry_msgs/Point start
	# geometry_msgs/Point goal

	pp_request.start = get_tr_position()

	goal_point = Point()
	goal_point.x = -16
	goal_point.y = 16
	goal_point.z = 0
	pp_request.goal = goal_point

	pp_request.costmap_ros = map_data
	pp_request.width = 42
	pp_request.height = 42

	rospy.wait_for_service('path_plan')
	try:
		path_plan = rospy.ServiceProxy('path_plan', PathPlan)
		resp = path_plan(pp_request)
		desired_path = resp.plan	
		rospy.loginfo(desired_path)
		desired_position_ = desired_path.pop()
	except rospy.ServiceException as e:
		print("Service call failed: %s"%e)

	rate = rospy.Rate(20)
	while not rospy.is_shutdown():
		if state_ == 0:
			tmp_yaw = yaw_
			fix_yaw(desired_position_)
			rate.sleep()
			if abs(yaw_ - tmp_yaw) < 0.001:
				change_state(1)
				# twist_msg = Twist()
				# twist_msg.linear.x = -1
				# twist_msg.angular.z = 1
				# pub.publish(twist_msg)
				# rate.sleep()
				# twist_msg.linear.x = 0
				# pub.publish(twist_msg)
				# rate.sleep()

		elif state_ == 1:
			go_straight_ahead(desired_position_)
		elif state_ == 2:
			done()
			pass
		else:
			rospy.logerr('Unknown state!')
			pass
		rate.sleep()


if __name__ == '__main__':
	main()		

