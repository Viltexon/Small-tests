#! /usr/bin/env python3

import rospy
from test_world.srv import *
from test_world.msg import *
from geometry_msgs.msg import Point
# from A_star import *


# TODO width&height
# TODO diagonal obstacles
# 0.5?
def tr_x_to(x, dist):
	return int(round(x + dist/2 - 0.5))

def tr_y_to(y, dist):
	return int(round(-y + dist/2 - 0.5))

def tr_x_from(x, dist):
	return x - dist/2 + 0.5

def tr_y_from(y, dist):
	return -y + dist/2 - 0.5


def handle_path_plan(req):

	rospy.loginfo("handle_path_plan +++++++++++++++++")

	# int8[] costmap_ros
	# int32 width
	# int32 height
	# geometry_msgs/Point[] start
	# geometry_msgs/Point[] goal 

	map_matr_tmp = [req.costmap_ros[i:i+req.width] for i in range(0, len(req.costmap_ros), req.width)]
	map_matr = []
	for i in range(len(map_matr_tmp)):
		map_matr.append(map_matr_tmp.pop())

	rospy.loginfo(map_matr)

	# start_index = []
	# start_index.append(tr_x_to(req.start.x, req.width))
	# start_index.append(tr_y_to(req.start.y, req.height)) 
	
	# rospy.loginfo(start_index)


	# goal_index = []
	# goal_index.append(tr_x_to(req.goal.x, req.width)) 
	# goal_index.append(tr_y_to(req.goal.y, req.height)) 
	
	# rospy.loginfo(goal_index)
	rospy.loginfo("handle_path_plan +++++++++++++++++")


	# side of each grid map square in meters
	resolution = 1

	# time statistics
	start_time = rospy.Time.now()

	# how TODO LRA* and WHCA* ?
	# tmp_path = A_star(map_matr, start_index, goal_index, resolution)

	goal_point = req.goal[0]
	goal_point2 = req.goal[1]
	# path = []
	# path.append(goal_point)

	# for p_point in tmp_path:
	# 	tmp_desired_position = Point()
	# 	tmp_desired_position.x = tr_x_from(p_point[0], req.width)
	# 	tmp_desired_position.y = tr_y_from(p_point[1], req.height)
	# 	tmp_desired_position.z = 0
	# 	path.append(tmp_desired_position)


	path1 = PathArray()
	path1.path.append(goal_point)
	tmp_desired_position = Point()
	tmp_desired_position.x = 7
	tmp_desired_position.y = 10
	tmp_desired_position.z = 0
	path1.path.append(tmp_desired_position)
	tmp_desired_position = Point()
	tmp_desired_position.x = -7
	tmp_desired_position.y = -3
	tmp_desired_position.z = 0
	path1.path.append(tmp_desired_position)


 
	path2 = PathArray()
	path2.path.append(goal_point2)
	tmp_desired_position = Point()
	tmp_desired_position.x = 4
	tmp_desired_position.y = 4
	tmp_desired_position.z = 0
	path2.path.append(tmp_desired_position)

	Plan_array = [path1, path2]



	if not path1:
		rospy.logwarn("No path returned by algorithm")
		path1 = []
	else:
		execution_time = rospy.Time.now() - start_time
		print("\n")
		rospy.loginfo('+++++++++++++++++ Metrics +++++++++++++++++')
		rospy.loginfo('Total execution time: %s seconds', str(execution_time.to_sec()))
		rospy.loginfo('++++++++++++++++++++++++++++++++++++++++++++')
		print("\n")

	resp = PathPlan2Response()
	resp.plan_array = Plan_array
	return resp

def path_plan_server2():
	rospy.init_node('path_plan_server2')
	s = rospy.Service('path_plan2', PathPlan2, handle_path_plan)
	rospy.spin()

if __name__ == "__main__":
	path_plan_server2()
