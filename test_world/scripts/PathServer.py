#! /usr/bin/env python3

import rospy
from test_world.srv import *
from geometry_msgs.msg import Point

def handle_path_plan(req):

	# costmap as 1-D array representation
	costmap = req.costmap_ros
	# number of columns in the occupancy grid
	width = req.width
	# number of rows in the occupancy grid
	height = req.height
	# start and goal points
	start_point = req.start
	goal_point = req.goal

	# side of each grid map square in meters
	resolution = 1
	# origin of grid map
	origin = [-7.4, -7.4, 0]

	# time statistics
	start_time = rospy.Time.now()

	# calculate the shortes path using Dijkstra, A*
	# how TODO LRA* and WHCA*
	# goal to index?
	# path = dijkstra(start_index, goal_index, width, height, costmap, resolution, origin, viz)


	# tmp response
	path = []

	path.append(goal_point)
	tmp_desired_position = Point()
	tmp_desired_position.x = 10
	tmp_desired_position.y = 12
	tmp_desired_position.z = 0
	path.append(tmp_desired_position)
	tmp_desired_position = Point()
	tmp_desired_position.x = -10
	tmp_desired_position.y = -3
	tmp_desired_position.z = 0
	path.append(tmp_desired_position)



	if not path:
		rospy.logwarn("No path returned by algorithm")
		path = []
	else:
		execution_time = rospy.Time.now() - start_time
		print("\n")
		rospy.loginfo('+++++++++++++++++ Metrics +++++++++++++++++')
		rospy.loginfo('Total execution time: %s seconds', str(execution_time.to_sec()))
		rospy.loginfo('++++++++++++++++++++++++++++++++++++++++++++')
		print("\n")

	resp = PathPlanResponse()
	resp.plan = path
	return resp

def path_plan_server():
	rospy.init_node('path_plan_server')
	s = rospy.Service('path_plan', PathPlan, handle_path_plan)
	rospy.spin()

if __name__ == "__main__":
	path_plan_server()