#! /usr/bin/env python3

import rospy
from test_world.srv import *
from test_world.msg import *
from geometry_msgs.msg import Point
from A_star import *


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
		map_matr.append(list(map_matr_tmp.pop()))

	rospy.loginfo(map_matr)

	start_index1 = []
	start_index1.append(tr_x_to(req.start[0].x, req.width))
	start_index1.append(tr_y_to(req.start[0].y, req.height))

	start_index2 = []
	start_index2.append(tr_x_to(req.start[1].x, req.width))
	start_index2.append(tr_y_to(req.start[1].y, req.height))  
	
	# rospy.loginfo(start_index)


	goal_index1 = []
	goal_index1.append(tr_x_to(req.goal[0].x, req.width)) 
	goal_index1.append(tr_y_to(req.goal[0].y, req.height))

	goal_index2 = []
	goal_index2.append(tr_x_to(req.goal[1].x, req.width)) 
	goal_index2.append(tr_y_to(req.goal[1].y, req.height))  
	
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


	tmp_path1 = A_star(map_matr, start_index1, goal_index1, resolution)

	path1 = PathArray()
	path1.path.append(goal_point)

	for p_point in tmp_path1:
		tmp_desired_position = Point()
		tmp_desired_position.x = tr_x_from(p_point[0], req.width)
		tmp_desired_position.y = tr_y_from(p_point[1], req.height)
		tmp_desired_position.z = 0
		path1.path.append(tmp_desired_position)

	tmp_path2 = A_star(map_matr, start_index2, goal_index2, resolution)

	path2 = PathArray()
	path2.path.append(goal_point2)

	for p_point in tmp_path2:
		tmp_desired_position = Point()
		tmp_desired_position.x = tr_x_from(p_point[0], req.width)
		tmp_desired_position.y = tr_y_from(p_point[1], req.height)
		tmp_desired_position.z = 0
		path2.path.append(tmp_desired_position)


	tmp_i1 = 0
	cr_point = []
	for r1_point in tmp_path1:
		tmp_i2 = 0
		for r2_point in tmp_path2:
			# if r1_point==r2_point and tmp_i1==tmp_i2:
			if r1_point==r2_point and abs(tmp_i1-tmp_i2)<3:
				rospy.loginfo("AAAAAAAAAAAAAAAAAAAAAAAAAA")
				rospy.loginfo(r1_point)
				rospy.loginfo(tr_x_from(r1_point[0], req.width))
				rospy.loginfo(tr_y_from(r1_point[1], req.height))
				# rospy.loginfo(map_matr[r1_point[0]][r1_point[1]])
				# cr_point = [r1_point[0], r1_point[1]]

				cr_point.append([r1_point[0], r1_point[1]])
				rospy.loginfo(tmp_i1)
				rospy.loginfo("AAAAAAAAAAAAAAAAAAAAAAAAAA")

			tmp_i2 = tmp_i2+1
		tmp_i1 = tmp_i1+1


	# change map if cr_point
	if cr_point:
		# inverted????
		# map_matr[cr_point[1]][cr_point[0]] = 95

		for i_cr_point in cr_point:
			map_matr[i_cr_point[1]][i_cr_point[0]] = 95

		tmp_path3 = A_star(map_matr, start_index2, goal_index2, resolution)
		rospy.loginfo("tmp_path3")
		rospy.loginfo(tmp_path3)

		path2 = PathArray()
		path2.path.append(goal_point2)

		for p_point in tmp_path3:
			tmp_desired_position = Point()
			tmp_desired_position.x = tr_x_from(p_point[0], req.width)
			tmp_desired_position.y = tr_y_from(p_point[1], req.height)
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
