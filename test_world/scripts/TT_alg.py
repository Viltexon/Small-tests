#! /usr/bin/env python3


import rospy


D2 = 1.41421
D = 1

def get_eur(node, goal):
	dx = abs(node[0] - goal[0])
	dy = abs(node[1]- goal[1])
	return D * (dx + dy) + (D2 - 2 * D) * min(dx, dy)


# use eur for time table

# use A_star value as eur 

# use TT only for windowed time

