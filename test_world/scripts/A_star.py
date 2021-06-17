#! /usr/bin/env python3


import rospy


D2 = 1.41421
D = 1

def get_eur(node, goal):
	dx = abs(node[0] - goal[0])
	dy = abs(node[1]- goal[1])
	return D * (dx + dy) + (D2 - 2 * D) * min(dx, dy)


# so costmap[j][i] or [row][column]
# child.g = currentNode.g + distance between child and current
# child.h = distance from child to end
# child.f = child.g + child.h
def generate_n(costmap, node_ind, node_g, goal_ind):

	n_list = []

	# why is costmap[tmp_ind[1]][tmp_ind[0]] inverted lol?
	tmp_ind = [0, 0]
	tmp_ind[0] = node_ind[0]+1
	tmp_ind[1] = node_ind[1]
	if 0 <= tmp_ind[0] < len(costmap):
		if costmap[tmp_ind[1]][tmp_ind[0]] == 0:
			h = get_eur(tmp_ind, goal_ind)
			g = node_g+D
			n_list.append([[tmp_ind[0], tmp_ind[1]], g+h, g, node_ind])

	tmp_ind[0] = node_ind[0]-1
	tmp_ind[1] = node_ind[1]
	if 0 <= tmp_ind[0] < len(costmap):
		if costmap[tmp_ind[1]][tmp_ind[0]] == 0:
			h = get_eur(tmp_ind, goal_ind)
			g = node_g+D
			n_list.append([[tmp_ind[0], tmp_ind[1]], g+h, g, node_ind])


	tmp_ind[0] = node_ind[0]
	tmp_ind[1] = node_ind[1]+1
	if 0 <= tmp_ind[1] < len(costmap[0]):
		if costmap[tmp_ind[1]][tmp_ind[0]] == 0:
			h = get_eur(tmp_ind, goal_ind)
			g = node_g+D
			n_list.append([[tmp_ind[0], tmp_ind[1]], g+h, g, node_ind])

	tmp_ind[0] = node_ind[0]
	tmp_ind[1] = node_ind[1]-1
	if 0 <= tmp_ind[1] < len(costmap[0]):
		if costmap[tmp_ind[1]][tmp_ind[0]] == 0:
			h = get_eur(tmp_ind, goal_ind)
			g = node_g+D
			n_list.append([[tmp_ind[0], tmp_ind[1]], g+h, g, node_ind])


	tmp_ind[0] = node_ind[0]+1
	tmp_ind[1] = node_ind[1]+1
	if 0 <= tmp_ind[0] < len(costmap) and 0 <= tmp_ind[1] < len(costmap[0]):
		if costmap[tmp_ind[1]][tmp_ind[0]] == 0:
			h = get_eur(tmp_ind, goal_ind)
			g = node_g+D2
			n_list.append([[tmp_ind[0], tmp_ind[1]], g+h, g, node_ind])


	tmp_ind[0] = node_ind[0]+1
	tmp_ind[1] = node_ind[1]-1
	if 0 <= tmp_ind[0] < len(costmap) and 0 <= tmp_ind[1] < len(costmap[0]):
		if costmap[tmp_ind[1]][tmp_ind[0]] == 0:
			h = get_eur(tmp_ind, goal_ind)
			g = node_g+D2
			n_list.append([[tmp_ind[0], tmp_ind[1]], g+h, g, node_ind])


	tmp_ind[0] = node_ind[0]-1
	tmp_ind[1] = node_ind[1]-1
	if 0 <= tmp_ind[0] < len(costmap) and 0 <= tmp_ind[1] < len(costmap[0]):
		if costmap[tmp_ind[1]][tmp_ind[0]] == 0:
			h = get_eur(tmp_ind, goal_ind)
			g = node_g+D2
			n_list.append([[tmp_ind[0], tmp_ind[1]], g+h, g, node_ind])


	tmp_ind[0] = node_ind[0]-1
	tmp_ind[1] = node_ind[1]+1
	if 0 <= tmp_ind[0] < len(costmap) and 0 <= tmp_ind[1] < len(costmap[0]):
		if costmap[tmp_ind[1]][tmp_ind[0]] == 0:
			h = get_eur(tmp_ind, goal_ind)
			g = node_g+D2
			n_list.append([[tmp_ind[0], tmp_ind[1]], g+h, g, node_ind])

	return n_list

def A_star(costmap, start_index, goal_index, resolution):
	D = resolution
	D2 = D*1.41421

	path = []


	openList = []
	closedList = []


	openList.append([start_index, 0, 0, []])

	while openList:
	
		ind = 0
		for i in range(1, len(openList)):
			if openList[i][1] < openList[ind][1]:
				ind = i

		currentNode = openList.pop(ind)
		closedList.append(currentNode)

		if currentNode[0]==goal_index:
			break
	
		neighbor_list = generate_n(costmap, currentNode[0], currentNode[2], goal_index)
	
		# print(currentNode[0])

		for n_node in neighbor_list:
			if not any((c_node[0] == n_node[0] for c_node in closedList)):
				for o_node in openList:
					if o_node[0]==n_node[0]:
						if o_node[2]>n_node[2]:
							# print(n_node[0])
							o_node[2] = n_node[2]
							o_node[1] = n_node[1]
							o_node[3] = n_node[3]
				if not any((o_node[0] == n_node[0] for o_node in openList)):
					openList.append(n_node)



	tmp = goal_index
	while tmp!=start_index:
		# print(tmp)
		for node in closedList:
			if node[0]==tmp:
				tmp=node[3]
				if tmp!=start_index:
					path.append([tmp[0], tmp[1]])
				break

	return path

