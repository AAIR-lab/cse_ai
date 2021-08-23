#!/usr/bin/env python
# encoding: utf-8

__copyright__ = "Copyright 2019, AAIR Lab, ASU"
__authors__ = ["Naman Shah", "Ketan Patil"]
__credits__ = ["Siddharth Srivastava"]
__license__ = "MIT"
__version__ = "1.0"
__maintainers__ = ["Pulkit Verma", "Abhyudaya Srinet"]
__contact__ = "aair.lab@asu.edu"
__docformat__ = 'reStructuredText'

from hw2.srv import *
import rospy
from mazeGenerator import *
import sys
import argparse
import time
from action_server import RobotActionsServer 
import problem_generator
import pickle
import os
import subprocess

root_path = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir))
books = None
mazeInfo = None
parser = argparse.ArgumentParser()
parser.add_argument('-sub', help='for providing no. of subjects', metavar='5', action='store', dest='n_subjects', default=5, type=int)
parser.add_argument('-b', help='for providing no. of books for each subject, min=1, max=5', metavar='5', action='store', dest='n_books', default=5, type=int)
parser.add_argument('-s', help='for providing random seed, min=1, max=10', metavar='32', action='store', dest='seed', default=int(time.time()), type=int)

def check_is_edge(edge, valueFlag):
    """
    This function checks if two points are connected via edge or not.
    """
    global mazeInfo
    invalid_edges = mazeInfo[1]

    if valueFlag == "changedValuesLater":
        if edge[2] < mazeInfo[0][0] or edge[2] > mazeInfo[0][1]*1 or edge[3] < mazeInfo[0][0] or edge[3] > mazeInfo[0][1]*1:
            return False
    elif valueFlag == "changedValuesBefore":
        if edge[0] < mazeInfo[0][0] or edge[0] > mazeInfo[0][1]*1 or edge[1] < mazeInfo[0][0] or edge[1] > mazeInfo[0][1]*1:
            return False

    if edge in invalid_edges:
        return False
    else:
        return True

def handle_get_successor(req):
	"""
		This function returns all successors of a given state 
				
		parameters:	x_cord - current x-cordinate of turtlebot
				    y_cord - current y-cordinate of turtlebot
				    direction - current orientation

		output:   
			GetSuccessorResponse (search/srv/GetSuccessor.srv)
	"""
	global mazeInfo
	action_list = ["TurnCW", "TurnCCW", "MoveF"]
	direction_list = ["NORTH", "EAST", "SOUTH", "WEST"]
	state_x = []
	state_y = []
	state_direction = []
	state_cost = []
	
	for action in action_list:
		#Checking requested action and making changes in states
		x_cord, y_cord, direction = req.x, req.y, req.direction
		if action == 'TurnCW':
			index = direction_list.index(req.direction)
			direction = direction_list[(index+1)%4]
			g_cost = 1

		elif action == 'TurnCCW':
			index = direction_list.index(req.direction)
			direction = direction_list[(index-1)%4]
			g_cost = 1

		elif action == 'MoveF':
			if direction == "NORTH":
				y_cord += 1.0
			elif direction == "EAST":
				x_cord += 1.0
			elif direction == "SOUTH":
				y_cord -= 1.0
			elif direction == "WEST":
				x_cord -= 1.0
			g_cost = 1
		
		if req.x <= x_cord and req.y <= y_cord:
			isValidEdge = check_is_edge((req.x, req.y, x_cord, y_cord), "changedValuesLater")
		else:
			isValidEdge = check_is_edge((x_cord, y_cord, req.x, req.y), "changedValuesBefore")

		if not isValidEdge:
			state_x.append(-1)
			state_y.append(-1)
			state_direction.append(direction)
			state_cost.append(-1)
		else:
			state_x.append(x_cord)
			state_y.append(y_cord)
			state_direction.append(direction)
			state_cost.append(g_cost)

	return GetSuccessorResponse(state_x, state_y, state_direction, state_cost, action_list)
  

def handle_get_initial_state(req):
    """
    This function will return initial state of turtlebot3.
    """
    global mazeInfo

    initial_state = mazeInfo[0]
    return GetInitialStateResponse(initial_state[0],initial_state[0],initial_state[2])


def handle_is_goal_state(req):
    """
    This function will return True if turtlebot3 is at goal state otherwise it will return False.
    """
    global mazeInfo

    goal_state = mazeInfo[0][1]*1

    if req.x == req.y and req.x == goal_state:
        return IsGoalStateResponse(1)

    return IsGoalStateResponse(0)

def handle_get_goal_state(req):
    global mazeInfo
    goal_state = mazeInfo[0][1]*1
    return GetGoalStateResponse(goal_state,goal_state)


def remove_blocked_edge(req):

    bookname = req.bookname
    
    global books
    global mazeInfo
    location_of_blocked_edge_list = books["books"][bookname]["load_loc"]
    if location_of_blocked_edge_list[0][0] <= location_of_blocked_edge_list[1][0] and location_of_blocked_edge_list[0][1] <= location_of_blocked_edge_list[1][1]:
        blocked_edge = (location_of_blocked_edge_list[0][0], location_of_blocked_edge_list[0][1], location_of_blocked_edge_list[1][0], location_of_blocked_edge_list[1][1])
    else:
        blocked_edge = (location_of_blocked_edge_list[1][0], location_of_blocked_edge_list[1][1], location_of_blocked_edge_list[0][0], location_of_blocked_edge_list[0][1])
    mazeInfo[1].remove(blocked_edge)
    return "1"

def ros_generate_maze(req):
    
    n_subjects=req.subjects
    n_books=req.books
    seed=req.seed
    book_sizes = 2
    book_count_of_each_subject = n_books * book_sizes
    book_count_list = [n_books] * n_subjects * book_sizes
    number_of_trollies = n_subjects * 2
    grid_size = 6 * n_subjects
    
    global books
    global mazeInfo
    books, mazeInfo = generate_blocked_edges(grid_size, book_count_list, seed,  number_of_trollies, root_path, 1.0)
    RobotActionsServer(books)
    path = root_path + "/problem.pddl"
    problem_generator.write_pddl(path ,books)
    
    return GenerateMazeResponse(0)

def server():

    rospy.init_node('server')
    rospy.Service('get_successor', GetSuccessor, handle_get_successor)
    rospy.Service('get_initial_state', GetInitialState, handle_get_initial_state)
    rospy.Service('is_goal_state', IsGoalState, handle_is_goal_state)
    rospy.Service('get_goal_state',GetGoalState,handle_get_goal_state)
    rospy.Service("generate_maze", GenerateMaze, ros_generate_maze)
    rospy.Service('remove_blocked_edge', RemoveBlockedEdgeMsg,remove_blocked_edge)
    print "Ready!"

    rospy.spin()
    
def generate_maze(subjects, books, seed):
    rospy.wait_for_service('generate_maze')
    response =  rospy.ServiceProxy('generate_maze', GenerateMaze) \
        (subjects, books, seed)
    if response.done != 0:    
        raise Exception("Cannot generate the maze")

def initialize_planning_server():

    fileHandle=open("/dev/null", "w")
    p = subprocess.Popen("rosrun hw2 server.py", shell=True, stdout=fileHandle, 
        stderr=fileHandle)

if __name__ == "__main__":


    server()
