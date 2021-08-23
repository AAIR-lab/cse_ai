#!/usr/bin/env python

from reinforcement.srv import *
import rospy
from mazeGenerator import *
import sys
import argparse
import time
from action_server import RobotActionsServer 
import pickle
import copy
import os
import subprocess

root_path = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir))
books = None
mazeInfo = None
mazeInfoCopy = None
robot_action_server = None
parser = argparse.ArgumentParser()
parser.add_argument('-sub', help='for providing no. of subjects', metavar='2', action='store', dest='n_subjects', default=1, type=int)
parser.add_argument('-b', help='for providing no. of books for each subject of each size', metavar='1', action='store', dest='n_books', default=1, type=int)
parser.add_argument('-s', help='for providing random seed', metavar='32', action='store', dest='seed', default=int(time.time()), type=int)
parser.add_argument('-action_seed', help='for providing action selection random seed', metavar='32', action='store', dest='action_seed', default=int(time.time()), type=int)
parser.add_argument('-headless', help='1 to run in the headless mode, 0 to launch gazebo', metavar='1', action='store', dest='headless', default=1, type=int)
robot_action_server = None


def check_is_edge(req):
    """
    This function checks if two points are connected via edge or not.
    """
    global root_path
    books = robot_action_server.books
    mazeInfo = robot_action_server.mazeInfo

    edge = (req.x1,req.y1,req.x2,req.y2)
    for edge_point in edge:
        if edge_point < mazeInfo.grid_start or edge_point > mazeInfo.grid_dimension * 1.0:
            return 0
    if edge in mazeInfo.blocked_edges or (edge[2],edge[3],edge[0],edge[1]) in mazeInfo.blocked_edges:
        return 0
    else:
        return 1


def handle_reset_world(req):

    robot_action_server.mazeInfo = copy.deepcopy(robot_action_server.mazeInfoCopy)
    robot_action_server.current_state = robot_action_server.generate_init_state()
    return 1


def remove_blocked_edge(req):
    bookname = req.bookname
    books = robot_action_server.books
    mazeInfo = robot_action_server.mazeInfo

    location_of_blocked_edge_list = books["books"][bookname]["load_loc"]
    if location_of_blocked_edge_list[0][0] <= location_of_blocked_edge_list[1][0] and location_of_blocked_edge_list[0][1] <= location_of_blocked_edge_list[1][1]:
        blocked_edge = (location_of_blocked_edge_list[0][0], location_of_blocked_edge_list[0][1], location_of_blocked_edge_list[1][0], location_of_blocked_edge_list[1][1])
    else:
        blocked_edge = (location_of_blocked_edge_list[1][0], location_of_blocked_edge_list[1][1], location_of_blocked_edge_list[0][0], location_of_blocked_edge_list[0][1])
    mazeInfo.blocked_edges.remove(blocked_edge)
    return "1"

def ros_generate_maze(req):
    
    n_subjects=req.subjects
    n_books=req.books
    seed=req.seed
    headless = req.headless
    book_sizes = 2
    book_count_of_each_subject = n_books * book_sizes
    book_count_list = [n_books] * n_subjects * book_sizes
    number_of_trollies = n_subjects * 2
    grid_size = 6 * n_subjects

    global robot_action_server
    books, mazeInfo = generate_blocked_edges(grid_size, book_count_list, seed,  number_of_trollies, root_path, 1.0)
    
    if robot_action_server is None:
        
        robot_action_server = RobotActionsServer(books, root_path, headless == 1, seed)
        robot_action_server.mazeInfoCopy = mazeInfo
        robot_action_server.mazeInfo = copy.deepcopy(mazeInfo)
        robot_action_server.register_ros_functions()
    else:
        robot_action_server = RobotActionsServer(books, root_path, headless == 1, seed)
        robot_action_server.mazeInfoCopy = mazeInfo
        robot_action_server.mazeInfo = copy.deepcopy(mazeInfo)
    
    return GenerateMazeResponse(0)

def generate_maze(subjects, books, seed, headless):
    rospy.wait_for_service('generate_maze')
    response =  rospy.ServiceProxy('generate_maze', GenerateMaze) \
        (subjects, books, seed, headless)
    if response.done != 0:    
        raise Exception("Cannot generate the maze")

def server():

    rospy.init_node('server')
    rospy.Service('remove_blocked_edge', RemoveBlockedEdgeMsg,remove_blocked_edge)
    rospy.Service('check_is_edge',CheckEdge,check_is_edge)
    rospy.Service('reset_world',ResetWorldMsg,handle_reset_world)
    rospy.Service("generate_maze", GenerateMaze, ros_generate_maze)
    print "Ready!"
    rospy.spin()    
        

def initialize_planning_server():

    fileHandle=open("/dev/null", "w")
    p = subprocess.Popen("rosrun reinforcement server.py", shell=True, stdout=fileHandle, 
        stderr=fileHandle)

if __name__ == "__main__":

    server()
