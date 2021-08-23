#!/usr/bin/env python
# encoding: utf-8

__copyright__ = "Copyright 2019, AAIR Lab, ASU"
__authors__ = ["Abhyudaya Srinet", "Pulkit Verma"]
__credits__ = ["Siddharth Srivastava"]
__license__ = "MIT"
__version__ = "1.0"
__maintainers__ = ["Pulkit Verma", "Abhyudaya Srinet"]
__contact__ = "aair.lab@asu.edu"
__docformat__ = 'reStructuredText'

import rospy
import problem
import heapq
import argparse
import os
import json
import random
from std_msgs.msg import String

from parser import parse_args
from server import initialize_planning_server
from server import generate_maze
from utils import initialize_ros
from utils import cleanup_ros

from hw2.srv import MoveActionMsg
from hw2.srv import PlaceActionMsg
from hw2.srv import PickActionMsg

import planner
import utils
import search
import traceback
import sys

"""
Do not change anything above this line, except if you want to import some package.
"""

class Refine:
    """
    This class has code to refine the high level actions used by PDDL to the the low level actions used by the TurtleBot.

    """

    def __init__(self, action_list):
        """
        :param plan_file: Path to the file where plan is stored. This is generally stored at the path from where the planner was called.
        :type plan_file: str
        :param planner: Planner that was used to generate this plan. This is needed to parse the plan and convert to/store in a data structure in Python. Only possible values are FF, FD, and PP.
        :type planner: str 

        Attributes:
            **status_subscriber**: To subscribe to the "/status" topic, which gives the State of the robot.

            **actions_queue** (tuple(action_name, action_params)): Used to store the refined action tuples. You can decide what parameters you want to store for each action. Store the parameters that you need to pass to execute_<action_name>_action methods of RobotActionServer class.

            **action_index** (int): Stores the index of action that needs to be sent to RobotActionServer for execution.
        """

        self.helper = problem.Helper()
        with open('%s/objects.json' % (utils.ROOT_PATH)) as f:
            self.env_data = json.load(f)

        self.action_index = 0
        self.actions_queue = self.refine_plan(action_list)

    def get_load_locations(self, location):
        obj = location[:location.index("_iloc")]
        
        if(obj.startswith("book")):
            return self.env_data["books"][obj]["load_loc"]
        elif(obj.startswith("trolly")):
            return self.env_data["bins"][obj]["load_loc"]

    def refine_plan(self, action_list):
        """
        Perform downward refinement to convert the high level plan into a low level executable plan.
        
        :param plan_file: Path to the file where plan is stored. This is generally stored at the path from where the planner was called.
        :type plan_file: str
        :param planner: Planner that was used to generate this plan. This is needed to parse the plan and convert to/store in a data structure in Python. Only possible values are FF, FD, and PP.
        :type planner: str 

        :returns: List of refined action tuples that will be added to the execution queue.
        :rtype: list(tuples)

        .. note::

            .. hlist::
                :columns: 1
                
                * Parse the plan from plan_file. This has to be according to the planner you are using.
                * use get_path(current_state, load_locations) to refine Move action.
                * Subject of the book and bin does not match.
                * Robot Location is not within the load location of the bin, i.e. robot is not in the viscinity of the bin.
        """

        current_state = self.helper.get_initial_state()
        actions = []

        for high_level_action in action_list:
            
            # Write your code to perform downward refinement.
            raise NotImplementedError

        return actions

    # --------------- HELPER FUNCTIONS --------------- #

    def is_goal_state(self, current_state, goal_state):
        """
        Checks if the current_state is goal_state or not. 
        If you are wondering why we are checking orientation, remember this is a different Homework. :)

        """
        if(current_state.x == goal_state.x and current_state.y == goal_state.y and current_state.orientation == goal_state.orientation):
            return True
        return False


    def build_goal_states(self, locations):
        """
        Creates a State representations for given list of locations
        
        """
        states = []
        for location in locations:
            states.append(problem.State(location[0], location[1], "EAST"))
            states.append(problem.State(location[0], location[1], "WEST"))
            states.append(problem.State(location[0], location[1], "NORTH"))
            states.append(problem.State(location[0], location[1], "SOUTH"))
        return states


    def get_path(self, init_state, goal_locations):
        """
        This method searches for a path from init_state to one of the possible goal_locations

        :param init_state: Current state of robot
        :type init_state: State
        :param goal_locations: list of target locations to search the path e.g. [(x1, y1), (x2, y2), ..]. This is important if there are multiple books of a subject.
        :type goal_locations: list(State)

        :returns: 
            .. hlist::
                :columns: 1

                * **action_list**: list of actions to execute to go from source to target
                * **final_state**: target state that is reached (will be one of goal_locations)
                * **goal_reached**: True/False indicating if one of the goal_locations was reached
        
        """
        final_state = None
        goal_states = self.build_goal_states(goal_locations)
        goal_reached = False

        for goal_state in goal_states:
        
            action_list, nodes_expanded = search.search(init_state, goal_state, self.helper, "gbfs")
            if action_list is not None:

                return action_list, goal_state, True

        return [], init_state, False

def generate_plan():
    '''
    Run a planner to evaluate problem.pddl and domain.pddl to generate a plan.
    Writes the plan to an output file
    '''

    action_list = planner.run_planner("fd", utils.DOMAIN_FILEPATH, 
        utils.PROBLEM_FILEPATH)

    return action_list
  
if __name__ == "__main__":

    random.seed(0xDEADC0DE)

    args = parse_args()
    initialize_ros()
    initialize_planning_server()
    
    # Generate the world.
    generate_maze(args.subjects, args.books, args.seed)

    if args.generate_only:
    
        cleanup_ros()
        sys.exit(0)
    
    FILE_PATH = utils.ROOT_PATH + "/" + args.file_name
    if os.path.exists(FILE_PATH) and args.clean:

        assert not os.path.isdir(FILE_PATH)                
        os.remove(FILE_PATH)

    if os.path.exists(FILE_PATH):
    
        assert not os.path.isdir(FILE_PATH)        
        file_handle = open(FILE_PATH, "a")
    else:
    
        file_handle = open(FILE_PATH, "w")
        file_handle.write("subjects; books; seed; exception; refined_plan\n")
    
    try:

        # Call the external planner for finding a high level plan.
        action_list = generate_plan()
        
        # Perform downward refinement.
        refinement = Refine(action_list)
        
        file_handle.write("%u; %u; %u; %s; %s\n" % (args.subjects,
            args.books,
            args.seed,
            None,
            str(refinement.actions_queue)))
    except Exception as e:
    
        print("Exception caught!")
        file_handle.write("%u; %u; %u; %s; %s\n" % (args.subjects,
            args.books,
            args.seed,
            type(e),
            []))

        traceback.print_exc()

    cleanup_ros()
