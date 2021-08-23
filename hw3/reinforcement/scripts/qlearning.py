#!/usr/bin/env python
# encoding: utf-8

__copyright__ = "Copyright 2019, AAIR Lab, ASU"
__authors__ = ["Abhyudaya Srinet"]
__credits__ = ["Siddharth Srivastava"]
__license__ = "MIT"
__version__ = "1.0"
__maintainers__ = ["Pulkit Verma", "Abhyudaya Srinet"]
__contact__ = "aair.lab@asu.edu"
__docformat__ = 'reStructuredText'

import rospy
from std_msgs.msg import String
import sys
import problem
import json
import os
import argparse
import random
import utils
import hw3_task
from tqdm.auto import trange

from parser import parse_args
from server import initialize_planning_server
from server import generate_maze
from utils import initialize_ros
from utils import cleanup_ros


class QLearning:

    def __init__(self, subjects, num_books, seed, file_path, alpha, gamma,
        episodes, max_steps, epsilon_task):
        
        self.subjects = subjects
        self.num_books = num_books
        self.seed = seed
        self.epsilon_task = epsilon_task
        self.books_json_file = utils.ROOT_PATH + "/books.json"
        self.books = json.load(open(self.books_json_file))
        self.helper = problem.Helper()
        self.helper.reset_world()
        
        assert not os.path.exists(file_path) or not os.path.isdir(file_path)
        self.file_handle = open(file_path, "w")
        self.write_file_header(self.file_handle)
        
        self.alpha = alpha
        self.gamma = gamma
        self.max_steps = max_steps

        q_values = self.learn(episodes)

        with open(utils.ROOT_PATH + "/q_values.json", "w") as fout:
            json.dump(q_values, fout)
            
    def write_file_header(self, file_handle):
       
        file_handle.write("Subjects;Books;Seed;Gamma;Episode #;Alpha;Epsilon;Cumulative Reward;Total Steps;Goal Reached\n")

    def write_to_file(self, file_handle, episode_num, alpha, epsilon,
        cumulative_reward, total_steps, is_goal_satisfied):

        file_handle.write("%u;%u;%u;%.6f;%u;%.6f;%.6f;%.2f;%u;%s\n" % (
            self.subjects,
            self.num_books,
            self.seed,
            self.gamma,
            episode_num,
            alpha,
            epsilon,
            cumulative_reward,
            total_steps,
            is_goal_satisfied))
        pass

    def learn(self, episodes):

        # The q-table.
        q_values = {}

        root_path = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir))
        actions_config_file = open(root_path + "/action_config.json",'r')
        actions_config = json.load(actions_config_file)

        books_file = open(root_path + "/books.json",'r')
        books = json.load(books_file)

        pick_loc=[]
        place_loc=[]

        # Initialize the pick and place locations here.
        raise NotImplementedError
        
        epsilon = 1.0
        for i in trange(0, episodes, desc="Episode", unit="episode"):

            epsilon = hw3_task.get_epsilon(epsilon, i, self.epsilon_task)
            curr_state = self.helper.get_current_state()
            
            for step in trange(0, self.max_steps, leave=False, desc="Step", unit="step"):

                # Write the q-learning algorithm here.
                raise NotImplementedError

                # Finally, increment the time step.
                step = step + 1

            # Write the episode stats as per the header here.
            raise NotImplementedError

            self.helper.reset_world()

        return q_values

if __name__ == "__main__":

    random.seed(0xDEADC0DE)

    args = parse_args()

    file_path = utils.ROOT_PATH + "/" + args.file_name
    initialize_ros()
    initialize_planning_server()
    
    # Generate the world.
    generate_maze(args.subjects, args.books, args.seed, 1)
   
    QLearning(args.subjects, args.books, args.seed, file_path, args.alpha, 
        args.gamma, args.episodes, args.max_steps, args.epsilon_task)
    
    cleanup_ros()
