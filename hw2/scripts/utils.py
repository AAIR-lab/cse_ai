import subprocess
import time
import os

# Initialize the root path.
ROOT_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir))

DOMAIN_FILEPATH = ROOT_PATH + "/domain.pddl"
PROBLEM_FILEPATH = ROOT_PATH + "/problem.pddl"

PLANNER_LOG_FILEPATH = ROOT_PATH + "/planner_output.log"
PLANNER_TIMEOUT_IN_SECS = 300

def initialize_ros():

    fileHandle=open("/dev/null", "w")

    # Cleanup any previous instances of roscore.
    subprocess.call("pkill roscore", shell=True,
        stdout=fileHandle, stderr=fileHandle)
    
    # Start a new instance.
    subprocess.Popen("roscore", shell=True,
        stdout=fileHandle, stderr=fileHandle)
    
    # Wait a few seconds for ROS Core to come up.
    time.sleep(5)

def cleanup_ros():

    fileHandle=open("/dev/null", "w")
    
    # Cleanup any previous instances of roscore.
    subprocess.call("pkill roscore", shell=True,
        stdout=fileHandle, stderr=fileHandle)
