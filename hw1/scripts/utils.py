import subprocess
import time

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
