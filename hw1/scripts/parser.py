import argparse
import time

# Create the argument parser.
parser = argparse.ArgumentParser()
parser.add_argument("--dimension", 
    default=5,
    type=int,
    help="The dimension 'n' of the n x n grid.")
    
parser.add_argument("--obstacles",
    default=0,
    type=int,
    help="The number of obstacles in the grid.")
    
parser.add_argument("--seed",
    default=int(time.time()),
    type=int,
    help="The random seed")
    
parser.add_argument("--algorithm",
    default=None,
    choices=["all", "bfs", "ucs", "gbfs", "astar", "custom-astar"],
    type=str,
    help="The algorithm to run")
    
parser.add_argument("--submit",
    default=False,
    action="store_true",
    help="Run the experiments required for submission")
    
parser.add_argument("--output-file",
    default=None,
    help="Store the output in the specified file")
    
def parse_args():
    """
        Parses the cmd line arguments.
        
        Returns
        ========
            args: Namespace
                The parsed args.
                
        Raises
        =======
            ArgumentError
                If the arguments do not parse correctly.
    """
    args = parser.parse_args()
   
    if args.dimension <= 2:
   
        raise Exception("Number of dimensions must be greater than 2")
   
    max_obstacles = args.dimension * (args.dimension + 1) * 2
    if args.obstacles > max_obstacles:

        raise Exception("Invalid number of obstacles (%u) specified. "
            "Max obstacles possible for a grid of dimension (%u) is [0, %u]." % 
            (args.obstacles, args.dimension, max_obstacles))
   
    return args
