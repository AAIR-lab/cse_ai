import argparse
import os
import time
import utils

# Create the argument parser.
parser = argparse.ArgumentParser(formatter_class=argparse.RawTextHelpFormatter)

parser.add_argument("--subjects", 
    default=1,
    type=int,
    help="No of book subjects.")
    
parser.add_argument("--books",
    default=1,
    type=int,
    help="The number of books in the grid.")
    
parser.add_argument("--seed",
    default=int(time.time()),
    type=int,
    help="The random seed.")
    
parser.add_argument("--alpha",
    default=0.3,
    type=float,
    help="The learning rate.")
    
parser.add_argument("--gamma",
    default=0.9,
    type=float,
    help="The discount factor.")
    
parser.add_argument("--episodes",
    default=500,
    type=int,
    help="The total number of episodes.")
    
parser.add_argument("--max-steps",
    default=500,
    type=int,
    help="The max. steps per episode.")
    
parser.add_argument("--epsilon-task",
    default=1,
    type=int,
    help="The epsilon task to use.")

    
parser.add_argument("--file-name",
    default="results.csv",
    help="Store results in <file> in the project root directory.")

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

    if args.subjects > 3:
        raise Exception('Maximum no. of subjects available is: 3')
        
    elif args.subjects < 1:
        raise Exception('Minimum no. of subjects available is: 1')

    if args.books > 3:
        raise Exception('Maximum no. of books available for each subject is: 3')
        
    elif args.books < 1:
        raise Exception('Minimum no. of books available for each subject is: 1')
   
    return args
