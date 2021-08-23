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
    help="The random seed")
    
parser.add_argument("--generate-only",
    default=False,
    action="store_true",
    help="Only generate a new problem file and exit.")
    
parser.add_argument("--file-name",
    default="results.csv",
    help="Store results in <file> in the project root directory.")

parser.add_argument("--clean",
    default=False,
    action="store_true",
    help="Cleanup the existing csv files")

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
