#/bin/bash -e

# This file generates a fresh batch of test cases in test_cases.csv at the root
# of the project.

# You can submit the test_cases.csv generated to Gradescope to check your
# assignment.

# You can also change the test cases here and submit new ones to Gradescope!

rosrun hw2 refinement.py --subjects 1 --books 1 --seed 0 --file-name test_cases.csv --clean
rosrun hw2 refinement.py --subjects 1 --books 2 --seed 1 --file-name test_cases.csv
rosrun hw2 refinement.py --subjects 2 --books 1 --seed 2 --file-name test_cases.csv
rosrun hw2 refinement.py --subjects 2 --books 2 --seed 3 --file-name test_cases.csv
rosrun hw2 refinement.py --subjects 3 --books 2 --seed 4 --file-name test_cases.csv
