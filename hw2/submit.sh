#/bin/bash -e

# This file generates a fresh batch of test cases in test_cases.csv at the root
# of the project.

# You can submit the test_cases.csv generated to Gradescope to check your
# assignment.

# You can also change the test cases here and submit new ones to Gradescope!

rosrun hw2 refinement.py --subjects 1 --books 1 --seed 111 --file-name submit.csv --clean
rosrun hw2 refinement.py --subjects 1 --books 2 --seed 222 --file-name submit.csv
rosrun hw2 refinement.py --subjects 2 --books 1 --seed 333 --file-name submit.csv
rosrun hw2 refinement.py --subjects 2 --books 2 --seed 444 --file-name submit.csv
rosrun hw2 refinement.py --subjects 3 --books 2 --seed 555 --file-name submit.csv
