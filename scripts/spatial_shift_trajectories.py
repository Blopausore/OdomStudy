__doc__ = """
Shift the trajectories in a rosbag file.
Different methods can be used to shift the trajectories.

Usage:
    python spatial_shift_trajectories.py <input_bag> <output_bag> --ref <reference_topic> [--shift_method <shift_method>] [method options]

Arguments:
    input_bag: The input bag file
    output_bag: The output bag file

Optional Arguments:
    -h, --help: Show this help message and exit
    -v, --verbose: Show the debug messages
    --shift_method: The method to shift the trajectories 

Methods:
    starting_matcher: Match the firsts points of the trajectories
    general_matcher: Shift the trajectories to minimize the distance between the points

Method Options:
    --matching_time_percentage: The percentage of the time to use for the matching [default: 0.1]

"""

import sys
import os
import rosbag
from rospy.rostime import Duration
import numpy as np

if '-h' in sys.argv or '--help' in sys.argv:
    print(__doc__)
    sys.exit(0)

if len(sys.argv) < 4:
    print("[ERROR]: Not enough arguments")
    print(__doc__)
    sys.exit(1)

verbose = '-v' in sys.argv or '--verbose' in sys.argv
input_bag = sys.argv[1]
if not os.path.exists(input_bag):
    print("[ERROR]: Input bag file does not exist")
    sys.exit(1)

if not input_bag.endswith('.bag'):
    print("[ERROR]: Input bag file is not a bag file")
    sys.exit(1)

output_bag = sys.argv[2]

if not output_bag.endswith('.bag'):
    print("[INFO]: Output bag file is not a bag file, it will be converted to a bag file")
    output_bag = output_bag + '.bag'

if os.path.exists(output_bag):
    print("[WARNING]: Output bag file already exists, it will be overwritten")

if verbose:
    print(f"[INFO] Input bag: {input_bag}")
    print(f"[INFO] Output bag: {output_bag}")

shift_method = 'starting_matcher'
if '--shift_method' in sys.argv:
    shift_method = sys.argv[sys.argv.index('--shift_method') + 1]
    if shift_method not in ['starting_matcher', 'general_matcher']:
        print(f"[ERROR] Unknown shift method: {shift_method}")
        sys.exit(1)

matching_time_percentage = 0.1
if '--matching_time_percentage' in sys.argv:
    matching_time_percentage = float(sys.argv[sys.argv.index('--matching_time_percentage') + 1])

if verbose:
    print(f"[INFO] Shift method: {shift_method}")
    if shift_method == 'starting_matcher':
        print(f"[INFO] Matching time percentage: {matching_time_percentage}")
 




def starting_matcher(traj1, traj2, matching_time_percentage=0.1):