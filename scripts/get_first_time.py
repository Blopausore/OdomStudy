__doc__ = """
Check the first message timestamp in a rosbag file.
If the timestamp is zero, check the first message that is not zero and print it if it exeeds a threshold.
Otherwise, return zero.

Usage:
    rosbag_check_first_time.py <input_file> [--time_threshold <time_threshold>]

Arguments:
    input_file: The input rosbag file

Options:
    -h, --help: Display this help message
    -time_threshold <time_threshold>: The time threshold to consider the origin of the first message [default: 1] seconds
    -v, --verbose: Print more information
"""
import rosbag
import sys
from rospy.rostime import Duration
from tools.rosbag import get_first_message_timestamp

print("Checking first message timestamp in a rosbag file...")
# Check arguments
if len(sys.argv) < 2:
    print(__doc__)
    sys.exit(1)

if '-h' in sys.argv or '--help' in sys.argv:
    print(__doc__)
    sys.exit(0)

input_file = sys.argv[1]

if not input_file.endswith('.bag'):
    print("Input file must be a .bag file")
    sys.exit(1)

print(f"Input file: {input_file}")

# Check options
time_threshold = float(sys.argv[sys.argv.index('--time_threshold') + 1]) if '--time_threshold' in sys.argv else 1
verbose = '-v' in sys.argv or '--verbose' in sys.argv

print(f"Time threshold: {time_threshold}")
print(f"Verbose: {verbose}")

# Check the first message timestamp
first_time = get_first_message_timestamp(input_file, time_threshold, verbose)
print(f"FIRST MESSAGE TIMESTAMP: {first_time.to_sec()} ")
with rosbag.Bag(input_file, 'r') as bag:
    print(f"START TIME: {Duration(bag.get_start_time()).to_sec()}")
sys.exit(0)
