"""
Contains functions to work with rosbag files.
"""

import rosbag
from rospy.rostime import Duration
import sys
import signal

def signal_handler(sig, frame, input_file):
    print('You pressed Ctrl+C!')
    input_file.close()
    sys.exit(0)

def get_first_message_timestamp(input_file, time_threshold=1, verbose=False) -> Duration:
    """
    Get the first message timestamp in a rosbag file

    :param input_file: The input rosbag file
    :param time_threshold: The time threshold to consider the origin of the first message [default: 1] seconds
    :param verbose: Print more information
    """
    if not input_file.endswith('.bag'):
        print("Input file must be a .bag file")
        sys.exit(1)

    if verbose:
        print(f"Input file: {input_file}")
        print(f"Time threshold: {time_threshold}")
        print(f"Verbose: {verbose}")

    # Check the first message timestamp
    with rosbag.Bag(input_file, 'r') as bag:
        first_time = Duration(bag.get_start_time())
        for i, (_, msg, t) in enumerate(bag.read_messages()):
            if msg._has_header:
                if msg.header.stamp.to_sec() > 0.0 and msg.header.stamp.to_sec() < first_time.to_sec():
                    first_time = msg.header.stamp
                    break

    if first_time is None:
        print("No message found or all messages have a timestamp of zero")
        return first_time

    if verbose:
        print(f"First message timestamp: {first_time}")
    if first_time.to_sec() == 0.0:
        if verbose:
            print("First message has a timestamp of zero : we are looking for the first non-zero timestamp")
        with rosbag.Bag(input_file, 'r') as bag:
            for i, (_, _, t) in enumerate(bag.read_messages()):
                if verbose: print(f"Reading message {i}", end='\r')
                if t.to_sec() > 0:
                    if verbose:
                        print(f"First non-zero timestamp : {t.to_sec()}")
                    if t.to_sec() > time_threshold:
                        first_time = t
                        if verbose:
                            print(f"First message timestamp exceeds the time threshold: {first_time.to_sec()}")
                    break
    if type(first_time) == float:
        first_time = Duration(first_time)
    return first_time

