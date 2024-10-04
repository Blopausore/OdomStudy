__doc__ = """
This script removes useless messages from a rosbag file.
It will remove the messages that have the same content as the previous message.

Usage:
    python crop_useless_msg.py <input_bag> [<output_bag>]

Arguments:
    input_bag: The input bag file
    output_bag: The output bag file [default: input_bag.cropped.bag]

Optional Arguments:
    -h, --help: Show this help message and exit
    -v, --verbose: Show the debug messages
"""

import sys
import os
import rosbag
from rospy.rostime import Duration

if '-h' in sys.argv or '--help' in sys.argv:
    print(__doc__)
    sys.exit(0)

if len(sys.argv) < 2:
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

if len(sys.argv) > 2 and not sys.argv[2].startswith('-'):
    output_bag = sys.argv[2]
    if not output_bag.endswith('.bag'):
        print("[INFO]: Output bag file is not a bag file, it will be converted to a bag file")
        output_bag = output_bag + '.bag'
else:
    output_bag = f"{input_bag[:-4]}.cropped.bag"

if os.path.exists(output_bag):
    print("[WARNING]: Output bag file already exists, it will be overwritten")

if verbose:
    print(f"[INFO] Input bag: {input_bag}")
    print(f"[INFO] Output bag: {output_bag}")

def have_same_content(msg1, msg2):
    for field in msg1.__slots__: # Check all fields
        if field != 'header' and field != 'header.stamp':
            if getattr(msg1, field) != getattr(msg2, field):
                return False
    return True

def main():
    last_msgs = {}
    with rosbag.Bag(input_bag, 'r') as inbag, rosbag.Bag(output_bag, 'w') as outbag:
        for topic, msg, t in inbag.read_messages():
            if topic not in last_msgs:
                last_msgs[topic] = msg
                outbag.write(topic, msg, t)

            elif not have_same_content(msg, last_msgs[topic]):
                last_msgs[topic] = msg
                outbag.write(topic, msg, t)
            else:
                if verbose:
                    print(f"[INFO] Message skipped: {topic}", end='\r')

if __name__ == '__main__':
    print("[INFO] Cropping useless messages...")
    main()
    print("[INFO] Done")
    sys.exit(0)

