__doc__ = """
Get the transformation matrices from tf topic in a rosbag file
This will create a file with the transformation matrices in a json format
Usage:
    python get_tf.py <input_bag> <output_file>
Arguments:
    input_bag: The input bag file
    output_file: The output file [default: tf.json]
Optional Arguments:
    -h, --help: Show this help message and exit
    -v, --verbose: Show the debug messages

"""

import sys
import os
import rosbag
import json
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

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
    output_file = sys.argv[2]
    if not output_file.endswith('.json'):
        print("[INFO]: Output file is not a json file, it will be converted to a json file")
        output_file = output_file + '.json'
else:
    output_file = 'tf.json'

if os.path.exists(output_file):
    print("[WARNING]: Output file already exists, it will be overwritten")

if verbose:
    print(f"[INFO] Input bag: {input_bag}")
    print(f"[INFO] Output file: {output_file}")

tf_data = {}
def main():
    with rosbag.Bag(input_bag, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=['/tf']):
            if msg.transforms:
                for transform in msg.transforms:
                    if transform.child_frame_id not in tf_data:
                        tf_data[transform.child_frame_id] = {
                            'translation': {
                                'x': transform.transform.translation.x,
                                'y': transform.transform.translation.y,
                                'z': transform.transform.translation.z
                            },
                            'rotation': {
                                'x': transform.transform.rotation.x,
                                'y': transform.transform.rotation.y,
                                'z': transform.transform.rotation.z,
                                'w': transform.transform.rotation.w
                            }
                        }
                        if verbose:
                            print(f"[INFO] Added {transform.child_frame_id} to the tf data")
        if verbose:
            print(f"[INFO] Finished reading the tf messages")
            print(f"[INFO] Writing the tf data to the output file")
        with open(output_file, 'w') as f:
            json.dump(tf_data, f, indent=4)

if __name__ == '__main__':
    print("[INFO] Starting the process")
    main()
    print("[INFO] Finished the process")

