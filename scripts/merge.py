__doc__ = """Merge multiple rosbags into a single rosbag file

Usage:
    python merge.py [ROBAG1, ROBAG2, ...] 

Arguments:
    ROSBAG1, ROSBAG2, ...: The input rosbag files
    OUTPUT_ROSBAG: The output rosbag file

Options:
    -h, --help: Display this help message
    -v, --verbose: Print more information
    -o, --output <output>: The output rosbag file [default: output.bag]
    --time_setter: Set the time to zero for the first message
"""

import rosbag
import os
import sys
from rospy.rostime import Duration


print("Merging multiple rosbags into a single rosbag file...")
# Check arguments
if len(sys.argv) < 2:
    print(__doc__)
    sys.exit(1)

if '-h' in sys.argv or '--help' in sys.argv:
    print(__doc__)
    sys.exit(0)

verbose = '-v' in sys.argv or '--verbose' in sys.argv
time_setter = '--time_setter' in sys.argv

# Check the input files
inputs = []
for arg in sys.argv[1:]:
    if arg.startswith('-'):
        break
    if not arg.endswith('.bag'):
        print(f"Input file must be a .bag file {arg}")
        sys.exit(1)
    inputs.append(arg)

# Check the output file
if '-o' in sys.argv or '--output' in sys.argv:
    if '--output' in sys.argv:
        out_arg_index = sys.argv.index('--output')
    else:
        out_arg_index = sys.argv.index('-o')

    output_file = sys.argv[out_arg_index + 1]

else:
    output_file = 'output.bag'

if not output_file.endswith('.bag'):
    if verbose: print("Output file .bag missing ...")
    output_file = output_file + '.bag'

print(f"Output file: {output_file}")


# Read the messages from the input rosbags
if verbose: print("Reading messages from input rosbags...")
timestamp_offset = {}
messages = []
for inbag_file_path in inputs:
    if verbose: print(f"Reading message from {inbag_file_path}")
    with rosbag.Bag(inbag_file_path, 'r') as inbag_file:
        simulation_duration = inbag_file.get_end_time() - inbag_file.get_start_time()
        for topic, msg, t in inbag_file.read_messages():
            if time_setter:  
                messages.append((t.to_sec() - inbag_file.get_start_time(), (topic, msg, Duration(t.to_sec() - inbag_file.get_start_time()))))
            else:
                messages.append((t.to_sec(), (topic, msg, t)))



# Sort the messages by timestamp
if verbose: print("Sorting messages by timestamp...")
messages.sort(key=lambda x: x[0])

if verbose: print("Writing messages to the output rosbag...")
for i, message in enumerate(messages):
    if len(message) != 2:
        print(f"Message {i} has wrong length")
        raise ValueError("Message has wrong length")

# Write the messages to the output rosbag
if verbose: print("Writing messages to the output rosbag...")
with rosbag.Bag(output_file, 'w') as output_bag:
    for i, (_, msg) in enumerate(messages):
        if verbose: print(f"Writing message {i}", end='\r')
        output_bag.write(*msg)
    
print("Done")
sys.exit(0)
