__doc__ = """
Display a rosbag file content

Usage:
    rosbag_display.py <input_file> [-p] [-t <topic1, topic2, ...>]

Arguments:
    input_file: The input rosbag file

Options:
    -h, --help: Display this help message
    -v, --verbose: Print more information
    -p, --pause: Pause after each message
    -t, --topics <topic>: The topic to display [default: all]
"""

import rosbag
import sys
import signal


def signal_handler(sig, frame, input_file):
    print('You pressed Ctrl+C!')
    input_file.close()
    sys.exit(0)


print("Displaying a rosbag file content...")
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

verbose = '-v' in sys.argv or '--verbose' in sys.argv
pause = '-p' in sys.argv or '--pause' in sys.argv

if '-t' in sys.argv or '--topic' in sys.argv:
    if '--topic' in sys.argv:
        topic_arg_index = sys.argv.index('--topic')
    else:
        topic_arg_index = sys.argv.index('-t')
    topic_arg_index_end = topic_arg_index
    while topic_arg_index_end + 1< len(sys.argv) and not sys.argv[topic_arg_index_end + 1].startswith('-'):
        topic_arg_index_end += 1
    topics = sys.argv[topic_arg_index + 1:topic_arg_index_end]
else:
    topics = None

# Display the rosbag content
with rosbag.Bag(input_file, 'r') as bag:
    signal.signal(signal.SIGINT, lambda sig, frame: signal_handler(sig, frame, bag))
    for topic, msg, t in bag.read_messages(topics=topics):
        print(f"Topic: {topic}")
        print(f"Message {type(msg)}: {msg}")
        print(f"Timestamp: {t.to_sec()}")
        if verbose:
            print(f"Message: {msg}")
        if pause:
            input("Press Enter to continue...")
        # Intercept sigint 
        
        print()