__doc__ = """

This script ajusts the timestamps of each message in a rosbag.
For each topic, it will set the first message timestamp to the rosbag timestamp and adjust the rest of the messages accordingly.
WARNING: This transformation reduces the correlation between the messages timestamps because it make the supposition that the first message is emitted at the rosbag timestamp, which is false but is an approximation.

Usage:
    python rosbag_zero_time_setter.py <input_bag> [<output_bag>]

Arguments:
    input_bag: The input bag file
    output_bag: The output bag file [default: input_bag.zero_time.bag]

Optional Arguments:
    -h, --help: Show this help message and exit
    -v, --verbose: Show the debug messages
    --shift_method: The method to shift the timestamps [default: gross_time_setter] [options: gross_time_setter, setter_on_rosbag_timeline, idiotic_setter_on_rosbag_timeline]

Shift methods:
    - gross_time_setter: Set the first message timestamp to zero adjust the rest of the messages accordingly
    - setter_on_rosbag_timeline: Set the first message timestamp to the rosbag timestamp and adjust the rest of the messages accordingly, but also shift the rosbag timeline to the first message timestamp
    - idiotic_setter_on_rosbag_timeline: Set the first message timestamp to the rosbag timestamp and adjust the rest of the messages accordingly, but also set the rosbag timeline to the first message timestamp

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
    output_bag = f"{input_bag[:-4]}.zero_time.bag"

if os.path.exists(output_bag):
    print("[WARNING]: Output bag file already exists, it will be overwritten")

if verbose:
    print(f"[INFO] Input bag: {input_bag}")
    print(f"[INFO] Output bag: {output_bag}")

shift_method = 'gross_time_setter'
if '--shift_method' in sys.argv:
    shift_method = sys.argv[sys.argv.index('--shift_method') + 1]
    if shift_method not in ['gross_time_setter', 'setter_on_rosbag_timeline', 'idiotic_setter_on_rosbag_timeline']:
        print(f"[ERROR] Unknown shift method: {shift_method}")
        sys.exit(1)


def gross_time_setter():
    ## Initialize our structure to store the messages and their timestamps evolution
    topic_offset = {} # {topic: offset}
    # Find timestamp offset for each topic
    with rosbag.Bag(input_bag, 'r') as inbag:
        simulation_duration = inbag.get_end_time() - inbag.get_start_time() 
        for topic, msg, t in inbag.read_messages():
            if topic not in topic_offset:
                topic_offset[topic] = msg.header.stamp.to_sec()
            elif abs(msg.header.stamp.to_sec() - topic_offset[topic]) > simulation_duration:
                print(f"[WARNING] Topic {topic} has a timestamp offset greater than the simulation duration, so we change the offset")
                topic_offset[topic] = msg.header.stamp.to_sec()
    
    if verbose:
        print(f"[INFO] Topic offset: ")
        for topic, offset in topic_offset.items():
            print(f"[INFO]  -- {topic}: {offset}")

    with rosbag.Bag(output_bag, 'w') as outbag, rosbag.Bag(input_bag, 'r') as inbag:
        simulation_duration = inbag.get_end_time() - inbag.get_start_time()
        for topic, msg, t in inbag.read_messages():
            if msg.header.stamp.to_sec() - topic_offset[topic] > simulation_duration:
                print(f"[ERROR] Topic {topic} has a timestamp offset greater than the simulation duration, so we skip the message")
                raise Exception(f"Topic {topic} has a timestamp offset greater than the simulation duration")
            d = Duration(max(msg.header.stamp.to_sec() - topic_offset[topic], 0.0))
            msg.header.stamp.secs = d.secs
            msg.header.stamp.nsecs = d.nsecs
            outbag.write(topic, msg, t)

def setter_on_rosbag_timeline():
    ## Initialize our structure to store the messages and their timestamps evolution
    topic_offset = {} # {topic: offset}

    # Find timestamp offset for each topic
    with rosbag.Bag(input_bag, 'r') as inbag:
        ros_offset = 0
        if inbag.get_start_time() > 0:
            if verbose: print(f"[WARNING] The rosbag has a start time different than 0, so we shift the ros timeline of {inbag.get_start_time()} seconds")
            ros_offset = inbag.get_start_time()

        simulation_duration = inbag.get_end_time() - inbag.get_start_time() 
        for topic, msg, t in inbag.read_messages():
            if topic not in topic_offset:
                topic_offset[topic] = msg.header.stamp.to_sec() - (t.to_sec() - ros_offset) 
            elif abs(msg.header.stamp.to_sec() - topic_offset[topic]) > simulation_duration:
                if verbose: print(f"[WARNING] Topic {topic} has a timestamp offset greater than the simulation duration, so we change the offset")
                topic_offset[topic] = msg.header.stamp.to_sec() - (t.to_sec() - ros_offset)

    if verbose:
        print(f"[INFO] Ros offset: {ros_offset}")
        print(f"[INFO] Topic offset: ")
        for topic, offset in topic_offset.items():
            print(f"[INFO]  -- {topic}: {offset}")

    # Write the messages to the output rosbag
    with rosbag.Bag(output_bag, 'w') as outbag, rosbag.Bag(input_bag, 'r') as inbag:
        for topic, msg, t in inbag.read_messages():
            if msg.header.stamp.to_sec() > topic_offset[topic]:
                d = Duration(msg.header.stamp.to_sec() - topic_offset[topic])
                msg.header.stamp.secs = d.secs
                msg.header.stamp.nsecs = d.nsecs
                outbag.write(topic, msg, t)
            else:
                d = Duration(0)
                msg.header.stamp.secs = d.secs
                msg.header.stamp.nsecs = d.nsecs
                outbag.write(topic, msg, t)

def idiotic_setter_on_rosbag_timeline():
    ## Initialize our structure to store the messages and their timestamps evolution
    topic_offset = {} # {topic: offset}
    with rosbag.Bag(output_bag, 'w') as outbag, rosbag.Bag(input_bag, 'r') as inbag:
        _ = Duration(inbag.get_start_time())
        for topic, msg, t in inbag.read_messages():
            if topic not in topic_offset:
                topic_offset[topic] = t
            elif t < topic_offset[topic]:
                topic_offset[topic] = t
            
            msg.header.stamp.secs = t.secs 
            msg.header.stamp.nsecs = t.nsecs
            outbag.write(topic, msg, t)

shift_methods = {
    'gross_time_setter': gross_time_setter,
    'setter_on_rosbag_timeline': setter_on_rosbag_timeline,
    'idiotic_setter_on_rosbag_timeline': idiotic_setter_on_rosbag_timeline
}

if __name__ == '__main__':
    print(f"[INFO] Processing {input_bag}")
    shift_methods[shift_method]()
    print(f"[INFO] Output bag: {output_bag}")
    print("[INFO] Done")
    sys.exit(0)

