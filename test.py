import rosbag
import sys

from scripts.tools.rosbag import get_first_timestamp

print("Zeroing timestamps in a rosbag file...")
print(get_first_timestamp(sys.argv[1], verbose=True))
with rosbag.Bag(sys.argv[1], 'r') as inbag:
    print(inbag.get_end_time())