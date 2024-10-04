# ODOM Study

In this folder, we will compare different ways of having the position of the robot.

For doing so, we will use [evo][1], a library that help us comparing differents trajectories.

## Scripts
This folder contains a lot of script that help manipulate rosbag files.
You can extract the doc for them with `python3 scripts/script_name.py --help`.


### Get first time
This script checks the first message timestamp in a rosbag file. If the timestamp is zero, it checks the first non-zero message and prints it if it exceeds a specified threshold. Otherwise, it returns zero.

Usage:
```bash
get_first_time.py <input_file> [--time_threshold <time_threshold>]
```

Arguments:
- `input_file`: The input rosbag file

Options:
- `-h, --help`: Display this help message
- `--time_threshold <time_threshold>`: The time threshold to consider the origin of the first message [default: 1] seconds
- `-v, --verbose`: Print more information

### Merge
This script merges multiple rosbag files into a single rosbag file.

Usage:
```bash
merge.py [ROSBAG1, ROSBAG2, ...] -o <output>
```

Arguments:
- `ROBAG1, ROBAG2, ...`: The input rosbag files

Options:
- `-h, --help`: Display this help message
- `-v, --verbose`: Print more information
- `-o, --output <output>`: The output rosbag file [default: output.bag]
- `--time_setter`: Set the time to zero for the first message


### Zero time setter
This script adjusts the timestamps of each message in a rosbag. For each topic, it will set the first message timestamp to the rosbag timestamp and adjust the rest of the messages accordingly. WARNING: This transformation reduces the correlation between the messages timestamps because it makes the supposition that the first message is emitted at the rosbag timestamp, which is false but is an approximation.

Usage:
```bash
python rosbag_zero_time_setter.py <input_bag> [<output_bag>]
```

Arguments:
- `input_bag`: The input bag file
- `output_bag`: The output bag file [default: input_bag.zero_time.bag]

Optional Arguments:
- `-h, --help`: Show this help message and exit
- `-v, --verbose`: Show the debug messages
- `--shift_method`: The method to shift the timestamps [default: gross_time_setter] [options: gross_time_setter, setter_on_rosbag_timeline, idiotic_setter_on_rosbag_timeline]

Shift methods:
- `gross_time_setter`: Set the first message timestamp to zero and adjust the rest of the messages accordingly
- `setter_on_rosbag_timeline`: Set the first message timestamp to the rosbag timestamp and adjust the rest of the messages accordingly, but also shift the rosbag timeline to the first message timestamp
- `idiotic_setter_on_rosbag_timeline`: Set the first message timestamp to the rosbag timestamp and adjust the rest of the messages accordingly, but also set the rosbag timeline to the first message timestamp

### Display
This script displays the content of a rosbag file. You can specify which topics to display and whether to pause after each message.

Usage:
```bash
display.py <input_file> [-p] [-t <topic1, topic2, ...>]
```

Arguments:
- `input_file`: The input rosbag file

Options:
- `-h, --help`: Display this help message
- `-v, --verbose`: Print more information
- `-p, --pause`: Pause after each message
- `-t, --topics <topic1> <topic2> ...`: The topics to display [default: all]

### Crop useless msg
This script removes useless messages from a rosbag file. It will remove the messages that have the same content as the previous message.

Usage:
```bash
python crop_useless_msg.py <input_bag> [<output_bag>]
```

Arguments:
- `input_bag`: The input bag file
- `output_bag`: The output bag file [default: input_bag.cropped.bag]

Optional Arguments:
- `-h, --help`: Show this help message and exit
- `-v, --verbose`: Show the debug messages
### TMP Folder
In this folder, you can find some scripts that are not usually used but can be useful in some cases.
#### Converter SBGNav to Pose
This script converts the SBG EKF Nav message to a PoseWithCovarianceStamped message.

Usage:
```bash
python converter_sbgnav2pose.py <input_bag> <output_bag>
```

Arguments:
- `input_bag`: The input bag file containing the SBG EKF Nav messages
- `output_bag`: The output bag file containing the PoseWithCovarianceStamped messages

Options:
- `-h, --help`: Show this help message and exit
- `-v, --verbose`: Show the debug messages
- `-p, --pub_name <name>`: The name of the topic to publish [default: /sbg/pose_cov]
- `-s, --sub_name <name>`: The name of the topic to subscribe [default: /sbg/ekf_nav]


## Encountered problems


### Origin timestamp difference
#### Problem
The differents sensors of your robot may not have the same initializing timestamp.
This difference will be located in the headers of messages from the topics.

In my case I have encountered this problem with the topics `/imu/nav_sat_fix` and `/zed2i/zed_node/pose_with_covariance`.



```bash
root@SmaugENSTA:/home/augustinbresset# rostopic echo /zed2i/zed_node/pose_with_covariance

header: 
  seq: 16661
  stamp: 
    secs: 1662635175
    nsecs: 824176656
  frame_id: "map"
```

In here I had to transform `/imu/nav_sat_fix` (gps coordinates) to position, to do so I used [geodetic utils][2] to convert it.

```bash
root@SmaugENSTA:/home/augustinbresset# rostopic echo /gps/pose_cova
header: 
  seq: 1
  stamp: 
    secs: 1713452027
    nsecs: 186256576
  frame_id: "world"
```

We can see here that we have as origins of time :
* /gps/pose : **1713452027**
* /zed2i/zed_node/pose_with_covariance : **1662635175**

It results with the following error when you try to align those topics :

```bash
root@SmaugENSTA:/home/augustinbresset/catkin_ws/analysis/odom_study# evo_traj bag --plot data/rosbags/gps_zedpose.bag --all_topics --ref /gps/pose_cova --plot --align        
/usr/local/lib/python3.8/dist-packages/scipy/__init__.py:146: UserWarning: A NumPy version >=1.16.5 and <1.23.0 is required for this version of SciPy (detected version 1.24.4
  warnings.warn(f"A NumPy version >={np_minversion} and <{np_maxversion}"
[WARNING] geometry_msgs/PointStamped does not contain rotation, evo will use unit quaternion. Note that rotation metrics will be invalid and RPE will only be valid with point_distance metric.
[ERROR] found no matching timestamps between reference and /zed2i/zed_node/pose_with_covariance with max. time diff 0.01 (s) and time offset 0.0 (s)

```

We can think that we just have to use the timestamp from the rosbag but be careful, this timestamp is recorded when the topic had published the message, not when the mesure was taken by the sensor.
Using it will end with a loss of accurancy.

#### Solution

##### Change the rosbag file

The first solution is not exactly correct.
In the `scripts` folder, there is the script `rosbag_zero_time.py` that will put the origins of time at 0

##### Create a transformation independant of time

###### Method 1 :
Using the first non-null pose (with quaternion) to create a tranformation of one topic to an other.

###### Method 2 :
Taking the transformation that minimize the difference thanks to an algorithm such as ICP.


###### Using --t_offset
Much more easier than my precedents idea ...

```bash
evo_traj bag --plot data/rosbags/zed_gps_pose.bag /zed2i/zed_node/pose_with_covariance --ref /gps/pose_cova --plot --t_offset 50816852  --align
```

## References

[1]: <https://github.com/MichaelGrupp/evo> "evo github"
[2]: <https://github.com/ethz-asl/geodetic_utils/tree/master/geodetic_utils> "geodetic utils github"