# ODOM Study

In this folder, we will compare different ways of having the position of the robot.

For doing so, we will use [evo][1], a library that help us comparing differents trajectories.

## Scripts
This folder contains a lot of script that help manipulate rosbag files.
You can extract the doc for them with `python3 scripts/script_name.py --help`.


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