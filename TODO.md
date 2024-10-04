# Odometry and localisation study

- [x] Create `geometry_msgs/PoseWithCovarianceStamped` rosbag file from `sbg_driver/SbgEkfNav`


### OrbSlam3
- [ ] Complete the installation
- [ ] Record a rosbag
- [ ] Formatting it to be used with evo


### Kiss ICP
- [x] Install and try
- [ ] Create a new publisher for the odometry
- [ ] Record it + formatting for evo

### Comparing studies
- [x] Barakuda sensors -> give a reference?
- [ ] Barakuda sensors + dlio + orb
- [ ] Barakuda sensors + dlio + orb + kiss-icp

### Mix Kiss-ICP/ DLIO -> KISS-DLIO
- [ ] Add the first step (dynamic integration of each points from a lidar scan) to kiss-icp
  - [ ] Change the velocity constant integration to full dynamic integration with imu
  - [ ] Save this transformation and apply to adaptative threshold