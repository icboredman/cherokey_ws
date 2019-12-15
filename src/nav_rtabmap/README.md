# nav_rtabmap

A ROS package that implements appearance-based SLAM and navigation using ROS wrapper for RTAB-Map package: [rtabmap_ros](http://wiki.ros.org/rtabmap_ros).

## Mapping:
Mapping could be achieved in three different ways:
#### Online
  - launch file: [local](rtab_slam.launch)
  - everything done within robot itself
  - high load requirement on embedded computer
#### Remote
  - launch files: [local](rtab_slam_remote.launch), [remote](remote/remote_slam.launch)
  - RTAB-Map runs on remotely connected computer
  - mapping is achieved in real-time
  - computing power partially offloaded to remote processor
  - high network bandwidth, low latency requirements
#### Offline
  - launch file: [remote](remote/offline_slam.launch)
  - sensor and odometry messages are recorded in a ROS bag
  - RTAB-Map runs on remote computer at a later time
  - map is generated not in real time
Currently, only Offline mapping has been successfully tested.

## Navigation:
Configuration file: [rtab_move.launch](rtab_move.launch)

See also [nav_controller_node](https://github.com/icboredman/nav_controller_node.git)

---
More info in my blog page: https://BoredomProjects.net/index.php/projects/robot-navigation-using-stereo-vision-part-2

