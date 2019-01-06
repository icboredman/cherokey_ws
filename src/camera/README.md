# Camera

A [ROS](http://www.ros.org) node that processes data received from [TeensyCam](https://github.com/icboredman/TeensyCam-HW) - stereo camera module.

Depending on various options defined in camera.launch, it can generate and broadcast the following ROS topics:
* raw (both left and right) images
* rectified images
* disparity image
* point cloud
* emulated laser scan

The node implements [ELAS dense stereo algorithm](http://www.cvlibs.net/software/libelas/), specifically, its OpenMP-version.


---
More info about the project is here: https://BoredomProjects.net/index.php/projects/robot-navigation-using-stereo-vision

