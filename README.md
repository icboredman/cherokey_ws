# cherokey_ws

A [ROS](http://www.ros.org) workspace that implements a configuration of ROS Navigation Stack for a four-wheeled robot that uses a **stereo camera** as a primary sensor.

### Components (nodes):

* [camera_node](https://github.com/icboredman/camera_node.git) - stereo camera driver for [TeensyCam](https://github.com/icboredman/TeensyCam-HW)
* [base_serial_node](https://github.com/icboredman/base_serial_node.git) - communication with base
* [teleop_key_node](https://github.com/icboredman/teleop_key_node.git) - move robot using keyboard
* [nav_controller_node](https://github.com/icboredman/nav_controller_node.git) - send goals to robot and process feedback using ActionLib API


---
More info in my blog page: https://BoredomProjects.net/index.php/projects/robot-navigation-using-stereo-vision-part-2
