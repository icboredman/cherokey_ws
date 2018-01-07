# minorucam

A [ROS](http://www.ros.org) node that processes stereo images from two usb webcams, such as [Minoru](http://www.minoru3d.com/).

Depending on various options defined in camera.launch, it can generate and broadcast the following ROS topics:
* raw (left) image
* disparity image
* point cloud
* laser scan

The software is based on original work of Bob Mottram and Giacomo Spigler (fuzzgun@gmail.com) called v4l2stereo, which was part of Sentience project:
https://code.google.com/archive/p/sentience/wikis/MinoruWebcam.wiki and https://github.com/bashrc/libv4l2cam

The node implements ELAS dense stereo algorithm. More info at: http://www.cvlibs.net/software/libelas/

To remove dependance on SSE instructions and make it run on Raspberry Pi, this code uses contributions of Ralph Campbell: https://github.com/RalphCampbell/v4l2stereo_no_sse

---
More info about the project is here: https://BoredomProjects.net/index.php/projects/robot-navigation-using-stereo-vision
