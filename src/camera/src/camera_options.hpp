/***********************************************************************************
 *  StereoCamera
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * ------------------------------
 *  boredman@BoredomProjects.net
 * ------------------------------
 *
 ***********************************************************************************/
#ifndef CAMERA_OPTIONS_HPP
#define CAMERA_OPTIONS_HPP

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/LaserScan.h>
//#include <omp.h>
#include <nav_msgs/Odometry.h>

#include <sstream>
#include <iostream>
#include <string>
#include <iostream>
#include <cstdio>
#include <stdio.h>
#include <pthread.h>

#include <time.h>
// OS Specific sleep
//#include <unistd.h>

#include "camera_serial.hpp"


class Options
{
  public:

  std::string usb_hwid;
  std::string calibration_filename;
  serial::Camera::camconfig camconfig;
  bool publish_raw_image;
  bool zoom_center;
  bool publish_rectified;
  bool histogram_equalization;
  bool histogram_show;
  int disparity_lines;
  bool publish_disparity;
  bool publish_pointcloud;
  bool publish_laserscan;
  float cloud_range_max;
  float scan_range_min;
  float scan_range_max;
  bool  scan_range_def_infinity;
  int scan_height_min;
  int scan_height_max;
  bool profiling;

  // Load options from ROS command line or launch file
  void LoadOptions(ros::NodeHandle& nh)
  {
    nh.param("usb_hwid", usb_hwid, (std::string)"1fc9:0094");
    nh.param("calibration_filename", calibration_filename, (std::string)"calibration.txt");

    nh.param("exposure", camconfig.exposure_us, 10000);
    nh.param("analogGain", camconfig.analogGain, 16);
    nh.param("digitalGain", camconfig.digitalGain, 4);
    nh.param("numLines", camconfig.n_lines, 480);
    nh.param("countsPerFrame", camconfig.cpf, 2);
    nh.param("autoExposure", camconfig.aec_enable, 0);
    nh.param("autoGain", camconfig.agc_enable, 0);
    nh.param("compEnable", camconfig.cmp_enable, 0);

    nh.param("pub_image", publish_raw_image, true);
    nh.param("zoom_center", zoom_center, true);
    nh.param("pub_rect", publish_rectified, false);
    nh.param("hist_equal", histogram_equalization, false);
    nh.param("hist_show", histogram_show, false);

    nh.param("disparity_lines", disparity_lines, 480);
    nh.param("pub_disparity", publish_disparity, false);

    nh.param("pub_cloud", publish_pointcloud, false);
    nh.param("cloud_range_max", cloud_range_max, (float)6000);

    nh.param("pub_laser", publish_laserscan, false);
    nh.param("scan_range_min", scan_range_min, (float)0);
    nh.param("scan_range_max", scan_range_max, (float)6000);
    nh.param("scan_range_def_infinity", scan_range_def_infinity, true);
    nh.param("scan_height_min", scan_height_min, -5);
    nh.param("scan_height_max", scan_height_max, 5);

    nh.param("profiling", profiling, false);
  }

};


#endif
