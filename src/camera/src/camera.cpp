/***********************************************************************************
 *  StereoCamera
 *  ROS node to process stereo images from a USB-CDC based stereo camera.
 *  Depending on various configuration options, it can generate and broadcast:
 *    raw image
 *    disparity image
 *    point cloud
 *    simulated laser scan
 *
 *  For details of the ELAS dense stereo algorithm see:
 *  http://www.cvlibs.net/software/libelas/
 *
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
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <image_transport/image_transport.h>

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

//#include <pcl_ros/point_cloud.h>
//#include <pcl/point_types.h>
//#include <pcl_conversions/pcl_conversions.h>
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
#include "camera_options.hpp"

/*
using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;
*/


using namespace std;
using namespace cv;



pthread_t imgCaptureThread;
pthread_mutex_t imgCopyMutex;

Mat imageR(serial::Camera::MAX_IMAGE_HEIGHT, serial::Camera::MAX_IMAGE_WIDTH, CV_8UC1, Scalar(0));
Mat imageL(serial::Camera::MAX_IMAGE_HEIGHT, serial::Camera::MAX_IMAGE_WIDTH, CV_8UC1, Scalar(0));

bool got_camera_frame = false;


string FindCameraDevice(string hwid);
void* CaptureImages(void* param);




/**************************************************************
 * MAIN
 **************************************************************/
int main(int argc, char** argv)
{
  ros::init(argc, argv, "stereocamera");

  Options opt;
  ros::NodeHandle nh("~");
  opt.LoadOptions(nh);

  if (pthread_create(&imgCaptureThread, NULL, &CaptureImages, (void*)&opt) != 0)
  {
    ROS_ERROR_STREAM("Couldn't create Camera THREAD");
//    cvReleaseImage(&l);
//    cvReleaseImage(&r);
    exit(1);
  }

  // publish and subscribe under this name space
  ros::NodeHandle n;

  image_transport::ImageTransport it1(n), it2(n);
  image_transport::Publisher image_pub, disp_pub;
  ros::Publisher cloud_pub, scan_pub;

//  cv_bridge::CvImage img_bridge = cv_bridge::CvImage( std_msgs::Header(),
//                                                      sensor_msgs::image_encodings::MONO8,
//                                                      imgL );


  if (opt.publish_raw_image) {
    image_pub = it1.advertise("left/image_raw", 1);
  }

  if (opt.publish_disparity) {
    disp_pub = it2.advertise("disparity", 1);
  }

  if (opt.publish_pointcloud) {
//    cloud_pub = n.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("pointcloud", 1);
  }

  if (opt.publish_laserscan) {
//    scan_pub = n.advertise<sensor_msgs::LaserScan>("laserscan", 1);
  }


  if (opt.publish_raw_image)
    ROS_INFO("Publishing left raw image...");
  if (opt.publish_disparity)
    ROS_INFO("Publishing disparity image...");
  if (opt.publish_pointcloud)
    ROS_INFO("Publishing pointcloud...");
  if (opt.publish_laserscan)
    ROS_INFO("Publishing laserscan...");

  ros::Time capture_time;
  clock_t start;

  Mat imR(serial::Camera::MAX_IMAGE_HEIGHT, serial::Camera::MAX_IMAGE_WIDTH, CV_8UC1, Scalar(0));
  Mat imL(serial::Camera::MAX_IMAGE_HEIGHT, serial::Camera::MAX_IMAGE_WIDTH, CV_8UC1, Scalar(0));

  while (n.ok())
  {
    if (got_camera_frame)
    {
      if (opt.profiling)
        start = clock();
      capture_time = ros::Time::now();

      // copy out images
      pthread_mutex_lock(&imgCopyMutex);
      imageR.copyTo(imR);
      imageL.copyTo(imL);
      pthread_mutex_unlock(&imgCopyMutex);
      // clear flag
      got_camera_frame = false;

      if (opt.publish_raw_image) {
        //img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
        //pub_img.publish(img_msg); // ros::Publisher pub_img = node.advertise<sensor_msgs::Image>("topic", queuesize);
        // Convert to sensor_msgs::Image
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(),
                                                       sensor_msgs::image_encodings::MONO8,
                                                       imL).toImageMsg();
        // publish image
        image_pub.publish(msg);
        //memcpy ((void*)(&raw_image.data[0]), (void*)img, opt.ww*opt.hh*3);
        //image_pub.publish(raw_image);
      }

      if (opt.profiling) {
        ROS_INFO("TIME: total=%f capture=%f cap_cntr=%ld", ((float)(clock()-start))/CLOCKS_PER_SEC, 0.0, 0L);
      }

    }
    ros::spinOnce();
  }

  return 0;
}




/**************************************************************
 * Thread to capture camera images
 **************************************************************/
void* CaptureImages(void* param)
{
  ROS_DEBUG_STREAM("Capture Thread started");
  Options *opt = (Options*)param;

  // locate serial port
  string port = FindCameraDevice(opt->usb_hwid);
  if (port == "")
  {
    ROS_ERROR_STREAM("No camera - exit thread");
    pthread_exit(NULL);
  }
  ROS_DEBUG_STREAM("Camera found on port " << port);

  // configure camera serial port
  serial::Camera camera(port, 115200, serial::Timeout(0,0,0,250,0));
  ROS_DEBUG_STREAM("Camera port configured");

  if (!camera.isOpen())
  {
    ROS_ERROR_STREAM("Unable to open port " << port);
    pthread_exit(NULL);
  }
  ROS_DEBUG_STREAM("Camera port opened");

  // configure camera parameters
  camera.Configure(opt->camconfig);
  ROS_DEBUG_STREAM("Camera initialized");

  Mat imgR(camera.MAX_IMAGE_HEIGHT, camera.MAX_IMAGE_WIDTH, CV_8UC1, Scalar(0));
  Mat imgL(camera.MAX_IMAGE_HEIGHT, camera.MAX_IMAGE_WIDTH, CV_8UC1, Scalar(0));

  ROS_DEBUG_STREAM("streaming...");
  // capture loop
  while(1)
  {
    if (camera.RecvLine(imgR, imgL) == 0)
    {
      pthread_mutex_lock(&imgCopyMutex);
      imgR.copyTo(imageR);
      imgL.copyTo(imageL);
      got_camera_frame = true;
      pthread_mutex_unlock(&imgCopyMutex);
      // blank out temp images
      //imgR.setTo(Scalar(0));
      //imgL.setTo(Scalar(0));
    }

  }

}



/**************************************************************
 * Find camera device with a given USB HWID
 * and return serial port allocated to this device.
 **************************************************************/
string FindCameraDevice(string hwid)
{
  // enumerate available devices
  vector<serial::PortInfo> devices_found = serial::list_ports();

  // find one with matching ID
  hwid.insert(0, "USB VID:PID=");
  ROS_DEBUG_STREAM("Looking for device " << hwid);

  vector<serial::PortInfo>::iterator iter = devices_found.begin();
  while( iter != devices_found.end() )
  {
    serial::PortInfo device = *iter++;
    if (device.hardware_id.compare(0,hwid.length(),hwid) == 0)
    {
      ROS_DEBUG_STREAM("Found device " << hwid);
      return device.port;
    }
  }

  ROS_WARN_STREAM("Device " << hwid << " not found!");
  return "";
}

