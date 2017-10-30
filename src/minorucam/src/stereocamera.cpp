/***********************************************************************************
 *  StereoCamera
 *  ROS driver to process stereo images from a stereo webcam (eg. Minoru)
 *  Depending on various command line options, it can generate and broadcast:
 *    raw image
 *    disparity image
 *    point cloud
 *    laser scan
 *
 *  The software is based on original work of Bob Mottram and Giacomo Spigler
 *  (fuzzgun@gmail.com) called v4l2stereo, which was part of Sentience project:
 *  https://code.google.com/archive/p/sentience/wikis/MinoruWebcam.wiki
 *  https://github.com/bashrc/libv4l2cam
 *
 *  For details of the ELAS dense stereo algorithm see:
 *  http://www.cvlibs.net/software/libelas/
 *
 *  To remove dependance on SSE instructions and make it run on Raspberry Pi,
 *  this code uses contributions of Ralph Campbell:
 *  https://github.com/RalphCampbell/v4l2stereo_no_sse
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
 *  Rev 1.0 - 2017.03.12
 *    fisrt major cleanup
 *
 ***********************************************************************************/

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <image_transport/image_transport.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/LaserScan.h>
#include <sstream>
#include <omp.h>
#include <nav_msgs/Odometry.h>

#include <iostream>
#include <stdio.h>

#include <time.h>

#include "v4l2stereo_no_sse/src/anyoption.h"
#include "v4l2stereo_no_sse/src/drawing.h"
#include "v4l2stereo_no_sse/src/stereo.h"
#include "v4l2stereo_no_sse/src/fast.h"
#include "v4l2stereo_no_sse/src/libcam.h"
#include "v4l2stereo_no_sse/src/camcalib.h"
#include "v4l2stereo_no_sse/src/pointcloud.h"
#include "v4l2stereo_no_sse/src/elas/elas.h"

#include <pthread.h>

using namespace std;
using namespace cv;

class Options
{
public:
  void LoadOptions(ros::NodeHandle& nh);
  std::string dev_left;
  std::string dev_right;
  std::string calibration_filename;
  int ww;
  int camhh;
  int hh;
  int fps;
  int exposure;
  bool profiling;
  bool publish_raw_image;
  bool publish_disparity;
  bool publish_pointcloud;
  bool publish_laserscan;
  bool flip_left_image;
  bool flip_right_image;
  bool histogram_equalization;
  float cloud_range_max;
  float scan_range_min;
  float scan_range_max;
  bool  scan_range_def_infinity;
  int scan_height_min;
  int scan_height_max;
  float turn_speed_limit;
};


pthread_t imgCaptureThread;
pthread_mutex_t imgCopyMutex;

Camera *left_camera = NULL;
Camera *right_camera = NULL;

IplImage *l=NULL;
IplImage *r=NULL;

unsigned long cap_cntr = 0;
float cap_avg_time = 0;

void *CaptureImages(void *param);
void flip( unsigned char* raw_image, int ww, int hh );
void colorize_disparities( unsigned char *img, float *disparities, int ww, int hh, int min_disparity );
void pointcloud_publish( unsigned char *image, IplImage *points, Options &opt, ros::Time &time, ros::Publisher &pub );
void laserscan_publish( IplImage *points, camcalib &camera_calibration, Options &opt, ros::Time &time, ros::Publisher &pub );
void stop_cameras(Camera *&left_camera, Camera *&right_camera);

float turn_speed = 0;
void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);


/**************************************************************
 * MAIN
 **************************************************************/
int main(int argc, char** argv)
{
  ros::init(argc, argv, "stereocamera");

  Options opt;
  ros::NodeHandle nh("~");
  opt.LoadOptions(nh);

  if (left_camera != NULL) {
    stop_cameras(left_camera,right_camera);
  }

  l = cvCreateImage(cvSize(opt.ww, opt.hh), 8, 3);
  r = cvCreateImage(cvSize(opt.ww, opt.hh), 8, 3);

  left_camera = new Camera(opt.dev_left.c_str(), opt.ww, opt.camhh, opt.fps);
  right_camera = new Camera(opt.dev_right.c_str(), opt.ww, opt.camhh, opt.fps);

  if (opt.exposure == 0) {
    left_camera->setExposureAuto();
    right_camera->setExposureAuto();
  } else {
    left_camera->setExposureAutoOff();
    right_camera->setExposureAutoOff();
    left_camera->setExposure(opt.exposure);
    right_camera->setExposure(opt.exposure);
  }

  if ( pthread_create(&imgCaptureThread, NULL, &CaptureImages, (void*)&opt) != 0 ) {
    ROS_ERROR("Couldn't create Camera THREAD");
    stop_cameras(left_camera,right_camera);
    cvReleaseImage(&l);
    cvReleaseImage(&r);
    exit(1);
  }

  camcalib camera_calibration;
  camera_calibration.ParseCalibrationFile(opt.calibration_filename);
  bool rectify_images = camera_calibration.rectification_loaded;

  //correct -Cy due to different image height
  float cam_Cy = cvmGet(camera_calibration.disparityToDepth,1,3);
  float new_Cy = cam_Cy + (opt.camhh-opt.hh) / 2;
  cvmSet(camera_calibration.disparityToDepth,1,3, new_Cy );

  IplImage* hist_image0 = cvCreateImage(cvSize(opt.ww, opt.hh), IPL_DEPTH_8U, 1);
  IplImage* hist_image1 = cvCreateImage(cvSize(opt.ww, opt.hh), IPL_DEPTH_8U, 1);

  // publish and subscribe under this name space
  ros::NodeHandle n;

  ros::Subscriber odom_sub;
  if (opt.turn_speed_limit > 0)
  {
    ROS_INFO("limiting stereo processing to twists below %f rad/sec", opt.turn_speed_limit);
    odom_sub = n.subscribe("odom", 100, odom_callback);
  }

  sensor_msgs::Image raw_image;
  raw_image.width  = opt.ww;
  raw_image.height = opt.hh;
  raw_image.step = opt.ww * 3;
  raw_image.encoding = "bgr8";
  raw_image.data.resize(opt.ww*opt.hh*3);

  image_transport::ImageTransport it1(n), it2(n);
  image_transport::Publisher image_pub, disp_pub;
  ros::Publisher cloud_pub, scan_pub;

  if (opt.publish_raw_image) {
    image_pub = it1.advertise("left/image_raw", 1);
  }

  if (opt.publish_disparity) {
    disp_pub = it2.advertise("disparity", 1);
  }

  if (opt.publish_pointcloud) {
    cloud_pub = n.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("pointcloud", 1);
  }

  if (opt.publish_laserscan) {
    scan_pub = n.advertise<sensor_msgs::LaserScan>("laserscan", 1);
  }

  unsigned char *l_ = new unsigned char[opt.ww * opt.hh * 3];
  unsigned char *r_ = new unsigned char[opt.ww * opt.hh * 3];
  unsigned char *buffer = NULL;

  Elas::parameters param;
//  param.speckle_size = 1000;
//  param.ipol_gap_width = 500;
  Elas elas(param);
  uint8_t* I1 = new uint8_t[opt.ww*opt.hh];
  uint8_t* I2 = new uint8_t[opt.ww*opt.hh];
  float* left_disparities = new float[opt.ww*opt.hh];
  float* right_disparities = new float[opt.ww*opt.hh];
  IplImage * disparity_image = NULL;
  IplImage * points_image = NULL;

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

  while (n.ok()) {

    // do not try to process stereo if robot is turning too fast
    // due to unsynchronized cameras
    if (opt.turn_speed_limit > 0 && turn_speed > opt.turn_speed_limit)
    {
      ros::spinOnce();
      continue;
    }

    if (opt.profiling)
      start = clock();

    capture_time = ros::Time::now();

    pthread_mutex_lock(&imgCopyMutex);
    memcpy((void*)l_, l->imageData, opt.ww*opt.hh*3);
    memcpy((void*)r_, r->imageData, opt.ww*opt.hh*3);
    pthread_mutex_unlock(&imgCopyMutex);

    // flip images
    if (opt.flip_left_image) {
      flip(l_, opt.ww, opt.hh);
    }

    if (opt.flip_right_image) {
      flip(r_, opt.ww, opt.hh);
    }

    if (opt.publish_disparity || opt.publish_pointcloud || opt.publish_laserscan) {
      // rectify images
      if (!rectify_images) {
        ROS_ERROR("Images need to be recrified before using ELAS");
        break;
      }
#pragma omp parallel for
      for (int cam = 0; cam < 2; cam++) {
        if (cam == 0)
          camera_calibration.RectifyImage(0, opt.ww, opt.hh, l_, -camera_calibration.v_shift);
        else
          camera_calibration.RectifyImage(1, opt.ww, opt.hh, r_, +camera_calibration.v_shift);
      }

      // histogram equalization
      if (opt.histogram_equalization) {
        if (buffer == NULL) {
          buffer = new unsigned char[opt.ww*opt.hh*3];
        }
        memcpy((void*)buffer, l_, opt.ww*opt.hh*3);

#pragma omp parallel for
        for (int i = 0; i < 2; i++) {
          unsigned char *img = l_;
          IplImage* hist_image = hist_image0;
          if (i > 0) {
            img = r_;
            hist_image = hist_image1;
          }
          svs::histogram_equalise( hist_image, img, opt.ww, opt.hh );
        }
      }

      //calculate disparities
      // convert to single byte format
      for (int i = 0; i < opt.ww*opt.hh; i++) {
        I1[i] = (uint8_t)l_[i*3+2];
        I2[i] = (uint8_t)r_[i*3+2];
      }
      const int32_t dims[3] = {opt.ww, opt.hh, opt.ww};
      elas.process(I1,I2,left_disparities,right_disparities,dims);
    }

    if (opt.publish_pointcloud || opt.publish_laserscan) {
      // convert disparity map to 3D points
      pointcloud::disparity_map_to_3d_points(
          left_disparities, opt.ww, opt.hh,
          camera_calibration.disparityToDepth,
          camera_calibration.pose,
          disparity_image, points_image );
    }

    unsigned char *img;
    if ( opt.histogram_equalization && (opt.publish_disparity || opt.publish_pointcloud || opt.publish_laserscan) )
      img = buffer;
    else
      img = l_;

    if (opt.publish_raw_image) {
      // Convert to sensor_msgs::Image
      memcpy ((void*)(&raw_image.data[0]), (void*)img, opt.ww*opt.hh*3);
      // publish image
      image_pub.publish(raw_image);
    }

    if (opt.publish_pointcloud) {
      pointcloud_publish( img, points_image, opt, capture_time, cloud_pub );
    }

    if (opt.publish_disparity) {
      // Color code image based on disparity
      colorize_disparities( img, left_disparities, opt.ww, opt.hh, 0 );
      // Convert to sensor_msgs::Image
      memcpy ((void*)(&raw_image.data[0]), (void*)img, opt.ww*opt.hh*3);
      // publish image
      disp_pub.publish(raw_image);
    }

    if (opt.publish_laserscan) {
      laserscan_publish( points_image, camera_calibration, opt, capture_time, scan_pub );
    }

    if (opt.profiling) {
      ROS_INFO("TIME: total=%f capture=%f cap_cntr=%ld", ((float)(clock()-start))/CLOCKS_PER_SEC, cap_avg_time, cap_cntr);
    }

    ros::spinOnce();
  }

  // cleanup
  pthread_cancel(imgCaptureThread);
  stop_cameras(left_camera,right_camera);
  cvReleaseImage(&l);
  cvReleaseImage(&r);
  cvReleaseImage(&hist_image0);
  cvReleaseImage(&hist_image1);
  delete[] l_;
  delete[] r_;
  if (buffer != NULL) delete[] buffer;
  delete[] I1;
  delete[] I2;
  delete[] left_disparities;
  delete[] right_disparities;
  if (disparity_image != NULL) cvReleaseImage(&disparity_image);
  if (points_image != NULL) cvReleaseImage(&points_image);

  ROS_INFO("Stereo camera stopped");
}



/**************************************************************
 * stop the stereo camera
 *   left_camera left camera object
 *   right_camera right camera object
 **************************************************************/
void stop_cameras(Camera *&left_camera, Camera *&right_camera)
{
  if (left_camera != NULL) {
    delete left_camera;
    delete right_camera;
    left_camera = NULL;
    right_camera = NULL;
  }
}


/**************************************************************
 * Load options from ROS command line or launch file
 **************************************************************/
void Options::LoadOptions(ros::NodeHandle& nh)
{
  nh.param("profiling", profiling, false);

  nh.param("width", ww, 320);
  nh.param("cam_height", camhh, 240);
  nh.param("height", hh, 240);
  nh.param("fps", fps, 30);
  nh.param("dev_left", dev_left, (std::string)"/dev/video1");
  nh.param("dev_right", dev_right, (std::string)"/dev/video0");
  nh.param("flip_left", flip_left_image, false);
  nh.param("flip_right", flip_right_image, false);

  nh.param("exposure", exposure, 0);

  nh.param("calibration_filename", calibration_filename, (std::string)"calibration.txt");

  nh.param("hist_equal", histogram_equalization, false);

  nh.param("pub_image", publish_raw_image, true);

  nh.param("pub_disparity", publish_disparity, true);

  nh.param("pub_cloud", publish_pointcloud, true);
  nh.param("cloud_range_max", cloud_range_max, (float)6000);

  nh.param("pub_laser", publish_laserscan, true);
  nh.param("scan_range_min", scan_range_min, (float)0);
  nh.param("scan_range_max", scan_range_max, (float)6000);
  nh.param("scan_range_def_infinity", scan_range_def_infinity, true);
  nh.param("scan_height_min", scan_height_min, -5);
  nh.param("scan_height_max", scan_height_max, 5);

  nh.param("turn_speed_limit", turn_speed_limit, (float)0);
}



/**************************************************************
 * flip the given image so that the camera can be mounted upside down
 **************************************************************/
void flip(unsigned char* raw_image, int ww, int hh)
{
  unsigned char* flipped_frame_buf = new unsigned char[ww*hh*3];
  int max = ww * hh * 3;
  for (int i = 0; i < max; i += 3) {
    flipped_frame_buf[i] = raw_image[(max - 3 - i)];
    flipped_frame_buf[i + 1] = raw_image[(max - 3 - i + 1)];
    flipped_frame_buf[i + 2] = raw_image[(max - 3 - i + 2)];
  }
  memcpy(raw_image, flipped_frame_buf, max * sizeof(unsigned char));
  delete[] flipped_frame_buf;
}


/**************************************************************
 * Color-code image based on disparities
 **************************************************************/
void colorize_disparities( unsigned char * img,
                           float * disparities,
                           int ww,
                           int hh,
                           int min_disparity )
{
  for (int i = 0; i < ww*hh; i++) {
    if (disparities[i] > min_disparity) {
      float val = min(( *(((float*)disparities)+i) )*0.01f,1.0f);
      if (val <= 0) {
        img[3*i+0] = 0;
        img[3*i+1] = 0;
        img[3*i+2] = 0;
      } else {
        float h2 = 6.0f * (1.0f - val);
        unsigned char x  = (unsigned char)((1.0f - fabs(fmod(h2, 2.0f) - 1.0f))*255);
        if (0 <= h2&&h2<1) {
          img[3*i+0] = 255;
          img[3*i+1] = x;
          img[3*i+2] = 0;
        }
        else if (1<=h2&&h2<2)  {
          img[3*i+0] = x;
          img[3*i+1] = 255;
          img[3*i+2] = 0;
        }
        else if (2<=h2&&h2<3)  {
          img[3*i+0] = 0;
          img[3*i+1] = 255;
          img[3*i+2] = x;
        }
        else if (3<=h2&&h2<4)  {
          img[3*i+0] = 0;
          img[3*i+1] = x;
          img[3*i+2] = 255;
        }
        else if (4<=h2&&h2<5)  {
          img[3*i+0] = x;
          img[3*i+1] = 0;
          img[3*i+2] = 255;
        }
        else if (5<=h2&&h2<=6) {
          img[3*i+0] = 255;
          img[3*i+1] = 0;
          img[3*i+2] = x;
        }
      }
    }
    else {
      img[3*i+0] = 0;
      img[3*i+1] = 0;
      img[3*i+2] = 0;
    }
  }

}



/**************************************************************
 * Publish Pointcloud
 **************************************************************/
void pointcloud_publish( unsigned char *image, IplImage *points, Options &opt, ros::Time &time, ros::Publisher &pub )
{
  pcl::PointCloud<pcl::PointXYZRGB> cloud;

  pcl_conversions::toPCL(time, cloud.header.stamp);
  cloud.header.frame_id = "camera_frame";
  cloud.points.resize(0);
  cloud.is_dense = true;

  float * points_image_data = (float*)points->imageData;
  int pixels = points->width * points->height;
  int n = 0, ctr = 0;
  pcl::PointXYZRGB point;
  for (int i = 0; i < pixels; i++, n += 3) {
    point.y = -points_image_data[n] / 1000;
    point.z = -points_image_data[n+1] / 1000;
    point.x = points_image_data[n+2] / 1000;

    if ((fabs(point.x)+fabs(point.y)+fabs(point.z) > 0.001) &&
        (fabs(point.x) < opt.cloud_range_max) &&
        (fabs(point.y) < opt.cloud_range_max) &&
        (fabs(point.z) < opt.cloud_range_max)) {
      point.b = image[n];
      point.g = image[n+1];
      point.r = image[n+2];
      cloud.points.push_back(point);
      ctr++;
    }
  }
  if (ctr > 0) {
    cloud.height = 1;
    cloud.width = ctr;
    pub.publish(cloud);
  }
}



/**************************************************************
 * Publish Laser Scan
 **************************************************************/
void laserscan_publish( IplImage *points, camcalib &camera_calibration, Options &opt, ros::Time &time, ros::Publisher &pub )
{
  sensor_msgs::LaserScan scan;

  float center_x = -cvmGet(camera_calibration.disparityToDepth,0,3);
  float center_y = -cvmGet(camera_calibration.disparityToDepth,1,3);
  float focal_length = cvmGet(camera_calibration.disparityToDepth,2,3);

  //determine amount of rays to create
  uint32_t ranges_size = points->width;

  float scan_angle_min = atan2(center_x - ranges_size, focal_length);
  float scan_angle_max = atan2(ranges_size, focal_length);
  float scan_angle_increment = (scan_angle_max - scan_angle_min) / ranges_size;

  scan.header.stamp = time;
  scan.header.frame_id = "camera_frame";
  scan.angle_min = scan_angle_min;
  scan.angle_max = scan_angle_max;
  scan.angle_increment = scan_angle_increment;
  scan.time_increment = 0.0;
  scan.range_min = opt.scan_range_min;
  scan.range_max = opt.scan_range_max;

  if (opt.scan_range_def_infinity)
    scan.ranges.assign(ranges_size, std::numeric_limits<double>::infinity());
  else
    scan.ranges.assign(ranges_size, opt.scan_range_max - 0.01);

  scan.intensities.resize(0);

  float * points_image_data = (float*)points->imageData;
  int width = points->width;
  int height = points->height;
  int pixels = width * height;

  vector<double> ranges[width];

  int n = 0, ctr = 0, w = 0, h = 0;
  for (int i = 0; i < pixels; i++, n += 3) {
    float x = points_image_data[n] / 1000;
    float y = points_image_data[n+1] / 1000;
    float z = points_image_data[n+2] / 1000;

    if (++w >= width) {
      w = 0;
      ++h;
    }

//  if (y < scan_height_min || y > scan_height_max) {
    if ( (h - center_y) > opt.scan_height_max || (h - center_y) < opt.scan_height_min ) {
      continue;
    }

    double range = hypot(x,z);
    if (range < opt.scan_range_min) {
      continue;
    }

    // sign in front of atan2 controls the scan direction (pos = CW; neg = CCW)
    double angle = -atan2(x,z);
    if (angle < scan_angle_min || angle > scan_angle_max) {
      continue;
    }

    //overwrite range at laserscan ray if new range is smaller
    int index = (angle - scan_angle_min) / scan_angle_increment;
//    int index = width - w - 1;
/*
    if (range < scan.ranges[index]) {
      scan.ranges[index] = range;
    }
*/
    ranges[index].push_back(range);

    ctr++;
  }

  if (ctr > 0) {

    for(int i=0; i<width; i++) {
      int length = ranges[i].size();
      if( length == 0 )
        if (opt.scan_range_def_infinity)
          scan.ranges[i] = std::numeric_limits<double>::infinity();
        else
          scan.ranges[i] = opt.scan_range_max - 0.01;
      else {
        std::sort(ranges[i].begin(), ranges[i].end());
        double median = ranges[i].at(length/2);
        scan.ranges[i] = median;
      }
    }

    pub.publish(scan);
  }
}



/**************************************************************
 * Thread to capture camera images
 **************************************************************/
void *CaptureImages(void *param)
{
  Options *opt = (Options*)param;
  bool profiling = opt->profiling;

  clock_t cap_start;
  float cap_time;

  while(1)
  {
    if (profiling) {
      cap_start = clock();
    }

    // capture frames
    if (!left_camera->Update(right_camera, 25, 1000)) {
      pthread_exit(NULL);
    }

    // Convert to IplImage
    pthread_mutex_trylock(&imgCopyMutex);
    left_camera->toIplImage(l);
    right_camera->toIplImage(r);
    pthread_mutex_unlock(&imgCopyMutex);

    if (profiling) {
      cap_time = ((float)(clock() - cap_start))/CLOCKS_PER_SEC;
      cap_avg_time = cap_avg_time * 0.9 + cap_time * 0.1;
      cap_cntr++;
    }
  }

}


/**************************************************************
 * Callback to receive "odom" messages
 * extract angular (turn) speed and save it in a global variable
 **************************************************************/
void odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  turn_speed = fabs(msg->twist.twist.angular.z);

//  ROS_INFO("Seq: [%d]", msg->header.seq);
//  ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
//  ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
//  ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x, msg->twist.twist.angular.z);
}

