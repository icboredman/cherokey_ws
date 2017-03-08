/*
    ROS driver to broadcast stereo images from a stereo webcam (eg. Minoru)
    This doesn't do any stsreo correspondence.  It merely broadcasts the images.
    Copyright (C) 2012 Bob Mottram
    fuzzgun@gmail.com

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <image_transport/image_transport.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sstream>
#include <omp.h>

#include <iostream>
#include <stdio.h>

#include <time.h>


#include "v4l2stereo_no_sse/anyoption.h"
#include "v4l2stereo_no_sse/drawing.h"
#include "v4l2stereo_no_sse/stereo.h"
#include "v4l2stereo_no_sse/fast.h"
#include "v4l2stereo_no_sse/libcam.h"
#include "v4l2stereo_no_sse/camcalib.h"
#include "v4l2stereo_no_sse/pointcloud.h"
#include "v4l2stereo_no_sse/elas/elas.h"

using namespace std;
using namespace cv;

std::string dev_left = "";
std::string dev_right = "";
int ww = 320;
int hh = 240;
int fps = 30;

Camera *left_camera = NULL;
Camera *right_camera = NULL;
sensor_msgs::Image raw_image;

IplImage *l=NULL;
IplImage *r=NULL;
unsigned char *l_=NULL;
unsigned char *r_=NULL;
unsigned char * buffer=NULL;

bool flip_left_image=false;
bool flip_right_image=false;
bool histogram_equalization=false;
bool publish_raw_image=true;

image_transport::Publisher image_pub;

ros::Publisher cloud_pub;
pcl::PointCloud<pcl::PointXYZRGB> cloud;

camcalib *camera_calibration=NULL;
std::string calibration_filename = "calibration.txt";
bool rectify_images = false;

uint8_t * I1 = NULL;
uint8_t * I2 = NULL;
float * left_disparities = NULL;
float * right_disparities = NULL;
Elas * elas = NULL;

IplImage* hist_image0 = NULL;
IplImage* hist_image1 = NULL;

IplImage * disparity_image = NULL;
IplImage * points_image = NULL;


void elas_disparity_map(
  unsigned char * left_image,
  unsigned char * right_image,
  int image_width,
  int image_height,
  uint8_t * &I1,
  uint8_t * &I2,
  float * &left_disparities,
  float * &right_disparities,
  Elas * &elas)
{
  if (elas==NULL) {
    Elas::parameters param;
    elas = new Elas(param);
    I1 = new uint8_t[image_width*image_height];
    I2 = new uint8_t[image_width*image_height];
    left_disparities = new float[image_width*image_height];
    right_disparities = new float[image_width*image_height];
  }

  // convert to single byte format
  for (int i = 0; i < image_width*image_height; i++) {
    I1[i] = (uint8_t)left_image[i*3+2];
    I2[i] = (uint8_t)right_image[i*3+2];
  }

  const int32_t dims[3] = {image_width, image_height, image_width};
  elas->process(I1,I2,left_disparities,right_disparities,dims);
}


/*!
 * \brief stop the stereo camera
 * \param left_camera left camera object
 * \param right_camera right camera object
 */
void stop_cameras(
                  Camera *&left_camera,
                  Camera *&right_camera)
{
  if (left_camera != NULL) {
    delete left_camera;
    delete right_camera;
    left_camera = NULL;
    right_camera = NULL;
  }
}

void start_cameras(
                   Camera *&left_camera,
                   Camera *&right_camera,
                   std::string dev_left, std::string dev_right,
                   int width, int height,
                   int fps)
{
  if (left_camera != NULL) {
    stop_cameras(left_camera,right_camera);
  }

  l = cvCreateImage(cvSize(width, height), 8, 3);
  r = cvCreateImage(cvSize(width, height), 8, 3);

  l_=(unsigned char *)l->imageData;
  r_=(unsigned char *)r->imageData;

  left_camera = new Camera(dev_left.c_str(), width, height, fps);
  right_camera = new Camera(dev_right.c_str(), width, height, fps);

  camera_calibration = new camcalib();
  camera_calibration->ParseCalibrationFile(calibration_filename);
  rectify_images = camera_calibration->rectification_loaded;

  hist_image0 = cvCreateImage( cvGetSize(l), IPL_DEPTH_8U, 1 );
  hist_image1 = cvCreateImage( cvGetSize(l), IPL_DEPTH_8U, 1 );

  ros::NodeHandle n;

  cloud_pub = n.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("camera/pointcloud", 1);  // 50 ?

  if (publish_raw_image) {
    raw_image.width  = width;
    raw_image.height = height;
    raw_image.step = width * 3;
    raw_image.encoding = "bgr8";
    raw_image.data.resize(width*height*3);

    image_transport::ImageTransport it(n);

    std::string topic_str = "camera/left/image_raw";
    image_pub = it.advertise(topic_str.c_str(), 1);
  }
}

// flip the given image so that the camera can be mounted upside down
void flip(unsigned char* raw_image, unsigned char* flipped_frame_buf) {
  int max = ww * hh * 3;
  for (int i = 0; i < max; i += 3) {
    flipped_frame_buf[i] = raw_image[(max - 3 - i)];
    flipped_frame_buf[i + 1] = raw_image[(max - 3 - i + 1)];
    flipped_frame_buf[i + 2] = raw_image[(max - 3 - i + 2)];
  }
  memcpy(raw_image, flipped_frame_buf, max * sizeof(unsigned char));
}


void pointcloud_publish( unsigned char *image_buf,
                         IplImage *points_image,
                         int max_range_mm,
                         CvMat *pose,
                         float baseline )
{
  pcl_conversions::toPCL(ros::Time::now(), cloud.header.stamp);
  cloud.header.frame_id = "sensor_frame";
  cloud.points.resize(0);
  cloud.is_dense = true;

  float pose_x = (float)cvmGet(pose,0,3);
  float pose_y = (float)cvmGet(pose,1,3);
  float pose_z = (float)cvmGet(pose,2,3);

  float * points_image_data = (float*)points_image->imageData;
  int pixels = points_image->width * points_image->height;
  int n = 0, ctr = 0;
  pcl::PointXYZRGB point;
  for (int i = 0; i < pixels; i++, n += 3) {
    point.x = points_image_data[n];
    point.y = points_image_data[n+1];
    point.z = points_image_data[n+2];

    float dx = point.x - pose_x;
    float dy = point.y - pose_y;
    float dz = point.z - pose_z;

    if ((fabs(dx)+fabs(dy)+fabs(dz) > 1) &&
        (fabs(dx) < max_range_mm) &&
        (fabs(dy) < max_range_mm) &&
        (fabs(dz) < max_range_mm)) {
      point.b = image_buf[n];
      point.g = image_buf[n+1];
      point.r = image_buf[n+2];
      cloud.points.push_back(point);
      ctr++;
    }
  }
  if (ctr > 0) {
    cloud.height = 1;
    cloud.width = ctr;
    cloud_pub.publish(cloud);
  }
}


void cleanup()
{
  cvReleaseImage(&l);
  cvReleaseImage(&r);

  if (hist_image0 != NULL) cvReleaseImage(&hist_image0);
  if (hist_image1 != NULL) cvReleaseImage(&hist_image1);
  if (disparity_image != NULL) cvReleaseImage(&disparity_image);
  if (points_image != NULL) cvReleaseImage(&points_image);

  if (elas != NULL) delete elas;
  delete camera_calibration;

  if (buffer != NULL) delete[] buffer;

  stop_cameras(left_camera,right_camera);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "stereocamera_pointcloud");
  ros::NodeHandle nh("~");

  // set some default values
  int val=0;
  std::string str="";
  dev_left = "/dev/video1";
  dev_right = "/dev/video0";

  nh.getParam("width", val);
  if (val>0) ww=val;
  nh.getParam("height", val);
  if (val>0) hh=val;
  nh.getParam("fps", val);
  if (val>0) fps=val;
  nh.getParam("dev_left", str);
  if (str!="") dev_left=str;
  nh.getParam("dev_right", str);
  if (str!="") dev_right=str;
  nh.getParam("flip_left", flip_left_image);
  nh.getParam("flip_right", flip_right_image);

  nh.getParam("calibration_filename", str);
  if (str!="") calibration_filename=str;

  nh.getParam("hist_equal", histogram_equalization);
  nh.getParam("pub_image", publish_raw_image);

  start_cameras(left_camera,right_camera,
                dev_left, dev_right,
                ww, hh, fps);

  ros::NodeHandle n;
  bool publishing = false;

  while (n.ok()) {

clock_t start = clock();

    // pre-capture to reduce lag
    left_camera->Update(right_camera, 25, 1000);
    // capture frames
    if (!left_camera->Update(right_camera, 25, 1000)) {
      ROS_ERROR("Camera capture error");
      break;
    }
//ROS_INFO("TIME: capture: %f", (float)(clock()-start)/CLOCKS_PER_SEC);

    // Convert to IplImage
    left_camera->toIplImage(l);
    right_camera->toIplImage(r);

    // flip images
    if (flip_left_image) {
      if (buffer == NULL) {
        buffer = new unsigned char[ww * hh * 3];
      }
      flip(l_, buffer);
    }

    if (flip_right_image) {
      if (buffer == NULL) {
        buffer = new unsigned char[ww * hh * 3];
      }
      flip(r_, buffer);
    }

    // rectify images
    if (!rectify_images) {
      ROS_ERROR("Images need to be recrified before using ELAS");
      break;
    }
#pragma omp parallel for
    for (int cam = 0; cam <= 1; cam++) {
      if (cam == 0) {
        camera_calibration->RectifyImage(0, ww, hh, l_, -camera_calibration->v_shift);
      }
      else {
        camera_calibration->RectifyImage(1, ww, hh, r_, +camera_calibration->v_shift);
      }
    }

    // histogram equalization
    if (histogram_equalization) {
      if (buffer == NULL) {
        buffer = new unsigned char[ww * hh * 3];
      }
      memcpy((void*)buffer, l_, ww*hh*3);

#pragma omp parallel for
      for (int i = 0; i < 2; i++) {
        unsigned char *img = l_;
        IplImage* hist_image = hist_image0;
        if (i > 0) {
          img = r_;
          hist_image = hist_image1;
        }
        svs::histogram_equalise( hist_image, img, ww, hh );
      }
    }

    ros::spinOnce();
//ROS_INFO("TIME: rectify:   %f", (float)(clock()-start)/CLOCKS_PER_SEC);

    //calculate disparities
    elas_disparity_map(l_, r_, ww, hh, I1, I2, left_disparities, right_disparities, elas);
//ROS_INFO("TIME:    elas:     %f", (float)(clock()-start)/CLOCKS_PER_SEC);
    ros::spinOnce();

    // convert disparity map to 3D points
    pointcloud::disparity_map_to_3d_points(
        left_disparities, ww, hh,
        camera_calibration->disparityToDepth,
        camera_calibration->pose,
        disparity_image, points_image );

    if (!publishing) {
      ROS_INFO("Publishing pointcloud...");
      if (publish_raw_image)
        ROS_INFO("Publishing left raw image...");
      publishing = true;
    }

    unsigned char *img;
    if (histogram_equalization)
      img = buffer;
    else
      img = l_;

    // publish pointcloud
    pointcloud_publish( img, points_image, 5000,
                        camera_calibration->pose,
                        cvmGet(camera_calibration->extrinsicTranslation,0,0) );

    if (publish_raw_image) {
      // Convert to sensor_msgs::Image
      memcpy ((void*)(&raw_image.data[0]), (void*)img, ww*hh*3);
      // publish image
      image_pub.publish(raw_image);
    }
ROS_INFO("TIME: publish:         %f", (float)(clock()-start)/CLOCKS_PER_SEC);

    ros::spinOnce();
  }

  cleanup();
  ROS_INFO("Stereo camera stopped");
}


