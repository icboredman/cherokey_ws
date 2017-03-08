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
sensor_msgs::Image left_image;
sensor_msgs::Image right_image;

IplImage *l=NULL;
IplImage *r=NULL;
unsigned char *l_=NULL;
unsigned char *r_=NULL;
unsigned char * buffer=NULL;

unsigned char *l_bak_=NULL;
unsigned char *r_bak_=NULL;

bool flip_left_image=false;
bool flip_right_image=false;

std::string calibration_filename = "calibration.txt";

std::string left_camera_filename = "left_camera.txt";
std::string right_camera_filename = "right_camera.txt";
std::string stereo_camera_filename = "stereo_camera.txt";

image_transport::Publisher left_pub, right_pub;

ros::Publisher left_camera_info_pub;
sensor_msgs::CameraInfo left_camera_info;

ros::Publisher right_camera_info_pub;
sensor_msgs::CameraInfo right_camera_info;

ros::Publisher stereo_camera_info_pub;
sensor_msgs::CameraInfo stereo_camera_info;

ros::ServiceServer set_cam_info_left;
ros::ServiceServer set_cam_info_right;
ros::ServiceServer set_cam_info;

camcalib *camera_calibration=NULL;
bool rectify_images = false;

uint8_t * I1 = NULL;
uint8_t * I2 = NULL;
float * left_disparities = NULL;
float * right_disparities = NULL;
Elas * elas = NULL;

IplImage* hist_image0 = NULL;
IplImage* hist_image1 = NULL;


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


bool load_camera_info(std::string filename, sensor_msgs::CameraInfo &info)
{
  FILE * fp;
  void * ptr = (void*)&info;

  fp = fopen(filename.c_str(),"rb");
  if (fp!=NULL) {
    fread (ptr, 1 , sizeof(sensor_msgs::CameraInfo) , fp);
    fclose(fp);
    ROS_INFO("Loaded %s",filename.c_str());
    return true;
  }

  return false;
}

bool save_camera_info(std::string filename, sensor_msgs::CameraInfo &info)
{
  FILE * fp;
  void * ptr = (void*)&info;

  fp = fopen(filename.c_str(),"wb");
  if (fp!=NULL) {
    fwrite (ptr, 1 , sizeof(sensor_msgs::CameraInfo) , fp);
    fclose(fp);
    ROS_INFO("Saved %s",filename.c_str());
    return true;
  }

  return false;
}

bool set_camera_info_left(
                          sensor_msgs::SetCameraInfo::Request &req,
                          sensor_msgs::SetCameraInfo::Response &res)
{
  ROS_INFO("Set camera info left");
  left_camera_info = req.camera_info;
  res.status_message = "";
  res.success = true;
  return save_camera_info(left_camera_filename,left_camera_info);
}

bool set_camera_info_right(
                           sensor_msgs::SetCameraInfo::Request &req,
                           sensor_msgs::SetCameraInfo::Response &res)
{
  ROS_INFO("Set camera info right");
  right_camera_info = req.camera_info;
  res.status_message = "";
  res.success = true;
  return save_camera_info(right_camera_filename,right_camera_info);
}

bool set_camera_info(
                     sensor_msgs::SetCameraInfo::Request &req,
                     sensor_msgs::SetCameraInfo::Response &res)
{
  ROS_INFO("Set camera info");
  stereo_camera_info = req.camera_info;
  res.success = true;
  res.status_message = "";
  return save_camera_info(stereo_camera_filename,stereo_camera_info);
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

  ros::NodeHandle n;
  image_transport::ImageTransport it(n);

  std::string topic_str = "stereocamera/left/image_raw";
  left_pub = it.advertise(topic_str.c_str(), 1);

  topic_str = "stereocamera/right/image_raw";
  right_pub = it.advertise(topic_str, 1);

  left_image.width  = width;
  left_image.height = height;
  left_image.step = width * 3;
  left_image.encoding = "bgr8";
  left_image.data.resize(width*height*3);

  right_image.width  = width;
  right_image.height = height;
  right_image.step = width * 3;
  right_image.encoding = "bgr8";
  right_image.data.resize(width*height*3);

  l = cvCreateImage(cvSize(width, height), 8, 3);
  r = cvCreateImage(cvSize(width, height), 8, 3);

  l_=(unsigned char *)l->imageData;
  r_=(unsigned char *)r->imageData;

  left_camera = new Camera(dev_left.c_str(), width, height, fps);
  right_camera = new Camera(dev_right.c_str(), width, height, fps);

  set_cam_info_left = n.advertiseService("stereocamera/left/set_camera_info", set_camera_info_left);
  set_cam_info_right = n.advertiseService("stereocamera/right/set_camera_info", set_camera_info_right);
  set_cam_info = n.advertiseService("stereocamera/set_camera_info", set_camera_info);

  left_camera_info_pub = n.advertise<sensor_msgs::CameraInfo>("stereocamera/left/camera_info", 1);
  right_camera_info_pub = n.advertise<sensor_msgs::CameraInfo>("stereocamera/right/camera_info", 1);
  stereo_camera_info_pub = n.advertise<sensor_msgs::CameraInfo>("stereocamera/camera_info", 1);

  load_camera_info(left_camera_filename, left_camera_info);
  load_camera_info(right_camera_filename, right_camera_info);
  load_camera_info(stereo_camera_filename, stereo_camera_info);

  camera_calibration = new camcalib();
  camera_calibration->ParseCalibrationFile(calibration_filename);
  rectify_images = camera_calibration->rectification_loaded;

  hist_image0 = cvCreateImage( cvGetSize(l), IPL_DEPTH_8U, 1 );
  hist_image1 = cvCreateImage( cvGetSize(l), IPL_DEPTH_8U, 1 );
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



void cleanup()
{
  cvReleaseImage(&l);
  cvReleaseImage(&r);

  if (hist_image0 != NULL) cvReleaseImage(&hist_image0);
  if (hist_image1 != NULL) cvReleaseImage(&hist_image1);

  if (elas != NULL) delete elas;
  delete camera_calibration;

  if (buffer != NULL) delete[] buffer;

  stop_cameras(left_camera,right_camera);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "stereocamera_broadcast");
  ros::NodeHandle nh("~");

  // set some default values
  int val=0;
  std::string str="";
  dev_left = "/dev/video1";
  dev_right = "/dev/video0";
  fps = 30;

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

  nh.getParam("left_camera_filename", str);
  if (str!="") left_camera_filename=str;
  nh.getParam("right_camera_filename", str);
  if (str!="") right_camera_filename=str;
  nh.getParam("stereo_camera_filename", str);
  if (str!="") stereo_camera_filename=str;

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

/*
    // histogram equalization
    if (l_bak_ == NULL) {
      l_bak_ = new unsigned char [ww*hh*3];
      r_bak_ = new unsigned char [ww*hh*3];
    }
    memcpy((void*)l_bak_, l_, ww*hh*3);
    memcpy((void*)r_bak_, r_, ww*hh*3);

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
*/
    ros::spinOnce();
//ROS_INFO("TIME: rectify:   %f", (float)(clock()-start)/CLOCKS_PER_SEC);

    //calculate disparities
    elas_disparity_map(l_, r_, ww, hh, I1, I2, left_disparities, right_disparities, elas);
//ROS_INFO("TIME:    elas:     %f", (float)(clock()-start)/CLOCKS_PER_SEC);
    ros::spinOnce();

    int min_disparity = 0;
    // colour coded disparities
    for (int i = 0; i < ww*hh; i++) {
      if (left_disparities[i] > min_disparity) {
        float val = min(( *(((float*)left_disparities)+i) )*0.01f,1.0f);
        if (val <= 0) {
          l_[3*i+0] = 0;
          l_[3*i+1] = 0;
          l_[3*i+2] = 0;
        } else {
          float h2 = 6.0f * (1.0f - val);
          unsigned char x  = (unsigned char)((1.0f - fabs(fmod(h2, 2.0f) - 1.0f))*255);
          if (0 <= h2&&h2<1) {
            l_[3*i+0] = 255;
            l_[3*i+1] = x;
            l_[3*i+2] = 0;
          }
          else if (1<=h2&&h2<2)  {
            l_[3*i+0] = x;
            l_[3*i+1] = 255;
            l_[3*i+2] = 0;
          }
          else if (2<=h2&&h2<3)  {
            l_[3*i+0] = 0;
            l_[3*i+1] = 255;
            l_[3*i+2] = x;
          }
          else if (3<=h2&&h2<4)  {
            l_[3*i+0] = 0;
            l_[3*i+1] = x;
            l_[3*i+2] = 255;
          }
          else if (4<=h2&&h2<5)  {
            l_[3*i+0] = x;
            l_[3*i+1] = 0;
            l_[3*i+2] = 255;
          }
          else if (5<=h2&&h2<=6) {
            l_[3*i+0] = 255;
            l_[3*i+1] = 0;
            l_[3*i+2] = x;
          }
        }
      }
      else {
        l_[3*i+0] = 0;
        l_[3*i+1] = 0;
        l_[3*i+2] = 0;
      }
    }
//ROS_INFO("TIME:colorize:       %f", (float)(clock()-start)/CLOCKS_PER_SEC);

/*
    //calculate disparities
    elas_disparity_map(l_bak_, r_bak_, ww, hh, I1, I2, left_disparities, right_disparities, elas);
//ROS_INFO("TIME:    elas:     %f", (float)(clock()-start)/CLOCKS_PER_SEC);
    ros::spinOnce();

    min_disparity = 0;
    // colour coded disparities
    for (int i = 0; i < ww*hh; i++) {
      if (left_disparities[i] > min_disparity) {
        float val = min(( *(((float*)left_disparities)+i) )*0.01f,1.0f);
        if (val <= 0) {
          l_bak_[3*i+0] = 0;
          l_bak_[3*i+1] = 0;
          l_bak_[3*i+2] = 0;
        } else {
          float h2 = 6.0f * (1.0f - val);
          unsigned char x  = (unsigned char)((1.0f - fabs(fmod(h2, 2.0f) - 1.0f))*255);
          if (0 <= h2&&h2<1) {
            l_bak_[3*i+0] = 255;
            l_bak_[3*i+1] = x;
            l_bak_[3*i+2] = 0;
          }
          else if (1<=h2&&h2<2)  {
            l_bak_[3*i+0] = x;
            l_bak_[3*i+1] = 255;
            l_bak_[3*i+2] = 0;
          }
          else if (2<=h2&&h2<3)  {
            l_bak_[3*i+0] = 0;
            l_bak_[3*i+1] = 255;
            l_bak_[3*i+2] = x;
          }
          else if (3<=h2&&h2<4)  {
            l_bak_[3*i+0] = 0;
            l_bak_[3*i+1] = x;
            l_bak_[3*i+2] = 255;
          }
          else if (4<=h2&&h2<5)  {
            l_bak_[3*i+0] = x;
            l_bak_[3*i+1] = 0;
            l_bak_[3*i+2] = 255;
          }
          else if (5<=h2&&h2<=6) {
            l_bak_[3*i+0] = 255;
            l_bak_[3*i+1] = 0;
            l_bak_[3*i+2] = x;
          }
        }
      }
      else {
        l_bak_[3*i+0] = 0;
        l_bak_[3*i+1] = 0;
        l_bak_[3*i+2] = 0;
      }
    }
//ROS_INFO("TIME:colorize:       %f", (float)(clock()-start)/CLOCKS_PER_SEC);
*/

    if (!publishing) {
      ROS_INFO("Publishing stereo images...");
      publishing = true;
    }
    // Convert to sensor_msgs::Image
    memcpy ((void*)(&left_image.data[0]), (void*)l_, ww*hh*3);
    memcpy ((void*)(&right_image.data[0]), (void*)r_, ww*hh*3);
    // publish
    left_pub.publish(left_image);
    right_pub.publish(right_image);

    left_camera_info_pub.publish(left_camera_info);
    right_camera_info_pub.publish(right_camera_info);
    stereo_camera_info_pub.publish(stereo_camera_info);
ROS_INFO("TIME: publish:         %f", (float)(clock()-start)/CLOCKS_PER_SEC);

    ros::spinOnce();
  }

/*
  delete l_bak_;
  delete r_bak_;
*/
  cleanup();
  ROS_INFO("Stereo camera stopped");
}


