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

//#include <opencv2/contrib/contrib.hpp>
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
#include "elas/elas.h"

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

//Mat imageR(serial::Camera::MAX_IMAGE_HEIGHT, serial::Camera::MAX_IMAGE_WIDTH, CV_8UC1, Scalar(0));
//Mat imageL(serial::Camera::MAX_IMAGE_HEIGHT, serial::Camera::MAX_IMAGE_WIDTH, CV_8UC1, Scalar(0));
Mat imageR, imageL;

bool got_camera_frame = false;


void ZoomCenter(Mat& image, int size, int factor);
void EqualizeHistogram1(Mat& image, int imgLines);
void EqualizeHistogram2(Mat& image, int imgLines);
void DrawHistogram(Mat& image, int imgLines);
void GetRectificationMap(FileStorage &calib_file, Mat &lx, Mat &ly, Mat &rx, Mat &ry, Size calImgSize, Size outImgSize);
Mat GenerateDisparityImg(Mat &left, Mat &right, Size out_img_size);

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

  image_transport::ImageTransport it1(n), it2(n), it3(n);
  image_transport::Publisher image_pub_l, image_pub_r, image_pub_disp;
  ros::Publisher cloud_pub, scan_pub;


  if (opt.publish_raw_image) {
    image_pub_l = it1.advertise("camera/left/image", 1);
    image_pub_r = it2.advertise("camera/right/image", 1);
  }

  Mat map_lx, map_ly, map_rx, map_ry;

  if (opt.publish_rectified ||
      opt.publish_disparity ||
      opt.publish_laserscan ||
      opt.publish_pointcloud)
  {
    FileStorage calib_file = FileStorage(opt.calibration_filename, FileStorage::READ);
    if (!calib_file.isOpened())
    {
      ROS_ERROR_STREAM("Unable to open calibration file: " << opt.calibration_filename);
      exit(2);
    }
    ROS_DEBUG_STREAM("Calibration file opened");

    GetRectificationMap(calib_file,
                        map_lx, map_ly, map_rx, map_ry,
                        Size(serial::Camera::MAX_IMAGE_WIDTH, serial::Camera::MAX_IMAGE_HEIGHT),
                        Size(serial::Camera::MAX_IMAGE_WIDTH, opt.camconfig.n_lines));
    ROS_DEBUG_STREAM("Rectification done");

    if (opt.publish_rectified)
    {
    }

    if (opt.publish_disparity)
    {
      image_pub_disp = it3.advertise("camera/disparity", 1);
    }

    if (opt.publish_pointcloud)
    {
      //    cloud_pub = n.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("pointcloud", 1);
    }

    if (opt.publish_laserscan)
    {
      //    scan_pub = n.advertise<sensor_msgs::LaserScan>("laserscan", 1);
    }
  }

  if (opt.publish_rectified)
    ROS_INFO("Publishing rectified images...");
  else if (opt.publish_raw_image)
    ROS_INFO("Publishing raw images...");

  if (opt.publish_disparity)
    ROS_INFO("Publishing disparity image...");
  if (opt.publish_pointcloud)
    ROS_INFO("Publishing pointcloud...");
  if (opt.publish_laserscan)
    ROS_INFO("Publishing laserscan...");

  ros::Time capture_time;
  clock_t start;

  Mat imR(opt.camconfig.n_lines, serial::Camera::MAX_IMAGE_WIDTH, CV_8UC1, Scalar(0));
  Mat imL(opt.camconfig.n_lines, serial::Camera::MAX_IMAGE_WIDTH, CV_8UC1, Scalar(0));

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

      if (opt.histogram_equalization)
      {
        EqualizeHistogram2(imL, opt.camconfig.n_lines);
        EqualizeHistogram2(imR, opt.camconfig.n_lines);
      }

      if (opt.publish_rectified ||
          opt.publish_disparity ||
          opt.publish_laserscan ||
          opt.publish_pointcloud)
      {
        remap(imL, imL, map_lx, map_ly, cv::INTER_LINEAR);
        remap(imR, imR, map_rx, map_ry, cv::INTER_LINEAR);
      }

      if (opt.publish_rectified || opt.publish_raw_image)
      {
        if (opt.histogram_show)
        {
          DrawHistogram(imR, opt.camconfig.n_lines);
          DrawHistogram(imL, opt.camconfig.n_lines);
        }
        if (opt.zoom_center)
        {
          ZoomCenter(imR, 100, 4);
          ZoomCenter(imL, 100, 4);
        }
        //img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
        //pub_img.publish(img_msg); // ros::Publisher pub_img = node.advertise<sensor_msgs::Image>("topic", queuesize);
        // Convert to sensor_msgs::Image
        std_msgs::Header header = std_msgs::Header();
        header.stamp = capture_time;
        // publish left image
        sensor_msgs::ImagePtr msgL = cv_bridge::CvImage(header,
                                                        sensor_msgs::image_encodings::MONO8,
                                                        imL).toImageMsg();
        image_pub_l.publish(msgL);
        // publish right image
        sensor_msgs::ImagePtr msgR = cv_bridge::CvImage(header,
                                                        sensor_msgs::image_encodings::MONO8,
                                                        imR).toImageMsg();
        image_pub_r.publish(msgR);
        //memcpy ((void*)(&raw_image.data[0]), (void*)img, opt.ww*opt.hh*3);
        //image_pub.publish(raw_image);
      }

      if (opt.publish_disparity)
      {
        //cvtColor(img_left, img_left_color, CV_GRAY2BGR);
        Mat disp = GenerateDisparityImg(imL, imR, Size(serial::Camera::MAX_IMAGE_WIDTH, serial::Camera::MAX_IMAGE_HEIGHT));
        // scale disparity image values for better visualization
        double min;
        double max;
        minMaxIdx(disp, &min, &max);
        Mat disp_scaled;
        convertScaleAbs( disp, disp_scaled, 255 / ( max - min ) );
        // colorize disparity image
        Mat disp_colored;
        applyColorMap(disp_scaled, disp_colored, COLORMAP_JET);
        // publish disparity image
        std_msgs::Header header = std_msgs::Header();
        header.stamp = capture_time;
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header,
                                                       sensor_msgs::image_encodings::BGR8,
                                                       disp_colored).toImageMsg();
        image_pub_disp.publish(msg);
      }

      if (opt.profiling)
      {
        ROS_INFO("TIME: total=%f capture=%f cap_cntr=%ld", ((float)(clock()-start))/CLOCKS_PER_SEC, 0.0, 0L);
      }

    }
    ros::spinOnce();
  }

  return 0;
}


void ZoomCenter(Mat& image, int size, int factor)
{
  Mat region(size, size, CV_8UC1, Scalar(0));

	for (int y = 0; y < size; y++)
		for (int x = 0; x < size; x++)
			region.at<uint8_t>(y, x) = image.at<uint8_t>(image.rows/2 - size/2/factor + y/factor, image.cols/2 - size/2/factor + x/factor);

	for (int y = 0; y < size; y++)
		for (int x = 0; x < size; x++)
			image.at<uint8_t>(image.rows/2 - size/2 + y, image.cols/2 - size/2 + x) = region.at<uint8_t>(y, x);
}

void EqualizeHistogram1(Mat& image, int imgLines)
{
	Mat mSrc(imgLines, image.cols, CV_8UC1, Scalar(0));
	// limit height and copy:
	for (int y = 0; y < imgLines; y++)
		for (int x = 0; x < image.cols; x++)
			mSrc.at<uint8_t>(y, x) = image.at<uint8_t>(y + (image.rows - imgLines) / 2, x);

	// do equalization:
	Mat mDst(imgLines, image.cols, CV_8UC1, Scalar(0));
	equalizeHist(mSrc, mDst);

	// copy back:
	for (int y = 0; y < imgLines; y++)
		for (int x = 0; x < image.cols; x++)
			image.at<uint8_t>(y + (image.rows - imgLines) / 2, x) = mDst.at<uint8_t>(y, x);
}

void EqualizeHistogram2(Mat& image, int imgLines)
{
	Mat mSrc(imgLines, image.cols, CV_8UC1, Scalar(0));
	// limit height and copy:
	for (int y = 0; y < imgLines; y++)
		for (int x = 0; x < image.cols; x++)
			mSrc.at<uint8_t>(y, x) = image.at<uint8_t>(y + (image.rows - imgLines) / 2, x);

	// do equalization:
	Mat mDst;
	Ptr<CLAHE> clahe = createCLAHE();
	clahe->setClipLimit(2);
	clahe->apply(mSrc, mDst);

	// copy back:
	for (int y = 0; y < imgLines; y++)
		for (int x = 0; x < image.cols; x++)
			image.at<uint8_t>(y + (image.rows - imgLines) / 2, x) = mDst.at<uint8_t>(y, x);

}

void DrawHistogram(Mat& image, int imgLines)
{
	/// Establish the number of bins
	int hist_bins = 256;

	/// Set the ranges:
	float range[] = { 0, 256 };	// 8-bit image
	const float* histRange = { range };

	// histogram dimensions:
	int hist_w = image.cols;
	int hist_h = image.rows;
	int bin_w = cvRound((double)hist_w / hist_bins);

	// configure mask:
	Mat mMask(hist_h, hist_w, CV_8UC1, Scalar(0));
	for (int y = hist_h / 2 - imgLines / 2; y < hist_h / 2 + imgLines / 2; y++)
		for (int x = 0; x < hist_w; x++)
			mMask.at<uint8_t>(y, hist_w - x - 1) = 255;

	Mat mHist;
	/// Compute the histogram:
	calcHist(&image, 1, 0, mMask, mHist, 1, &hist_bins, &histRange, true, false);

	/// Normalize amplitudes to [ 0, 128 ]
	normalize(mHist, mHist, 0, 128, NORM_MINMAX, -1, Mat());	// hist_h

	/// Draw histogram as white line:
	for (int i = 1; i < hist_bins; i++)
		line(image, Point(bin_w*(i - 1), hist_h - cvRound(mHist.at<float>(i - 1))),
			Point(bin_w*(i), hist_h - cvRound(mHist.at<float>(i))),
			255, 2, 8, 0);

}


void GetRectificationMap(FileStorage &calib_file, Mat &lx, Mat &ly, Mat &rx, Mat &ry, Size calImgSize, Size outImgSize)
{
  Mat XR, XT, Q, P1, P2;
  Mat R1, R2, K1, K2, R;
  Vec4d D1, D2;
  Vec3d T;

  calib_file["K1"] >> K1;
  calib_file["K2"] >> K2;
  calib_file["D1"] >> D1;
  calib_file["D2"] >> D2;
  calib_file["R"] >> R;
  calib_file["T"] >> T;
  calib_file["XR"] >> XR;
  calib_file["XT"] >> XT;

	cv::fisheye::stereoRectify(K1, D1, K2, D2, calImgSize, R, T, R1, R2, P1, P2,
		Q, CV_CALIB_ZERO_DISPARITY, calImgSize, 0.0, 1.1);
  // use temporary map images initially
  Mat clx, cly, crx, cry;
	cv::fisheye::initUndistortRectifyMap(K1, D1, R1, P1, calImgSize, CV_32FC1, clx, cly);
	cv::fisheye::initUndistortRectifyMap(K2, D2, R2, P2, calImgSize, CV_32FC1, crx, cry);

  // allocate output map images
  lx.create(outImgSize.height, outImgSize.width, CV_32FC1);
  ly.create(outImgSize.height, outImgSize.width, CV_32FC1);
  rx.create(outImgSize.height, outImgSize.width, CV_32FC1);
  ry.create(outImgSize.height, outImgSize.width, CV_32FC1);

  // copy to output maps while compensating for vertical crop
  int offs = (calImgSize.height - outImgSize.height) / 2;

  for (int y=0; y<outImgSize.height; y++)
  {
    for (int x=0; x<outImgSize.width; x++)
    {
      lx.at<float>(y,x) = clx.at<float>(y+offs,x);
      ly.at<float>(y,x) = cly.at<float>(y+offs,x) - (float)offs;
      rx.at<float>(y,x) = crx.at<float>(y+offs,x);
      ry.at<float>(y,x) = cry.at<float>(y+offs,x) - (float)offs;
    }
  }
}


Mat GenerateDisparityImg(Mat &left, Mat &right, Size out_img_size)
{
  if (left.empty() || right.empty())
    return left;
  const Size imsize = left.size();
  const int32_t dims[3] = { imsize.width, imsize.height, imsize.width };
  Mat leftdpf = Mat::zeros(imsize, CV_32F);
  Mat rightdpf = Mat::zeros(imsize, CV_32F);

  Elas::parameters param(Elas::MIDDLEBURY);
  param.postprocess_only_left = true;
  Elas elas(param);
  elas.process(left.data, right.data, leftdpf.ptr<float>(0), rightdpf.ptr<float>(0), dims);
  Mat dmap = Mat(out_img_size, CV_8UC1, Scalar(0));
  leftdpf.convertTo(dmap, CV_8U, 1.);
  return dmap;
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
  serial::Camera camera(port, 115200, serial::Timeout(2,10,0,250,0));
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

  Mat imgR(opt->camconfig.n_lines, camera.MAX_IMAGE_WIDTH, CV_8UC1, Scalar(0));
  Mat imgL(opt->camconfig.n_lines, camera.MAX_IMAGE_WIDTH, CV_8UC1, Scalar(0));

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
