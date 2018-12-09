/***********************************************************************************
 *  StereoCamera
 *  ROS node to process stereo images from a USB-CDC based stereo camera.
 *  Depending on various configuration options, it can generate and broadcast:
 *    raw image
 *    disparity image
 *    point cloud
 *    simulated laser scan
 *
 *  Based on work of Sourish Ghosh:
 *    https://github.com/umass-amrl/stereo_dense_reconstruction
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

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

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
Mat GetRectificationMap(FileStorage &calib_file, Mat &lx, Mat &ly, Mat &rx, Mat &ry, Size calImgSize, Size outImgSize);
Mat GenerateDisparityImg(Mat &left, Mat &right, int disp_rows);

void PublishImage(image_transport::Publisher &pub, Mat &img, ros::Time &cap_time);
Mat  PublishDisparity(image_transport::Publisher &pub, Mat &disp, ros::Time &cap_time);
void PublishPointCloud(ros::Publisher &pub, Mat &img3D, ros::Time &cap_time, float max_range);
void PublishLaserScan(ros::Publisher &pub, Mat &img3D, ros::Time &cap_time,
                      float cam_center_x, float cam_focal_length, int active_img_height,
                      float scan_range_min, float scan_range_max, bool scan_range_def_inf);


string FindCameraDevice(string hwid);
void* CaptureImages(void* param);

void WriteImage(Mat &img, string filestr, int file_counter);

//typedef pcl::PointCloud<pcl::PointXYZ> PointCloud_t;


#include <signal.h>
#include <sys/ioctl.h>
#include <termios.h>

struct termios cooked, raw;
// quit handler
void quit(int sig)
{
  (void)sig;
  ROS_DEBUG("Restoring keyboard before shutdown");
  tcsetattr(STDIN_FILENO, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}


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

  image_transport::ImageTransport it1(n), it2(n), it3(n), it4(n), it5(n);
  image_transport::Publisher pub_raw_l, pub_raw_r, pub_rect_l, pub_rect_r, pub_disp;
  ros::Publisher pub_cloud, pub_scan;


  if (opt.publish_raw_image) {
    ROS_INFO("Publishing raw images...");
    pub_raw_l = it1.advertise("camera/left/image_raw", 1);
    pub_raw_r = it2.advertise("camera/right/image_raw", 1);
  }

  Mat mapLx, mapLy, mapRx, mapRy; // disparity coordinate maps
  Mat disp2depth; // aka Q

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

    disp2depth = GetRectificationMap(calib_file,
                                     mapLx, mapLy, mapRx, mapRy,
                                     Size(serial::Camera::MAX_IMAGE_WIDTH, serial::Camera::MAX_IMAGE_HEIGHT),
                                     Size(serial::Camera::MAX_IMAGE_WIDTH, opt.camconfig.n_lines));
    //correct -Cy in matrix Q due to different image height
    disp2depth.at<double>(1,3) *= (double)opt.disparity_lines / (double)serial::Camera::MAX_IMAGE_HEIGHT;
    ROS_DEBUG_STREAM("Rectification done");

    if (opt.publish_rectified)
    {
      ROS_INFO("Publishing rectified images...");
      pub_rect_l = it3.advertise("camera/left/image_rect", 1);
      pub_rect_r = it4.advertise("camera/right/image_rect", 1);
    }

    if (opt.publish_disparity)
    {
      ROS_INFO("Publishing disparity image...");
      pub_disp = it5.advertise("camera/disparity", 1);
    }

    if (opt.publish_pointcloud)
    {
      ROS_INFO("Publishing pointcloud...");
      pub_cloud = n.advertise<pcl::PointCloud<pcl::PointXYZ>>("camera/pointcloud", 1);
    }

    if (opt.publish_laserscan)
    {
      ROS_INFO("Publishing laserscan...");
      pub_scan = n.advertise<sensor_msgs::LaserScan>("camera/laserscan", 1);
    }
  }


  if (opt.image_save)
  {
    // register quit handler
    signal(SIGINT,quit);

    // set the console in raw mode
    tcgetattr(STDIN_FILENO, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);

    ROS_INFO("Listening for keypress 's' to save images");
  }
  bool write_image = false;
  int  write_image_counter = -1;


  ros::Time capture_time;
  clock_t clk_start, clk_image, clk_rect=-1, clk_disp=-1, clk_cloud=-1, clk_laser=-1, clk_stop;

  Mat imL(opt.camconfig.n_lines, serial::Camera::MAX_IMAGE_WIDTH, CV_8UC1, Scalar(0));
  Mat imR(opt.camconfig.n_lines, serial::Camera::MAX_IMAGE_WIDTH, CV_8UC1, Scalar(0));

  while (n.ok())
  {
    if (got_camera_frame)
    {
      if (opt.profiling)
        clk_start = clock();
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

      if (opt.publish_raw_image)
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
        PublishImage(pub_raw_l, imL, capture_time);
        PublishImage(pub_raw_r, imR, capture_time);
        if (write_image)
        {
          WriteImage(imL, opt.image_save_path + "raw_left", write_image_counter);
          WriteImage(imR, opt.image_save_path + "raw_right", write_image_counter);
        }
      }

      if (opt.profiling)
        clk_image = clock();

      if (opt.publish_rectified ||
          opt.publish_disparity ||
          opt.publish_laserscan ||
          opt.publish_pointcloud)
      {
        Mat imLrec(imL.size(), imL.type());
        Mat imRrec(imR.size(), imR.type());
        // rectify images
        cv::remap(imL, imLrec, mapLx, mapLy, cv::INTER_LINEAR);
        cv::remap(imR, imRrec, mapRx, mapRy, cv::INTER_LINEAR);

        if (opt.publish_rectified)
        {
          PublishImage(pub_rect_l, imLrec, capture_time);
          PublishImage(pub_rect_r, imRrec, capture_time);
          if (write_image)
          {
            WriteImage(imLrec, opt.image_save_path + "rect_left", write_image_counter);
            WriteImage(imRrec, opt.image_save_path + "rect_right", write_image_counter);
          }
        }

        if (opt.profiling)
          clk_rect = clock();

        if (opt.publish_disparity ||
            opt.publish_pointcloud ||
            opt.publish_laserscan)
        {
          Mat disparity = GenerateDisparityImg(imLrec, imRrec, opt.disparity_lines);

          if (opt.publish_disparity)
          {
            Mat disp_colored = PublishDisparity(pub_disp, disparity, capture_time);
            if (write_image)
              WriteImage(disp_colored, opt.image_save_path + "disparity", write_image_counter);
          }

          if (opt.profiling)
            clk_disp = clock();

          if (opt.publish_pointcloud ||
              opt.publish_laserscan)
          {
            // prepare 3-channel matrix containing reprojected 3D world coordinates
            Mat im3D = cv::Mat::zeros(disparity.size(), CV_32FC3);
            // convert disparity to array of 3D points
            cv::reprojectImageTo3D(disparity, im3D, disp2depth);

            if (opt.publish_pointcloud)
            {
              PublishPointCloud(pub_cloud, im3D, capture_time, opt.cloud_range_max);
            }

            if (opt.profiling)
              clk_cloud = clock();

            if (opt.publish_laserscan)
            {
              PublishLaserScan( pub_scan, im3D, capture_time,
                                -disp2depth.at<double>(0,3), disp2depth.at<double>(2,3), opt.scan_used_img_height,
                                opt.scan_range_min, opt.scan_range_max, opt.scan_range_def_infinity );

              if (opt.profiling)
                clk_laser = clock();
            }
          }
        }
      }

      if (opt.image_save)
      {
        write_image = false;

        int bytesWaiting = 0;
        ioctl(STDIN_FILENO, FIONREAD, &bytesWaiting);
        if (bytesWaiting > 0)
        {
          char c;
          // get the next event from the keyboard
          if (read(STDIN_FILENO, &c, 1) < 0)
          {
            ROS_DEBUG("ERROR: read()");   //perror("read():");
            exit(-1);
          }
          if (c == 's')
          {
            write_image = true;
            write_image_counter++;
          }
        }
      }

      if (opt.profiling)
      {
        clk_stop = clock();
        static clock_t clk_last_stop = -1;

        clock_t clk_image_ = clk_start;
        clock_t clk_rect_ = clk_image;
        clock_t clk_disp_ = (clk_rect != -1) ? clk_rect : clk_rect_;
        clock_t clk_cloud_ = (clk_disp != -1) ? clk_disp : clk_disp_;
        clock_t clk_laser_ = (clk_cloud != -1) ? clk_cloud : clk_cloud_;

        ROS_INFO( "TIME[ms]: image=%.2f rect=%.2f disp=%.2f cloud=%.2f laser=%.2f TOTAL=%.2f FPS=%.1f",
                  (float)(clk_image - clk_image_) / (CLOCKS_PER_SEC / 1000),
                  (clk_rect != -1) ? (float)(clk_rect - clk_rect_) / (CLOCKS_PER_SEC / 1000) : NAN,
                  (clk_disp != -1) ? (float)(clk_disp - clk_disp_) / (CLOCKS_PER_SEC / 1000) : NAN,
                  (clk_cloud != -1) ? (float)(clk_cloud - clk_cloud_) / (CLOCKS_PER_SEC / 1000) : NAN,
                  (clk_laser != -1) ? (float)(clk_laser - clk_laser_) / (CLOCKS_PER_SEC / 1000) : NAN,
                  (float)(clk_stop - clk_start) / (CLOCKS_PER_SEC / 1000),
                  (clk_last_stop != -1) ? CLOCKS_PER_SEC / (float)(clk_stop - clk_last_stop) : NAN);
        clk_last_stop = clk_stop;
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
	cv::equalizeHist(mSrc, mDst);

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


Mat GetRectificationMap(FileStorage &calib_file, Mat &lx, Mat &ly, Mat &rx, Mat &ry, Size calImgSize, Size outImgSize)
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
		Q, CV_CALIB_ZERO_DISPARITY, calImgSize, 0.0, 1.0);
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

  // return disparity-to-depth mapping matrix
  return Q;
}


// disp_rows <-- crop source images from top and bottom to this many rows
Mat GenerateDisparityImg(Mat &left, Mat &right, int disp_rows)
{
  if (left.empty() || right.empty())
    return left;

  assert(disp_rows <= left.rows);

  const Size disp_size = cv::Size(left.cols, disp_rows);
  const int32_t dims[3] = { disp_size.width,
                            disp_size.height,
                            disp_size.width };
  Mat leftdpf = Mat::zeros(disp_size, CV_32F);
  Mat rightdpf = Mat::zeros(disp_size, CV_32F);

  Elas::parameters param(Elas::ROBOTICS);
  //param.support_threshold = 0.85;
  //param.speckle_size = 200;
  param.ipol_gap_width = 40;
  Elas elas(param);
  elas.process( left.ptr<uchar>((left.rows - disp_rows) / 2),
                right.ptr<uchar>((left.rows - disp_rows) / 2),
                leftdpf.ptr<float>(0),
                rightdpf.ptr<float>(0),
                dims );
  return leftdpf;
}


void PublishImage(image_transport::Publisher &pub, Mat &img, ros::Time &cap_time)
{
  std_msgs::Header header = std_msgs::Header();
  header.stamp = cap_time;
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header,
                                                 sensor_msgs::image_encodings::MONO8,
                                                 img).toImageMsg();
  pub.publish(msg);
}


Mat PublishDisparity(image_transport::Publisher &pub, Mat &disp, ros::Time &cap_time)
{
  // scale disparity image values for better visualization
  double min;
  double max;
  cv::minMaxIdx(disp, &min, &max);
  Mat disp_scaled;
  cv::convertScaleAbs(disp, disp_scaled, 255 / (max - min));

  // colorize disparity image
  Mat disp_colored;
  cv::applyColorMap(disp_scaled, disp_colored, COLORMAP_JET);

  // publish disparity image
  std_msgs::Header header = std_msgs::Header();
  header.stamp = cap_time;
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header,
                                                 sensor_msgs::image_encodings::BGR8,
                                                 disp_colored).toImageMsg();
  pub.publish(msg);

  return disp_colored;
}


void PublishPointCloud(ros::Publisher &pub, Mat &img3D, ros::Time &cap_time, float max_range)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.header.frame_id = "camera_frame";
  pcl_conversions::toPCL(cap_time, cloud.header.stamp);
  cloud.points.resize(0);
  //cloud.is_dense = true;

  float *points_image_data = (float*)img3D.data;
  int pixels = img3D.rows * img3D.cols;
  pcl::PointXYZ point;
  for (int i = 0; i < pixels*3; i+=3)
  {
    point.y = -points_image_data[i];
    point.z = -points_image_data[i+1];
    point.x = points_image_data[i+2];

    if ( point.x > 0 &&
        (fabs(point.x)+fabs(point.y)+fabs(point.z) > 0.001) &&
        (fabs(point.x) < max_range) &&
        (fabs(point.y) < max_range) &&
        (fabs(point.z) < max_range) )
    {
      cloud.points.push_back(point);
    }
  }

  if (cloud.points.size() > 0)
  {
    cloud.height = 1;
    cloud.width = cloud.points.size();
    pub.publish(cloud);
  }
}


void PublishLaserScan(ros::Publisher &pub, Mat &img3D, ros::Time &cap_time,
                      float cam_center_x, float cam_focal_length, int active_img_height,
                      float scan_range_min, float scan_range_max, bool scan_range_def_inf)
{
  //determine amount of rays to create
  uint32_t ranges_size = img3D.cols;

  float scan_angle_min = atan2(cam_center_x-ranges_size, cam_focal_length);
  float scan_angle_max = atan2(cam_center_x, cam_focal_length);
  float scan_angle_increment = (scan_angle_max - scan_angle_min) / ranges_size;

  sensor_msgs::LaserScan scan;

  scan.header.stamp = cap_time;
  scan.header.frame_id = "camera_frame";
  scan.angle_min = scan_angle_min;
  scan.angle_max = scan_angle_max;
  scan.angle_increment = scan_angle_increment;
  scan.time_increment = 0.0;
  scan.range_min = scan_range_min;
  scan.range_max = scan_range_max;

  if (scan_range_def_inf)
    scan.ranges.assign(ranges_size, std::numeric_limits<double>::infinity());
  else
    scan.ranges.assign(ranges_size, scan_range_max - 0.01);

  scan.intensities.resize(0);

  uint width = img3D.cols;
  vector<double> ranges[width];

  assert(active_img_height <= img3D.rows);
  int start_row = (img3D.rows - active_img_height) / 2;
  int stop_row = start_row + active_img_height;

  for (int h = start_row; h < stop_row; h++)
  {
    // get pointer to row
    const float *p_row = img3D.ptr<float>(h);
    // access elements in that row
    for (uint w = 0; w < width*3; w += 3)
    {
      float y = p_row[w];
      float z = p_row[w+1];
      float x = p_row[w+2];

      double range = hypot(y,x);
      if (range < scan_range_min)
        continue;
      
      // sign in front of atan2 controls the scan direction (pos = CW; neg = CCW)
      double angle = -atan2(y,x);
      if (angle < scan_angle_min || angle > scan_angle_max)
        continue;
      
      uint index = (angle - scan_angle_min) / scan_angle_increment;
      if (index >= width)
        index = width - 1;
      ranges[index].push_back(range);
    }
  }

  for (int i = 0; i < img3D.cols; i++)
  {
    int length = ranges[i].size();
    if (length == 0)
      scan.ranges[i] = scan_range_def_inf ? std::numeric_limits<double>::infinity() : scan_range_max - 0.01;
    else
    { // median works better than mean as it is less influenced by outliers
      std::sort(ranges[i].begin(), ranges[i].end());
      double median = ranges[i].at(length/2);
//    double mean = std::accumulate(ranges[i].begin(), ranges[i].end(), 0.0) / ranges[i].size();
      scan.ranges[i] = median;
    }
  }

  pub.publish(scan);

}



/** @brief Writes the given image as .PNG file.
 *  @param img Mat object containing image
 *  @param filestr filename including full path
 *  @param file_counter will be appended to pathname
 *  @return Void.
 */
void WriteImage(Mat &img, string filestr, int file_counter)
{
  string filename = filestr + "_" + to_string(file_counter) + ".png";
  ROS_DEBUG_STREAM("Writing image to: " << filename);
  cv::imwrite(filename, img);
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
