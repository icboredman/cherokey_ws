#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>



void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv::imshow("image", cv_bridge::toCvShare(msg, "bgr8")->image);
    ROS_INFO("Received frame: %s", msg->header.frame_id.c_str());
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "show_image");
  ros::NodeHandle nh;

  cv::namedWindow("image", 0);

  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("stereocamera/left/image_raw", 1, imageCallback);
  ros::spin();
  cv::destroyWindow("image");
}

