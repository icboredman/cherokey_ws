// This program subscribes to cherokey/odomTemp and
// republishes on cherokey/odom.
#include <ros/ros.h>
#include <geometry_msgs/InertiaStamped.h>
#include <nav_msgs/Odometry.h>

ros::Publisher *pubPtr;

void odomTempReceived(const geometry_msgs::InertiaStamped & msgIn)
{
   nav_msgs::Odometry msgOut;

   //msgOut.header.seq = msgIn.header.seq;
   msgOut.header.stamp = msgIn.header.stamp;
   msgOut.header.frame_id = "odom";
   msgOut.child_frame_id = "base_link";
/*
   geometry_msgs::Quaternion Quat = geometry_msgs::Quaternion();
   Quat.x = msgIn.inertia.ixz;
   Quat.y = msgIn.inertia.iyy;
   Quat.z = msgIn.inertia.iyz;
   Quat.w = msgIn.inertia.izz;
*/
   msgOut.pose.pose.position.x = msgIn.inertia.com.x;	// x
   msgOut.pose.pose.position.y = msgIn.inertia.com.y;	// y
   msgOut.pose.pose.position.z = 0.0;
   msgOut.pose.pose.orientation.x = msgIn.inertia.ixz;
   msgOut.pose.pose.orientation.y = msgIn.inertia.iyy;
   msgOut.pose.pose.orientation.z = msgIn.inertia.iyz;
   msgOut.pose.pose.orientation.w = msgIn.inertia.izz;
   msgOut.twist.twist.linear.x = msgIn.inertia.ixx;     // vx
   msgOut.twist.twist.linear.y = 0.0;	// vy
   msgOut.twist.twist.angular.z = msgIn.inertia.ixy;	// vth

   pubPtr->publish(msgOut);
}


int main(int argc, char **argv)
{
   ros::init(argc, argv, "cherokey_odom_remap");
   ros::NodeHandle nh;

   pubPtr = new ros::Publisher( nh.advertise<nav_msgs::Odometry>("odom", 100) );

   ros::Subscriber sub = nh.subscribe("odomTemp", 100, &odomTempReceived);

   ros::spin();

   delete pubPtr;
}
