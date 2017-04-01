// This program subscribes to cherokey/odomTemp and
// republishes on cherokey/odom.
#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

#include <sys/time.h>
#include <time.h>

#include <RadioHead.h>
#include <RHutil/HardwareSerial.h>
#include "base_serial.h"


double cmd_vel_linear_x;
double cmd_vel_angular_z;
bool cmd_vel_received = false;

void callback_cmd_vel(const geometry_msgs::Twist& cmd_vel)
{
  if( ! cmd_vel_received )
  {
    cmd_vel_linear_x = cmd_vel.linear.x;
    cmd_vel_angular_z = cmd_vel.angular.z;
    cmd_vel_received = true;
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "base_serial");
  ros::NodeHandle nh;

  // On Unix we connect to a physical serial port
  // You can override this with RH_HARDWARESERIAL_DEVICE_NAME environment variable
  HardwareSerial hardwareserial("/dev/ttyAMA0");
  Base_Serial uart(hardwareserial);

  uart.serial().begin(115200);
  if( ! uart.init() )
  {
    ROS_ERROR("fail init serial device");
    exit(1);
  }

  ros::Subscriber sub_vel = nh.subscribe("cmd_vel", 10, callback_cmd_vel);

  sensor_msgs::BatteryState bat_msg;
  ros::Publisher pub_bat = nh.advertise<sensor_msgs::BatteryState>("battery", 5);

  ros::Publisher pub_odom = nh.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster odom_broadcaster;

  // populate some constant values
  bat_msg.design_capacity = 2500.0;
  bat_msg.power_supply_technology = sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_NIMH;
  bat_msg.present = true;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ROS_INFO("Base connected over serial");
  bool bat_connected = false;
  bool odom_connected = false;

  while( ros::ok() )
  {

    if( cmd_vel_received )
    {
      uart.drive.speed_mm_s = (int16_t)(cmd_vel_linear_x * 1000.0);
      uart.drive.turn_mrad_s = (int16_t)(cmd_vel_angular_z * 1000.0);
      if( ! uart.sendDrive() )
        ROS_ERROR("sendDrive failed");
      cmd_vel_received = false;
    }

    if( uart.recvPower() )
    {
      bat_msg.current = uart.power.battery_miA / 1023.0;
      bat_msg.voltage = uart.power.battery_miV / 1023.0;
      bat_msg.charge = uart.power.vsupply_miV / 1023.0; // use for VS
      bat_msg.power_supply_status = uart.power.charger_state;
      bat_msg.percentage = uart.power.bat_percentage / 100.0;
      pub_bat.publish(bat_msg);

      if( ! bat_connected )
      {
        ROS_INFO("Base sending BatteryState");
        bat_connected = true;
      }
    }

    if( uart.recvOdom() )
    {
      current_time = ros::Time::now();

      double theta = uart.odom.theta;
      double dx = uart.odom.dx_mm / 1000.0;
      double dth = uart.odom.dth;
      double dt = uart.odom.dt_ms / 1000.0;

      double vx = dx / dt;
      double vth = dth / dt;

      // calculate position while translating to "odom" frame
      static double x, y;
      x += dx * cos(theta);
      y += dx * sin(theta);

      //since all odometry is 6DOF we'll need a quaternion created from yaw
      geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);

      //first, we'll publish the transform over tf
      geometry_msgs::TransformStamped odom_tf;
      odom_tf.header.stamp = current_time;
      odom_tf.header.frame_id = "odom";
      odom_tf.child_frame_id = "base_link";

      odom_tf.transform.translation.x = x;
      odom_tf.transform.translation.y = y;
      odom_tf.transform.translation.z = 0.0;
      odom_tf.transform.rotation = odom_quat;

      //send the transform
      odom_broadcaster.sendTransform(odom_tf);

      //next, we'll publish the odometry message over ROS
      nav_msgs::Odometry odom;
      odom.header.stamp = current_time;
      odom.header.frame_id = "odom";

      //set the position
      odom.pose.pose.position.x = x;
      odom.pose.pose.position.y = y;
      odom.pose.pose.position.z = 0.0;
      odom.pose.pose.orientation = odom_quat;

      //set the velocity
      odom.child_frame_id = "base_link";
      odom.twist.twist.linear.x = vx;
      odom.twist.twist.linear.y = 0.0;
      odom.twist.twist.angular.z = vth;

      //publish the message
      pub_odom.publish(odom);

      if( ! odom_connected )
      {
        ROS_INFO("Base sending tf and odom");
        odom_connected = true;
      }
    }

    ros::spinOnce();
  }

}





/*****************************************************************
 * Copy these function from RHutil/RasPi.cpp to make linker happy
 *****************************************************************/

//Initialize the values for sanity
timeval RHStartTime;

unsigned long millis() {
  //Declare a variable to store current time
  struct timeval RHCurrentTime;
  //Get current time
  gettimeofday(&RHCurrentTime,NULL);
  //Calculate the difference between our start time and the end time
  unsigned long difference = ((RHCurrentTime.tv_sec - RHStartTime.tv_sec) * 1000);
  difference += ((RHCurrentTime.tv_usec - RHStartTime.tv_usec)/1000);
  //Return the calculated value
  return difference;
}

void delay (unsigned long ms) {
  //Implement Delay function
  struct timespec ts;
  ts.tv_sec=0;
  ts.tv_nsec=(ms * 1000);
  nanosleep(&ts,&ts);
}

long random(long min, long max) {
  long diff = max - min;
  long ret = diff * rand() + min;
  return ret;
}
