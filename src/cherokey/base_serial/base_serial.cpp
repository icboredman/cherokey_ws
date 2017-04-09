// =============================================================================
// This program enables serial communication with slave base microcontroller and
// publishes odometry and tf messages over ROS.
// In addition, it subscribes to cmd_vel messages and retransmitts them to slave.
//
// Copyright (c) 2017 boredman <http://BoredomProjects.net>
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//
// =============================================================================

#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

#include <sys/time.h>
#include <time.h>

#include "serial/serial.h"
#include "MessageSerial.h"


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

  // configure hardware serial port
  serial::Serial uart("/dev/ttyAMA0", 115200, serial::Timeout(0,0,0,250,0));

  // MessageSerial object will use the above port
  MessageSerial serial(uart);

  // define actual messages and create corresponding Message objects
  typedef struct Power {
    uint16_t battery_miV;
    int16_t  battery_miA;
    uint16_t vsupply_miV;
    uint8_t  charger_state;
    uint8_t  bat_percentage;
  } tPower;
  // Message objects should reference the above MessageSerial object
  Message<tPower,2> power(serial);

  typedef struct {
    float theta;
    float dx_mm;
    float dth;
    uint16_t dt_ms;
  } tOdom;
  Message<tOdom,5> odom(serial);

  typedef struct {
    int16_t speed_mm_s;
    int16_t turn_mrad_s;
  } tDrive;
  Message<tDrive,7> drive(serial);

  typedef struct {
    char str[100];
  } tStr;
  Message<tStr,1> text(serial); // message id 1 is reserved for character string messages

  if( ! uart.isOpen() )
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

  ros::Time last_recv_time = ros::Time::now();
  ros::Duration conn_lost_duration(5);	//sec
  bool conn_lost_timeout = false;
  bool bat_connected = false;
  bool odom_connected = false;

  while( ros::ok() )
  {
    // message processing function
    serial.update();

    if( cmd_vel_received )
    {
      drive.data.speed_mm_s = (int16_t)(cmd_vel_linear_x * 1000.0);
      drive.data.turn_mrad_s = (int16_t)(cmd_vel_angular_z * 1000.0);
      if( ! drive.send() )
        ROS_ERROR("Drive send failed");
      cmd_vel_received = false;
    }

    if( power.available() )
    {
      bat_msg.current = power.data.battery_miA / 1023.0;
      bat_msg.voltage = power.data.battery_miV / 1023.0;
      bat_msg.charge = power.data.vsupply_miV / 1023.0; // use for VS
      bat_msg.power_supply_status = power.data.charger_state;
      bat_msg.percentage = power.data.bat_percentage / 100.0;
      power.ready();
      pub_bat.publish(bat_msg);

      if( ! bat_connected )
      {
        ROS_INFO("Base sending BatteryState");
        bat_connected = true;
      }
      last_recv_time = ros::Time::now();
    }

    if( odom.available() )
    {
      ros::Time current_time = ros::Time::now();

      double theta = odom.data.theta;
      double dx = odom.data.dx_mm / 1000.0;
      double dth = odom.data.dth;
      double dt = odom.data.dt_ms / 1000.0;
      odom.ready();

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
      nav_msgs::Odometry odom_msg;
      odom_msg.header.stamp = current_time;
      odom_msg.header.frame_id = "odom";

      //set the position
      odom_msg.pose.pose.position.x = x;
      odom_msg.pose.pose.position.y = y;
      odom_msg.pose.pose.position.z = 0.0;
      odom_msg.pose.pose.orientation = odom_quat;

      //set the velocity
      odom_msg.child_frame_id = "base_link";
      odom_msg.twist.twist.linear.x = vx;
      odom_msg.twist.twist.linear.y = 0.0;
      odom_msg.twist.twist.angular.z = vth;

      //publish the message
      pub_odom.publish(odom_msg);

      if( ! odom_connected )
      {
        ROS_INFO("Base sending tf and odom");
        odom_connected = true;
      }
      last_recv_time = ros::Time::now();
    }

    if( text.available() )
    {
      ROS_INFO(text.data.str);
      text.ready();
    }

    if( !conn_lost_timeout && ros::Time::now() - last_recv_time > conn_lost_duration )
    {
      ROS_ERROR("Lost communication with Base");
      bat_connected = false;
      odom_connected = false;
      conn_lost_timeout = true;
    }
    if( bat_connected || odom_connected )
      conn_lost_timeout = false;

    ros::spinOnce();
  }

}

