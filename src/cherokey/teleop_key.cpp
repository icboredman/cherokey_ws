// This program publishes velocity messages for cherokey robot
//
// based on https://github.com/ros/ros_tutorials/blob/kinetic-devel/turtlesim/tutorials/teleop_turtle_key.cpp
//
//


#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <signal.h>
#include <termios.h>
#include <stdio.h>

#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71
#define KEYCODE_SPC 0x20


const double speed_step =  0.05;  // [m/s]
const double speed_top  =  1.0;   // [m/s]
const double turn_step  =  0.5;   // [r/s]
const double turn_top   = 10.0;   // [r/s]


struct termios cooked, raw;

// restore normal keyboard behaviour before exit
void quit(int sig)
{
  (void)sig;
  tcsetattr(STDIN_FILENO, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}



int main(int argc, char **argv)
{
    // Initialize the ROS system and become a node
    ros::init(argc, argv, "cherokey_teleop_key");
    ros::NodeHandle nh;

    // Create a publisher object
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("base/cmd_vel", 1000);

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

    ROS_INFO_STREAM("Reading from keyboard");
    ROS_INFO_STREAM("----------------------------");
    ROS_INFO_STREAM("Use arrow keys to move robot");

    double speed = 0;
    double turn = 0;

    char c;
    bool dirty = false;

    //ros::Rate rate(10);	// every 100 ms
    while( ros::ok() )
    {
        // get the next event from the keyboard
        if(read(STDIN_FILENO, &c, 1) < 0)
        {
            //perror("read():");
            ROS_INFO_STREAM("ERROR: read()");
            exit(-1);
        }

        ROS_DEBUG("value: 0x%02X\n", c);

        switch(c)
        {
            case KEYCODE_U:
                ROS_DEBUG("UP");
                speed += speed_step;
                if( speed > speed_top )
                    speed = speed_top;
                dirty = true;
                break;
            case KEYCODE_D:
                ROS_DEBUG("DOWN");
                speed -= speed_step;
                if( speed < -speed_top )
                    speed = -speed_top;
                dirty = true;
                break;
            case KEYCODE_L:
                ROS_DEBUG("LEFT");
                turn += turn_step;
                if( turn > turn_top )
                    turn = turn_top;
                dirty = true;
                break;
            case KEYCODE_R:
                ROS_DEBUG("RIGHT");
                turn -= turn_step;
                if( turn < -turn_top )
                    turn = -turn_top;
                dirty = true;
                break;
            case KEYCODE_SPC:
                ROS_DEBUG("SPACE");
                speed = 0;
                turn = 0;
                dirty = true;
                break;
            case KEYCODE_Q:
                ROS_DEBUG("QUIT");
                exit(0);
                break;
            default :
                break;
        }

        if( dirty )
        {
            // create and fill in the message
            geometry_msgs::Twist msg;
            msg.linear.x = speed;
            msg.angular.z = turn;
            // publish the message
            pub.publish(msg);
            dirty = false;
        }

        // wait until it's time for another iteration
        //rate.sleep();
    }

    quit(0);
}
