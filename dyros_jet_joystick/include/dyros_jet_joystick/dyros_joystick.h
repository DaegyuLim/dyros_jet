#ifndef _DYROS_JET_JOYSTICK_H
#define _DYROS_JET_JOYSTICK_H
// ROS LIBRARY
#include <ros/ros.h>
// ROS MESSAGE
#include <dyros_jet_msgs/WalkingCommand.h>
#include <sensor_msgs/Joy.h>
    


class DyrosJoystick
    {
   public:
     DyrosJoystick();
     void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
     
   private:
     ros::NodeHandle nh_;
     ros::Publisher walking_cmd_pub_;
     ros::Subscriber joy_sub_;
     bool walk_cmd_;
     bool walk_cmd_pre_;
   
    dyros_jet_msgs::WalkingCommand walk_cmd_msg_;

   };
 

 int main(int argc, char** argv);

#endif
