#include "../include/dyros_jet_joystick/dyros_joystick.h"

DyrosJoystick::DyrosJoystick()
{
    walking_cmd_pub_ = nh_.advertise<dyros_jet_msgs::WalkingCommand>("/dyros_jet/joystick_walking_command", 3);
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &DyrosJoystick::joyCallback, this);
    
}

void DyrosJoystick::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    if (joy-> axes[1] > 0.5)
    {
        walk_cmd_ = true;
        walk_cmd_msg_.x = 1.0;
    }
    else
    {
        walk_cmd_ = false;
        walk_cmd_msg_.x = 0.0;
    }

    walk_cmd_msg_.y = joy->axes[0];
    walk_cmd_msg_.z = 0.0;
    walk_cmd_msg_.height = 0.0;
    walk_cmd_msg_.theta = 0.0;
    walk_cmd_msg_.step_length_x = 0.0;
    walk_cmd_msg_.step_length_y = 0.0;

    if (walk_cmd_ != walk_cmd_pre_)
        walking_cmd_pub_.publish(walk_cmd_msg_);
    walk_cmd_pre_ = walk_cmd_;
}




int main(int argc, char** argv)
{
    std::cout << "Started1"<<std::endl;
    ros::init(argc, argv, "dyros_jet_joystick");
    DyrosJoystick dyrosjoystick;
    std::cout << "Joystick Controller Started"<<std::endl;
    while(ros::ok())
    {
        ros::spinOnce();
    }
}