#include "car_commands/car_commands.h"

#include "ackermann_msgs/AckermannDriveStamped.h"

CarCommands::CarCommands(ros::NodeHandle n)
{
    node_handle_ = n;

    node_handle_.getParam("/car_control/initial_speed", speed_);
    node_handle_.getParam("/car_control/initial_turn", turn_);
    direction_speed_ = 1;
    direction_theta_ = 0;
}

void CarCommands::prepare()
{
    pub_command_ = node_handle_.advertise<ackermann_msgs::AckermannDriveStamped>("/vesc/ackermann_cmd_mux/input/teleop", 1000);
}

void CarCommands::runPeriodically()
{
    ackermann_msgs::AckermannDriveStamped msg;

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "base_link";

    msg.drive.speed = direction_speed_ * speed_;
    msg.drive.acceleration = 1;
    msg.drive.jerk = 1;
    msg.drive.steering_angle = direction_theta_ * turn_;
    msg.drive.steering_angle_velocity = 1;

    pub_command_.publish(msg);
}

void CarCommands::shutDown()
{

}