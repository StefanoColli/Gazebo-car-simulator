#include "car_odometry_reader/car_odometry_reader.h"

CarOdometryReader::CarOdometryReader(ros::NodeHandle n)
{
    node_handle_ = n;
}

void CarOdometryReader::prepare()
{
    sub_odometry_ = node_handle_.subscribe("/vesc/odom", 1000, &CarOdometryReader::odometryCallback, this);
}

void CarOdometryReader::runPeriodically()
{

}

void CarOdometryReader::shutDown()
{

}

void CarOdometryReader::odometryCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
    ROS_INFO("Seq: [%d]", odom_msg->header.seq);

    /*
    * Read robot position over all the axis (3D space). Robot doesn't fly, so z position should be 0.0 or near it.
    */
    ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", 
        odom_msg->pose.pose.position.x,
        odom_msg->pose.pose.position.y, 
        odom_msg->pose.pose.position.z);

    /*
    * Read robot orientation, for this application we are not interested in it.
    * This is expressed in term of Quaternions, a different (better) way to describe the orientation of a frame.
    * It's an alternative to yaw, pitch and roll. 
    * Note: Quaternion x, y and z are NOT a position vector. 
    */
    ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", 
        odom_msg->pose.pose.orientation.x, 
        odom_msg->pose.pose.orientation.y, 
        odom_msg->pose.pose.orientation.z, 
        odom_msg->pose.pose.orientation.w);

    /*
    * Read velocity over all the axis. Robot rotate over axis z.
    */
    ROS_INFO("Vel-> Linear{ x: [%f], y: [%f] }, Angular: [%f]", 
        odom_msg->twist.twist.linear.x,
        odom_msg->twist.twist.linear.y,
        odom_msg->twist.twist.angular.z);

    ROS_INFO("-----");
}
