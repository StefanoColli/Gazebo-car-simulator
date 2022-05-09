#include "ros/ros.h"

#include "car_odometry_reader/car_odometry_reader.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "car_odometry_reader_node");
    
    constexpr int LOOP_RATE {2}; /**< loop rate in Hz */

    ros::NodeHandle node_handle;
    ros::Rate loop_rate(LOOP_RATE);
    CarOdometryReader car_odometry_reader {node_handle};
    
    car_odometry_reader.prepare();
    
    ROS_INFO("Ready to listen for odometry info...");

    ros::spin();

    car_odometry_reader.shutDown();

    return 0;
}
