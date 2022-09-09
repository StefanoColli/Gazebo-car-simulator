#include "ros/ros.h"

#include "car_commands_fr/car_commands_fr.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "car_commands_friction_node");
    
    constexpr int LOOP_RATE {2}; /**< loop rate in Hz */

    ros::NodeHandle node_handle;
    ros::Rate loop_rate(LOOP_RATE);
    CarCommandsFr car_commands_friction {node_handle};
    
    car_commands_friction.prepare();
    
    ROS_INFO("Ready to listen for commands...");

    ros::spin();

    car_commands_friction.shutDown();

    return 0;
}