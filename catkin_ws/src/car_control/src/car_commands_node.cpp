#include "ros/ros.h"

#include "car_commands/car_commands.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "car_commands_node");
    
    constexpr int LOOP_RATE {2}; /**< loop rate in Hz */

    ros::NodeHandle node_handle;
    ros::Rate loop_rate(LOOP_RATE);
    CarCommands car_commands {node_handle};
    int count = 0;
    
    car_commands.prepare();

    std::cout << "Starting Iterations... Loop Rate " << LOOP_RATE << " [Hz]" << std::endl;

    while (ros::ok())
    {
        car_commands.runPeriodically();

        ROS_INFO("Running (count=%d); Message Sent;", count);

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }

    car_commands.shutDown();

    return 0;
}

