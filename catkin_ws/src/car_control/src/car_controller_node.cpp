#include "ros/ros.h"

#include "car_control/car_controller.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "car_controller_node");
    
    constexpr int LOOP_RATE {2}; /**< loop rate in Hz */

    ros::NodeHandle node_handle;
    ros::Rate loop_rate(LOOP_RATE);
    CarController car_controller {node_handle};
    int count = 0;
    
    car_controller.prepare();

    std::cout << "Starting Iterations... Loop Rate " << LOOP_RATE << " [Hz]" << std::endl;

    while (ros::ok())
    {
        car_controller.runPeriodically();

        ROS_INFO("Running (count=%d);", count);

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }

    car_controller.shutDown();

    return 0;
}