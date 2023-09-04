#include "ros/ros.h"

#include "car_dyn_linearizer/car_dyn_linearizer.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "car_dyn_linearizer_node");
    
    constexpr double LOOP_RATE {100.0}; /**< loop rate in Hz */
    constexpr double LOOP_PERIOD {1 / LOOP_RATE}; /**< loop period in [s] */

    ros::NodeHandle node_handle;
    ros::Rate loop_rate(LOOP_RATE);
    CarDynLinearizer car_dyn_linearizer {node_handle};
    int count = 0;
    
    car_dyn_linearizer.prepare();

    ROS_INFO("Starting Iterations... (Loop Rate = %lf [Hz], Loop Period = %lf [s])", LOOP_RATE, LOOP_PERIOD);

    while (ros::ok())
    {
        car_dyn_linearizer.runPeriodically();

        //ROS_INFO("Running (count=%d);", count);

        ros::spinOnce();

        loop_rate.sleep();

        ++count;
    }

    car_dyn_linearizer.shutDown();

    return 0;
}