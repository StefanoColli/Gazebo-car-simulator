#include "ros/ros.h"

#include "std_msgs/Float64MultiArray.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "car_kin_velocitypub_node");
    
    constexpr double LOOP_RATE {3}; /**< loop rate in Hz */

    ros::NodeHandle node_handle;
    ros::Rate loop_rate(LOOP_RATE);
    ros::Publisher pub_velocity = node_handle.advertise<std_msgs::Float64MultiArray>("/robot_as_point_velocity", 1);
    int count = 0;
    constexpr double Vx_p = 0.5;
    constexpr double Vy_p = -0.25;

    std::cout << "Starting Iterations... Loop Rate " << LOOP_RATE << " [Hz]" << std::endl;

    while (ros::ok())
    {
        ROS_INFO("Running (count=%d);", count);

        std_msgs::Float64MultiArray msg;

        // 0 -> Vx_p
        msg.data.push_back(Vx_p);
        // 1 -> Vy_p
        msg.data.push_back(Vy_p);

        pub_velocity.publish(msg);
        ROS_INFO("Pub speed: x = %lf, y = %lf;", Vx_p, Vy_p);

        ros::spinOnce();

        loop_rate.sleep();
        
        ++count;
    }

    return 0;
}