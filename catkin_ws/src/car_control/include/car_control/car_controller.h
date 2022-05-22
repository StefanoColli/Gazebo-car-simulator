#ifndef CAR_CONTROLLER_H
#define CAR_CONTROLLER_H

#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "nav_msgs/Odometry.h"

class CarController
{
public:
    CarController(ros::NodeHandle n);

    /**
     * @brief Mainly set topics to publish or to subscribe.
     * 
     */
    void prepare();

    /**
     * @brief Action executed at every ROS Spin.
     * 
     * Not implemented 
     */
    void runPeriodically();
    void shutDown();

private:
    struct dyn_mod_params
    {
        double M;
        double Cf;
        double Cr;
        double a;
        double b;
    };

    /**
     * @brief Exact linearization of nonlinear system: the bicycle dynamic model
     * 
     * Called every time new actual velocities are received. Perform the computation of general velocity vector of the car and angular velocity.
     * 
     * @param point_velocity_x 
     * @param point_velocity_y 
     * @param yaw
     */
    void linearize(const double point_velocity_x, const double point_velocity_y, const double yaw);

    /**
     * @brief Receive input velocities of a point.
     * 
     * These are the active input velocities which will be linearized.
     * When this callback is called, the last available angular position of the car will be used.
     * 
     * @param input_vel_msg
     */
    void inputVelocityCallback(const std_msgs::Float64MultiArray::ConstPtr& input_vel_msg);

    /**
     * @brief Receive Odometry data (position, orientation and twist). Extract angular position of the car.
     * 
     * @param odom_msg
     */
    void odometryCallback(const nav_msgs::Odometry::ConstPtr& odom_msg);

    ros::NodeHandle node_handle_;
    ros::Subscriber sub_input_velocity_;
    ros::Subscriber sub_odometry_;
    struct dyn_mod_params model_params;
    double last_available_yaw_;
    double last_available_yaw_rate_;
};

#endif /* CAR_CONTROLLER_H */