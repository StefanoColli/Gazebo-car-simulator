#ifndef CAR_KIN_CONTROLLER_H
#define CAR_KIN_CONTROLLER_H

#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "nav_msgs/Odometry.h"

class CarKinController
{
public:
    /**
     * @brief Construct a new CarKinController object.
     * 
     * Get parameters from parameter server.
     * 
     * @param n Handle of the node.
     * @requires "existance of parameter labelled 'L' representing the distance between wheels"
     * @ensures if parameters.L exists than parameters.L >= 0.0
     */
    CarKinController(ros::NodeHandle n);

    /**
     * @brief Mainly set topics to publish or to subscribe.
     * 
     */
    void prepare();

    /**
     * @brief Action executed at every ROS Spin.
     * 
     * Call to perform linearization.
     * Call to send results of linearization.
     */
    void runPeriodically();

    void shutDown();

private:
    /**
     * @brief Parameters of the Car.
     * 
     */
    struct s_kin_model_params
    {
        double L {0.0}; /**< Distance between front and rear wheels. */
    };

    /**
     * @brief Perform the computation of general velocity of the car, the steer angular velocity and the steer orientation.
     * 
     * When the method is called, the last available point velocity of the car will be used. The same for the yaw.
     * Note: velocity is referred as front wheel speed.
     * 
     * @see (pag. 43-47, 03-Kinematics) (pag. 37-38, 07-Control)
     */
    void linearize();

    /**
     * @brief Send an ackermann command to the topic from which the model will read;
     * 
     */
    void sendCommand() const;

    /* --- EVENT HANDLERS --- */

    /**
     * @brief Called every time new actual point velocities are received. 
     * 
     * These are the active input velocities.
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

    ros::Subscriber sub_input_velocity_; /**< Callback for point velocity info sent by Trajectory Tracker. */
    ros::Subscriber sub_odometry_; /**< Callback for odometry info sent by Model. */

    ros::Publisher pub_command_; /**< Publisher of Ackerman steering info for the Model. */

    struct s_kin_model_params model_params_; /**< Model parameters which are useful to perform linearization. */

    double last_point_velocity_x_ {0.0} /**< Last point velocity (x) value received (controller input). */;
    double last_point_velocity_y_ {0.0}; /**< Last point velocity (y) value received (controller input). */

    double last_yaw_ {0.0}; /**< Degree of rotation of the veichle on the z-axis (controller input). */
    double last_yaw_rate_ {0.0}; /**< Rotation speed of the veichle on the z-axis (controller input). */

    double new_front_wheel_speed_ {0.0}; /**< Speed value to send to the veichle Model (controller output). */
    double new_steering_angle_ {0.0}; /**< Steering angle to send to the veichle Model (controller output).*/
    double new_steering_angle_velocity_ {0.0}; /**< Steering angle velocity to send to the veichle Model (controller output).*/
};

#endif /* CAR_KIN_CONTROLLER_H */