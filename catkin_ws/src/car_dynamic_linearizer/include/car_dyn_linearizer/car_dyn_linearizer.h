#ifndef CAR_DYN_LINEARIZER_H
#define CAR_DYN_LINEARIZER_H

#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "nav_msgs/Odometry.h"

class CarDynLinearizer
{
public:
    /**
     * @brief Construct a new CarDynLinearizer object.
     * 
     * Get parameters from parameter server.
     * 
     * @param n Handle of the node.
     * @requires "existance of parameter labelled 'cog_a'
     * @requires "existance of parameter labelled 'cog_b'
     * @requires "existance of parameter labelled 'm'
     * @requires "existance of parameter labelled 'Cf'
     * @requires "existance of parameter labelled 'Cr'
     */
    CarDynLinearizer(ros::NodeHandle n);

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
    typedef struct s_dyn_mod_params
    {
        double m {0.0}; /**< Vehicle mass */
        double a {0.0}; /**< Distance of COG from front axle */
        double b {0.0}; /**< Distance of COG from rear axle */
        double Cf {0.0}; /**< Front cornering stiffness */
        double Cr {0.0}; /**< Rear cornering stiffness */

        double epsilon {0.0}; /**< Distance between COG and a reference point P. */
    } t_dyn_model_params;

     /**
     * @brief Perform the computation of general velocity of the car, the steer angular velocity and the steer orientation.
     * 
     * When the method is called, the last available point velocity of the car will be used. The same for the yaw.
     * Note: velocity is referred as front wheel speed.
     * 
     * @see (pag. 21-28, 04-Dynamics) (pag. 44-48, 07-Control)
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
     * @param input_vel_msg desired velocities received from Trajectory Tracker
     */
    void inputVelocityCallback(const std_msgs::Float64MultiArray::ConstPtr& input_vel_msg);

     /**
     * @brief Receive Odometry data (position, orientation and twist). Extract angular position of the car.
     * 
     * @param odom_msg contains information about Odometry received from Gazebo Model, such as current position and orientation.
     */
    void odometryCallback(const nav_msgs::Odometry::ConstPtr& odom_msg);

    ros::NodeHandle node_handle_;

    ros::Subscriber sub_input_velocity_; /**< Callback for point velocity info sent by Trajectory Tracker. */
    ros::Subscriber sub_odometry_; /**< Callback for odometry info sent by Model. */

    ros::Publisher pub_command_; /**< Publisher of Ackerman steering info for the Model. */

    t_dyn_model_params model_params_; /**< Model parameters which are useful to perform linearization. */

    double last_point_velocity_x_ {0.0} /**< Last point velocity (x) value received (controller input). */;
    double last_point_velocity_y_ {0.0}; /**< Last point velocity (y) value received (controller input). */

    double last_yaw_ {0.0}; /**< Degree of rotation of the veichle on the z-axis (controller input). */
    double last_yaw_rate_ {0.0}; /**< Rotation speed of the veichle on the z-axis (controller input). */

    double new_front_wheel_speed_ {0.0}; /**< Speed value to send to the veichle Model (controller output). */
    double new_steering_angle_ {0.0}; /**< Steering angle to send to the veichle Model (controller output).*/
    double new_steering_angle_velocity_ {0.0}; /**< Steering angle velocity to send to the veichle Model (controller output).*/
};

#endif /* CAR_DYN_LINEARIZER_H */