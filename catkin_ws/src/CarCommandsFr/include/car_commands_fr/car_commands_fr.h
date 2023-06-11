#ifndef CAR_COMMANDS_FR_H
#define CAR_ODOMETRY_FR_H

#include "ros/ros.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include <std_msgs/Float64MultiArray.h>

/**
 * @class CarCommandsFr
 * 
 * Read commands from topic AckermannDriveStamped.
 * Apply Magic Formula to car's model and print results.
 */
class CarCommandsFr
{
public:
    CarCommandsFr(ros::NodeHandle n);
    
    int surface_type;

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
    
    ros::NodeHandle Handle;

    /**
     * @brief Calculation of longitudinal slip of the tire: difference between the tire tangential speed and the speed of the axle relative to the road.
     * 
     * @param speed
     * @param steer_angle
     * @param wheel_R
     */
    double longitudinalSlip(const double speed, const double steer_angle, const double wheel_R);

    /**
     * @brief Calculation of lateral slip of the tire: angle between the direction of the trajectory of the tire and the pointing direction of the tire.
     * 
     * @param speed
     * @param dir_x
     * @param dir_y
     */
    double lateralSlip(const double speed, const double dir_x, const double dir_y);

    /**
     * @brief Calculation of acceleration: done by using magic formula.
     * Longitudinal force Fx as a function of the longitudinal slip.
     * 	Fx(k) = D*sin[Cx*arctan{B*k-Ex(B*k-arctan(B*k))]
     * Lateral force Fy as a function of the lateral slip.
     * 	Fy(alpha) = D*sin[Cy*arctan{B*alpha-Ey(B*alpha-arctan(B*alpha))]
     * 
     * @param slip
     * @param D
     * @param C
     * @param B
     * @param E
     * @param mass
     */
    double compute_acc(const double slip, const double D, const double C, const double B, const double E, const double mass);

    /**
     * @brief Display received data (speed and steering angle) and the calculated acceleration data 
     * 
     * @param ackmsg
     */
    void inputCommandsCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& ackmsg);

    /**
     * @brief Set parameters to compute wheels' accelerations using Pacejka's Magic Formula, given a surface type. 
     */
    void setParameters();

    ros::NodeHandle node_handle_;
    ros::Subscriber sub_input_commands_;
    ros::Publisher pub_acc_left_rear_wheel;
    ros::Publisher pub_acc_right_rear_wheel;
    ros::Publisher pub_acc_left_front_wheel;
    ros::Publisher pub_acc_right_front_wheel;
    ros::Publisher pub_reference_speed;
    ros::Publisher pub_pos_left_steering_hinge;
    ros::Publisher pub_pos_right_steering_hinge;

    /**
     * @brief Pacejka Magic Formula parameters
     */
    double Bx;
    double By;
    double Cx;
    double Cy;
    double D;
    double E;

    double t; //current time

};

#endif /* CAR_ODOMETRY_READER_H */
