#ifndef CAR_COMMANDS_FR_H
#define CAR_ODOMETRY_FR_H

#include "ros/ros.h"
#include "ackermann_msgs/AckermannDriveStamped.h"

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
     * @brief Calculation of x component of acceleration: done by using magic formula, longitudinal force Fx as a function of the longitudinal slip.
     * Fx(k) = D*sin[Cx*arctan{B*k-Ex(B*k-arctan(B*k))]
     * 
     * @param long_Slip
     * @param D
     * @param Cx
     * @param B
     * @param Ex
     * @param mass
     */
    double acceleration_X(const double long_Slip, const double D, const double Cx, const double B, const double Ex, const double mass);

    /**
     * @brief Calculation of y component of acceleration: done by using magic formula, lateral force Fy as a function of the lateral slip.
     * Fy(alpha) = D*sin[Cy*arctan{B*alpha-Ey(B*alpha-arctan(B*alpha))]
     * 
     * @param lat_Slip
     * @param D
     * @param Cy
     * @param B
     * @param Ey
     * @param mass
     */
    double acceleration_Y(const double lat_Slip, const double D, const double Cy, const double B, const double Ey, const double mass);

    /**
     * @brief Display received data (speed and steering angle) and the calculated acceleration data 
     * 
     * @param ackmsg
     */
    void inputsCommandsCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& ackmsg);

    ros::NodeHandle node_handle_;
    ros::Subscriber sub_input_commands_;
};

#endif /* CAR_ODOMETRY_READER_H */