#include "car_kin_controller.h"
#include "ros/assert.h"
#include "ros/exceptions.h"

#include "tf/tf.h"
#include "ackermann_msgs/AckermannDriveStamped.h"

CarKinController::CarKinController(ros::NodeHandle n)
{    
    ROS_ASSERT_MSG(n.hasParam("/car_kinematic_control/wheel_distance"), "YAML param does not exists.");
    node_handle_.getParam("/car_kinematic_control/wheel_distance", model_params_.L);
    if (model_params_.L < 0.0) throw new ros::InvalidParameterException("Wheel distance cannot be below 0.0");

    ROS_ASSERT_MSG(n.hasParam("/car_kinematic_control/epsilon"), "YAML param does not exists.");
    node_handle_.getParam("/car_kinematic_control/epsilon", model_params_.epsilon);
    if (model_params_.epsilon < 0.0) throw new ros::InvalidParameterException("Distance from point P and COG cannot be below 0.0");
    
    node_handle_ = n;
}

void CarKinController::prepare()
{
    sub_input_velocity_ = node_handle_.subscribe("/virtual_velocities", 1000, &CarKinController::inputVelocityCallback, this);
    sub_odometry_ = node_handle_.subscribe("/vesc/odom", 1000, &CarKinController::odometryCallback, this);
    pub_command_ = node_handle_.advertise<ackermann_msgs::AckermannDriveStamped>("/vesc/ackermann_cmd_mux/input/navigation", 1000);
    
    ROS_ASSERT(sub_input_velocity_);
    ROS_ASSERT(sub_odometry_);
    ROS_ASSERT(pub_command_);
}

void CarKinController::runPeriodically()
{
    linearize();
    sendCommand();
}

void CarKinController::shutDown()
{

}

void CarKinController::linearize()
{
    double epsilon = model_params_.epsilon;

    /** Compute new velocity absolute value. */
    new_front_wheel_speed_ = last_point_velocity_x_ * std::cos(last_yaw_) + last_point_velocity_y_ * std::sin(last_yaw_);
    
    //ROS_DEBUG("velocity x = %lf, velocity y = %lf", point_velocity_x, point_velocity_y);

    /** Compute new steer angle. */
    const double num = last_point_velocity_y_ * std::cos(last_yaw_) - last_point_velocity_x_ * std::sin(last_yaw_);
    const double denom = last_point_velocity_x_ * std::cos(last_yaw_) + last_point_velocity_y_ * std::sin(last_yaw_);
    new_steering_angle_ = std::atan( (model_params_.L / epsilon) * (num / denom) );
    
    //ROS_DEBUG("num = %lf, denom = %lf, atan_arg = %lf, steering_angle = %lf", num, denom, (model_params_.L / epsilon) * (num / denom), steering_angle);
    
    /** Compute new steering velocity. */
    new_steering_angle_velocity_ = (new_front_wheel_speed_ / model_params_.L) * std::tan(new_steering_angle_);
}

void CarKinController::sendCommand() const
{
    ackermann_msgs::AckermannDriveStamped msg;

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "base_link";

    msg.drive.speed = new_front_wheel_speed_;
    msg.drive.acceleration = 1;
    msg.drive.jerk = 1;
    msg.drive.steering_angle = new_steering_angle_;
    msg.drive.steering_angle_velocity = 0; //new_steering_angle_velocity_;

    pub_command_.publish(msg);

    ROS_DEBUG("Ackermann Command Sent: speed = %lf, steer_angle = %lf, steer_vel = %lf;", new_front_wheel_speed_, new_steering_angle_, new_steering_angle_velocity_);
}

/* --- HANDLERS --- */

void CarKinController::inputVelocityCallback(const std_msgs::Float64MultiArray::ConstPtr& input_vel_msg)
{
    /** velocity x: input_vel_msg[0]; velocity y: input_vel_msg[1]; */
    last_point_velocity_x_ = input_vel_msg->data.at(0);
    last_point_velocity_y_ = input_vel_msg->data.at(1);
}

void CarKinController::odometryCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
    double roll {0.0};
    double pitch {0.0};
    double yaw {0.0};

    /** Extract 'yaw' from odometry orientation.
     * 'yaw' is the orientation of the veichle on z-axis (referred as 'theta' in the bicycle kinematic model of the slides).
     * It is NOT the steering angle.
     * 
     * Note:
     * - Quaternion: a different way to describe the orientation of a frame only. It's an alternative to YAW, PITCH, ROLL.
     *               (x, y, z, w) used to construct quaternion is NOT a position vector.
     * - Position: position of the robot (x, y, z) in 3D space.
     * - Pose: position + orientation
     */

    const tf::Quaternion quaternion(
        odom_msg->pose.pose.orientation.x, 
        odom_msg->pose.pose.orientation.y, 
        odom_msg->pose.pose.orientation.z, 
        odom_msg->pose.pose.orientation.w);
    
    const tf::Matrix3x3 matrix3x3(quaternion);

    matrix3x3.getRPY(roll, pitch, yaw);

    last_yaw_ = yaw;

    /** Extract 'yaw rate' from odometry twist */
    last_yaw_rate_ = odom_msg->twist.twist.angular.z;
}