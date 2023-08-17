#include "car_dyn_linearizer/car_dyn_linearizer.h"
#include "ros/assert.h"
#include "ros/exceptions.h"

#include "tf/tf.h"
#include "ackermann_msgs/AckermannDriveStamped.h"

CarDynLinearizer::CarDynLinearizer(ros::NodeHandle n)
{
    node_handle_ = n;

    /* Retrieve parameters from ROS parameter server */
    std::string full_param_name;

    full_param_name = ros::this_node::getName() + "/cog_a";
    if (false == node_handle_.getParam(full_param_name, model_params_.a))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), full_param_name.c_str());

    full_param_name = ros::this_node::getName() + "/cog_b";
    if (false == node_handle_.getParam(full_param_name, model_params_.b))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), full_param_name.c_str());

    full_param_name = ros::this_node::getName() + "/m";
    if (false == node_handle_.getParam(full_param_name, model_params_.m))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), full_param_name.c_str());

    full_param_name = ros::this_node::getName() + "/Cf";
    if (false == node_handle_.getParam(full_param_name, model_params_.Cf))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), full_param_name.c_str());

    full_param_name = ros::this_node::getName() + "/Cr";
    if (false == node_handle_.getParam(full_param_name, model_params_.Cr))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), full_param_name.c_str());

    full_param_name = ros::this_node::getName() + "/epsilon";
    if (false == node_handle_.getParam(full_param_name, model_params_.epsilon))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), full_param_name.c_str());
}

void CarDynLinearizer::prepare()
{
    sub_input_velocity_ = node_handle_.subscribe("/virtual_velocities", 1000, &CarDynLinearizer::inputVelocityCallback, this);
    sub_odometry_ = node_handle_.subscribe("/vesc/odom", 1000, &CarDynLinearizer::odometryCallback, this);
    pub_command_ = node_handle_.advertise<ackermann_msgs::AckermannDriveStamped>("/vesc/ackermann_cmd_mux/input/navigation", 1000);
    
    ROS_ASSERT(sub_input_velocity_);
    ROS_ASSERT(sub_odometry_);
    ROS_ASSERT(pub_command_);
}

void CarDynLinearizer::runPeriodically()
{
    linearize();
    sendCommand();
}

void CarDynLinearizer::shutDown()
{

}

void CarDynLinearizer::linearize()
{
    /* [command 1/2] absolute_velocity is one of the two information we want to pass to the model */
    double absolute_velocity {0.0};
    /* angular_velocity must be computed in order to get the steering angle */
    double angular_velocity {0.0};

    /* sideslip_angle: in theory slides it is referred as beta */
    const double sideslip_angle = std::atan(last_point_velocity_y_ / last_point_velocity_x_);
    double epsilon = model_params_.epsilon;

    /* [command 2/2] steering_angle is one of the two information we want to pass to the model */
    double steering_angle_part1 {0.0};
    double steering_angle_part2 {0.0};
    double steering_angle_part3 {0.0};
    double steering_angle {0.0};

    /* useful service variable to clarify code */
    const double theta = sideslip_angle + last_yaw_;

    /* Change of coordinates: [V omega]' = CHANGE_MATRIX * [Vp_x Vp_y]' */
    absolute_velocity = (std::cos(theta) * last_point_velocity_x_) + (std::sin(theta) * last_point_velocity_y_);
    angular_velocity = ((- std::sin(theta) / epsilon) * last_point_velocity_x_) + ((std::cos(theta) / epsilon) * last_point_velocity_y_);

    /* Computation of steering angle */
    steering_angle_part1 = ((model_params_.m * absolute_velocity) / model_params_.Cf) * angular_velocity;
    steering_angle_part2 = ((model_params_.Cf + model_params_.Cr) / model_params_.Cf) * sideslip_angle;
    steering_angle_part3 = ((model_params_.b * model_params_.Cr - model_params_.a * model_params_.Cf) / model_params_.Cf) 
                            * (last_yaw_rate_ / absolute_velocity);
                            
    steering_angle = steering_angle_part1 + steering_angle_part2 + steering_angle_part3;

    new_front_wheel_speed_ = absolute_velocity;
    new_steering_angle_ = steering_angle;
}

void CarDynLinearizer::sendCommand() const
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

void CarDynLinearizer::inputVelocityCallback(const std_msgs::Float64MultiArray::ConstPtr& input_vel_msg)
{
    /** velocity x: input_vel_msg[0]; velocity y: input_vel_msg[1]; */
    last_point_velocity_x_ = input_vel_msg->data.at(0);
    last_point_velocity_y_ = input_vel_msg->data.at(1);
}

void CarDynLinearizer::odometryCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
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