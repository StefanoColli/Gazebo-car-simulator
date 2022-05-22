#include "car_control/car_controller.h"

#include "tf/tf.h"

CarController::CarController(ros::NodeHandle n)
{
    node_handle_ = n;

    model_params.M = 0.8;
    model_params.Cf = 1.0;
    model_params.Cr = 1.0;
    model_params.a = 1.0;
    model_params.b = 1.0;

    last_available_yaw_ = 0.0;
    last_available_yaw_rate_ = 0.0;
}

void CarController::prepare()
{
    sub_input_velocity_ = node_handle_.subscribe("/robot_as_point_velocity", 1000, &CarController::inputVelocityCallback, this);
    sub_odometry_ = node_handle_.subscribe("/vesc/odom", 1000, &CarController::odometryCallback, this);
}

void CarController::runPeriodically()
{

}

void CarController::shutDown()
{

}

void CarController::linearize(const double point_velocity_x, const double point_velocity_y, const double yaw)
{
    /* [command 1/2] absolute_velocity is one of the two information we want to pass to the model */
    double absolute_velocity {0.0};
    /* angular_velocity must be computed in order to get the steering angle */
    double angular_velocity {0.0};

    /* sideslip_angle: in theory slides it is referred as beta */
    const double sideslip_angle = std::atan(point_velocity_y / point_velocity_x);
    double epsilon {1.0};

    /* [command 2/2] steering_angle is one of the two information we want to pass to the model */
    double steering_angle_part1 {0.0};
    double steering_angle_part2 {0.0};
    double steering_angle_part3 {0.0};
    double steering_angle {0.0};

    /* useful service variable to clarify code */
    const double theta = sideslip_angle + yaw;

    /* Change of coordinates: [V omega]' = CHANGE_MATRIX * [Vp_x Vp_y]' */
    absolute_velocity = (std::cos(theta) * point_velocity_x) + (std::sin(theta) * point_velocity_y);
    angular_velocity = ((- std::sin(theta) / epsilon) * point_velocity_x) + ((std::cos(theta) / epsilon) * point_velocity_y);

    /* Computation of steering angle */
    steering_angle_part1 = ((model_params.M * absolute_velocity) / model_params.Cf) * angular_velocity;
    steering_angle_part2 = ((model_params.Cf + model_params.Cr) / model_params.Cf) * sideslip_angle;
    steering_angle_part3 = ((model_params.b * model_params.Cr - model_params.a * model_params.Cf) / model_params.Cf) 
                            * (last_available_yaw_rate_ / absolute_velocity);
                            
    steering_angle = steering_angle_part1 + steering_angle_part2 + steering_angle_part3;
}

void CarController::inputVelocityCallback(const std_msgs::Float64MultiArray::ConstPtr& input_vel_msg)
{
    double point_velocity_x {0.0};
    double point_velocity_y {0.0};

    /* velocity x: input_vel_msg[0]; velocity y: input_vel_msg[1]; */
    point_velocity_x = input_vel_msg->data.at(0);
    point_velocity_y = input_vel_msg->data.at(1);

    linearize(point_velocity_x, point_velocity_y, last_available_yaw_);
}

void CarController::odometryCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
    double roll {0.0};
    double pitch {0.0};
    double yaw {0.0};

    /* Extract 'yaw' from odometry orientation. */

    tf::Quaternion quaternion(
        odom_msg->pose.pose.orientation.x, 
        odom_msg->pose.pose.orientation.y, 
        odom_msg->pose.pose.orientation.z, 
        odom_msg->pose.pose.orientation.w);
    
    tf::Matrix3x3 matrix3x3(quaternion);

    matrix3x3.getRPY(roll, pitch, yaw);

    last_available_yaw_ = yaw;

    /* Extract 'yaw rate' from odometry twist */

    last_available_yaw_rate_ = odom_msg->twist.twist.angular.z;
}