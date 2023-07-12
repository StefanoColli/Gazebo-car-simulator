#include "ode_simulator/ode_simulator.h"

#include <unistd.h>

void OdeSimulator::Prepare(void)
{
    /* Retrieve parameters from ROS parameter server */
    std::string full_param_name;

    // Simulator parameters

    full_param_name = ros::this_node::getName() + "/tyre_model";
    if (false == handle_.getParam(full_param_name, server_params_.tyre_model))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), full_param_name.c_str());

    full_param_name = ros::this_node::getName() + "/dt";
    if (false == handle_.getParam(full_param_name, server_params_.dt))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), full_param_name.c_str());

    // Vehicle parameters

    full_param_name = ros::this_node::getName() + "/cog_a";
    if (false == handle_.getParam(full_param_name, server_params_.a))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), full_param_name.c_str());

    full_param_name = ros::this_node::getName() + "/cog_b";
    if (false == handle_.getParam(full_param_name, server_params_.b))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), full_param_name.c_str());

    full_param_name = ros::this_node::getName() + "/m";
    if (false == handle_.getParam(full_param_name, server_params_.m))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), full_param_name.c_str());

    full_param_name = ros::this_node::getName() + "/mu";
    if (false == handle_.getParam(full_param_name, server_params_.mu))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), full_param_name.c_str());

    full_param_name = ros::this_node::getName() + "/Cf";
    if (false == handle_.getParam(full_param_name, server_params_.Cf))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), full_param_name.c_str());

    full_param_name = ros::this_node::getName() + "/Cr";
    if (false == handle_.getParam(full_param_name, server_params_.Cr))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), full_param_name.c_str());

    full_param_name = ros::this_node::getName() + "/Iz";
    if (false == handle_.getParam(full_param_name, server_params_.Iz))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), full_param_name.c_str());

    // Vehicle initial state

    full_param_name = ros::this_node::getName() + "/r0";
    if (false == handle_.getParam(full_param_name, server_params_.r0))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), full_param_name.c_str());

    full_param_name = ros::this_node::getName() + "/beta0";
    if (false == handle_.getParam(full_param_name, server_params_.beta0))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), full_param_name.c_str());

    full_param_name = ros::this_node::getName() + "/x0";
    if (false == handle_.getParam(full_param_name, server_params_.x0))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), full_param_name.c_str());

    full_param_name = ros::this_node::getName() + "/y0";
    if (false == handle_.getParam(full_param_name, server_params_.y0))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), full_param_name.c_str());

    full_param_name = ros::this_node::getName() + "/psi0";
    if (false == handle_.getParam(full_param_name, server_params_.psi0))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), full_param_name.c_str());

    /* ROS topics */

    vehicle_command_subscriber_ = handle_.subscribe("/vesc/ackermann_cmd_mux/input/navigation", 1, &OdeSimulator::vehicleCommand_MessageCallback, this);
    vehicle_odom_publisher_ = handle_.advertise<nav_msgs::Odometry>("/vesc/odom", 1);
    
    vehicle_state_publisher_ = handle_.advertise<std_msgs::Float64MultiArray>("/car_state", 1);    
    clock_publisher_ = handle_.advertise<rosgraph_msgs::Clock>("/clock", 1);

    /* Create simulator class */
    switch (server_params_.tyre_model)
    {
        case 0: // linear
            simulator_ = new SingletrackODE(server_params_.dt, SingletrackODE::LINEAR);
            break;

        case 1: // fiala with saturation
            simulator_ = new SingletrackODE(server_params_.dt, SingletrackODE::FIALA_WITH_SATURATION);
            break;

        case 2: // fiala without saturation
            simulator_ = new SingletrackODE(server_params_.dt, SingletrackODE::FIALA_WITHOUT_SATURATION);
            break;

        default:
            simulator_ = new SingletrackODE(server_params_.dt, SingletrackODE::LINEAR);
            break;
    }

    /* Initialize simulator class */
    simulator_->setInitialState(server_params_.r0, server_params_.beta0, server_params_.x0, server_params_.y0, server_params_.psi0);
    simulator_->setVehicleParams(server_params_.m, server_params_.a, server_params_.b, server_params_.Cf, server_params_.Cr, server_params_.mu, server_params_.Iz);

    ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}

void OdeSimulator::RunPeriodically(void)
{
    /*  Integrate the model */
    simulator_->integrate();

    /*  Extract measurement from simulator */
    double x, y, theta;
    simulator_->getPose(x, y, theta);

    double ay, yawrate, vy;
    simulator_->getLateralDynamics(ay, yawrate, vy);

    double sideslip;
    simulator_->getSideslip(sideslip);

    double slip_front, slip_rear;
    simulator_->getSlip(slip_front, slip_rear);

    double force_front, force_rear;
    simulator_->getLateralForce(force_front, force_rear);

    double velocity_act, steer_act;
    simulator_->getCommands(velocity_act, steer_act);

    double time;
    simulator_->getTime(time);

    /*  Print simulation time every 5 sec */
    if (std::fabs(std::fmod(time,5.0)) < 1.0e-3)
    {
        ROS_INFO("Simulator time: %d seconds", (int) time);
    }

    /*  Publish vehicle state */
    std_msgs::Float64MultiArray vehicleStateMsg;
    vehicleStateMsg.data.push_back(time);
    vehicleStateMsg.data.push_back(x);
    vehicleStateMsg.data.push_back(y);
    vehicleStateMsg.data.push_back(theta);
    vehicleStateMsg.data.push_back(yawrate);
    vehicleStateMsg.data.push_back(vy);
    vehicleStateMsg.data.push_back(ay);
    vehicleStateMsg.data.push_back(sideslip);
    vehicleStateMsg.data.push_back(slip_front);
    vehicleStateMsg.data.push_back(slip_rear);
    vehicleStateMsg.data.push_back(force_front);
    vehicleStateMsg.data.push_back(force_rear);
    vehicleStateMsg.data.push_back(velocity_act);
    vehicleStateMsg.data.push_back(steer_act);
    vehicle_state_publisher_.publish(vehicleStateMsg);

    /* Publish veichle odometry for Tracker and Linearizator. */
    nav_msgs::Odometry veichle_odometry;
    veichle_odometry.pose.pose.position.x = x; // For Tracker
    veichle_odometry.pose.pose.position.y = y; // For Tracker

    tf::Quaternion quaternion;
    quaternion.setRPY(0, 0, theta); // yaw (=theta) will be retrieved by both Tracker and Linearizator

    veichle_odometry.pose.pose.orientation.x = quaternion.getX();
    veichle_odometry.pose.pose.orientation.y = quaternion.getY();
    veichle_odometry.pose.pose.orientation.z = quaternion.getZ();
    veichle_odometry.pose.pose.orientation.w = quaternion.getW();

    veichle_odometry.twist.twist.angular.z = yawrate; // For Linearizator
    
    vehicle_odom_publisher_.publish(veichle_odometry);

    /*  Publish clock */
    rosgraph_msgs::Clock clockMsg;
    clockMsg.clock = ros::Time(time);
    clock_publisher_.publish(clockMsg);
}

void OdeSimulator::Shutdown(void)
{
    delete simulator_;

    ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
}

void OdeSimulator::vehicleCommand_MessageCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg)
{
    /*  Set vehicle commands */
    simulator_->setReferenceCommands(msg->drive.speed, msg->drive.steering_angle);
}
