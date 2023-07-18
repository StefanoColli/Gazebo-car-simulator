#include "trajectory_tracker/trajectory_tracker.h"

#include "tf/tf.h"

void TrajectoryTracker::Prepare(void)
{
    /* Retrieve parameters from ROS parameter server */
    std::string FullParamName;

    // Node parameters
    FullParamName = ros::this_node::getName()+"/trajectory_type";
    if (false == Handle.getParam(FullParamName, traj_type))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());
    
    // Straight line trajectory parameters
    FullParamName = ros::this_node::getName()+"/a_coeff";
    if (false == Handle.getParam(FullParamName, a_coeff))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());
    FullParamName = ros::this_node::getName()+"/b_coeff";
    if (false == Handle.getParam(FullParamName, b_coeff))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());
    // Parabolic trajectory parameter
    FullParamName = ros::this_node::getName()+"/focal_length";
    if (false == Handle.getParam(FullParamName, focal_length))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());
    // Circular trajectory parameters
    FullParamName = ros::this_node::getName()+"/R";
    if (false == Handle.getParam(FullParamName, R))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());
    FullParamName = ros::this_node::getName()+"/W";
    if (false == Handle.getParam(FullParamName, W))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());
    // Eight-shape trajectory parameters
    FullParamName = ros::this_node::getName()+"/a";
    if (false == Handle.getParam(FullParamName, a))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());
    FullParamName = ros::this_node::getName()+"/w";
    if (false == Handle.getParam(FullParamName, w))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());
    // Cycloidal trajectory parameter
    FullParamName = ros::this_node::getName()+"/cycloid_radius";
    if (false == Handle.getParam(FullParamName, cycloid_radius))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());
    FullParamName = ros::this_node::getName()+"/cycloid_distance";
    if (false == Handle.getParam(FullParamName, cycloid_distance))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    // Controller parameters
    FullParamName = ros::this_node::getName()+"/Kp";
    if (false == Handle.getParam(FullParamName, Kp))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());
    FullParamName = ros::this_node::getName()+"/Ki";
    if (false == Handle.getParam(FullParamName, Ki))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());
    FullParamName = ros::this_node::getName()+"/Kd";
    if (false == Handle.getParam(FullParamName, Kd))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());
    FullParamName = ros::this_node::getName()+"/FFWD";
    if (false == Handle.getParam(FullParamName, FFWD))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    // Robot parameters
    FullParamName = ros::this_node::getName()+"/PL_distance";
    if (false == Handle.getParam(FullParamName, PL_distance))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    /* ROS topics */
    virtual_velocities_publisher = Handle.advertise<std_msgs::Float64MultiArray>("/virtual_velocities", 1);
    reference_publisher = Handle.advertise<std_msgs::Float64MultiArray>("/reference_trajectory", 1);
    odometry_subscriber = Handle.subscribe("/vesc/odom", 1, &TrajectoryTracker::Odometry_message_callback, this);
    
    // set up dynamic reconfiguration
    f = boost::bind(&TrajectoryTracker::Reconfigure_callback, this, _1, _2);
    config_server.setCallback(f);
    ROS_INFO("Dynamic reconfigure initialized");

    /* Initialize node state */
    RunPeriod = 1/100.0f; //100Hz
    t = ros::Time::now().toSec();
    vPx = vPy = theta = 0.0;
    x_int_term = y_int_term = 0.0;
    prev_xP_error = prev_yP_error = 0.0;
    

    ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}

void TrajectoryTracker::Odometry_message_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // Input odometry

    /*  we have to transform the coordinate back to the P point */
    L_to_P(msg->pose.pose.position.x, msg->pose.pose.position.y, xP, yP);

    double roll = 0;
    double pitch = 0;
    double yaw = 0;

    const tf::Quaternion quaternion(
        msg->pose.pose.orientation.x, 
        msg->pose.pose.orientation.y, 
        msg->pose.pose.orientation.z, 
        msg->pose.pose.orientation.w);
    
    const tf::Matrix3x3 matrix3x3(quaternion);

    matrix3x3.getRPY(roll, pitch, yaw);

    theta = yaw;
}

void TrajectoryTracker::Reconfigure_callback(const trajectory_tracker::TrajTrackerConfig& config, uint32_t level)
{
    // update controller gains with values coming from the dynamic reconfigure server
    Kp = config.Kp;
    Ki = config.Ki;
    Kd = config.Kd;
    FFWD = config.FFWD;

    if (!active && config.active) //active state changed
    { 
        ROS_INFO("Trajectory started");
        traj_starting_time = ros::Time::now().toSec(); //set trajectory starting time
    }
    active = config.active;
}

void TrajectoryTracker::L_to_P(const double xlocal, const double ylocal, double &xP, double &yP)
{
    xP = xlocal + PL_distance * std::cos(theta);
    yP = ylocal + PL_distance * std::sin(theta);
}

void TrajectoryTracker::Compute_trajectory_step()
{
    double traj_time = ros::Time::now().toSec() - traj_starting_time;
    /* Selected trajectory generation */
    switch (traj_type){

        case LINEAR:

            // Linear trajectory computation
            xref    = a_coeff * traj_time;
            dxref   = a_coeff;
            yref    = b_coeff * traj_time;
            dyref   = b_coeff;

            break;

        case PARABOLIC:

            // Parabolic trajectory computation
            xref    = 2 * focal_length * traj_time;
            dxref   = 2 * focal_length;
            yref    = focal_length * std::pow(traj_time,2.0);
            dyref   = 2 * focal_length * traj_time;

            break;

        case CIRCLE:

            // Circular trajectory computation
            xref    = R*std::cos(W*traj_time - M_PI/2);
            dxref   = -W*R*std::sin(W*traj_time - M_PI/2);
            yref    = R*std::sin(W*traj_time - M_PI/2) + R;
            dyref   = W*R*std::cos(W*traj_time - M_PI/2);

            break;

        case EIGHT:

            // Eight shaped trajectory computation
            xref    = a*std::sin(w*traj_time);
            dxref   = w*a*std::cos(w*traj_time);  
            yref    = a*std::sin(w*traj_time)*std::cos(w*traj_time);
            dyref   = w*a*(std::pow(std::cos(w*traj_time),2.0)-std::pow(std::sin(w*traj_time),2.0)); 

            break;

        case CYCLOIDAL:

            // Curtate cycloid trajectory computation
            xref    = cycloid_radius * traj_time - cycloid_distance * std::sin(traj_time);
            dxref   = cycloid_radius - cycloid_distance * std::cos(traj_time);
            yref    = cycloid_distance * (1 - std::cos(traj_time));
            dyref   = cycloid_distance * std::sin(traj_time);

            break;

        default:
            ROS_INFO("Uknown selected trajectory type! (%d was selected).", traj_type);
            throw std::invalid_argument( "Uknown selected trajectory type!" );
            break;
    }
}

void TrajectoryTracker::Control_law()
{
    /* Trajectory tracking law */ 
    // compute position error
    double xP_error = xPref - xP;
    double yP_error = yPref - yP;

    // get current time
    double t_new = ros::Time::now().toSec();
    double delta_t = t_new - t;

    // compute integral term
    x_int_term += xP_error * delta_t;
    y_int_term += yP_error * delta_t;

    // compute derivative term
    double x_der_term = (xP_error - prev_xP_error) / delta_t;
    double y_der_term = (yP_error - prev_yP_error) / delta_t;

    // PI + FFWD controller 
    vPx = FFWD * dxref + Kp * xP_error + Ki * x_int_term + Kd * x_der_term;
    vPy = FFWD * dyref + Kp * yP_error + Ki * y_int_term + Kd * y_der_term;

    ROS_DEBUG("Error = (%f,%f) - Speed = (%f,%f)", xP_error, yP_error, vPx,vPy);

    // store last error
    prev_xP_error = xP_error;
    prev_yP_error = yP_error;

    
}

void TrajectoryTracker::Run_periodically(float Period)
{
    ros::Rate LoopRate(1.0/Period);

    ROS_INFO("Node %s running periodically (T=%.2fs, f=%.2fHz).", ros::this_node::getName().c_str(), Period, 1.0/Period);

    while (ros::ok())
    {
        Periodic_task();

        ros::spinOnce();

        LoopRate.sleep();
    }
}

void TrajectoryTracker::Shutdown(void)
{
    ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
}

void TrajectoryTracker::Periodic_task(void)
{
    if(active)
    {
        // compute reference point for the desired trajectory
        Compute_trajectory_step();
    
        // translate trajectory point from point L to point P 
        L_to_P(xref, yref, xPref, yPref);

        // compute control action as virtual velocities
        Control_law();

        /* Publish vehicle commands (msg->data[0] = velocity of point P along x direction
                                        msg->data[1] = velocity of point P along y direction
                                        msg->data[2] = t )*/
        std_msgs::Float64MultiArray msg;
        msg.data.push_back(vPx);
        msg.data.push_back(vPy);
        msg.data.push_back(t);
        virtual_velocities_publisher.publish(msg);

        /* Publish reference trajectory (msg_ref->data[0] = x reference
                                        msg_ref->data[1] = y reference
                                        msg->data[2] = t )*/
        std_msgs::Float64MultiArray msg_ref;
        msg_ref.data.push_back(xref);
        msg_ref.data.push_back(yref);
        msg_ref.data.push_back(t);
        reference_publisher.publish(msg_ref);
    }
        
    // update time
    t = ros::Time::now().toSec();
}
