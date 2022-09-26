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
    FullParamName = ros::this_node::getName()+"/parabola_convexity";
    if (false == Handle.getParam(FullParamName, parabola_convexity))
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

    // Controller parameters
    FullParamName = ros::this_node::getName()+"/Kp";
    if (false == Handle.getParam(FullParamName, Kp))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());
    FullParamName = ros::this_node::getName()+"/Ki";
    if (false == Handle.getParam(FullParamName, Ki))
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
    odometry_subscriber = Handle.subscribe("/vesc/odom", 1, &TrajectoryTracker::odometry_MessageCallback, this);
    
    // set up dynamic reconfiguration
    f = boost::bind(&TrajectoryTracker::reconfigure_callback, this, _1, _2);
    config_server.setCallback(f);
    ROS_INFO("Dynamic reconfigure initialized");

    /* Initialize node state */
    RunPeriod = RUN_PERIOD_DEFAULT;
    t = ros::Time::now().toSec();
    vPx = vPy = theta = 0.0;
    

    ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}

void TrajectoryTracker::odometry_MessageCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // Input odometry

    /*  we have to transorm the coordinate back to the P point */
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

void TrajectoryTracker::reconfigure_callback(const trajectory_tracker::TrajTrackerConfig& config, uint32_t level)
{
    // update controller gains with values coming from the dynamic reconfigure server
    Kp = config.Kp;
    Ki = config.Ki;
    FFWD = config.FFWD;
    ROS_INFO("Changed local parameters: Kp = %f, Ki = %f, FFWD = %d!", Kp, Ki, FFWD);
}

void TrajectoryTracker::L_to_P(const double xlocal, const double ylocal, double &xP, double &yP)
{
    xP = xlocal + PL_distance * std::cos(theta);
    yP = ylocal + PL_distance * std::sin(theta);
}

void TrajectoryTracker::RunPeriodically(float Period)
{
    ros::Rate LoopRate(1.0/Period);

    ROS_INFO("Node %s running periodically (T=%.2fs, f=%.2fHz).", ros::this_node::getName().c_str(), Period, 1.0/Period);

    while (ros::ok())
    {
        PeriodicTask();

        ros::spinOnce();

        LoopRate.sleep();
    }
}

void TrajectoryTracker::Shutdown(void)
{
    ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
}

void TrajectoryTracker::PeriodicTask(void)
{

    /* Selected trajectory generation */
    switch (traj_type){
    
        case LINEAR:

            // Linear trajectory computation
            xref    = a_coeff * t;
            dxref   = a_coeff;
            yref    = b_coeff * t;
            dyref   = b_coeff;

            break;

        case PARABOLIC:

            // Parabolic trajectory computation
            xref    = t;
            dxref   = 1;
            yref    = parabola_convexity * std::pow(std::cos(w*t),2.0);
            dyref   = 2 * parabola_convexity * t;

            break;

        case CIRCLE:

            // Circular trajectory computation
            xref    = R*(std::cos(w*t)-1.0);
            dxref   = -w*R*std::sin(w*t);
            yref    = R*std::sin(w*t);
            dyref   = w*R*std::cos(w*t);

            break;

        case EIGHT:

            // Eight shaped trajectory computation
            xref    = a*std::sin(w*t);
            dxref   = w*a*std::cos(w*t);  
            yref    = a*std::sin(w*t)*std::cos(w*t);
            dyref   = w*a*(std::pow(std::cos(w*t),2.0)-std::pow(std::sin(w*t),2.0)); 

            break;

        case CYCLOIDAL:

            // Cycloidal trajectory computation
            xref    = cycloid_radius * (t - std::sin(t));
            dxref   = cycloid_radius - std::cos(t);
            yref    = cycloid_radius * (1 - std::cos(t));
            dyref   = std::sin(t);

            break;

        default:
            ROS_INFO("Uknown selected trajectory type! (%d was selected).", traj_type);
            throw std::invalid_argument( "Uknown selected trajectory type!" );
            break;
    }
    
    // translate trajectory from point L to point P 
    L_to_P(xref, yref, xPref, yPref);

    /* Trajectory tracking law */ 
    // compute position error
    double xP_error = xPref - xP;
    double yP_error = yPref - yP;

    // get current time
    double t_new = ros::Time::now().toSec();

    // compute integral term
    x_int_term += xP_error * (t_new - t);
    y_int_term += yP_error * (t_new - t);

    // PI + FFWD controller 
    vPx = FFWD * dxref + Kp * xP_error + Ki * x_int_term;
    vPy = FFWD * dyref + Kp * yP_error + Ki * y_int_term;

    // update time
    t = t_new;

    /* Publishing vehicle commands (msg->data[0] = velocity of point P along x direction
                                    msg->data[1] = velocity of point P along y direction)
                                    msg->data[2] = t */
    std_msgs::Float64MultiArray msg;
    msg.data.push_back(vPx);
    msg.data.push_back(vPy);
    msg.data.push_back(t);
    virtual_velocities_publisher.publish(msg);
}
