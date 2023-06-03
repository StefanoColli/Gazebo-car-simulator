#ifndef TRAJECTORY_TRACKER_H
#define TRAJECTORY_TRACKER_H

#include "ros/ros.h"
#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <dynamic_reconfigure/server.h>
#include <trajectory_tracker/TrajTrackerConfig.h>

#define RUN_PERIOD_DEFAULT 0.1
/* Used only if the actual value of the period is not retrieved from the ROS parameter server */

#define NAME_OF_THIS_NODE "trajectory_tracker"

class TrajectoryTracker
{
  private: 
    ros::NodeHandle Handle;

    /* ROS topics */
    ros::Publisher virtual_velocities_publisher, reference_publisher;
    ros::Subscriber odometry_subscriber;

    dynamic_reconfigure::Server<trajectory_tracker::TrajTrackerConfig> config_server;
    dynamic_reconfigure::Server<trajectory_tracker::TrajTrackerConfig>::CallbackType f;

    /* Parameters from ROS parameter server */
    int traj_type; // desired trajectory to follow
    double a_coeff, b_coeff; // linear trajectory parameters
    double parabola_convexity; // parabolic trajectory parameter
    double R, W; // circular trajectory parameters
    double a, w; // eight-shape trajectory parameters 
    double cycloid_radius; // cycloidal trajectory parameter
    double PL_distance; // distance from the center of the bicycle L to the selected point P (L and P are on the bicycle chassis line) 


    /* Node state variables */
    double xref, dxref, yref, dyref;  //trajectory of the robot frame origin
    double xPref, yPref; //virtual trajectory for point P
    double vPx, vPy; //computed virtual velocities for point P
    double xP, yP, theta; //virtual positions of P and orientation theta from the robot
    double prev_xP_error, prev_yP_error; //error of the previous iteration
    bool active = false; //if the controller is active or not (if active it starts trajectory tracking)
    double t; //current time
    double traj_starting_time; //trajectory time and trajectory starting time 

    /* Controller variables */
    bool FFWD; //speed feedforward flag (on/off)
    double Kp, Ki, Kd;  //proportional, integral and derivative gains
    double x_int_term, y_int_term;  //integral terms

    /* Node periodic task */
    void Periodic_task(void);
    /* Convert coordinates from local robot frame into selected point P frame */ 
    void L_to_P(const double xlocal, const double ylocal, double &xP, double &yP);
    /* Compute the coordinates of hte next setpoint for the trajectory chosen */
    void Compute_trajectory_step();
    /* Compute the control action as suitable virtual velocities */
    void Control_law();

  public:

    typedef enum e_trajectory_type {
        LINEAR = 0,
        PARABOLIC = 1,
        CIRCLE = 2,
        EIGHT = 3,
        CYCLOIDAL = 4
    } trajectory_type;

    double RunPeriod;

    void Prepare(void);
    
    void Run_periodically(float Period);
    
    void Shutdown(void);
    
    void Odometry_message_callback(const nav_msgs::Odometry::ConstPtr& msg);

    void Reconfigure_callback(const trajectory_tracker::TrajTrackerConfig& config, uint32_t level);

};

#endif /* TRAJECTORY_TRACKER_H */
