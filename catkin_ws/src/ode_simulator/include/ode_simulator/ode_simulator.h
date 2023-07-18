#ifndef ODE_SIMULATOR_H_
#define ODE_SIMULATOR_H_

#include "ros/ros.h"
#include "rosgraph_msgs/Clock.h"
#include "singletrack_ode.h"
#include "tf/tf.h"
#include "std_msgs/Float64MultiArray.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "nav_msgs/Odometry.h"

class OdeSimulator
{
public:
    void Prepare(void);

    void RunPeriodically(void);

    void Shutdown(void);

private:
    typedef struct s_parameters 
    {
        double dt; /**< Integration step */

        int tyre_model; /**< Tyre model */

        double r0; /**< Initial yaw rate */
        double beta0; /**< Initial sideslip */
        double x0; /**< Initial position x */
        double y0; /**< Initial position y */
        double psi0; /**< Initial heading */

        double m; /**< Vehicle mass */
        double a; /**< Distance of COG from front axle */
        double b; /**< Distance of COG from rear axle */
        double Cf; /**< Front cornering stiffness */
        double Cr; /**< Rear cornering stiffness */
        double mu; /**< Ground friction coefficient */
        double Iz; /**< Yaw moment of inertia */
    } t_parameters;

    ros::NodeHandle handle_;

    /* Node state variables */
    SingletrackODE* simulator_;

    /* Parameters from ROS parameter server */
    t_parameters server_params_;

    /* ROS topics */
    ros::Subscriber vehicle_command_subscriber_;
    ros::Publisher vehicle_state_publisher_;
    ros::Publisher vehicle_odom_publisher_;
    ros::Publisher clock_publisher_;

    /* ROS topic callbacks */
    void vehicleCommand_MessageCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg);
};

#endif /* ODE_SIMULATOR_H_ */
