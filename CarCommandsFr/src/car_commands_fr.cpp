#include "car_commands_fr/car_commands_fr.h"
#include <std_msgs/Float64.h>

#include "ackermann_msgs/AckermannDriveStamped.h"

#define M_PI           3.14159265358979323846  /* pi */

CarCommandsFr::CarCommandsFr(ros::NodeHandle n)
{
    node_handle_ = n;
}

void CarCommandsFr::prepare()
{
    std::string FullParamName;

    // Surface type
    FullParamName = ros::this_node::getName()+"/surface_type";
    if (false == Handle.getParam(FullParamName, surface_type))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());
    
   sub_input_commands_ = node_handle_.subscribe("/vesc/low_level/ackermann_cmd_mux/output", 1000, &CarCommandsFr::inputCommandsCallback, this);
   pub_acc_left_rear_wheel = node_handle_.advertise<std_msgs::Float64>("/racecar/left_rear_wheel_velocity_controller/command", 1000);
   pub_acc_right_rear_wheel = node_handle_.advertise<std_msgs::Float64>("/racecar/right_rear_wheel_velocity_controller/command", 1000);
   pub_acc_left_front_wheel = node_handle_.advertise<std_msgs::Float64>("/racecar/left_front_wheel_velocity_controller/command", 1000);
   pub_acc_right_front_wheel = node_handle_.advertise<std_msgs::Float64>("/racecar/right_front_wheel_velocity_controller/command", 1000);

   pub_pos_left_steering_hinge = node_handle_.advertise<std_msgs::Float64>("/racecar/left_steering_hinge_position_controller/command", 1000);
   pub_pos_right_steering_hinge = node_handle_.advertise<std_msgs::Float64>("/racecar/right_steering_hinge_position_controller/command", 1000);
}

void CarCommandsFr::setParameters()
{
   switch(surface_type){
	case DRY:
		B = 10;
		C = 1.9;
		D = 1;
		E = 0.97;
		break;
	case WET:
		B = 12;
		C = 2.3;
		D = 0.82;
		E = 1;
		break;
	case SNOW:
		B = 5;
		C = 2;
		D = 0.3;
		E = 1;
		break;
	case ICE:
		B = 4;
		C = 2;
		D = 0.1;
		E = 1;
		break;
	case CUSTOM:
		
   }
		
}

double CarCommandsFr::longitudinalSlip(const double speed, const double steer_angle, const double wheel_R)
{
   double w = speed*std::sin(steer_angle)/wheel_R; //Angular velocity
   if(speed > wheel_R*w)
      return (speed - wheel_R*w)/speed;
   else
      return (wheel_R*w - speed)/(wheel_R*w);
}

double CarCommandsFr::lateralSlip(const double speed, const double dir_x, const double dir_y)
{
   double t1 = speed*dir_y - (M_PI/180)*0.35;
   double t2 = speed*dir_x - (M_PI/180)*0.2;
   double slip = std::atan(t1/t2);
   return slip;
}

double CarCommandsFr::acceleration_X(const double long_Slip, const double D, const double Cx, const double B, const double Ex, const double mass)
{
   double Fx = D*std::sin(Cx*(std::atan(B*long_Slip - Ex*(B*long_Slip - std::atan(B*long_Slip)))));
   double accx = -Fx/mass;
   return accx;
}

double CarCommandsFr::acceleration_Y(const double lat_Slip, const double D, const double Cy, const double B, const double Ey, const double mass)
{
   double Fy = D*std::sin(Cy*(std::atan(B*lat_Slip - Ey*(B*lat_Slip - std::atan(B*lat_Slip)))));
   double accy = Fy/mass;
   return accy; 
}

void CarCommandsFr::runPeriodically()
{
   
}

void CarCommandsFr::inputCommandsCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& ackmsg)
{
   double speed_ = ackmsg->drive.speed;
   double steering_angle_ = ackmsg->drive.steering_angle;
   double direction_x_ = speed_*std::cos(steering_angle_);
   double direction_y_ = speed_*std::sin(steering_angle_);

   //MF coefficients
   /*double B = 10000000.0; //tiffness factor kp
   double Cy = 1.30; //Lateral force shape factor
   double Cx = 1.65; //Longitudinal force shape factor
   double D = 1; //Peak factor mu*Fz
   double Ey = (B - std::tan(M_PI/(2*Cy)))/(B-std::atan(B)); //Lateral curvature factor
   double Ex = (B - std::tan(M_PI/(2*Cx)))/(B-std::atan(B)); //Longitudinal curvature factor
*/

   //Model values
   double mass = 5.6922;
   double wheel_radius = 0.5;

   double longSlip = longitudinalSlip(speed_, steering_angle_, wheel_radius);

   double latSlip = lateralSlip(speed_, direction_x_, direction_y_);

   double acceleration = std::sqrt(std::pow(acceleration_X(longSlip, D, Cx, B, Ex, mass), 2) + std::pow(acceleration_Y(latSlip, D, Cy, B, Ey, mass), 2));
   std_msgs::Float64 acc;
   acc.data = acceleration/0.01;
   std_msgs::Float64 s_angle;
   s_angle.data = steering_angle_;

   pub_acc_left_rear_wheel.publish(acc);
   pub_acc_right_rear_wheel.publish(acc);
   pub_acc_left_front_wheel.publish(acc);
   pub_acc_right_front_wheel.publish(acc);

   //pub_pos_right_steering_hinge.publish(s_angle);
   //pub_pos_left_steering_hinge.publish(s_angle);
   
   ROS_INFO("Speed: [%f], Steering_Angle: [%f], Acceleration: [%f]", 
      speed_,
      s_angle, 
      acc);

   ROS_INFO("-----");   
}

void CarCommandsFr::shutDown()
{

}
