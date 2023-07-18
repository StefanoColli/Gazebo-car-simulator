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
    t = ros::Time::now().toSec();

    // Surface type
    FullParamName = ros::this_node::getName()+"/surface_type";
    if (false == Handle.getParam(FullParamName, surface_type))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());
    
   sub_input_commands_ = node_handle_.subscribe("/vesc/low_level/ackermann_cmd_mux/output", 1000, &CarCommandsFr::inputCommandsCallback, this);
   pub_acc_left_rear_wheel = node_handle_.advertise<std_msgs::Float64>("/racecar/left_rear_wheel_velocity_controller/command", 1000);
   pub_acc_right_rear_wheel = node_handle_.advertise<std_msgs::Float64>("/racecar/right_rear_wheel_velocity_controller/command", 1000);
   pub_acc_left_front_wheel = node_handle_.advertise<std_msgs::Float64>("/racecar/left_front_wheel_velocity_controller/command", 1000);
   pub_acc_right_front_wheel = node_handle_.advertise<std_msgs::Float64>("/racecar/right_front_wheel_velocity_controller/command", 1000);
   pub_reference_speed = node_handle_.advertise<std_msgs::Float64MultiArray>("/reference_speed", 1000);

   pub_pos_left_steering_hinge = node_handle_.advertise<std_msgs::Float64>("/racecar/left_steering_hinge_position_controller/command", 1000);
   pub_pos_right_steering_hinge = node_handle_.advertise<std_msgs::Float64>("/racecar/right_steering_hinge_position_controller/command", 1000);
}

void CarCommandsFr::setParameters()
{
   switch(surface_type){
	case 1:
		Bx = 10;
		By = 10;
		Cx = 1.9;
		Cy = 1.9;
		D = 1;
		E = 0.97;
		break;
	case 2:
		Bx = 12;
		By = 12;
		Cx = 2.3;
		Cy = 2.3;
		D = 0.82;
		E = 1;
		break;
	case 3:
		Bx = 5;
		By = 5;
		Cx = 2;
		Cy = 2;
		D = 0.3;
		E = 1;
		break;
	case 4:
		Bx = 4;
		By = 4;
		Cx = 2;
		Cy = 2;
		D = 0.1;
		E = 1;
		break;
	case 5:
		Bx = 0.21;
		By = 0.017;
		Cx = 1.65;
		Cy = 1.3;
		D = -548;
		E = 0.2;
		break;
	default:
		ROS_INFO("Any surface_type selected");
		throw std::invalid_argument("Unknown surface type");
		break;
   }
		
}

double CarCommandsFr::longitudinalSlip(const double speed, const double steer_angle, const double wheel_R)
{
   //double w = speed*std::sin(steer_angle)/wheel_R; //Angular velocity
   double w = speed/wheel_R;
   if(speed > wheel_R*w)
   return (speed - wheel_R*w)/speed;
   else
      return (wheel_R*w - speed)/(wheel_R*w);
}

double CarCommandsFr::lateralSlip(const double speed, const double dir_x, const double dir_y)
{
   //double t1 = speed*dir_y - (M_PI/180)*0.35;
   //double t2 = speed*dir_x - (M_PI/180)*0.2;
   double slip = std::atan(dir_y/dir_x);
   return slip;
}

double CarCommandsFr::compute_acc(const double slip, const double D, const double C, const double B, const double E, const double mass)
{
   double F = D*std::sin(C*(std::atan(B*slip - E*(B*slip - std::atan(B*slip)))));
   double acc;

   /*if(B == Bx)
   	acc = -F/mass;
   else
	acc = F/mass;*/
   acc = F/mass;

   return acc;
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

   setParameters();

   //Model values
   double mass = 5.5922;
   double wheel_radius = 0.05;

   double longSlip = longitudinalSlip(std::abs(speed_), steering_angle_, wheel_radius);

   double latSlip = lateralSlip(std::abs(speed_), direction_x_, direction_y_);

   double acceleration = std::sqrt(std::pow(compute_acc(longSlip, D, Cx, Bx, E, mass), 2) + std::pow(compute_acc(latSlip, D, Cy, By, E, mass), 2));

   double desired_speed = (speed_ + acceleration*1/100.0f)/0.1;
   std_msgs::Float64 acc;
   acc.data = desired_speed;
   std_msgs::Float64 s_ref;
   s_ref.data = speed_/0.1;

   pub_acc_left_rear_wheel.publish(acc);
   pub_acc_right_rear_wheel.publish(acc);
   pub_acc_left_front_wheel.publish(acc);
   pub_acc_right_front_wheel.publish(acc);

   //pub_acc.publish(acc);
   //pub_pos_right_steering_hinge.publish(s_angle);
   //pub_pos_left_steering_hinge.publish(s_angle);
   
   ROS_INFO("Speed: [%f], Steering_Angle: [%f], Acceleration: [%f]", 
      speed_,
      steering_angle_, 
      acc);

   ROS_INFO("-----");

   std_msgs::Float64MultiArray msg_ref;
   msg_ref.data.push_back(s_ref.data);
   msg_ref.data.push_back(acc.data);
   msg_ref.data.push_back(t);
   pub_reference_speed.publish(msg_ref);

   // update time
   t = ros::Time::now().toSec();   
}

void CarCommandsFr::shutDown()
{

}
