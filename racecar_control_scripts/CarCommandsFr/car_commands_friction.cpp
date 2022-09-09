#include "car_commands_fr/car_commands_fr.h"

#include "ackermann_msgs/AckermannDriveStamped.h"

#define M_PI           3.14159265358979323846  /* pi */

CarCommandsFr::CarCommandsFr(ros::NodeHandle n)
{
    node_handle_ = n;

    node_handle_.getParam("/car_control/initial_speed", speed_);
    node_handle_.getParam("/car_control/initial_turn", turn_);
    direction_x_ = 1;
    direction_y_ = 0;
}

void CarCommandsFr::prepare()
{
   sub_input_commands_ = node_handle_.subscribe("/vexc/ackermann_cmd_mux/input/teleop", 1000, &CarCommandsFr::inputCommandsCallback, this);
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

void CarCommandsFr::inputsCommandsCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& ackmsg)
{
   double speed_ = ackmsg->drive->speed;
   double steering_angle_ = ackmsg->drive->steering_angle;
   double direction_x_ = speed_*std::cos(steering_angle_);
   double direction_y_ = speed_*std::sin(steering_angle_);

   //MF coefficients
   double B = 10000000.0 //tiffness factor kp
   double Cy = 1.30 //Lateral force shape factor
   double Cx = 1.65 //Longitudinal force shape factor
   double D = 1 //Peak factor mu*Fz
   double Ey = (B - std::tan(M_PI/(2*Cy)))/(B-std::atan(B)) //Lateral curvature factor
   double Ex = (B - std::tan(M_PI/(2*Cx)))/(B-std::atan(B)) //Longitudinal curvature factor

   //Model values
   double mass = 5.6922
   double wheel_radius = 0.05

   double longSlip = longitudinalSlip(speed_, steering_angle_, wheel_radius);

   double latSlip = lateralSlip(speed_, direction_x_, direction_y_);

   double acceleration = acceleration_X(longSlip, D, Cx, B, Ex, mass)*direction_x_ + acceleration_Y(latSlip, D, Cy, B, Ey, mass)*direction_y_;

   ROS_INFO("Speed: [%f], Steering_Angle: [%f], Acceleration: [%f]", 
      speed_,
      steering_angle_, 
      acceleration);

   ROS_INFO("-----");   
}

void CarCommands::shutDown()
{

}