/*
 * Copyright (C) 2017 Open Source Robotics Foundation
 *
*/
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <sdf/sdf.hh>

#include "../include/gazebo_ros_wheel_slip.h"

namespace gazebo
{

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosWheelSlip)

/////////////////////////////////////////////////
GazeboRosWheelSlip::GazeboRosWheelSlip()
{
}

/////////////////////////////////////////////////
GazeboRosWheelSlip::~GazeboRosWheelSlip()
{
  // Custom Callback Queue
  this->queue_.clear();
  this->queue_.disable();

  delete this->dyn_srv_;

  this->rosnode_->shutdown();
  delete this->rosnode_;
}

/////////////////////////////////////////////////
void GazeboRosWheelSlip::configCallback(gazebo_plugins::WheelSlipConfig &config, uint32_t /*level*/)
{
  if (config.slip_compliance_unitless_lateral >= 0)
  {
    ROS_INFO_NAMED("wheel_slip", "Reconfigure request for the gazebo ros wheel_slip: %s. New lateral slip compliance: %.3e",
             this->GetParentModel()->GetScopedName().c_str(),
             config.slip_compliance_unitless_lateral);
    this->SetSlipComplianceLateral(config.slip_compliance_unitless_lateral);
  }
  if (config.slip_compliance_unitless_longitudinal >= 0)
  {
    ROS_INFO_NAMED("wheel_slip", "Reconfigure request for the gazebo ros wheel_slip: %s. New longitudinal slip compliance: %.3e",
             this->GetParentModel()->GetScopedName().c_str(),
             config.slip_compliance_unitless_longitudinal);
    this->SetSlipComplianceLongitudinal(config.slip_compliance_unitless_longitudinal);
  }

  /*Cx = config.Cx;
  Cy = config.Cx;
  Bx = config.Bx;
  By = config.By;
  Dx = config.Dx;
  Dy = config.Dy;
  Ex = config.Ex;
  Ey = config.Ey; 
  wheel_slip_pacejka::set_Pacejka_Params(Cx, Cy, Bx, By, Dx, Dy, Ex, Ey);
*/
}

/*void GazeboRosWheelSlip::reconfig_callback(gazebo_ros_wheel_slip::WheelPacejkaConfig &config, uint32_t level)
{
  Cx = config.Cx;
  Cy = config.Cx;
  Bx = config.Bx;
  By = config.By;
  Dx = config.Dx;
  Dy = config.Dy;
  Ex = config.Ex;
  Ey = config.Ey; 
  wheel_slip_pacejka::set_Pacejka_Params(Cx, Cy, Bx, By, Dx, Dy, Ex, Ey);

}*/

/////////////////////////////////////////////////
// Load the controller
void GazeboRosWheelSlip::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  std::cout <<"gazebo wheel_slip_plugin LOADING"<<std::endl;
  // Load the plugin
  wheel_slip_pacejka::Load(_parent, _sdf);
  
  if (_sdf->HasElement("robotNamespace"))
  {
    this->robotNamespace_ = _sdf->Get<std::string>("robotNamespace") + "/";
  }
  if (this->robotNamespace_.empty() ||
      this->robotNamespace_ == "/" ||
      this->robotNamespace_ == "//")
  {
    this->robotNamespace_ = "wheel_slip/";
  }
  this->robotNamespace_ = _parent->GetName() + "/" + this->robotNamespace_;

  // Init ROS
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("wheel_slip", "Not loading plugin since ROS hasn't been "
          << "properly initialized.  Try starting gazebo with ros plugin:\n"
          << "  gazebo -s libgazebo_ros_api_plugin.so\n");
    return;
  }

  this->rosnode_ = new ros::NodeHandle(this->robotNamespace_);

  // set up dynamic reconfigure
  dyn_srv_ =
    new dynamic_reconfigure::Server<gazebo_plugins::WheelSlipConfig>
    (*this->rosnode_);
  dynamic_reconfigure::Server<gazebo_plugins::WheelSlipConfig>
    ::CallbackType f =
    boost::bind(&GazeboRosWheelSlip::configCallback, this, _1, _2);
  dyn_srv_->setCallback(f);
  // Custom Callback Queue
  this->callbackQueueThread_ =
    boost::thread(boost::bind(&GazeboRosWheelSlip::QueueThread, this));

  /*dyn_srv_pac_ =
    new dynamic_reconfigure::Server<gazebo_ros_wheel_slip::WheelPacejkaConfig>
    (*this->rosnode_);
  dynamic_reconfigure::Server<gazebo_ros_wheel_slip::WheelPacejkaConfig>
    ::CallbackType f_p =
    boost::bind(&GazeboRosWheelSlip::reconfig_callback, this, _1, _2);
  dyn_srv_pac_->setCallback(f_p);
  // Custom Callback Queue
  /*this->callbackQueueThread_ =
    boost::thread(boost::bind(&GazeboRosWheelSlip::QueueThread, this));*/
  ROS_INFO("Dynamic reconfigure initialized");
}

/////////////////////////////////////////////////
void GazeboRosWheelSlip::QueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->queue_.callAvailable(ros::WallDuration(timeout));
  }
}
}
