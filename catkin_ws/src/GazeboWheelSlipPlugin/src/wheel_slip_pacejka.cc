/*
 * Copyright (C) 2018 Open Source Robotics Foundation
*/
#include <map>
#include <mutex>
#include <ros/ros.h>

#include <ignition/common/Profiler.hh>

#include <std_msgs/Float64.h>

#include <gazebo/common/Assert.hh>
#include <gazebo/common/CommonTypes.hh>
#include <gazebo/common/Console.hh>
#include <gazebo/common/Events.hh>

#include <gazebo/physics/Collision.hh>
#include <gazebo/physics/CylinderShape.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Shape.hh>
#include <gazebo/physics/SphereShape.hh>
#include <gazebo/physics/SurfaceParams.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/ode/ODESurfaceParams.hh>
#include <gazebo/physics/ode/ODETypes.hh>

#include <gazebo/transport/Node.hh>
#include <gazebo/transport/Publisher.hh>
#include <gazebo/transport/Subscriber.hh>


#include "../include/wheel_slip_pacejka.hh"

namespace gazebo
{
  namespace physics
  {
    typedef boost::weak_ptr<physics::Joint> joint_weak_ptr;
    typedef boost::weak_ptr<physics::Link> link_weak_ptr;
    typedef boost::weak_ptr<physics::Model> model_weak_ptr;
    typedef boost::weak_ptr<physics::ODESurfaceParams> ODE_surface_params_weak_ptr;
  }

  class wheel_slip_pacejka_private
  {

    public: class link_surface_params
    {
      /// \brief Pointer to wheel spin joint.
      public: physics::joint_weak_ptr joint;

      /// \brief Pointer to ODESurfaceParams object.
      public: physics::ODE_surface_params_weak_ptr surface;

      /// \brief Wheel normal force estimate used to compute slip
      /// compliance for ODE, which takes units of 1/N.
      public: double wheel_normal_force = 0;

      /// \brief Wheel radius extracted from collision shape if not
      /// specified as xml parameter.
      public: double wheel_radius = 0;

      public: double longitudinal_slip;
      public: double lateral_slip;
      public: int road = 1; //1: Dry , 2: Wet , 3: Snow , 4: Ice
      public: int front_rear = 0; //0: Front , 1: Rear

      /// \brief Publish slip for each wheel.
      public: transport::PublisherPtr slip_pub;

      /// \brief Unitless wheel slip compliance in lateral direction.
      /// The parameter should be non-negative,
      /// with a value of zero allowing no slip
      /// and larger values allowing increasing slip.
      public: double slip_compliance_lateral;

      /// \brief Unitless wheel slip compliance in longitudinal direction.
      /// The parameter should be non-negative,
      /// with a value of zero allowing no slip
      /// and larger values allowing increasing slip.
      public: double slip_compliance_longitudinal;
      
    };
    public: ros::Publisher pub_long;
    public: ros::Publisher pub_lat;
    public: ros::Publisher pub_Fx;
    public: ros::Publisher pub_Fy;
    public: double wheel_longitudinal_force;
    public: double wheel_lateral_force;
    public: double slip_lat;
    public: double slip_long;
		
    /// \brief Initial gravity direction in parent model frame.
    public: ignition::math::Vector3d init_gravity_direction;

    /// \brief Model pointer.
    public: physics::model_weak_ptr model;

    /// \brief Protect data access during transport callbacks
    public: std::mutex mutex;

    /// \brief Gazebo communication node
    /// \todo: Transition to ignition-transport in gazebo8
    public: transport::NodePtr gz_node;

    /// \brief Link and surface pointers to update.
    public: std::map<physics::link_weak_ptr,
                        link_surface_params> map_link_surface_params;

    /// \brief Link names and their pointers
    public: std::map<std::string,
            physics::link_weak_ptr> map_link_names;

    /// \brief Lateral slip compliance subscriber.
    /// \todo: Transition to ignition-transport in gazebo8.
    public: transport::SubscriberPtr lateral_compliance_sub;

    /// \brief Longitudinal slip compliance subscriber.
    /// \todo: Transition to ignition-transport in gazebo8.
    public: transport::SubscriberPtr longitudinal_compliance_sub;

    /// \brief Pointer to the update event connection
    public: event::ConnectionPtr update_connection;
  };
}

using namespace gazebo;

/////////////////////////////////////////////////
wheel_slip_pacejka::wheel_slip_pacejka()
  : data_ptr(new wheel_slip_pacejka_private)
{
}

/////////////////////////////////////////////////
wheel_slip_pacejka::~wheel_slip_pacejka()
{
}

/////////////////////////////////////////////////
void wheel_slip_pacejka::Fini()
{
  this->data_ptr->update_connection.reset();

  this->data_ptr->lateral_compliance_sub.reset();
  this->data_ptr->longitudinal_compliance_sub.reset();
  for (auto link_surface : this->data_ptr->map_link_surface_params)
  {
    link_surface.second.slip_pub.reset();
  }
  if (this->data_ptr->gz_node)
    this->data_ptr->gz_node->Fini();
}

/////////////////////////////////////////////////
void wheel_slip_pacejka::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  std::cout <<"LOADING"<<std::endl;
  GZ_ASSERT(_model, "wheel_slip_pacejka model pointer is NULL");
  GZ_ASSERT(_sdf, "wheel_slip_pacejka sdf pointer is NULL");
  ros::NodeHandle nh;
  this->data_ptr->pub_long = nh.advertise<std_msgs::Float64>("long_pub/right_front", 10);
  this->data_ptr->pub_Fx = nh.advertise<std_msgs::Float64>("fx_pub/right_front", 10);
  this->data_ptr->pub_lat = nh.advertise<std_msgs::Float64>("lat_pub/right_front", 10);
  this->data_ptr->pub_Fy = nh.advertise<std_msgs::Float64>("fy_pub/right_front", 10);

  /*Cx = 1.45;
  Cy = 1.45;
  Dx = 1;
  Dy = 1.371;
  Bx = 18;
  Ex = -5;
  By = 10;
  Ey = 0.97;*/

  this->data_ptr->model = _model;
  auto world = _model->GetWorld();
  GZ_ASSERT(world, "world pointer is NULL");
  { 
    ignition::math::Vector3d gravity = world->Gravity();
    ignition::math::Quaterniond initial_model_rot =
        _model->WorldPose().Rot();
    this->data_ptr->init_gravity_direction =
        initial_model_rot.RotateVectorReverse(gravity.Normalized());
  }

  if (!_sdf->HasElement("wheel"))
  {
    gzerr << "No wheel tags specified, plugin is disabled" << std::endl;
    return;
  }

  // Read each wheel element
  for (auto wheel_elem = _sdf->GetElement("wheel"); wheel_elem;
      wheel_elem = wheel_elem->GetNextElement("wheel"))
  {
    if (!wheel_elem->HasAttribute("link_name"))
    {
      gzerr << "wheel element missing link_name attribute" << std::endl;
      continue;
    }

    // Get link name
    auto link_name = wheel_elem->Get<std::string>("link_name");

    wheel_slip_pacejka_private::link_surface_params params;
    if (wheel_elem->HasElement("slip_compliance_lateral"))
    {
      params.slip_compliance_lateral =
        wheel_elem->Get<double>("slip_compliance_lateral");
    }
    if (wheel_elem->HasElement("slip_compliance_longitudinal"))
    {
      params.slip_compliance_longitudinal =
        wheel_elem->Get<double>("slip_compliance_longitudinal");
    }
    if (wheel_elem->HasElement("road"))
    {
      params.road =
        wheel_elem->Get<int>("road");
    }
    if (wheel_elem->HasElement("front_rear"))
    {
      params.front_rear =
        wheel_elem->Get<int>("front_rear");
    }
    if (wheel_elem->HasElement("wheel_normal_force"))
    {
      params.wheel_normal_force = wheel_elem->Get<double>("wheel_normal_force");
    }

    if (wheel_elem->HasElement("wheel_radius"))
    {
      params.wheel_radius = wheel_elem->Get<double>("wheel_radius");
    }

    auto link = _model->GetLink(link_name);
    if (link == nullptr)
    {
      gzerr << "Could not find link named [" << link_name
            << "] in model [" << _model->GetScopedName() << "]"
            << std::endl;
      continue;
    }

    auto collisions = link->GetCollisions();
    if (collisions.empty() || collisions.size() != 1)
    {
      gzerr << "There should be 1 collision in link named [" << link_name
            << "] in model [" << _model->GetScopedName() << "]"
            << ", but " << collisions.size() << " were found"
            << std::endl;
      continue;
    }
    auto collision = collisions.front();
    if (collision == nullptr)
    {
      gzerr << "Could not find collision in link named [" << link_name
            << "] in model [" << _model->GetScopedName() << "]"
            << std::endl;
      continue;
    }

    auto surface = collision->GetSurface();
    auto ode_surface =
      boost::dynamic_pointer_cast<physics::ODESurfaceParams>(surface);
    if (ode_surface == nullptr)
    {
      gzerr << "Could not find ODE Surface "
            << "in collision named [" << collision->GetName()
            << "] in link named [" << link_name
            << "] in model [" << _model->GetScopedName() << "]"
            << std::endl;
      continue;
    }
    params.surface = ode_surface;

    auto joints = link->GetParentJoints();
    if (joints.empty() || joints.size() != 1)
    {
      gzerr << "There should be 1 parent joint for link named [" << link_name
            << "] in model [" << _model->GetScopedName() << "]"
            << ", but " << joints.size() << " were found"
            << std::endl;
      continue;
    }
    auto joint = joints.front();
    if (joint == nullptr)
    {
      gzerr << "Could not find parent joint for link named [" << link_name
            << "] in model [" << _model->GetScopedName() << "]"
            << std::endl;
      continue;
    }
    params.joint = joint;

    if (params.wheel_radius <= 0)
    {
      // get collision shape and extract radius if it is a cylinder or sphere
      auto shape = collision->GetShape();
      if (shape->HasType(physics::Base::CYLINDER_SHAPE))
      {
        auto cyl = boost::dynamic_pointer_cast<physics::CylinderShape>(shape);
        if (cyl != nullptr)
        {
          params.wheel_radius = cyl->GetRadius();
        }
      }
      else if (shape->HasType(physics::Base::SPHERE_SHAPE))
      {
        auto sphere = boost::dynamic_pointer_cast<physics::SphereShape>(shape);
        if (sphere != nullptr)
        {
          params.wheel_radius = sphere->GetRadius();
        }
      }
      else
      {
        gzerr << "A positive wheel radius was not specified in the"
              << " [wheel_radius] parameter, and the the wheel radius"
              << " could not be identified automatically because a"
              << " sphere or cylinder collision shape could not be found."
              << " Skipping link [" << link_name << "]."
              << std::endl;
        continue;
      }

      // if that still didn't work, skip this link
      if (params.wheel_radius <= 0)
      {
        gzerr << "Found wheel radius [" << params.wheel_radius
              << "], which is not positive"
              << " in link named [" << link_name
              << "] in model [" << _model->GetScopedName() << "]"
              << std::endl;
        continue;
      }
    }

    if (params.wheel_normal_force <= 0)
    {
      gzerr << "Found wheel normal force [" << params.wheel_normal_force
            << "], which is not positive"
            << " in link named [" << link_name
            << "] in model [" << _model->GetScopedName() << "]"
            << std::endl;
      continue;
    }

    this->data_ptr->map_link_surface_params[link] = params;
    this->data_ptr->map_link_names[link->GetName()] = link;
  }

  if (this->data_ptr->map_link_surface_params.empty())
  {
    gzerr << "No ODE links and surfaces found, plugin is disabled" << std::endl;
    return;
  }

  // Subscribe to slip compliance updates
  this->data_ptr->gz_node = transport::NodePtr(new transport::Node());
  this->data_ptr->gz_node->Init(world->Name());

  // add publishers
  for (auto &link_surface : this->data_ptr->map_link_surface_params)
  {
    auto link = link_surface.first.lock();
    GZ_ASSERT(link, "link should still exist inside Load");
    auto &params = link_surface.second;
    std::cout << "NAME : "<< _model->GetName() << std::endl;
    params.slip_pub = this->data_ptr->gz_node->Advertise<msgs::Vector3d>(
        "~/" + _model->GetName() + "/wheel_slip/" + link->GetName());
  }

  this->data_ptr->lateral_compliance_sub = this->data_ptr->gz_node->Subscribe(
      "~/" + _model->GetName() + "/wheel_slip/lateral_compliance",
      &wheel_slip_pacejka::OnLateralCompliance, this);

  this->data_ptr->longitudinal_compliance_sub = this->data_ptr->gz_node->Subscribe(
      "~/" + _model->GetName() + "/wheel_slip/longitudinal_compliance",
      &wheel_slip_pacejka::OnLongitudinalCompliance, this);

  // Connect to the update event
  this->data_ptr->update_connection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&wheel_slip_pacejka::Update, this));
}

/////////////////////////////////////////////////
physics::ModelPtr wheel_slip_pacejka::GetParentModel() const
{
  return this->data_ptr->model.lock();
}

double wheel_slip_pacejka::longitudinalSlip(const double _speed, const double _spin_speed) const
{
   if( isnan(_speed) || isnan(_spin_speed)) return 1e-5; //_spin_speed == 0.0 &&
   else{ 
	double long_slip;
	if(_speed > _spin_speed)
	{
    if (abs(_speed) <= DBL_EPSILON)
    {
      return DBL_MIN;
    }
    
	   long_slip = (_speed - _spin_speed)/_speed;
	}else
	{
    if ( abs(_spin_speed) <= DBL_EPSILON)
    {
      return DBL_MAX;
    }
    
	   long_slip = (_spin_speed - _speed)/_spin_speed;
	}
	return (_spin_speed - _speed)/_spin_speed;
   }
}

double wheel_slip_pacejka::lateralSlip(const double dir_x, const double dir_y) const
{
   double slip = 1;
   if(abs(dir_x) >= DBL_EPSILON && !isnan(dir_x) && !isnan(dir_y))
	    slip = -std::atan(dir_y/std::abs(dir_x));
   else{
        //std::cout <<"HERE"<<std::endl;
   }

   return slip;
}

/////////////////////////////////////////////////
void wheel_slip_pacejka::GetSlips(
        std::map<std::string, ignition::math::Vector3d> &_out, double &_long_slip, double &_lat_slip) const
{
  auto model = this->GetParentModel();
  if (!model)
  {
    gzerr << "Parent model does not exist" << std::endl;
    return;
  }
  auto model_world_pose = model->WorldPose();

  std::lock_guard<std::mutex> lock(this->data_ptr->mutex);
  for (const auto &link_surface : this->data_ptr->map_link_surface_params)
  {
    auto link = link_surface.first.lock();
    if (!link)
      continue;
    auto params = link_surface.second;

    // Compute wheel velocity in parent model frame
    auto wheel_world_linear_vel = link->WorldLinearVel();
    auto wheel_model_linear_vel =
        model_world_pose.Rot().RotateVectorReverse(wheel_world_linear_vel);
    // Compute wheel spin axis in parent model frame
    auto joint = params.joint.lock();
    if (!joint)
      continue;
    auto wheel_world_axis = joint->GlobalAxis(0).Normalized();
    auto wheel_model_axis =
        model_world_pose.Rot().RotateVectorReverse(wheel_world_axis);
    // Estimate longitudinal direction as cross product of initial gravity
    // direction with wheel spin axis.
    auto longitudinal_model_axis =
        this->data_ptr->init_gravity_direction.Cross(wheel_model_axis);
    
    double spin_speed = params.wheel_radius * joint->GetVelocity(0);
    double lateral_speed = wheel_model_axis.Dot(wheel_model_linear_vel);
    double longitudinal_speed = longitudinal_model_axis.Dot(wheel_model_linear_vel);
    
    ignition::math::Vector3d slip;
    slip.X(longitudinal_speed - spin_speed);
    slip.Y(lateral_speed);
    slip.Z(spin_speed);

    //params.longitudinal_slip = this->longitudinalSlip(longitudinal_speed, spin_speed);
    //params.lateral_slip = this->lateralSlip(longitudinal_speed, lateral_speed);
    double sl_x = slip.X();
    _long_slip = this->longitudinalSlip(longitudinal_speed, spin_speed);
    _lat_slip = this->lateralSlip(longitudinal_speed, lateral_speed);

    auto name = link->GetName();
    _out[name] = slip;

    
  }
}

 /////////////////////////////////////////////////
void wheel_slip_pacejka::OnLateralCompliance(ConstGzStringPtr &_msg)
{
  try
  {
    this->SetSlipComplianceLateral(std::stod(_msg->data()));
  }
  catch(...)
  {
    gzerr << "Invalid slip compliance data[" << _msg->data() << "]\n";
  }
}

/////////////////////////////////////////////////
void wheel_slip_pacejka::OnLongitudinalCompliance(ConstGzStringPtr &_msg)
{
  try
  {
    this->SetSlipComplianceLongitudinal(std::stod(_msg->data()));
  }
  catch(...)
  {
    gzerr << "Invalid slip compliance data[" << _msg->data() << "]\n";
  }
}


/////////////////////////////////////////////////
void wheel_slip_pacejka::SetSlipComplianceLateral(const double _compliance)
{
  std::lock_guard<std::mutex> lock(this->data_ptr->mutex);
  for (auto &link_surface : this->data_ptr->map_link_surface_params)
  {
    link_surface.second.slip_compliance_lateral = _compliance;
  }
}

/////////////////////////////////////////////////
void wheel_slip_pacejka::SetSlipComplianceLateral(std::string _wheel_name, const double _compliance)
{
  std::lock_guard<std::mutex> lock(this->data_ptr->mutex);
  if (this->data_ptr->map_link_names.count(_wheel_name) > 0)
  {
    auto link = this->data_ptr->map_link_names[_wheel_name];
    this->data_ptr->map_link_surface_params[link].slip_compliance_lateral = _compliance;
  }
}

/////////////////////////////////////////////////
void wheel_slip_pacejka::SetSlipComplianceLongitudinal(const double _compliance)
{
  std::lock_guard<std::mutex> lock(this->data_ptr->mutex);
  for (auto &link_surface : this->data_ptr->map_link_surface_params)
  {
    link_surface.second.slip_compliance_longitudinal = _compliance;
  }
}

/////////////////////////////////////////////////
void wheel_slip_pacejka::SetSlipComplianceLongitudinal(std::string _wheel_name, const double _compliance)
{
  std::lock_guard<std::mutex> lock(this->data_ptr->mutex);
  if (this->data_ptr->map_link_names.count(_wheel_name) > 0)
  {
    auto link = this->data_ptr->map_link_names[_wheel_name];
    this->data_ptr->map_link_surface_params[link].slip_compliance_longitudinal = _compliance;
  }
  //Computing slip compliance longitudinal with Pacejka MF equations

   /*Parameters:
	- slip_longitudinal
	- normal force Fz
	- Longitudinal force Fx = D * sin(C * arctan(B*(slip_long+H) - E*(B*(slip_long+H) - arctan(B*(slip_long+H))))) + V

   Values:
	- slip_longitudinal = slip_ratio longitudinal (v-w*R)/v OK
	- normal force = params.normal_force
	- Pacejka MF values*/

}
/*void wheel_slip_pacejka::set_Pacejka_Params(double _Cx, double _Cy, double _Bx, double _By, double _Dx, double _Dy, double _Ex, double _Ey)
{
   Cx = _Cx;
   Cy = _Cy;
   Bx = _Bx;
   By = _By;
   Dx = _Dx;
   Dy = _Dy;
   Ex = _Ex;
   Ey = _Ey;
}*/
double wheel_slip_pacejka::MF_Force(const double _slip, const int _road, const int _XY, const int _FR, const int _LatLong)
{
   double B;
   double C = 1.45;
   double D = 1.371;
   double E;

   //_XY == 1 -> lateral Y , 0 -> longitudinal X
   //_FR == 1 -> rear , 0 -> front
   //if(!_XY && !_FR) D = 0.1;
   //else D = 1.371;

   switch(_road){
	case 1:
		B = 20;
		E = 1;
		break;
	case 2:
		B = 12;
		E = 1;
		break;
	case 3:
		B = 5;
		E = 1;
		break;
	case 4:
		B = 4;
		E = 1;
		break;
	default:
		std::cout << "Any surface_type selected" << std::endl;
		throw std::invalid_argument("Unknown surface type");
		break;
   }
   
   double F;
   double Cx = 1.45;
   double Cy = 1.45;
   double Dx = 1390*0.01371;
   double Dy = 1.371;
   double Bx = 3; //2
   double Ex = 0.52;
   double By = 10;
   double Ey = 0.97;
   //_latLong == 1 -> lateral
   if(_LatLong == 1)
	F = Dy*std::sin(Cy*(std::atan(By*_slip - Ey*(By*_slip - std::atan(By*_slip)))));
   else
   {
        //X longitudinal
        F = Dx*std::sin(Cx*(std::atan(Bx*_slip - Ex*(Bx*_slip - std::atan(Bx*_slip)))));
   }
   return F;
}

/////////////////////////////////////////////////
std::map<std::string, ignition::math::Vector2d> wheel_slip_pacejka::GetFrictionCoefficients()
{
  std::lock_guard<std::mutex> lock(this->data_ptr->mutex);
  std::map<std::string, ignition::math::Vector2d> friction_coeffs;
  for (const auto &link_surface : this->data_ptr->map_link_surface_params)
  {
    auto link = link_surface.first.lock();
    auto surface = link_surface.second.surface.lock();
    if (!link || !surface)
      continue;

    ignition::math::Vector2d friction;
    friction.X(surface->FrictionPyramid()->MuPrimary());
    friction.Y(surface->FrictionPyramid()->MuSecondary());

    friction_coeffs[link->GetName()] = friction;
  }

  return friction_coeffs;
}

/////////////////////////////////////////////////
bool wheel_slip_pacejka::SetMuPrimary(const std::string &_wheel_name, double _mu)
{
  std::lock_guard<std::mutex> lock(this->data_ptr->mutex);
  if (this->data_ptr->map_link_names.count(_wheel_name) == 0)
    return false;

  auto link = this->data_ptr->map_link_names[_wheel_name];
  auto surface = this->data_ptr->map_link_surface_params[link].surface.lock();
  if (surface == nullptr)
    return false;

  surface->FrictionPyramid()->SetMuPrimary(_mu);
  return true;
}

/////////////////////////////////////////////////
bool wheel_slip_pacejka::SetMuSecondary(const std::string &_wheel_name, double _mu)
{
  std::lock_guard<std::mutex> lock(this->data_ptr->mutex);
  if (this->data_ptr->map_link_names.count(_wheel_name) == 0)
    return false;

  auto link = this->data_ptr->map_link_names[_wheel_name];
  auto surface = this->data_ptr->map_link_surface_params[link].surface.lock();
  if (surface == nullptr)
    return false;

  surface->FrictionPyramid()->SetMuSecondary(_mu);
  return true;
}

/////////////////////////////////////////////////
void wheel_slip_pacejka::Update()
{
  IGN_PROFILE("wheel_slip_pacejka::OnUpdate");
  IGN_PROFILE_BEGIN("Update");
  // Get slip data so it can be published later
  std::map<std::string, ignition::math::Vector3d> slips;
  double longitudinal_slip;
  double lateral_slip;
  this->GetSlips(slips, longitudinal_slip, lateral_slip);

  std::lock_guard<std::mutex> lock(this->data_ptr->mutex);
  for (const auto &link_surface : this->data_ptr->map_link_surface_params)
  {
    const auto &params = link_surface.second;

    // get user-defined normal force constant
    double force = params.wheel_normal_force;

    // get link angular velocity parallel to joint axis
    ignition::math::Vector3d wheel_angular_velocity;
    auto link = link_surface.first.lock();
    if (link)
      wheel_angular_velocity = link->WorldAngularVel();

    ignition::math::Vector3d joint_axis;
    auto joint = params.joint.lock();
    if (joint)
      joint_axis = joint->GlobalAxis(0);

    double spin_angular_velocity = wheel_angular_velocity.Dot(joint_axis);

    auto surface = params.surface.lock();

    if (surface)
    {
      // As discussed in wheel_slip_pacejka.hh, the ODE slip1 and slip2
      // parameters have units of inverse viscous damping:
      // [linear velocity / force] or [m / s / N].
      // Since the slip compliance parameters supplied to the plugin
      // are unitless, they must be scaled by a linear speed and force
      // magnitude before being passed to ODE.
      // The force is taken from a user-defined constant that should roughly
      // match the steady-state normal force at the wheel.
      // The linear speed is computed dynamically at each time step as
      // radius * spin angular velocity.
      // This choice of linear speed corresponds to the denominator of
      // the slip ratio during acceleration (see equation (1) in
      // Yoshida, Hamano 2002 DOI 10.1109/ROBOT.2002.1013712
      // "Motion dynamics of a rover with slip-based traction model").
      // The acceleration form is more well-behaved numerically at low-speed
      // and when the vehicle is at rest than the braking form,
      // so it is used for both slip directions.
      double speed = params.wheel_radius * std::abs(spin_angular_velocity);

      this->data_ptr->wheel_longitudinal_force = this->MF_Force(longitudinal_slip, params.road, 0, params.front_rear, 0); 
      this->data_ptr->wheel_lateral_force = this->MF_Force(lateral_slip, params.road, 1, params.front_rear, 1); 

      //if (abs(force) <= DBL_EPSILON)
      //{
      //  this->data_ptr->slip_lat = 0;
      //  this->data_ptr->slip_long = 0;
      //  surface->slip1 = 0;
      //  surface->slip2 = 0;
      //}
      //else
      //{
        this->data_ptr->slip_lat = lateral_slip/(this->data_ptr->wheel_lateral_force/force);
        this->data_ptr->slip_long = longitudinal_slip/(this->data_ptr->wheel_longitudinal_force/force);
        surface->slip1 = speed / force * this->data_ptr->slip_lat;
        surface->slip2 = speed / force * this->data_ptr->slip_long;
      //}
      
      
      //surface->slip1 = speed / force * params.slip_compliance_lateral;
      //surface->slip2 = speed / force * params.slip_compliance_lateral;
    }

    // Try to publish slip data for this wheel
    if (link)
    {
      msgs::Vector3d msg;
      auto name = link->GetName();
      msg = msgs::Convert(slips[name]);
      if (params.slip_pub)
      {
        params.slip_pub->Publish(msg);
      }
      if(link->GetName() == "left_front_wheel")
      {
	std_msgs::Float64 msg_long;
	std_msgs::Float64 msg_lat;
	std_msgs::Float64 msg_Fx;
	std_msgs::Float64 msg_Fy;
 
	msg_long.data = longitudinal_slip;
	msg_lat.data = lateral_slip;
	msg_Fx.data = this->data_ptr->wheel_longitudinal_force;
	msg_Fy.data = this->data_ptr->wheel_lateral_force;

	this->data_ptr->pub_long.publish(msg_long);
	this->data_ptr->pub_lat.publish(msg_lat);
	this->data_ptr->pub_Fx.publish(msg_Fx);
	this->data_ptr->pub_Fy.publish(msg_Fy);
      }
    }
  }
  IGN_PROFILE_END();
}
