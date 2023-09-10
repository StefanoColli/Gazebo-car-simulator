/*
 * Copyright (C) 2018 Open Source Robotics Foundation
 *
 */
#ifndef GAZEBO_PLUGINS_WHEELSLIPPLUGIN_HH_
#define GAZEBO_PLUGINS_WHEELSLIPPLUGIN_HH_

#include <map>
#include <memory>
#include <string>

#include <ignition/math/Vector3.hh>

#include <gazebo/common/Plugin.hh>
#include <gazebo/util/system.hh>

namespace gazebo
{
  // Forward declare private data class
  class wheel_slip_pacejka_private;

  /// \brief A plugin that updates ODE wheel slip parameters based
  /// on linear wheel spin velocity (radius * spin rate).
  /// It currently assumes that the fdir1 friction parameter is set
  /// parallel to the joint axis (often [0 0 1]) and that the link
  /// origin is on the joint axis.
  /// The ODE slip parameter is documented as Force-Dependent Slip
  /// (slip1, slip2) in the ODE user guide:
  /// http://ode.org/ode-latest-userguide.html#sec_7_3_7
  /// and it has units of velocity / force (m / s / N),
  /// similar to the inverse of a viscous damping coefficient.
  /// The slip_compliance parameters specified in this plugin
  /// are unitless, representing the lateral or longitudinal slip ratio
  /// (see https://en.wikipedia.org/wiki/Slip_(vehicle_dynamics) )
  /// to tangential force ratio (tangential / normal force).
  /// Note that the maximum force ratio is the friction coefficient.
  /// At each time step, these compliance are multipled by
  /// the linear wheel spin velocity and divided by the wheel_normal_force
  /// parameter specified below in order to match the units of the ODE
  /// slip parameters.
  ///
  /// ( Other resource : https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2950/page/1 )
  /// ( Other resource : http://code.eng.buffalo.edu/dat/sites/tire/tire.html )
  ///
  /// A graphical interpretation of these parameters is provided below
  /// for a positive value of slip compliance.
  /// The horizontal axis corresponds to the slip ratio at the wheel,
  /// and the vertical axis corresponds to the tangential force ratio
  /// (tangential / normal force).
  /// As wheel slip increases, the tangential force increases until
  /// it reaches the maximum set by the friction coefficient.
  /// The slip compliance corresponds to the inverse of the slope
  /// of the force before it reaches the maximum value.
  /// A slip compliance of 0 corresponds to a completely vertical
  /// portion of the plot below.
  /// As slip compliance increases, the slope decreases.
  ///
  /** \verbatim
        |                                            .
        |      _________ friction coefficient        .
        |     /                                      .
        |    /|                                      .
        |   /-┘ slope is inverse of                  .
        |  /    slip compliance                      .
        | /                                          .
        |/                                           .
      --+-------------------------- slipRatio
        |

    <plugin filename="libWheelSlipPlugin.so" name="wheel_slip">
      <wheel link_name="wheel_front_left">
        <slip_compliance_lateral>0</slip_compliance_lateral>
        <slip_compliance_longitudinal>0.1</slip_compliance_longitudinal>
        <wheel_normal_force>100</wheel_normal_force>
      </wheel>
      <wheel link_name="wheel_front_right">
        <slip_compliance_lateral>0</slip_compliance_lateral>
        <slip_compliance_longitudinal>0.1</slip_compliance_longitudinal>
        <wheel_normal_force>100</wheel_normal_force>
      </wheel>
      <wheel link_name="wheel_rear_left">
        <slip_compliance_lateral>0</slip_compliance_lateral>
        <slip_compliance_longitudinal>0.1</slip_compliance_longitudinal>
        <wheel_normal_force>80</wheel_normal_force>
      </wheel>
      <wheel link_name="wheel_rear_right">
        <slip_compliance_lateral>0</slip_compliance_lateral>
        <slip_compliance_longitudinal>0.1</slip_compliance_longitudinal>
        <wheel_normal_force>80</wheel_normal_force>
      </wheel>
    </plugin>
   \endverbatim */
  class GZ_PLUGIN_VISIBLE wheel_slip_pacejka : public ModelPlugin
  {
    /// \brief Constructor.
    public: wheel_slip_pacejka();

    /// \brief Destructor.
    public: virtual ~wheel_slip_pacejka();

    // Documentation inherited
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    // Documentation inherited
    public: virtual void Fini();

    /// \brief Get parent model.
    /// \return pointer to parent model.
    public: physics::ModelPtr GetParentModel() const;

    public: double longitudinalSlip(const double _speed, const double _spin_speed) const;
    public: double lateralSlip(const double dir_x, const double dir_y) const;

    /// \brief Get wheel slip measurements.
    /// \param[out] _out Map of wheel name to a Vector3 of slip velocities.
    /// The Vector3.X value is the longitudinal slip in m/s,
    /// the Vector3.Y value is the lateral slip in m/s, and
    /// the Vector3.Z value is the product of radius and spin rate in m/s.
    public: void GetSlips(std::map<std::string, ignition::math::Vector3d> &_out, double &_long_slip, double &_lat_slip)
            const;

    /// \brief Set unitless lateral slip compliance for all wheels.
    /// \param[in] _compliance unitless slip compliance to set.
    public: void SetSlipComplianceLateral(const double _compliance);

    /// \brief Set unitless lateral slip compliance for a particular wheel.
    /// \param[in] _wheel_name name of the wheel link on which _compliance should be set.
    /// \param[in] _compliance unitless slip compliance to set.
    public: void SetSlipComplianceLateral(std::string _wheel_name, const double _compliance);

    /// \brief Set unitless longitudinal slip compliance for all wheels.
    /// \param[in] _compliance unitless slip compliance to set.
    public: void SetSlipComplianceLongitudinal(const double _compliance);

    /// \brief Set unitless longitudinal slip compliance for a particular wheel.
    /// \param[in] _wheel_name name of the wheel link on which _compliance should be set.
    /// \param[in] _compliance unitless slip compliance to set.
    public: void SetSlipComplianceLongitudinal(std::string _wheel_name, const double _compliance);

    /// \brief Transport callback for setting lateral slip compliance.
    /// \param[in] _msg Slip compliance encoded as string.
    private: void OnLateralCompliance(ConstGzStringPtr &_msg);

    /// \brief Transport callback for setting longitudinal slip compliance.
    /// \param[in] _msg Slip compliance encoded as string.
    private: void OnLongitudinalCompliance(ConstGzStringPtr &_msg);

    public: double MF_Force(const double _slip, const int _road, const int _XY, const int _FR, const int LatLong);

    /// brief Get friction coefficients for each wheel
    /// \return Map of wheel name to a Vector2 of friction coefficients
    /// The Vector2.X value is the friction coefficient in the primary direction and
    /// the Vector2.Y value is the friction coefficient in the secondary direction.
    public: std::map<std::string, ignition::math::Vector2d> GetFrictionCoefficients();

    /// \brief Set the friction coefficient in the primary direction for a particular wheel.
    /// \param[in] _wheel_name name of the wheel link on which _mu should be set.
    /// \param[in] _mu Friction coefficient.
    /// \return True if the friction coefficient was successfully set. False otherwise.
    public: bool SetMuPrimary(const std::string &_wheel_name, double _mu);

    /// \brief Set the friction coefficient in the secondary direction for a particular wheel.
    /// \param[in] _wheel_name name of the wheel link on which _mu should be set.
    /// \param[in] _mu Friction coefficient.
    /// \return True if the friction coefficient was successfully set. False otherwise.
    public: bool SetMuSecondary(const std::string &_wheel_name, double _mu);

    /// \brief Update the plugin. This is updated every iteration of
    /// simulation.
    private: void Update();

    /// \brief Private data pointer.
    private: std::unique_ptr<wheel_slip_pacejka_private> data_ptr;
    //private:
	//double Cx, Cy, Bx, By, Dx, Dy, Ex, Ey;
    //public: void set_Pacejka_Params(double _Cx, double _Cy, double _Bx, double _By, double _Dx, double _Dy, double _Ex, double _Ey);
  };
}
#endif
