#ifndef ROS_GZ_EXAMPLE_GAZEBO_FORCE_TORQUE_PLUGIN_HH
#define ROS_GZ_EXAMPLE_GAZEBO_FORCE_TORQUE_PLUGIN_HH

#include <gz/sim/System.hh>
#include <gz/sim/Link.hh>
#include <ignition/math/Vector3.hh>

namespace ros_gz_example_gazebo
{
/// \brief Plugin to apply force and torque to a link.
class ForceTorquePlugin : public gz::sim::System,
                          public gz::sim::ISystemConfigure,
                          public gz::sim::ISystemPreUpdate
{
  public:
    /// \brief Configure the plugin.
    /// \param[in] _entity Entity to which the plugin is attached.
    /// \param[in] _sdf SDF configuration.
    /// \param[in] _ecm Entity component manager.
    /// \param[in] _eventMgr Event manager.
    void Configure(
        const gz::sim::Entity &_entity,
        const std::shared_ptr<const sdf::Element> &_sdf,
        gz::sim::EntityComponentManager &_ecm,
        gz::sim::EventManager &_eventMgr) override;

    /// \brief Apply force and torque in the pre-update phase.
    /// \param[in] _info Simulation update info.
    /// \param[in] _ecm Entity component manager.
    void PreUpdate(
        const gz::sim::UpdateInfo &_info,
        gz::sim::EntityComponentManager &_ecm) override;

  private:
    /// \brief The link to which the force and torque are applied.
    gz::sim::Link link;

    /// \brief Force to apply in the body frame.
    ignition::math::Vector3d bodyForce{0, 0, 0};

    /// \brief Torque to apply in the body frame.
    ignition::math::Vector3d bodyTorque{0, 0, 0};
};
} // namespace ros_gz_example_gazebo

#endif

