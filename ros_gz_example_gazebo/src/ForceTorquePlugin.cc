#include "ros_gz_example_gazebo/ForceTorquePlugin.hh"

#include <ignition/plugin/Register.hh>

IGNITION_ADD_PLUGIN(
    ros_gz_example_gazebo::ForceTorquePlugin,
    gz::sim::System,
    ros_gz_example_gazebo::ForceTorquePlugin::ISystemConfigure,
    ros_gz_example_gazebo::ForceTorquePlugin::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(
    ros_gz_example_gazebo::ForceTorquePlugin,
    "ros_gz_example_gazebo::ForceTorquePlugin")

namespace ros_gz_example_gazebo
{
void ForceTorquePlugin::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager & /*_eventMgr*/)
{
    gz::sim::Model model(_entity);
    if (!model.Valid(_ecm))
    {
        ignerr << "ForceTorquePlugin must be attached to a model entity.\n";
        return;
    }

    // 모델의 주요 링크 가져오기
    this->link = gz::sim::Link(model.CanonicalLink(_ecm));
    if (!this->link.Valid(_ecm))
    {
        ignerr << "Model has no valid canonical link.\n";
        return;
    }

    // SDF에서 force와 torque 읽기
    if (_sdf->HasElement("force"))
    {
        this->bodyForce = _sdf->Get<ignition::math::Vector3d>("force");
    }
    if (_sdf->HasElement("torque"))
    {
        this->bodyTorque = _sdf->Get<ignition::math::Vector3d>("torque");
    }

    ignmsg << "ForceTorquePlugin configured with force: " << this->bodyForce
           << ", torque: " << this->bodyTorque << "\n";
}

void ForceTorquePlugin::PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
{
    if (_info.paused)
    {
        return;
    }

    if (!this->link.Valid(_ecm))
    {
        ignerr << "Link is no longer valid.\n";
        return;
    }

    // 링크에 힘과 토크 적용
    this->link.AddWorldWrench(_ecm, this->bodyForce, this->bodyTorque);

    ignmsg << "Applying force: " << this->bodyForce
           << ", torque: " << this->bodyTorque << " to link.\n";
}
} // namespace ros_gz_example_gazebo

