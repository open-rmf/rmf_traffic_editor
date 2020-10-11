#include <ignition/plugin/Register.hh>

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/Static.hh>
#include <ignition/gazebo/components/AxisAlignedBox.hh>
#include <ignition/gazebo/components/JointAxis.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/gazebo/components/JointVelocityCmd.hh>
#include <ignition/gazebo/components/JointPositionReset.hh>
#include <ignition/gazebo/components/LinearVelocityCmd.hh>
#include <ignition/gazebo/components/AngularVelocityCmd.hh>

#include <rclcpp/rclcpp.hpp>

#include <building_sim_common/utils.hpp>
#include <building_sim_common/lift_common.hpp>

// TODO remove this
using namespace ignition;
using namespace gazebo;
using namespace systems;

using namespace building_sim_common;

namespace building_sim_ign {

//==============================================================================

class IGNITION_GAZEBO_VISIBLE LiftPlugin
  : public System,
  public ISystemConfigure,
  public ISystemPreUpdate
{
private:
  rclcpp::Node::SharedPtr _ros_node;
  Entity _cabin_joint;
  Entity _lift_en;

  std::unique_ptr<LiftCommon> _lift_common = nullptr;

  bool _initialized = false;

  void create_entity_components(Entity entity, EntityComponentManager& ecm)
  {
    if (!ecm.EntityHasComponentType(entity,
      components::JointPosition().TypeId()))
      ecm.CreateComponent(entity, components::JointPosition({0}));
    if (!ecm.EntityHasComponentType(entity,
      components::JointPositionReset().TypeId()))
      ecm.CreateComponent(entity, components::JointPositionReset({0}));
    if (!ecm.EntityHasComponentType(entity,
      components::JointVelocity().TypeId()))
      ecm.CreateComponent(entity, components::JointVelocity({0}));
    if (!ecm.EntityHasComponentType(entity,
      components::JointVelocityCmd().TypeId()))
      ecm.CreateComponent(entity, components::JointVelocityCmd({0}));
  }

  std::unordered_set<Entity> get_payloads(
    Entity& lift,
    EntityComponentManager& ecm)
  {
    const auto lift_aabb = ecm.Component<components::AxisAlignedBox>(_lift_en);
    std::unordered_set<Entity> payloads;
    ecm.Each<components::Model, components::Name, components::Pose,
      components::Static>(
      [&](const Entity& entity,
      const components::Model*,
      const components::Name* nm,
      const components::Pose* pose,
      const components::Static* is_static
      ) -> bool
      {
        // Object should not be static
        const auto payload_position = pose->Data().Pos();
        if (!(is_static->Data()) && entity != lift)
        {
          if (lift_aabb->Data().Contains(payload_position))
          {
            payloads.insert(entity);
          }
        }
        return true;
      });
    return payloads;
  }

public:
  LiftPlugin()
  {
    // TODO init ros node
    // Do nothing
  }

  void Configure(const Entity& entity,
    const std::shared_ptr<const sdf::Element>& sdf,
    EntityComponentManager& ecm, EventManager& /*_eventMgr*/) override
  {
    _lift_en = entity;
    //_ros_node = gazebo_ros::Node::Get(sdf);
    // TODO get properties from sdf instead of hardcoded (will fail for multiple instantiations)
    // TODO proper rclcpp init (only once and pass args)
    auto model = Model(entity);
    char const** argv = NULL;
    if (!rclcpp::is_initialized())
      rclcpp::init(0, argv);
    std::string plugin_name("plugin_" + model.Name(ecm));
    ignwarn << "Initializing plugin with name " << plugin_name << std::endl;
    _ros_node = std::make_shared<rclcpp::Node>(plugin_name);

    RCLCPP_INFO(_ros_node->get_logger(),
      "Loading LiftPlugin for [%s]",
      model.Name(ecm).c_str());

    _lift_common = LiftCommon::make(
      model.Name(ecm),
      _ros_node,
      sdf);

    if (!_lift_common)
      return;

    const auto joint = model.JointByName(ecm, _lift_common->get_joint_name());
    if (!joint)
    {
      RCLCPP_ERROR(_ros_node->get_logger(),
        " -- Model is missing the joint [%s]",
        _lift_common->get_joint_name().c_str());
      return;
    }

    create_entity_components(joint, ecm);
    _cabin_joint = joint;

    auto position_cmd = ecm.Component<components::JointPositionReset>(
      _cabin_joint);
    position_cmd->Data()[0] = _lift_common->get_elevation();

    if (!ecm.EntityHasComponentType(_lift_en,
      components::AxisAlignedBox().TypeId()))
    {
      ecm.CreateComponent(_lift_en, components::AxisAlignedBox());
    }

    _initialized = true;

    RCLCPP_INFO(_ros_node->get_logger(),
      "Finished loading [%s]",
      model.Name(ecm).c_str());
  }

  void PreUpdate(const UpdateInfo& info, EntityComponentManager& ecm) override
  {
    // TODO parallel thread executor?
    rclcpp::spin_some(_ros_node);
    if (!_initialized)
      return;

    // Send update request
    const double t =
      (std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).
      count()) * 1e-9;
    const double position = ecm.Component<components::JointPosition>(
      _cabin_joint)->Data()[0];
    const double velocity = ecm.Component<components::JointVelocity>(
      _cabin_joint)->Data()[0];

    auto result = _lift_common->update(t, position, velocity);

    // Apply motion to the joint
    auto vel_cmd = ecm.Component<components::JointVelocityCmd>(
      _cabin_joint);
    vel_cmd->Data()[0] = result.velocity;

    // For TPE: Move any payloads that need to be manually moved
    // (i.e. have a LinearVelocityCmd component that exists)
    std::unordered_set<Entity> payloads = get_payloads(_lift_en, ecm);
    for (const Entity& payload : payloads)
    {
      if (ecm.EntityHasComponentType(payload,
        components::LinearVelocityCmd().TypeId()))
      {
        ignition::math::Vector3d old_lin_vel_cmd =
          ecm.Component<components::LinearVelocityCmd>(payload)->Data();
        ecm.Component<components::LinearVelocityCmd>(payload)->Data() = {
          old_lin_vel_cmd[0], old_lin_vel_cmd[1], result.velocity};
      }
    }
  }
};


IGNITION_ADD_PLUGIN(
  LiftPlugin,
  System,
  LiftPlugin::ISystemConfigure,
  LiftPlugin::ISystemPreUpdate)

// TODO would prefer namespaced
IGNITION_ADD_PLUGIN_ALIAS(LiftPlugin, "lift")

} // namespace building_sim_ign
