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
#include <ignition/gazebo/components/PoseCmd.hh>
#include <ignition/gazebo/components/PhysicsEnginePlugin.hh>

#include <rclcpp/rclcpp.hpp>

#include <building_sim_common/utils.hpp>
#include <building_sim_common/lift_common.hpp>

// TODO remove this
using namespace ignition;
using namespace gazebo;
using namespace systems;

using namespace building_sim_common;

namespace building_sim_ign {

enum class PhysEnginePlugin {DEFAULT, TPE};
std::unordered_map<std::string, PhysEnginePlugin> plugin_names {
  {"ignition-physics-tpe-plugin", PhysEnginePlugin::TPE}};

//==============================================================================

class IGNITION_GAZEBO_VISIBLE LiftPlugin
  : public System,
  public ISystemConfigure,
  public ISystemPreUpdate
{
private:
  rclcpp::Node::SharedPtr _ros_node;
  Entity _cabin_joint;
  Entity _lift_entity;
  std::vector<Entity> _payloads;
  ignition::math::AxisAlignedBox _initial_aabb;
  ignition::math::Pose3d _initial_pose;
  bool _read_aabb_dimensions = true;

  PhysEnginePlugin _phys_plugin = PhysEnginePlugin::DEFAULT;
  bool _first_iteration = true;

  std::unique_ptr<LiftCommon> _lift_common = nullptr;

  bool _initialized = false;

  void create_entity_components(Entity entity, EntityComponentManager& ecm)
  {
    if (!ecm.EntityHasComponentType(entity,
      components::LinearVelocityCmd().TypeId()))
      ecm.CreateComponent(entity, components::LinearVelocityCmd({0, 0, 0}));
    if (!ecm.EntityHasComponentType(entity,
      components::WorldPoseCmd().TypeId()))
    {
      auto pos = ecm.Component<components::Pose>(entity);
      if (pos) // Set Pose cmd to current pose, instead of default location
      {
        ecm.CreateComponent(entity, components::WorldPoseCmd(pos->Data()));
      }
      else
      {
        ecm.CreateComponent(entity, components::WorldPoseCmd());
      }
    }
  }

  void create_joint_components(Entity entity, EntityComponentManager& ecm)
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

  void fill_physics_engine(Entity entity, EntityComponentManager& ecm)
  {
    Entity parent = entity;
    while (ecm.ParentEntity(parent))
    {
      parent = ecm.ParentEntity(parent);
    }
    if (ecm.EntityHasComponentType(parent,
      components::PhysicsEnginePlugin().TypeId()))
    {
      const std::string physics_plugin_name =
        ecm.Component<components::PhysicsEnginePlugin>(parent)->Data();
      const auto it = plugin_names.find(physics_plugin_name);
      if (it != plugin_names.end())
      {
        _phys_plugin = it->second;
      }
    }
  }

  std::vector<Entity> get_payloads(EntityComponentManager& ecm)
  {
    const auto& lift_pose =
      ecm.Component<components::Pose>(_lift_entity)->Data();
    const ignition::math::Vector3d displacement =
      lift_pose.CoordPositionSub(_initial_pose);
    // Calculate current AABB of lift assuming it hasn't tilted/deformed
    ignition::math::AxisAlignedBox lift_aabb = _initial_aabb + displacement;

    std::vector<Entity> payloads;
    ecm.Each<components::Model, components::Pose>(
      [&](const Entity& entity,
      const components::Model*,
      const components::Pose* pose
      ) -> bool
      {
        const auto payload_position = pose->Data().Pos();
        if (entity != _lift_entity)
        { // Could possibly check bounding box intersection too, but this suffices
          if (lift_aabb.Contains(payload_position))
          {
            payloads.push_back(entity);
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
    _lift_entity = entity;
    //_ros_node = gazebo_ros::Node::Get(sdf);
    // TODO get properties from sdf instead of hardcoded (will fail for multiple instantiations)
    // TODO proper rclcpp init (only once and pass args)
    auto model = Model(entity);
    char const** argv = NULL;
    if (!rclcpp::ok())
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

    if (!ecm.EntityHasComponentType(_lift_entity,
      components::AxisAlignedBox().TypeId()))
    {
      ecm.CreateComponent(_lift_entity, components::AxisAlignedBox());
    }

    _cabin_joint = model.JointByName(ecm, _lift_common->get_joint_name());
    if (!_cabin_joint)
    {
      RCLCPP_ERROR(_ros_node->get_logger(),
        " -- Model is missing the joint [%s]",
        _lift_common->get_joint_name().c_str());
      return;
    }

    _initialized = true;

    RCLCPP_INFO(_ros_node->get_logger(),
      "Finished loading [%s]",
      model.Name(ecm).c_str());
  }

  void PreUpdate(const UpdateInfo& info, EntityComponentManager& ecm) override
  {
    // Read from components that may not have been initialized in configure()
    if (_first_iteration)
    {
      fill_physics_engine(_lift_entity, ecm);

      double lift_elevation = _lift_common->get_elevation();
      if (_phys_plugin == PhysEnginePlugin::DEFAULT)
      {
        create_joint_components(_cabin_joint, ecm);
        auto position_cmd = ecm.Component<components::JointPositionReset>(
          _cabin_joint);
        position_cmd->Data()[0] = lift_elevation;
      }
      else
      {
        create_entity_components(_lift_entity, ecm);
        auto position_cmd = ecm.Component<components::WorldPoseCmd>(
          _lift_entity);
        position_cmd->Data().Pos().Z() = lift_elevation;
      }

      _first_iteration = false;
    }

    // Optimization: Read and store lift's pose and AABB whenever available, then
    // delete the AABB component once read. Not deleting it causes rtf to drop by
    // a 3-4x factor whenever the lift moves.
    if (_read_aabb_dimensions)
    {
      const auto& aabb_component =
        ecm.Component<components::AxisAlignedBox>(_lift_entity);
      const auto& pose_component =
        ecm.Component<components::Pose>(_lift_entity);

      if (aabb_component && pose_component)
      {
        const double volume = aabb_component->Data().Volume();
        if (volume > 0 && volume != std::numeric_limits<double>::infinity())
        {
          _initial_aabb = aabb_component->Data();
          _initial_pose = pose_component->Data();
          ecm.RemoveComponent(_lift_entity,
            components::AxisAlignedBox().TypeId());
          _read_aabb_dimensions = false;
        }
      }
    }

    // TODO parallel thread executor?
    rclcpp::spin_some(_ros_node);
    if (!_initialized)
      return;

    // Send update request
    const double t =
      (std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).
      count()) * 1e-9;
    double position = 0.0;
    double velocity = 0.0;

    // Read from either joint data or model data based on physics engine
    if (_phys_plugin == PhysEnginePlugin::DEFAULT)
    {
      position = ecm.Component<components::JointPosition>(
        _cabin_joint)->Data()[0];
      velocity = ecm.Component<components::JointVelocity>(
        _cabin_joint)->Data()[0];
    }
    else
    {
      position = ecm.Component<components::Pose>(
        _lift_entity)->Data().Pos().Z();
      auto lift_vel_cmd = ecm.Component<components::LinearVelocityCmd>(
        _lift_entity);
      velocity = lift_vel_cmd->Data()[2];
    }

    auto result = _lift_common->update(t, position, velocity);

    // Move either joint or lift cabin based on physics engine used
    if (_phys_plugin == PhysEnginePlugin::DEFAULT)
    {
      auto vel_cmd = ecm.Component<components::JointVelocityCmd>(
        _cabin_joint);
      vel_cmd->Data()[0] = result.velocity;
    }
    else
    {
      auto lift_vel_cmd = ecm.Component<components::LinearVelocityCmd>(
        _lift_entity);
      lift_vel_cmd->Data()[2] = result.velocity;
    }

    // Move any payloads that need to be manually moved
    // (i.e. have a LinearVelocityCmd component that exists)
    if (_lift_common->motion_state_changed())
    {
      _payloads = get_payloads(ecm);
    }

    for (const Entity& payload : _payloads)
    {
      if (ecm.EntityHasComponentType(payload,
        components::LinearVelocityCmd().TypeId()))
      {
        auto lin_vel_cmd =
          ecm.Component<components::LinearVelocityCmd>(payload);
        lin_vel_cmd->Data()[2] = result.velocity;
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
