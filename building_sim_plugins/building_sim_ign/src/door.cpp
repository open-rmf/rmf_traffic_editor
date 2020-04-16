//#include <gazebo/common/Plugin.hh>
//#include <gazebo/physics/Model.hh>
//#include <gazebo/physics/World.hh>
//#include <gazebo/physics/Joint.hh>
//#include <gazebo_ros/node.hpp>

#include <ignition/plugin/Register.hh>

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/components/JointAxis.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/gazebo/components/JointVelocityCmd.hh>

#include <rclcpp/rclcpp.hpp>

#include <building_sim_common/utils.hpp>
#include <building_sim_common/door_common.hpp>

// TODO remove this
using namespace ignition;
using namespace gazebo;
using namespace systems;

using namespace building_sim_common;

namespace building_sim_ign {

//==============================================================================

class IGNITION_GAZEBO_VISIBLE DoorPlugin
  : public System,
  public ISystemConfigure,
  public ISystemPreUpdate
{
private:
  rclcpp::Node::SharedPtr _ros_node;
  Model _model;
  std::shared_ptr<Entity> _left_door_joint = nullptr;
  std::shared_ptr<Entity> _right_door_joint = nullptr;

  std::shared_ptr<DoorCommon> _door_common = nullptr;

  bool _initialized = false;

  void create_entity_components(Entity entity, EntityComponentManager& ecm)
  {
    if (!ecm.EntityHasComponentType(entity,
      components::JointPosition().TypeId()))
        ecm.CreateComponent(entity, components::JointPosition({0}));
    if (!ecm.EntityHasComponentType(entity,
      components::JointVelocity().TypeId()))
        ecm.CreateComponent(entity, components::JointVelocity({0}));
    if (!ecm.EntityHasComponentType(entity,
      components::JointVelocityCmd().TypeId()))
        ecm.CreateComponent(entity, components::JointVelocityCmd({0}));
  }

public:
  DoorPlugin()
  {
    // TODO init ros node
    // Do nothing
  }

  void Configure(const Entity& entity,
    const std::shared_ptr<const sdf::Element>& sdf,
    EntityComponentManager& ecm, EventManager& _eventMgr) override
  {
    //_ros_node = gazebo_ros::Node::Get(sdf);
    // TODO get properties from sdf instead of hardcoded (will fail for multiple instantiations)
    // TODO proper rclcpp init (only once and pass args)
    _model = Model(entity);
    char const** argv = NULL;
    if (!rclcpp::is_initialized())
      rclcpp::init(0, argv);
    std::string plugin_name("plugin_" + _model.Name(ecm));
    ignwarn << "Initializing plugin with name " << plugin_name << std::endl;
    _ros_node = std::make_shared<rclcpp::Node>(plugin_name);

    RCLCPP_INFO(
      _ros_node->get_logger(),
      "Loading DoorPlugin for [%s]",
      _model.Name(ecm).c_str());
    auto sdfClone = sdf->Clone();

    _door_common = DoorCommon::make(
      _model.Name(ecm),
      _ros_node,
      sdfClone);

    if (!_door_common)
      return;

    const auto left_door_joint_name = _door_common->left_door_joint_name();
    const auto right_door_joint_name = _door_common->right_door_joint_name();

    if (left_door_joint_name != "empty_joint")
    {
      _left_door_joint = std::make_shared<Entity>(
        _model.JointByName(ecm, left_door_joint_name));
      if (!_left_door_joint)
      {
        RCLCPP_ERROR(
          _ros_node->get_logger(),
          " -- Model is missing the left door joint [%s]",
          left_door_joint_name.c_str());
        return;
      }      
      create_entity_components(*_left_door_joint, ecm);
      auto axis = ecm.Component<components::JointAxis>(*_left_door_joint)->Data();
      _door_common->add_left_door(_model.Name(ecm) == "chart_lift_door",
        axis.Upper(), axis.Lower());
    }

    if (right_door_joint_name != "empty_joint")
    {
      _right_door_joint = std::make_shared<Entity>(
        _model.JointByName(ecm,
        right_door_joint_name));
      if (!_right_door_joint)
      {
        RCLCPP_ERROR(
          _ros_node->get_logger(),
          " -- Model is missing the right door joint [%s]",
          right_door_joint_name.c_str());
        return;
      }
      create_entity_components(*_right_door_joint, ecm);
      auto axis = ecm.Component<components::JointAxis>(*_right_door_joint)->Data();
      _door_common->add_right_door(_model.Name(ecm) == "chart_lift_door",
        axis.Upper(), axis.Lower());
    }

    _initialized = true;

    RCLCPP_INFO(
      _ros_node->get_logger(),
      "Finished loading [%s]",
      _model.Name(ecm).c_str());
  }

  void PreUpdate(const UpdateInfo& info, EntityComponentManager& ecm) override
  {
    // TODO parallel thread executor?
    rclcpp::spin_some(_ros_node);
    if (!_initialized)
      return;

    double t =
      (std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).
      count()) * 1e-9;

    DoorCommon::DoorUpdateRequest request;
    if (_left_door_joint)
    {
      request.left_position = std::make_shared<double>(
        ecm.Component<components::JointPosition>(
          *_left_door_joint)->Data()[0]);
      request.left_velocity = std::make_shared<double>(
        ecm.Component<components::JointVelocity>(
          *_left_door_joint)->Data()[0]);
    }
    if (_right_door_joint)
    {
      request.right_position = std::make_shared<double>(
        ecm.Component<components::JointPosition>(
          *_right_door_joint)->Data()[0]);
      request.right_velocity = std::make_shared<double>(
        ecm.Component<components::JointVelocity>(
          *_right_door_joint)->Data()[0]);
    }

    auto result = _door_common->update(t, request);

    // Apply motions to the joints
    if (_left_door_joint)
    {
      auto vel_cmd = ecm.Component<components::JointVelocityCmd>(*_left_door_joint);
      vel_cmd->Data()[0] = *result.left_velocity;
    }
    if (_right_door_joint)
    {
      auto vel_cmd = ecm.Component<components::JointVelocityCmd>(*_right_door_joint);
      vel_cmd->Data()[0] = *result.right_velocity;
    }
  }

};

IGNITION_ADD_PLUGIN(
  DoorPlugin,
  System,
  DoorPlugin::ISystemConfigure,
  DoorPlugin::ISystemPreUpdate)

// TODO would prefer namespaced
IGNITION_ADD_PLUGIN_ALIAS(DoorPlugin, "door")

} // namespace building_sim_ign
