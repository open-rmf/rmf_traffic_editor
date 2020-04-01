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
class Door
{
public:

  bool _debuggable;

  Door(const bool debuggable,
    const Entity& entity,
    EntityComponentManager& ecm,
    const MotionParams& params,
    const bool flip_direction = false)
  : _debuggable(debuggable),
    _entity(entity),
    _params(params)
  {
    // Create components for joint position, velocity and velocitycmd
    if (!ecm.EntityHasComponentType(entity,
      components::JointPosition().TypeId()))
      ecm.CreateComponent(entity, components::JointPosition({0}));
    if (!ecm.EntityHasComponentType(entity,
      components::JointVelocity().TypeId()))
      ecm.CreateComponent(entity, components::JointVelocity({0}));
    if (!ecm.EntityHasComponentType(entity,
      components::JointVelocityCmd().TypeId()))
      ecm.CreateComponent(entity, components::JointVelocityCmd({0}));
    auto axis = ecm.Component<components::JointAxis>(entity)->Data();
    auto lower = axis.Lower();
    auto upper = axis.Upper();
    if (flip_direction)
    {
      _closed_position = lower;
      _open_position = upper;
    }
    else
    {
      _closed_position = upper;
      _open_position = lower;
    }
  }

  bool is_open(EntityComponentManager& ecm) const
  {
    auto pos = _get_door_position(ecm);
    return std::abs(_open_position - pos) <= _params.dx_min;
  }

  bool is_closed(EntityComponentManager& ecm) const
  {
    auto pos = _get_door_position(ecm);
    return std::abs(_closed_position - pos) <= _params.dx_min;
  }

  void open(EntityComponentManager& ecm, double dt)
  {
    _set_door_command(ecm, _open_position, dt);
  }

  void close(EntityComponentManager& ecm, double dt)
  {
    _set_door_command(ecm, _closed_position, dt);
  }


private:

  void _set_door_command(EntityComponentManager& ecm, const double target,
    const double dt)
  {
    double dx = target - _get_door_position(ecm);
    auto vel_cmd = ecm.Component<components::JointVelocityCmd>(_entity);

    if (std::abs(dx) < _params.dx_min/2.0)
      dx = 0.0;

    const double door_v = compute_desired_rate_of_change(
      dx, _get_door_velocity(ecm), _params, dt);

    vel_cmd->Data()[0] = door_v;
    // TODO f_max in the SDF file
    //_joint->SetParam("fmax", 0, _params.f_max);
  }

  // TODO template these two functions
  double _get_door_position(EntityComponentManager& ecm) const
  {
    return ecm.Component<components::JointPosition>(_entity)->Data()[0];
  }

  double _get_door_velocity(EntityComponentManager& ecm) const
  {
    return ecm.Component<components::JointVelocity>(_entity)->Data()[0];
  }

  Entity _entity;
  MotionParams _params;

  double _open_position;
  double _closed_position;

};

class IGNITION_GAZEBO_VISIBLE DoorPlugin
  : public System,
  public ISystemConfigure,
  public ISystemPreUpdate
{
private:
  rclcpp::Node::SharedPtr _ros_node;
  Model _model;
  std::shared_ptr<DoorCommon> _door_common = nullptr;

  std::vector<Door> _doors;

  double _last_pub_time;

  bool _initialized = false;

public:
  DoorPlugin()
  : _last_pub_time(0)
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
      sdf);

    if (!_door_common)
      return;

    const auto left_door_joint_name = _door_common->left_door_joint_name();
    const auto right_door_joint_name = _door_common->right_door_joint_name();

    if (left_door_joint_name != "empty_joint")
    {
      const auto left_door_joint =
        _model.JointByName(ecm, left_door_joint_name);
      if (!left_door_joint)
      {
        RCLCPP_ERROR(
          _ros_node->get_logger(),
          " -- Model is missing the left door joint [%s]",
          left_door_joint_name.c_str());
        return;
      }
      _doors.emplace_back(_model.Name(ecm) == "chart_lift_door",
        left_door_joint, ecm, params);
    }

    if (right_door_joint_name != "empty_joint")
    {
      const auto right_door_joint = _model.JointByName(ecm,
          right_door_joint_name);
      if (!right_door_joint)
      {
        RCLCPP_ERROR(
          _ros_node->get_logger(),
          " -- Model is missing the right door joint [%s]",
          right_door_joint_name.c_str());
        return;
      }
      _doors.emplace_back(_model.Name(ecm) == "chart_lift_door",
        right_door_joint, ecm, params, true);
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
    double dt =
      (std::chrono::duration_cast<std::chrono::nanoseconds>(info.dt).count())
      * 1e-9;

    double t =
      (std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).
      count()) * 1e-9;

    if (_door_common->requested_mode().value == DoorMode::MODE_OPEN)
    {
      for (auto& door : _doors)
        door.open(ecm, dt);
    }
    else
    {
      for (auto& door : _doors)
        door.close(ecm, dt);
    }

    if (t - _last_pub_time >= 1.0)
    {
      _last_pub_time = t;
      const int32_t t_sec = static_cast<int32_t>(t);
      const uint32_t t_nsec =
        static_cast<uint32_t>((t-static_cast<double>(t_sec)) *1e9);
      const rclcpp::Time now{t_sec, t_nsec, RCL_ROS_TIME};

      if (all_doors_open(ecm))
      {
        _door_common->publish_state(DoorMode::MODE_OPEN, now);
      }
      else if (all_doors_closed(ecm))
      {
        _door_common->publish_state(DoorMode::MODE_CLOSED, now);
      }
      else
      {
        _door_common->publish_state(DoorMode::MODE_MOVING, now);
      }
    }
  }

private:

  bool all_doors_open(EntityComponentManager& ecm) const
  {
    for (const auto& door : _doors)
    {
      if (!door.is_open(ecm))
        return false;
    }

    return true;
  }

  bool all_doors_closed(EntityComponentManager& ecm) const
  {
    for (const auto& door : _doors)
    {
      if (!door.is_closed(ecm))
        return false;
    }

    return true;
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
