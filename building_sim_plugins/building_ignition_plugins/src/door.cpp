#include <ignition/plugin/Register.hh>

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/components/JointAxis.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/gazebo/components/JointVelocityCmd.hh>
#include <ignition/gazebo/components/LinearVelocityCmd.hh>
#include <ignition/gazebo/components/AngularVelocityCmd.hh>
#include <ignition/gazebo/components/PhysicsEnginePlugin.hh>
#include <ignition/gazebo/components/Pose.hh>

#include <rclcpp/rclcpp.hpp>

#include <building_sim_common/utils.hpp>
#include <building_sim_common/door_common.hpp>

// TODO remove this
using namespace ignition;
using namespace gazebo;
using namespace systems;

using namespace building_sim_common;

namespace building_ignition_plugins {

//==============================================================================

class IGNITION_GAZEBO_VISIBLE DoorPlugin
  : public System,
  public ISystemConfigure,
  public ISystemPreUpdate
{
private:
  struct DoorElement {
    Entity link_entity;
    Entity joint_entity;
    ignition::math::Pose3d orig_position;
    double orig_rotation;
  };

  rclcpp::Node::SharedPtr _ros_node;
  std::unordered_map<std::string, Entity> _link_entities;
  std::unordered_map<std::string, Entity> _joint_entities;
  Entity _en;

  std::unordered_map<Entity, ignition::math::Pose3d> orig_positions;
  std::string _physics_plugin_name;
  ignition::math::Pose3d orig_pos;
  double orig_rot = 0.0;
  bool pos_set = false;
  int axis = 0;
  double vel_cmd = 0.0;

  std::shared_ptr<DoorCommon> _door_common = nullptr;

  bool _initialized = false;

  void create_joint_components(Entity entity, EntityComponentManager& ecm)
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

  void create_link_components(Entity entity, EntityComponentManager& ecm)
  {
    if (!ecm.EntityHasComponentType(entity,
      components::LinearVelocityCmd().TypeId()))
      ecm.CreateComponent(entity, components::LinearVelocityCmd());
    if (!ecm.EntityHasComponentType(entity,
      components::AngularVelocityCmd().TypeId()))
      ecm.CreateComponent(entity, components::AngularVelocityCmd());
    if (!ecm.EntityHasComponentType(entity,
        components::Pose().TypeId()))
      ecm.CreateComponent(entity, components::Pose());
  }

  bool is_tpe_plugin(const std::string& plugin_name)
  {
    static const std::string tpe_plugin = "ignition-physics-tpe-plugin";
    return plugin_name == tpe_plugin;
  }

public:
  DoorPlugin()
  {
    // TODO init ros node
    // Do nothing
  }

  void Configure(const Entity& entity,
    const std::shared_ptr<const sdf::Element>& sdf,
    EntityComponentManager& ecm, EventManager& /*_eventMgr*/) override
  {
    //_ros_node = gazebo_ros::Node::Get(sdf);
    // TODO get properties from sdf instead of hardcoded (will fail for multiple instantiations)
    // TODO proper rclcpp init (only once and pass args)
    _en = entity;
    auto model = Model(entity);
    char const** argv = NULL;
    std::string name;
    auto door_ele = sdf->GetElementImpl("door");
    get_sdf_attribute_required<std::string>(door_ele, "name", name);
    if (!rclcpp::is_initialized())
      rclcpp::init(0, argv);
    std::string plugin_name("plugin_" + name);
    ignwarn << "Initializing plugin with name " << plugin_name << std::endl;
    _ros_node = std::make_shared<rclcpp::Node>(plugin_name);

    RCLCPP_INFO(_ros_node->get_logger(),
      "Loading DoorPlugin for [%s]",
      name.c_str());

    _door_common = DoorCommon::make(
      name,
      _ros_node,
      sdf);

    if (!_door_common)
      return;

    for (const auto& door_elem : _door_common->_doors)
    {
      const auto link = model.LinkByName(ecm, door_elem.second.link_name);
      if (link == kNullEntity)
      {
        RCLCPP_ERROR(_ros_node->get_logger(),
          " -- Door model is missing the link [%s]",
          door_elem.second.link_name.c_str());
        return;
      }
      create_link_components(link, ecm);
      _link_entities.insert({door_elem.second.joint_name, link});

      const auto joint = model.JointByName(ecm, door_elem.second.joint_name);
      if (!joint)
      {
        RCLCPP_ERROR(_ros_node->get_logger(),
          " -- Door model is missing the joint [%s]",
          door_elem.second.joint_name.c_str());
        return;
      }
      create_joint_components(joint, ecm);
      _joint_entities.insert({door_elem.second.joint_name, joint});
    }

    _initialized = true;

    RCLCPP_INFO(_ros_node->get_logger(),
      "Finished loading [%s]",
      name.c_str());
  }

  void PreUpdate(const UpdateInfo& info, EntityComponentManager& ecm) override
  {
    // TODO parallel thread executor?
    rclcpp::spin_some(_ros_node);
    if (!_initialized)
      return;

    if(!pos_set){

      orig_pos = ecm.Component<components::Pose>(_en)->Data();
      orig_rot = ecm.Component<components::Pose>(_en)->Data().Yaw();

      Entity parent = _en;
      while(ecm.ParentEntity(parent)){
        parent = ecm.ParentEntity(parent);
      }
      if (ecm.EntityHasComponentType(parent,
          components::PhysicsEnginePlugin().TypeId())){
            _physics_plugin_name = ecm.Component<components::PhysicsEnginePlugin>(parent)->Data();
      }
      pos_set = true;
    }

    double t =
      (std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).
      count()) * 1e-9;

    // Create DoorUpdateRequest
    std::vector<DoorCommon::DoorUpdateRequest> requests;
    for (const auto& door : _door_common->_doors)
    {
      Entity link = _link_entities[door.first];
      DoorCommon::DoorUpdateRequest request;
      request.joint_name = door.first;

      if(is_tpe_plugin(_physics_plugin_name)) // No joint features support, use velocity commands instead. May need axis too
      {
        const DoorCommon::DoorElement& door_elem = door.second;
        constexpr double eps = 0.01;
        if(door_elem.door_type == "SwingDoor" || door_elem.door_type == "DoubleSwingDoor")
        {
          // In the event that Yaw angle of the door moves past -Pi rads, it experiences a discontinuous jump from -Pi to Pi (likewise when the Yaw angle moves past Pi)
          // We need to take this jump into account when calculating the relative orientation of the door w.r.t to its original orientation
          if(door_elem.closed_position > door_elem.open_position)
          {
            // Yaw may go past -Pi rads when opening, and experience discontinuous jump to +Pi. For e.g. when original angle is -(7/8)Pi, opening the door by Pi/2 rads would
            // push it past -Pi rads.
            // If closed position (0 rads) is larger than open position, then the relative orientation should theoretically never exceed 0,
            //which is how we identify when it has experienced a discontinuous jump
            request.position = ecm.Component<components::Pose>(link)->Data().Yaw() - orig_rot > eps ?
              -3.14 - fmod(ecm.Component<components::Pose>(link)->Data().Yaw() - orig_rot,3.14) : ecm.Component<components::Pose>(link)->Data().Yaw() - orig_rot;
          }
          else // Yaw may go past +180, and experience discontinuous jump to -180
          {
            request.position = ecm.Component<components::Pose>(link)->Data().Yaw() - orig_rot < -eps ?
              3.14 + abs(-3.14 - (ecm.Component<components::Pose>(link)->Data().Yaw() - orig_rot)) : ecm.Component<components::Pose>(link)->Data().Yaw() - orig_rot;
          }
        }
        else if(door_elem.door_type == "SlidingDoor" || door_elem.door_type == "DoubleSlidingDoor")
        {
            ignition::math::Pose3d curr_pos = ecm.Component<components::Pose>(link)->Data();
            ignition::math::Vector3d displacement = curr_pos.CoordPositionSub(orig_pos);
            //std::cout << "diff: " << displacement.Length() << std::endl;
            request.position = displacement.Length();
        }
        else
        {
          continue;
        }
        //std::cout << "pos: " << request.position << std::endl;
        //std::cout << "result vel: " << vel_cmd << std::endl;
        request.velocity = vel_cmd;
      }
      else // Default Physics Engine with Joint Features support
      {
        Entity joint = _joint_entities[door.first];
        request.position = ecm.Component<components::JointPosition>(
          joint)->Data()[0];
        request.velocity = ecm.Component<components::JointVelocity>(
          joint)->Data()[0];
      }

      requests.push_back(request);
    }

    // Get and apply motions to the joints
    auto results = _door_common->update(t, requests);
    for (const auto& result : results)
    {
      Entity link = _link_entities[result.joint_name];
      const auto it = _door_common->_doors.find(result.joint_name);
      assert(it != _door_common->_doors.end());

      //to get door element section
      if(is_tpe_plugin(_physics_plugin_name))
      {
        vel_cmd = result.velocity;
        const DoorCommon::DoorElement& door_elem = it->second;
        if(door_elem.door_type == "SwingDoor" || door_elem.door_type == "DoubleSwingDoor")
        {
          ecm.Component<components::LinearVelocityCmd>(link)->Data() = ignition::math::Vector3d(0,-0.5*vel_cmd,0);
          ecm.Component<components::AngularVelocityCmd>(link)->Data() = ignition::math::Vector3d(0,0,vel_cmd);
        }
        else if(door_elem.door_type == "SlidingDoor")
        {
          ecm.Component<components::LinearVelocityCmd>(link)->Data() = ignition::math::Vector3d(vel_cmd,0,0);
        }
        else
        {
          continue;
        }
        //std::cout << "velocity: " << result.velocity << std::endl;
        //std::cout << "Door Pose: " << ecm.Component<components::Pose>(_en)->Data() << std::endl;
        //std::cout << "Link Pose: " << ecm.Component<components::Pose>(link)->Data() << std::endl;
      }
      else // Default Physics Engine with Joint Features support
      {
        Entity joint = _joint_entities[result.joint_name];
        auto joint_vel_cmd = ecm.Component<components::JointVelocityCmd>(
          joint);
        joint_vel_cmd->Data()[0] = result.velocity;
      }
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

} // namespace building_ignition_plugins
