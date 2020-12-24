#include <ignition/plugin/Register.hh>

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/components/JointAxis.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/gazebo/components/JointType.hh>
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
  struct DoorElementIgnition
  {
    Entity link_entity;
    Entity joint_entity;
    sdf::JointType joint_type;
    ignition::math::Pose3d orig_position;
    double orig_rotation;
    double vel_cmd;
  };

  rclcpp::Node::SharedPtr _ros_node;
  // Map from joint_name of each door section to ignition properties for that section
  std::unordered_map<std::string, DoorElementIgnition> _doors_ign;
  Entity _en;
  std::string _physics_plugin_name;
  bool pos_set = false;

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
    if (!ecm.EntityHasComponentType(entity,
      components::JointType().TypeId()))
      ecm.CreateComponent(entity, components::JointType());
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
      auto door_ign_it = _doors_ign.insert(
        {door_elem.second.joint_name, DoorElementIgnition()}).first;

      // For Trivial Physics Engine (TPE)
      const auto link = model.LinkByName(ecm, door_elem.second.link_name);
      if (link == kNullEntity)
      {
        RCLCPP_ERROR(_ros_node->get_logger(),
          " -- Door model is missing the link [%s]",
          door_elem.second.link_name.c_str());
        return;
      }
      create_link_components(link, ecm);
      door_ign_it->second.link_entity = link;

      // For default physics engine
      const auto joint = model.JointByName(ecm, door_elem.second.joint_name);
      if (!joint)
      {
        RCLCPP_ERROR(_ros_node->get_logger(),
          " -- Door model is missing the joint [%s]",
          door_elem.second.joint_name.c_str());
        return;
      }
      create_joint_components(joint, ecm);
      door_ign_it->second.joint_entity = joint;
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
      // Code later on assumes that each link is originally at its closed position
      for (auto& [door_name, door_ign] : _doors_ign)
      {
        door_ign.orig_position =
          ecm.Component<components::Pose>(door_ign.link_entity)->Data()
          + ecm.Component<components::Pose>(_en)->Data();
        door_ign.orig_rotation = door_ign.orig_position.Yaw();
      }

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

    // Determine current angular position and velocity of the door elements
    std::vector<DoorCommon::DoorUpdateRequest> requests;
    for (const auto& [door_name, door] : _door_common->_doors)
    {
      const DoorElementIgnition& door_ign = _doors_ign[door_name];
      Entity link = door_ign.link_entity;
      DoorCommon::DoorUpdateRequest request;
      request.joint_name = door_name;

      // No joint features support, look at link pose instead
      if(is_tpe_plugin(_physics_plugin_name))
      {
        const double orig_rot = door_ign.orig_rotation;
        const double curr_rot = ecm.Component<components::Pose>(link)->Data().Yaw()
          + ecm.Component<components::Pose>(_en)->Data().Yaw();
        const ignition::math::Pose3d orig_pos = door_ign.orig_position;
        const ignition::math::Pose3d curr_pos = ecm.Component<components::Pose>(link)->Data()
          + ecm.Component<components::Pose>(_en)->Data();
        constexpr double eps = 0.01;

        if(door.type == "SwingDoor" || door.type == "DoubleSwingDoor")
        {
          // In the event that Yaw angle of the door moves past -Pi rads, it experiences
          // a discontinuous jump from -Pi to Pi (vice-versa when the Yaw angle moves past Pi).
          // We need to take this jump into account when calculating the relative orientation
          // of the door w.r.t to its original orientation.
          if(door.closed_position > door.open_position)
          {
            // Yaw may go past -Pi rads when opening, and experience discontinuous jump to +Pi.
            // For e.g. when original angle is -(7/8)Pi, opening the door by Pi/2 rads would
            // push it past -Pi rads. If closed position (0 rads) is larger than open position,
            // then the relative orientation should theoretically never exceed 0,
            // which is how we identify when it has experienced a discontinuous jump.
            request.position = curr_rot - orig_rot > eps ?
              -3.14 - fmod(curr_rot - orig_rot, 3.14) : curr_rot - orig_rot;
          }
          else // Yaw may go past +180, and experience discontinuous jump to -180
          {
            request.position = curr_rot - orig_rot < -eps ?
              3.14 + abs(-3.14 - (curr_rot - orig_rot)) : curr_rot - orig_rot;
          }
        }
        else if(door.type == "SlidingDoor" || door.type == "DoubleSlidingDoor")
        {
            ignition::math::Vector3d displacement = curr_pos.CoordPositionSub(orig_pos);
            request.position = displacement.Length();
            if (door.open_position < door.closed_position){
              // At any point the link's position must be negative relative to the closed pose
              request.position *= -1;
            }
        }
        else
        {
          continue;
        }
        request.velocity = door_ign.vel_cmd;
      }
      else // Default Physics Engine with Joint Features support
      {
        request.position = ecm.Component<components::JointPosition>(
          door_ign.joint_entity)->Data()[0];
        request.velocity = ecm.Component<components::JointVelocity>(
          door_ign.joint_entity)->Data()[0];
      }

      requests.push_back(request);
    }

    // Get and apply motions to the joints/links
    auto results = _door_common->update(t, requests);
    for (const auto& result : results)
    {
      DoorElementIgnition& door_ign = _doors_ign[result.joint_name];
      const Entity link = door_ign.link_entity;
      const auto it = _door_common->_doors.find(result.joint_name);
      assert(it != _door_common->_doors.end());

      // No joint features support, use velocity commands to mimic joint motion
      if(is_tpe_plugin(_physics_plugin_name))
      {
        door_ign.vel_cmd = result.velocity;
        double vel_cmd = result.velocity;
        const DoorCommon::DoorElement& door_elem = it->second;
        if(door_elem.type == "SwingDoor" || door_elem.type == "DoubleSwingDoor")
        {
          // The reference frame axes of the Linear Velocity Cmds at any point t = axes defined
          // by the link's global pose (link yaw + model yaw) at time t0, rotated by the change in
          // yaw component of the link since then.
          // Note: This is not always equal to the global reference frame's axes rotated by the
          // link's global yaw. In some cases, the link may rotate after time t0, but the change in
          // its yaw may equal 0 if the change is applied to the parent model instead.

          // Calculate the door link orientation, theta_local, relative to Linear Velocity cmds reference frame
          // theta_global = (current_link_yaw + current_model_yaw) - (original_link_yaw + original_model_yaw)
          // theta_local = theta_global - (current_link_yaw - original_link_yaw)
          // theta_local = current_model_yaw - (original_link_yaw + original_model_yaw) (assuming original link yaw = 0)
          double theta_local = ecm.Component<components::Pose>(_en)->Data().Yaw() - door_ign.orig_rotation;

          // Given the rotation of the door, calculate a vector parallel to the door length, with magnitude 1/2 * vel_cmd
          double x_cmd = cos(theta_local) * vel_cmd * (door_elem.length / 2.0);
          double y_cmd = sin(theta_local) * vel_cmd * (door_elem.length / 2.0);
          // Rotate the vector to calculate the normal to the door length. The Linear Velocity
          // Cmd should be applied normal to the door
          double x_rot_cmd = -y_cmd;
          double y_rot_cmd = x_cmd;
          // Flip direction of velocity based on door hinge (left/right).
          // Assumes closed position is closest to 0, i.e. along original axis of the joint
          if (door_elem.closed_position > door_elem.open_position)
          {
            x_rot_cmd *= -1;
            y_rot_cmd *= -1;
          }

          ecm.Component<components::LinearVelocityCmd>(link)->Data() = ignition::math::Vector3d(x_rot_cmd,y_rot_cmd,0);
          ecm.Component<components::AngularVelocityCmd>(link)->Data() = ignition::math::Vector3d(0,0,vel_cmd);
        }
        else if(door_elem.type == "SlidingDoor" || door_elem.type == "DoubleSlidingDoor")
        {
          ecm.Component<components::LinearVelocityCmd>(link)->Data() = ignition::math::Vector3d(vel_cmd,0,0);
        }
        else
        {
          continue;
        }
      }
      else // Default Physics Engine with Joint Features support
      {
        auto joint_vel_cmd = ecm.Component<components::JointVelocityCmd>(
          door_ign.joint_entity);
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
