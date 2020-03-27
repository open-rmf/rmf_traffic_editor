#include <ignition/plugin/Register.hh>

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/components/JointAxis.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/gazebo/components/JointVelocityCmd.hh>
#include <ignition/gazebo/components/Pose.hh>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <memory>

#include <rmf_fleet_msgs/msg/destination_request.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rmf_fleet_msgs/msg/path_request.hpp>
#include <rmf_fleet_msgs/msg/mode_request.hpp>
#include <rmf_fleet_msgs/msg/robot_state.hpp>
#include <rclcpp/logger.hpp>

#include <building_sim_common/utils.hpp>
#include <building_sim_common/slotcar_common.hpp>

// TODO remove this
using namespace ignition;
using namespace gazebo;
using namespace systems;

using namespace building_sim_common;

class IGNITION_GAZEBO_VISIBLE SlotcarPlugin
    : public System,
      public ISystemConfigure,
      public ISystemPreUpdate
{
public:
  SlotcarPlugin();
  ~SlotcarPlugin();

  void Configure(const Entity& entity, const std::shared_ptr<const sdf::Element> &sdf,
      EntityComponentManager& ecm, EventManager& eventMgr) override;
  void path_request_cb(const rmf_fleet_msgs::msg::PathRequest::SharedPtr msg);
  void mode_request_cb(const rmf_fleet_msgs::msg::ModeRequest::SharedPtr msg);
  void PreUpdate(const UpdateInfo& info, EntityComponentManager& ecm) override;

private:
  std::unique_ptr<SlotcarCommon> dataPtr;

  rclcpp::Node::SharedPtr _ros_node;
  Model _model;
  Entity _entity;
  std::string _name;

  // TODO check if Entity is OK here
  std::array<Entity, 2> joints;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor;
  bool emergency_stop = false;

  // TODO check
  //std::unordered_set<gazebo::physics::Model*> infrastructure;

  // Book keeping
  int update_count = 0;

  void send_control_signals(
      EntityComponentManager &ecm,
      const double x_target,
      const double yaw_target,
      const double dt)
  {
    std::array<double, 2> w_tire_actual;
    for (std::size_t i = 0; i < 2; ++i)
      w_tire_actual[i] = ecm.Component<components::JointVelocity>(joints[i])->Data()[0];
    auto joint_signals = dataPtr->calculate_control_signals(w_tire_actual,
        x_target, yaw_target, dt);
    for (std::size_t i = 0; i < 2; ++i)
    {
      auto vel_cmd = ecm.Component<components::JointVelocityCmd>(joints[i]);
      vel_cmd->Data()[0] = joint_signals[i];
    }
  }
};

SlotcarPlugin::SlotcarPlugin() :
  dataPtr(std::make_unique<SlotcarCommon>())
{
  // We do initialization only during ::Load
}

SlotcarPlugin::~SlotcarPlugin()
{
}

void SlotcarPlugin::Configure(const Entity& entity,
    const std::shared_ptr<const sdf::Element> &sdf,
    EntityComponentManager& ecm, EventManager& eventMgr)
{
  _entity = entity;
  _model = Model(entity);
  _name = _model.Name(ecm);
  dataPtr->set_model_name(_model.Name(ecm));
  dataPtr->read_sdf(sdf);
  // TODO proper argc argv
  char const** argv = NULL;
  if (!rclcpp::is_initialized())
    rclcpp::init(0, argv);
  std::string plugin_name("plugin_" + _name);
  _ros_node = std::make_shared<rclcpp::Node>(plugin_name);
  // TODO Check if executor is getting callbacks
  //executor = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
  //executor->add_node(_ros_node);
  //executor->spin();
  dataPtr->init_ros_node(_ros_node);

  RCLCPP_INFO(dataPtr->logger(), "hello i am " + _name);

  joints[0] = _model.JointByName(ecm, "joint_tire_left");
  if (!joints[0])
    RCLCPP_ERROR(dataPtr->logger(), "Could not find tire for [joint_tire_left]");

  joints[1] = _model.JointByName(ecm, "joint_tire_right");
  if (!joints[1])
    RCLCPP_ERROR(dataPtr->logger(), "Could not find tire for [joint_tire_right]");

  // Initialise JointVelocityCmd / JointVelocity components for velocity control
  for (const auto& joint : joints)
  {
    if (!ecm.EntityHasComponentType(joint, components::JointVelocityCmd().TypeId()))
      ecm.CreateComponent(joint, components::JointVelocityCmd({0}));
    if (!ecm.EntityHasComponentType(joint, components::JointVelocity().TypeId()))
      ecm.CreateComponent(joint, components::JointVelocity({0}));
  }
  // Initialize Pose3d component
  if (!ecm.EntityHasComponentType(entity, components::Pose().TypeId()))
    ecm.CreateComponent(entity, components::Pose());
}

void SlotcarPlugin::PreUpdate(const UpdateInfo& info, EntityComponentManager& ecm)
{
  // TODO parallel thread executor?
  rclcpp::spin_some(_ros_node);
  /*
  update_count++;
  const auto& world = _model->GetWorld();
  if (update_count <= 1)
  {
    infrastructure.insert(_model.get());
    const auto& all_models = world->Models();
    for (const auto& m : all_models)
    {
      if (m->IsStatic())
        continue;

      if (m->GetName().find("door") != std::string::npos)
        infrastructure.insert(m.get());

      if (m->GetName().find("lift") != std::string::npos)
        infrastructure.insert(m.get());
    }
  }
  */
  double x_target = 0.0;
  double yaw_target = 0.0;

  double dt = (std::chrono::duration_cast<std::chrono::nanoseconds>
      (info.dt).count()) * 1e-9;

  double time = (std::chrono::duration_cast<std::chrono::nanoseconds>
      (info.simTime).count()) * 1e-9;

  auto pose = ecm.Component<components::Pose>(_entity)->Data();

  if (!dataPtr->update(convert_pose(pose), time, x_target, yaw_target))
    return;

  const double current_yaw = pose.Rot().Yaw();
  ignition::math::Vector3d current_heading{
    std::cos(current_yaw), std::sin(current_yaw), 0.0};

  const ignition::math::Vector3d stop_zone =
      pose.Pos() + dataPtr->stop_distance()*current_heading;

  bool need_to_stop = false;
  // TODO implement infrastructure emergency stop
  /*
  for (const auto& m : world->Models())
  {
    if (m->IsStatic())
      continue;

    if (infrastructure.count(m.get()) > 0)
      continue;

    const auto p_obstacle = m->WorldPose().Pos();
    if ( (p_obstacle - stop_zone).Length() < dataPtr->stop_radius() )
    {
      need_to_stop = true;
      break;
    }
  }

  if (need_to_stop != emergency_stop)
  {
    emergency_stop = need_to_stop;
    if (need_to_stop)
      std::cout << "Stopping vehicle to avoid a collision" << std::endl;
    else
      std::cout << "No more obstacles; resuming course" << std::endl;
  }

  if (emergency_stop)
  {
    // Allow spinning but not translating
    x_target = 0.0;
  }
  */

  send_control_signals(ecm, x_target, yaw_target, dt);
}

IGNITION_ADD_PLUGIN(SlotcarPlugin,
                    System,
                    SlotcarPlugin::ISystemConfigure,
                    SlotcarPlugin::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(SlotcarPlugin,
                          "slotcar")
