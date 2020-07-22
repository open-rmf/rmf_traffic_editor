#include <ignition/plugin/Register.hh>

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/gazebo/components/JointVelocityCmd.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/Static.hh>

#include <rclcpp/rclcpp.hpp>

#include <building_sim_common/utils.hpp>
#include <building_sim_common/slotcar_common.hpp>

using namespace ignition::gazebo;

using namespace building_sim_common;

class IGNITION_GAZEBO_VISIBLE SlotcarPlugin
  : public System,
  public ISystemConfigure,
  public ISystemPreUpdate
{
public:
  SlotcarPlugin();
  ~SlotcarPlugin();

  void Configure(const Entity& entity,
    const std::shared_ptr<const sdf::Element>& sdf,
    EntityComponentManager& ecm, EventManager& eventMgr) override;
  void path_request_cb(const rmf_fleet_msgs::msg::PathRequest::SharedPtr msg);
  void mode_request_cb(const rmf_fleet_msgs::msg::ModeRequest::SharedPtr msg);
  void PreUpdate(const UpdateInfo& info, EntityComponentManager& ecm) override;

private:
  std::unique_ptr<SlotcarCommon> dataPtr;

  rclcpp::Node::SharedPtr _ros_node;
  Entity _entity;

  std::array<Entity, 2> joints;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor;

  std::unordered_set<Entity> _infrastructure;

  void send_control_signals(EntityComponentManager& ecm,
    const std::pair<double, double>& velocities,
    const double dt)
  {
    std::array<double, 2> w_tire;
    for (std::size_t i = 0; i < 2; ++i)
      w_tire[i] =
        ecm.Component<components::JointVelocity>(joints[i])->Data()[0];
    auto joint_signals = dataPtr->calculate_control_signals(w_tire,
        velocities, dt);
    for (std::size_t i = 0; i < 2; ++i)
    {
      auto vel_cmd = ecm.Component<components::JointVelocityCmd>(joints[i]);
      vel_cmd->Data()[0] = joint_signals[i];
    }
  }

  void init_infrastructure(EntityComponentManager& ecm);

  std::vector<Eigen::Vector3d> get_obstacle_positions(
    EntityComponentManager& ecm);
};

SlotcarPlugin::SlotcarPlugin()
: dataPtr(std::make_unique<SlotcarCommon>())
{
  // We do initialization only during ::Configure
}

SlotcarPlugin::~SlotcarPlugin()
{
}

void SlotcarPlugin::Configure(const Entity& entity,
  const std::shared_ptr<const sdf::Element>& sdf,
  EntityComponentManager& ecm, EventManager&)
{
  _entity = entity;
  auto model = Model(entity);
  std::string model_name = model.Name(ecm);
  dataPtr->set_model_name(model_name);
  dataPtr->read_sdf(sdf);
  // TODO proper argc argv
  char const** argv = NULL;
  if (!rclcpp::is_initialized())
    rclcpp::init(0, argv);
  std::string plugin_name("plugin_" + model_name);
  _ros_node = std::make_shared<rclcpp::Node>(plugin_name);
  // TODO Check if executor is getting callbacks
  //executor = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
  //executor->add_node(_ros_node);
  //executor->spin();
  dataPtr->init_ros_node(_ros_node);

  joints[0] = model.JointByName(ecm, "joint_tire_left");
  if (!joints[0])
    RCLCPP_ERROR(dataPtr->logger(),
      "Could not find tire for [joint_tire_left]");

  joints[1] = model.JointByName(ecm, "joint_tire_right");
  if (!joints[1])
    RCLCPP_ERROR(dataPtr->logger(),
      "Could not find tire for [joint_tire_right]");

  // Initialise JointVelocityCmd / JointVelocity components for velocity control
  for (const auto& joint : joints)
  {
    if (!ecm.EntityHasComponentType(joint,
      components::JointVelocityCmd().TypeId()))
      ecm.CreateComponent(joint, components::JointVelocityCmd({0}));
    if (!ecm.EntityHasComponentType(joint,
      components::JointVelocity().TypeId()))
      ecm.CreateComponent(joint, components::JointVelocity({0}));
  }
  // Initialize Pose3d component
  if (!ecm.EntityHasComponentType(entity, components::Pose().TypeId()))
    ecm.CreateComponent(entity, components::Pose());
}

void SlotcarPlugin::init_infrastructure(EntityComponentManager& ecm)
{
  // Cycle through all the static entities with Model and Name components
  ecm.Each<components::Model, components::Name, components::Pose,
    components::Static>(
    [&](const Entity& entity,
    const components::Model*,
    const components::Name* name,
    const components::Pose*,
    const components::Static* is_static
    ) -> bool
    {
      if (is_static->Data() == false)
      {
        std::string n = name->Data();
        std::for_each(n.begin(), n.end(), [](char& c)
        {
          c = ::tolower(c);
        });
        if (n.find("door") != std::string::npos ||
        n.find("lift") != std::string::npos)
          _infrastructure.insert(entity);
      }
      return true;
    });
  // Also add itself
  _infrastructure.insert(_entity);
}

std::vector<Eigen::Vector3d> SlotcarPlugin::get_obstacle_positions(
  EntityComponentManager& ecm)
{
  std::vector<Eigen::Vector3d> obstacle_positions;
  ecm.Each<components::Model, components::Name, components::Pose,
    components::Static>(
    [&](const Entity& entity,
    const components::Model*,
    const components::Name*,
    const components::Pose* pose,
    const components::Static* is_static
    ) -> bool
    {
      // Object should not be static
      // It should not be part of infrastructure (doors / lifts)
      // And it should be closer than the "stop" range (checked by common)
      const auto obstacle_position = pose->Data().Pos();
      if (is_static->Data() == false &&
      _infrastructure.find(entity) == _infrastructure.end())
      {
        obstacle_positions.push_back(convert_vec(obstacle_position));
      }
      return true;
    });
  return obstacle_positions;
}

void SlotcarPlugin::PreUpdate(const UpdateInfo& info,
  EntityComponentManager& ecm)
{
  // TODO parallel thread executor?
  rclcpp::spin_some(_ros_node);
  if (_infrastructure.empty())
    init_infrastructure(ecm);

  double dt =
    (std::chrono::duration_cast<std::chrono::nanoseconds>(info.dt).count()) *
    1e-9;
  double time =
    (std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count())
    * 1e-9;

  auto pose = ecm.Component<components::Pose>(_entity)->Data();
  auto obstacle_positions = get_obstacle_positions(ecm);

  auto velocities =
    dataPtr->update(convert_pose(pose), obstacle_positions, time);

  send_control_signals(ecm, velocities, dt);
}

IGNITION_ADD_PLUGIN(
  SlotcarPlugin,
  System,
  SlotcarPlugin::ISystemConfigure,
  SlotcarPlugin::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(SlotcarPlugin, "slotcar")
