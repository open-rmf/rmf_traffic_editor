#include <unordered_map>

#include <ignition/plugin/Register.hh>

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/gazebo/components/JointVelocityCmd.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/Static.hh>
#include <ignition/gazebo/components/AxisAlignedBox.hh>
#include <ignition/gazebo/components/LinearVelocityCmd.hh>
#include <ignition/gazebo/components/AngularVelocityCmd.hh>
#include <ignition/gazebo/components/PhysicsEnginePlugin.hh>

#include <ignition/msgs.hh>
#include <ignition/transport.hh>
#include <rclcpp/rclcpp.hpp>

#include <building_sim_common/utils.hpp>
#include <building_sim_common/slotcar_common.hpp>

using namespace ignition::gazebo;

using namespace building_sim_common;

enum class PhysEnginePlugin {DEFAULT, TPE};
std::unordered_map<std::string, PhysEnginePlugin> plugin_names {
  {"ignition-physics-tpe-plugin", PhysEnginePlugin::TPE}};

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
  void PreUpdate(const UpdateInfo& info, EntityComponentManager& ecm) override;

private:
  std::unique_ptr<SlotcarCommon> dataPtr;
  ignition::transport::Node _ign_node;
  rclcpp::Node::SharedPtr _ros_node;

  Entity _entity;
  std::unordered_set<Entity> _payloads;
  std::unordered_set<Entity> _infrastructure;
  double _height = 0;

  PhysEnginePlugin phys_plugin = PhysEnginePlugin::DEFAULT;

  bool first_iteration = true; // Flag for checking if it is first PreUpdate() call
  bool _read_aabb_dimensions = true;

  void charge_state_cb(const ignition::msgs::Selection& msg);

  void send_control_signals(EntityComponentManager& ecm,
    const std::pair<double, double>& velocities,
    const std::unordered_set<Entity> payloads,
    const double dt);
  void init_infrastructure(EntityComponentManager& ecm);
  void item_dispensed_cb(const ignition::msgs::UInt64_V& msg);
  void item_ingested_cb(const ignition::msgs::Entity& msg);
  bool get_slotcar_height(const ignition::msgs::Entity& req,
    ignition::msgs::Double& rep);
  std::vector<Eigen::Vector3d> get_obstacle_positions(
    EntityComponentManager& ecm);
};

SlotcarPlugin::SlotcarPlugin()
: dataPtr(std::make_unique<SlotcarCommon>())
{
  // Listen for messages that enable/disable charging
  if (!_ign_node.Subscribe("/charge_state", &SlotcarPlugin::charge_state_cb,
    this))
  {
    std::cerr << "Error subscribing to topic [/charge_state]" << std::endl;
  }
  // We do rest of initialization during ::Configure
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
  if (!rclcpp::ok())
    rclcpp::init(0, argv);
  std::string plugin_name("plugin_" + model_name);
  _ros_node = std::make_shared<rclcpp::Node>(plugin_name);
  // TODO Check if executor is getting callbacks
  //executor = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
  //executor->add_node(_ros_node);
  //executor->spin();
  dataPtr->init_ros_node(_ros_node);

  // Initialize Pose3d component
  if (!ecm.EntityHasComponentType(entity, components::Pose().TypeId()))
    ecm.CreateComponent(entity, components::Pose());
  // Initialize Bounding Box component
  if (!ecm.EntityHasComponentType(entity,
    components::AxisAlignedBox().TypeId()))
    ecm.CreateComponent(entity, components::AxisAlignedBox());
  // Initialize Linear/AngularVelocityCmd components to drive slotcar
  if (!ecm.EntityHasComponentType(_entity,
    components::LinearVelocityCmd().TypeId()))
    ecm.CreateComponent(_entity, components::LinearVelocityCmd());
  if (!ecm.EntityHasComponentType(_entity,
    components::AngularVelocityCmd().TypeId()))
    ecm.CreateComponent(_entity, components::AngularVelocityCmd());

  // Keep track of when a payload is dispensed onto/ingested from slotcar
  // Needed for TPE Plugin to know when to manually move payload via this plugin
  if (!_ign_node.Subscribe("/item_dispensed", &SlotcarPlugin::item_dispensed_cb,
    this))
  {
    std::cerr << "Error subscribing to topic [/item_dispensed]" << std::endl;
  }
  if (!_ign_node.Subscribe("/item_ingested", &SlotcarPlugin::item_ingested_cb,
    this))
  {
    std::cerr << "Error subscribing to topic [/item_ingested]" << std::endl;
  }
  // Respond to requests asking for height (e.g. for dispenser to dispense object)
  const std::string height_srv_name =
    "/slotcar_height_" + std::to_string(entity);
  if (!_ign_node.Advertise(height_srv_name, &SlotcarPlugin::get_slotcar_height,
    this))
  {
    std::cerr << "Error subscribing to topic [/slotcar_height]" << std::endl;
  }
}

void SlotcarPlugin::send_control_signals(EntityComponentManager& ecm,
  const std::pair<double, double>& velocities,
  const std::unordered_set<Entity> payloads,
  const double dt)
{
  auto lin_vel_cmd =
    ecm.Component<components::LinearVelocityCmd>(_entity);
  auto ang_vel_cmd =
    ecm.Component<components::AngularVelocityCmd>(_entity);

  double v_robot = lin_vel_cmd->Data()[0];
  double w_robot = ang_vel_cmd->Data()[2];
  std::array<double, 2> target_vels;
  target_vels = dataPtr->calculate_model_control_signals({v_robot, w_robot},
      velocities, dt);

  lin_vel_cmd->Data()[0] = target_vels[0];
  ang_vel_cmd->Data()[2] = target_vels[1];

  if (phys_plugin == PhysEnginePlugin::TPE) // Need to manually move any payloads
  {
    for (const Entity& payload : payloads)
    {
      if (!ecm.EntityHasComponentType(payload,
        components::LinearVelocityCmd().TypeId()))
      {
        ecm.CreateComponent(payload,
          components::LinearVelocityCmd({0, 0, 0}));
      }
      if (!ecm.EntityHasComponentType(payload,
        components::AngularVelocityCmd().TypeId()))
      {
        ecm.CreateComponent(payload,
          components::AngularVelocityCmd({0, 0, 0}));
      }
      ecm.Component<components::LinearVelocityCmd>(payload)->Data() =
        lin_vel_cmd->Data();
      ecm.Component<components::AngularVelocityCmd>(payload)->Data() =
        ang_vel_cmd->Data();
    }
  }
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

void SlotcarPlugin::charge_state_cb(const ignition::msgs::Selection& msg)
{
  dataPtr->charge_state_cb(msg.name(), msg.selected());
}

// First element of msg should be the slotcar, and the second should be the payload
void SlotcarPlugin::item_dispensed_cb(const ignition::msgs::UInt64_V& msg)
{
  if (msg.data_size() == 2 && msg.data(0) == _entity)
  {
    Entity new_payload = msg.data(1);
    this->_payloads.insert(new_payload);
  }
}

void SlotcarPlugin::item_ingested_cb(const ignition::msgs::Entity& msg)
{
  if (msg.IsInitialized())
  {
    const std::unordered_set<Entity>::iterator it = _payloads.find(msg.id());
    if (it != _payloads.end())
    {
      _payloads.erase(it);
    }
  }
}

bool SlotcarPlugin::get_slotcar_height(const ignition::msgs::Entity& req,
  ignition::msgs::Double& rep)
{
  if (req.id() == _entity)
  {
    rep.set_data(_height);
    return true;
  }
  return false;
}

void SlotcarPlugin::PreUpdate(const UpdateInfo& info,
  EntityComponentManager& ecm)
{
  // Read from components that may not have been initialized in configure()
  if (first_iteration)
  {
    Entity parent = _entity;
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
        phys_plugin = it->second;
      }
    }
    first_iteration = false;
  }

  // Optimization: Read and store slotcar's dimensions whenever available, then
  // delete the AABB component once read. Not deleting it causes rtf to drop by
  // a 3-4x factor whenever the slotcar moves.
  if (_read_aabb_dimensions)
  {
    const auto& aabb_component =
      ecm.Component<components::AxisAlignedBox>(_entity);
    if (aabb_component)
    {
      const double volume = aabb_component->Data().Volume();
      if (volume > 0 && volume != std::numeric_limits<double>::infinity())
      {
        _height = aabb_component->Data().ZLength();
        ecm.RemoveComponent(_entity, components::AxisAlignedBox().TypeId());
        _read_aabb_dimensions = false;
      }
    }
  }

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

  send_control_signals(ecm, velocities, _payloads, dt);
}

IGNITION_ADD_PLUGIN(
  SlotcarPlugin,
  System,
  SlotcarPlugin::ISystemConfigure,
  SlotcarPlugin::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(SlotcarPlugin, "slotcar")
