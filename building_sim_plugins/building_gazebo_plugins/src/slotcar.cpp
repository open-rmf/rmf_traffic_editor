#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo_ros/node.hpp>

#include <gazebo/common/Plugin.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <building_sim_common/utils.hpp>
#include <building_sim_common/slotcar_common.hpp>

using namespace building_sim_common;

class SlotcarPlugin : public gazebo::ModelPlugin
{
public:
  SlotcarPlugin();
  ~SlotcarPlugin();

  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;
  void OnUpdate();

private:
  std::unique_ptr<SlotcarCommon> dataPtr;

  gazebo::transport::NodePtr _gazebo_node;
  gazebo::transport::SubscriberPtr _charge_state_sub;

  gazebo::event::ConnectionPtr _update_connection;
  gazebo::physics::ModelPtr _model;

  std::array<gazebo::physics::JointPtr, 2> joints;

  std::unordered_set<gazebo::physics::Model*> infrastructure;

  // Book keeping
  double last_update_time = 0.0;

  void init_infrastructure();

  std::vector<Eigen::Vector3d> get_obstacle_positions(
    const gazebo::physics::WorldPtr& world);

  void charge_state_cb(ConstSelectionPtr& msg);

  void send_control_signals(const std::pair<double, double>& velocities,
    const double dt)
  {
    std::array<double, 2> w_tire;
    for (std::size_t i = 0; i < 2; ++i)
      w_tire[i] = joints[i]->GetVelocity(0);
    auto joint_signals = dataPtr->calculate_joint_control_signals(w_tire,
        velocities, dt);
    for (std::size_t i = 0; i < 2; ++i)
    {
      joints[i]->SetParam("vel", 0, joint_signals[i]);
      joints[i]->SetParam("fmax", 0, 10000000.0); // TODO(MXG): Replace with realistic torque limit
    }
  }
};

SlotcarPlugin::SlotcarPlugin()
: dataPtr(std::make_unique<SlotcarCommon>())
{
  // Listen for messages that enable/disable charging
  _gazebo_node = gazebo::transport::NodePtr(new gazebo::transport::Node());
  _gazebo_node->Init();
  _charge_state_sub = _gazebo_node->Subscribe("/charge_state",
      &SlotcarPlugin::charge_state_cb, this);
  // We do rest of initialization during ::Load
}

SlotcarPlugin::~SlotcarPlugin()
{
}

void SlotcarPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  _model = model;
  dataPtr->set_model_name(_model->GetName());
  dataPtr->read_sdf(sdf);
  gazebo_ros::Node::SharedPtr _ros_node = gazebo_ros::Node::Get(sdf);
  dataPtr->init_ros_node(_ros_node);

  RCLCPP_INFO(dataPtr->logger(),
    "Initialising slotcar for " + model->GetName());

  _update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&SlotcarPlugin::OnUpdate, this));

  joints[0] = _model->GetJoint("joint_tire_left");
  if (!joints[0])
    RCLCPP_ERROR(dataPtr->logger(),
      "Could not find tire for [joint_tire_left]");

  joints[1] = _model->GetJoint("joint_tire_right");
  if (!joints[1])
    RCLCPP_ERROR(dataPtr->logger(),
      "Could not find tire for [joint_tire_right]");

}

void SlotcarPlugin::charge_state_cb(ConstSelectionPtr& msg)
{
  dataPtr->charge_state_cb(msg->name(), msg->selected());
}

void SlotcarPlugin::init_infrastructure()
{
  const auto& world = _model->GetWorld();
  infrastructure.insert(_model.get());
  const auto& all_models = world->Models();
  for (const auto& m : all_models)
  {
    // Object should not be static and part of infrastructure
    if (!m->IsStatic())
    {
      std::string name = m->GetName();
      std::for_each(name.begin(), name.end(), [](char& c)
        {
          c = ::tolower(c);
        });
      if (name.find("door") != std::string::npos ||
        name.find("lift") != std::string::npos)
        infrastructure.insert(m.get());
    }
  }
}

std::vector<Eigen::Vector3d> SlotcarPlugin::get_obstacle_positions(
  const gazebo::physics::WorldPtr& world)
{
  std::vector<Eigen::Vector3d> obstacle_positions;

  for (const auto& m : world->Models())
  {
    // Object should not be static, not part of infrastructure
    // and close than a threshold (checked by common function)
    const auto p_obstacle = m->WorldPose().Pos();
    if (m->IsStatic() == false &&
      infrastructure.find(m.get()) == infrastructure.end())
      obstacle_positions.push_back(convert_vec(p_obstacle));
  }

  return obstacle_positions;
}

void SlotcarPlugin::OnUpdate()
{
  const auto& world = _model->GetWorld();
  if (infrastructure.empty())
    init_infrastructure();

  const double time = world->SimTime().Double();
  const double dt = time - last_update_time;
  last_update_time = time;

  auto pose = _model->WorldPose();
  auto obstacle_positions = get_obstacle_positions(world);

  auto velocities =
    dataPtr->update(convert_pose(pose), obstacle_positions, time);

  send_control_signals(velocities, dt);
}

GZ_REGISTER_MODEL_PLUGIN(SlotcarPlugin)
