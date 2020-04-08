#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo_ros/node.hpp>

#include <memory>

#include <gazebo/common/Plugin.hh>
#include <rclcpp/logger.hpp>

#include <building_sim_common/utils.hpp>
#include <building_sim_common/slotcar_common.hpp>

using namespace building_sim_common;

class SlotcarPlugin : public gazebo::ModelPlugin
{
public:
  SlotcarPlugin();
  ~SlotcarPlugin();

  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;
  void path_request_cb(const rmf_fleet_msgs::msg::PathRequest::SharedPtr msg);
  void mode_request_cb(const rmf_fleet_msgs::msg::ModeRequest::SharedPtr msg);
  void OnUpdate();

private:
  std::unique_ptr<SlotcarCommon> dataPtr;

  gazebo::event::ConnectionPtr _update_connection;
  gazebo::physics::ModelPtr _model;

  std::array<gazebo::physics::JointPtr, 2> joints;

  std::unordered_set<gazebo::physics::Model*> infrastructure;

  // Book keeping
  double last_update_time = 0.0;

  void init_infrastructure();

  std::vector<Eigen::Vector3d> get_obstacle_positions(const gazebo::physics::WorldPtr& world);

  void send_control_signals(
    const double x_target,
    const double yaw_target,
    const double dt)
  {
    std::array<double, 2> w_tire_actual;
    for (std::size_t i = 0; i < 2; ++i)
      w_tire_actual[i] = joints[i]->GetVelocity(0);
    auto joint_signals = dataPtr->calculate_control_signals(w_tire_actual,
        x_target, yaw_target, dt);
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
  // We do initialization only during ::Load
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

void SlotcarPlugin::init_infrastructure()
{
  const auto& world = _model->GetWorld();
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

std::vector<Eigen::Vector3d> SlotcarPlugin::get_obstacle_positions(const gazebo::physics::WorldPtr& world)
{
  std::vector<Eigen::Vector3d> obstacle_positions;

  for (const auto& m : world->Models())
  {
    if (m->IsStatic())
      continue;

    if (infrastructure.count(m.get()) > 0)
      continue;

    const auto p_obstacle = m->WorldPose().Pos();
    obstacle_positions.push_back(convert_vec(p_obstacle));
  }

  return obstacle_positions;
}

void SlotcarPlugin::OnUpdate()
{
  const auto& world = _model->GetWorld();
  if (infrastructure.empty())
    init_infrastructure();
  double x_target = 0.0;
  double yaw_target = 0.0;

  const double time = world->SimTime().Double();
  const double dt = time - last_update_time;
  last_update_time = time;

  ignition::math::Pose3d pose = _model->WorldPose();

  // Will return false if there is no more waypoints
  if (!dataPtr->update(convert_pose(pose), time, x_target, yaw_target))
    return;

  auto obstacle_positions = get_obstacle_positions(world);

  bool emergency_stop = dataPtr->emergency_stop(obstacle_positions);

  if (emergency_stop)
  {
    // Allow spinning but not translating
    x_target = 0.0;
  }

  send_control_signals(x_target, yaw_target, dt);
}

GZ_REGISTER_MODEL_PLUGIN(SlotcarPlugin)
