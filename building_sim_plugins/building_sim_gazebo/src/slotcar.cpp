#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <memory>

#include <gazebo/common/Plugin.hh>
#include <rmf_fleet_msgs/msg/destination_request.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rmf_fleet_msgs/msg/path_request.hpp>
#include <rmf_fleet_msgs/msg/mode_request.hpp>
#include <rmf_fleet_msgs/msg/robot_state.hpp>
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
  gazebo_ros::Node::SharedPtr _ros_node;
  gazebo::physics::ModelPtr _model;

  rclcpp::Subscription<rmf_fleet_msgs::msg::PathRequest>::SharedPtr traj_sub;
  rclcpp::Subscription<rmf_fleet_msgs::msg::ModeRequest>::SharedPtr mode_sub;
  rclcpp::Publisher<rmf_fleet_msgs::msg::RobotState>::SharedPtr robot_state_pub;
  rmf_fleet_msgs::msg::RobotState robot_state_msg;

  std::array<gazebo::physics::JointPtr, 2> joints;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf2_broadcaster;
  std::vector<ignition::math::Pose3d> traj;
  std::vector<rclcpp::Time> hold_times;
  rclcpp::Time start_time;
  std::vector<rmf_fleet_msgs::msg::Location> remaining_path;
  bool emergency_stop = false;
  rmf_fleet_msgs::msg::RobotMode current_mode;

  std::unordered_set<gazebo::physics::Model*> infrastructure;

  // Book keeping
  double goal_yaw_tolerance = 2 * M_PI;
  int traj_wp_idx = 0;
  double last_update_time = 0.0;
  bool load_complete = false;
  bool arrived_at_goal = false;
  int update_count = 0;
  std::string current_task_id;

  void init_infrastructure();


  std::pair<double, double> transform_coordinates(double x_target, double y_target)
  {
    // implement 2d transformation from traffic-editor coordinates to slotcar coordinates
    std::pair<double, double> x;
    x.first = x_target;
    x.second = y_target;
    return x;
  }

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

  // private:
  // Helper functions
  static double compute_heading(const ignition::math::Vector3d &dpos)
  {
    return std::atan2(dpos.Y(), dpos.X());
  }

  static ignition::math::Vector3d compute_dpos(
      const ignition::math::Pose3d &target,
      const ignition::math::Pose3d &actual)
  {
    return ignition::math::Vector3d(
        target.Pos().X() - actual.Pos().X(),
        target.Pos().Y() - actual.Pos().Y(),
        0.0);
  }

  static double clamp(const double value, const double limit)
  {
    const double abs_limit = std::abs(limit);
    if (std::abs(value) > abs_limit)
      return value < 0 ? -abs_limit : abs_limit;

    return value;
  }

  static double compute_change_in_rotation(
      const double actual_heading,
      const ignition::math::Vector3d& dpos,
      double* permissive = nullptr)
  {
    if (dpos.Length() < 1e-3)
    {
      // We're right next to the waypoint, so we don't really need any heading
      // to reach it.
      return 0.0;
    }

    ignition::math::Vector3d heading_vec(
        std::cos(actual_heading),
        std::sin(actual_heading),
        0.0);

    // Flip the heading vector if the dot product is less than zero. That way,
    // the robot will turn towards the heading that's closer.
    const double dot = heading_vec.Dot(dpos);
    if (permissive && dot < 0.0)
    {
      heading_vec = -1.0 * heading_vec;
      *permissive = -1.0;
    }
    else if (permissive)
    {
      *permissive = 1.0;
    }

    const ignition::math::Vector3d cross = heading_vec.Cross(dpos);
    const double direction = cross.Z() < 0.0 ? -1.0 : 1.0;
    const double denom = heading_vec.Length() * dpos.Length();
    const double d_yaw = direction * std::asin(cross.Length() / denom);

    return d_yaw;
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

void SlotcarPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  current_mode.mode = rmf_fleet_msgs::msg::RobotMode::MODE_MOVING;
  _model = model;
  dataPtr->set_model_name(_model->GetName());
  dataPtr->read_sdf(sdf);
  // TODO can we push _ros_node to common library?
  _ros_node = gazebo_ros::Node::Get(sdf);
  dataPtr->set_tf2_broadcaster(std::make_unique<tf2_ros::TransformBroadcaster>(_ros_node));

  RCLCPP_INFO(dataPtr->logger(), "hello i am " + model->GetName());

  _update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&SlotcarPlugin::OnUpdate, this));

  traj_sub = _ros_node->create_subscription<rmf_fleet_msgs::msg::PathRequest>(
      "/robot_path_requests", 10, std::bind(&SlotcarPlugin::path_request_cb, this, std::placeholders::_1));

  mode_sub = _ros_node->create_subscription<rmf_fleet_msgs::msg::ModeRequest>(
      "/robot_mode_requests", 10, std::bind(&SlotcarPlugin::mode_request_cb, this, std::placeholders::_1));

  robot_state_pub = _ros_node->create_publisher<rmf_fleet_msgs::msg::RobotState>(
      "/robot_state", 10);

  joints[0] = _model->GetJoint("joint_tire_left");
  if (!joints[0])
    RCLCPP_ERROR(dataPtr->logger(), "Could not find tire for [joint_tire_left]");

  joints[1] = _model->GetJoint("joint_tire_right");
  if (!joints[1])
    RCLCPP_ERROR(dataPtr->logger(), "Could not find tire for [joint_tire_right]");

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

void SlotcarPlugin::OnUpdate()
{
  update_count++;
  const auto& world = _model->GetWorld();
  if (update_count <= 1)
    init_infrastructure();

  const double time = world->SimTime().Double();
  const double dt = time - last_update_time;
  const int32_t t_sec = static_cast<int32_t>(time);
  const uint32_t t_nsec =
      static_cast<uint32_t>((time-static_cast<double>(t_sec)) *1e9);
  const rclcpp::Time now{t_sec, t_nsec, RCL_ROS_TIME};
  last_update_time = time;

  ignition::math::Pose3d pose = _model->WorldPose();
  auto wp = _model->WorldPose();
  // RCLCPP_INFO(impl_->ros_node_->get_logger(), "world_pose: [ %.3f, %.3f, %.3f ]",
  //             wp.Pos().X(), wp.Pos().Y(), wp.Pos().Z());

  // TODO make this sim time related
  if (update_count % 10 == 0)
    dataPtr->publish_tf2(wp, now);

  if (update_count % 100 == 0) // todo: be smarter, use elapsed sim time
  {
    robot_state_msg.name = dataPtr->model_name();

    robot_state_msg.location.x = wp.Pos().X();
    robot_state_msg.location.y = wp.Pos().Y();
    robot_state_msg.location.yaw = wp.Rot().Yaw();
    robot_state_msg.location.t = now;

    robot_state_msg.task_id = current_task_id;
    robot_state_msg.path = remaining_path;
    robot_state_msg.mode = current_mode;

    robot_state_pub->publish(robot_state_msg);
  }

  if (traj.empty())
    return;

  double x_target = 0.0;
  double yaw_target = 0.0;
  const double current_yaw = pose.Rot().Euler().Z();
  ignition::math::Vector3d current_heading{
    std::cos(current_yaw), std::sin(current_yaw), 0.0};

  if ((unsigned int)traj_wp_idx < traj.size())
  {
    const ignition::math::Vector3d dpos = compute_dpos(
          traj[traj_wp_idx], wp);

    auto dpos_mag = dpos.Length();
    const auto hold_time = hold_times[traj_wp_idx];

    const bool close_enough = (dpos_mag < 0.02);
    const bool rotate_towards_next_target = close_enough && (now < hold_time);

    if (rotate_towards_next_target)
    {
      const double goal_yaw = traj[traj_wp_idx].Rot().Yaw();
      const ignition::math::Vector3d goal_heading{
            std::cos(goal_yaw), std::sin(goal_yaw), 0.0};

      yaw_target = compute_change_in_rotation(
            pose.Rot().Euler().Z(), goal_heading);
    }
    else if (close_enough)
    {
      traj_wp_idx++;
      if (remaining_path.empty())
        return;

      remaining_path.erase(remaining_path.begin());
      RCLCPP_INFO(dataPtr->logger(),
                  "%s reached waypoint %d/%d",
                  dataPtr->model_name().c_str(),
                  traj_wp_idx,
                  (int)traj.size());
      if ((unsigned int)traj_wp_idx == traj.size())
      {
        RCLCPP_INFO(
            dataPtr->logger(),
            "%s reached goal -- rotating to face target",
            dataPtr->model_name().c_str());
      }
    }

    if (!rotate_towards_next_target)
    {
      const double d_yaw_tolerance = 5.0 * M_PI / 180.0;

      double dir = 1.0;
      yaw_target = compute_change_in_rotation(current_yaw, dpos, &dir);
      if (dir < 0.0)
        current_heading *= -1.0;

      // If d_yaw is less than a certain tolerance (i.e. we don't need to spin
      // too much), then we'll include the forward velocity. Otherwise, we will
      // only spin in place until we are oriented in the desired direction.
      x_target = std::abs(yaw_target) < d_yaw_tolerance ? dir * dpos_mag : 0.0;
    }
  }
  else
  {
    const double goal_yaw = traj.back().Rot().Yaw();
    yaw_target = compute_change_in_rotation(
        current_yaw,
        ignition::math::Vector3d(cos(goal_yaw), sin(goal_yaw), 0.0));

    // Put in a deadzone if yaw is small enough. This essentially locks the
    // tires. COMMENTED OUT as it breaks rotations for some reason...
    // if(std::abs(yaw_target) < std::max(0.1*M_PI/180.00, goal_yaw_tolerance))
    // {
    //   yaw_target = 0.0;
    // }

    x_target = 0.0;
  }

  const ignition::math::Vector3d stop_zone =
      wp.Pos() + dataPtr->stop_distance()*current_heading;

  bool need_to_stop = false;
  const auto& all_models = world->Models();
  for (const auto& m : all_models)
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

  send_control_signals(x_target, yaw_target, dt);
}

void SlotcarPlugin::path_request_cb(const rmf_fleet_msgs::msg::PathRequest::SharedPtr msg)
{
  if (msg->robot_name != dataPtr->model_name())
  {
    RCLCPP_INFO(
          dataPtr->logger(),
          "Ignoring path request for ["
          + msg->robot_name + "]");
    return;
  }

  if (msg->task_id == current_task_id)
  {
    RCLCPP_INFO(
          dataPtr->logger(),
          "Already received task [" + current_task_id
          + "] -- continuing as normal");
    return;
  }

  if (msg->path.size() == 0)
  {
    RCLCPP_WARN(dataPtr->logger(), "got a path with no waypoints");
    return;
  }

  RCLCPP_INFO(
      dataPtr->logger(),
      "greetings. got a path with %d waypoints",
      (int)msg->path.size());

  auto l_to_str = [](const ignition::math::Pose3d& p)
  {
    return std::to_string(p.Pos().X()) + ", "
        + std::to_string(p.Pos().Y()) + ", "
        + std::to_string(p.Rot().Yaw());
  };

  const auto initial_pose = _model->WorldPose();
  std::string path_str = "Path for [" + dataPtr->model_name() + "] starting from ("
      + l_to_str(initial_pose) + ") | [" + msg->task_id + "]:";

  // Reset this if we aren't at the final waypoint
  arrived_at_goal = false;
  traj.resize(msg->path.size());
  hold_times.resize(msg->path.size());
  for (size_t i = 0; i < msg->path.size(); ++i)
  {
    std::pair<double, double> local_coordinates =
        transform_coordinates(msg->path[i].x, msg->path[i].y);
    ignition::math::Vector3<double> v3(
        local_coordinates.first,
        local_coordinates.second,
        0);

    ignition::math::Vector3<double> yaw_euler(
        0,
        0,
        msg->path[i].yaw);

    ignition::math::Quaternion<double> quat = ignition::math::Quaternion<double>::EulerToQuaternion(yaw_euler);
    traj[i] = ignition::math::Pose3d(v3, quat);

    hold_times[i] = msg->path[i].t;

    const double s = hold_times[i].seconds();
    const uint64_t minutes = static_cast<uint64_t>(s/60.0);
    const double s_remainder = s - static_cast<double>(minutes)*60.0;
    path_str += "\n -- (" + std::to_string(minutes) + ":"
        + std::to_string(s_remainder) + ") [" + std::to_string(s) + "] "
        + l_to_str(traj[i]);
  }
  remaining_path = msg->path;
  traj_wp_idx = 0;

  if (!msg->path.empty())
    start_time = msg->path.front().t;

  goal_yaw_tolerance = 0.1; // TODO: Clarify this placeholder tolerance

  current_task_id = msg->task_id;

  std::cout << path_str << std::endl;

  const double initial_dist = compute_dpos(traj.front(), initial_pose).Length();
  if (initial_dist > 0.5)
  {
    RCLCPP_ERROR(
          _ros_node->get_logger(),
          "BIG ERROR IN INITIAL DISTANCE: " + std::to_string(initial_dist));
  }
}

void SlotcarPlugin::mode_request_cb(const rmf_fleet_msgs::msg::ModeRequest::SharedPtr msg)
{
  current_mode = msg->mode;
}

GZ_REGISTER_MODEL_PLUGIN(SlotcarPlugin)
