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

#include <building_map_msgs/msg/building_map.hpp>

#include "utils.hpp"

using namespace building_gazebo_plugins;

class SlotcarPlugin : public gazebo::ModelPlugin
{
public:
  SlotcarPlugin();
  ~SlotcarPlugin();

  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;
  void path_request_cb(const rmf_fleet_msgs::msg::PathRequest::SharedPtr msg);
  void mode_request_cb(const rmf_fleet_msgs::msg::ModeRequest::SharedPtr msg);
  void map_cb(const building_map_msgs::msg::BuildingMap::SharedPtr msg);

  void OnUpdate();

private:
  rclcpp::Logger logger();

  gazebo::event::ConnectionPtr _update_connection;
  gazebo_ros::Node::SharedPtr _ros_node;
  gazebo::physics::ModelPtr _model;

  rclcpp::Subscription<rmf_fleet_msgs::msg::PathRequest>::SharedPtr traj_sub;
  rclcpp::Subscription<rmf_fleet_msgs::msg::ModeRequest>::SharedPtr mode_sub;
  rclcpp::Publisher<rmf_fleet_msgs::msg::RobotState>::SharedPtr robot_state_pub;
  rclcpp::Subscription<building_map_msgs::msg::BuildingMap>::SharedPtr _building_map_sub;

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
  double _last_pub_time = 0.0;
  bool load_complete = false;
  bool arrived_at_goal = false;
  int update_count = 0;
  std::string name;
  std::string current_task_id;
  std::unordered_map<std::string, double> _level_to_elevation;
  bool _initialized_levels = false;
  bool adapter_error = false;

  // Vehicle dynamic constants
  // TODO(MXG): Consider fetching these values from model data
  // Radius of a tire
  double tire_radius = 0.1;
  // min_Distance of a tire from the origin
  double base_width = 0.52;

  double nominal_drive_speed = 0.5;         // nominal robot velocity (m/s)
  double nominal_drive_acceleration = 0.05; // nominal robot forward acceleration (m/s^2)
  double max_drive_acceleration = 0.1;      // maximum robot forward acceleration (m/s^2)

  double nominal_turn_speed = M_PI / 8.0;         // nominal robot turning speed (half a rotation per 8 seconds)
  double nominal_turn_acceleration = M_PI / 10.0; // nominal robot turning acceleration (rad/s^2)

  double max_turn_acceleration = M_PI; // maximum robot turning acceleration (rad/s^2)

  double stop_min_distance = 1.0;
  double stop_radius = 1.0;

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
    const double v_actual = (w_tire_actual[0] + w_tire_actual[1]) * tire_radius / 2.0;
    const double w_actual = (w_tire_actual[1] - w_tire_actual[0]) * tire_radius / base_width;

    const double v_target = compute_ds(x_target, v_actual, nominal_drive_speed,
                                       nominal_drive_acceleration, max_drive_acceleration, dt);

    const double w_target = compute_ds(yaw_target, w_actual, nominal_turn_speed,
                                       nominal_turn_acceleration, max_turn_acceleration, dt);
    for (std::size_t i = 0; i < 2; ++i)
    {
      const double yaw_sign = i == 0 ? -1.0 : 1.0;
      joints[i]->SetParam(
          "vel", 0, v_target / tire_radius + yaw_sign * w_target * base_width / (2.0 * tire_radius));
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

  std::string get_level_name(const double z)
  {
    std::string level_name = "";
    if (!_initialized_levels)
      return level_name;
    auto min_distance = std::numeric_limits<double>::max();
    for (auto it = _level_to_elevation.begin(); it != _level_to_elevation.end();
      ++it)
    {
      const double disp = std::abs(it->second - z);
      if (disp < min_distance)
      {
        min_distance = disp;
        level_name = it->first;
      }
    }
    return level_name;
  }

};

SlotcarPlugin::SlotcarPlugin()
{
  // We do initialization only during ::Load
}

SlotcarPlugin::~SlotcarPlugin()
{
}

rclcpp::Logger SlotcarPlugin::logger()
{
  return rclcpp::get_logger("slotcar_" + _model->GetName());
}

void SlotcarPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  current_mode.mode = rmf_fleet_msgs::msg::RobotMode::MODE_MOVING;
  _model = model;
  _ros_node = gazebo_ros::Node::Get(sdf);
  tf2_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(_ros_node);

  RCLCPP_INFO(logger(), "hello i am " + model->GetName());

  _update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&SlotcarPlugin::OnUpdate, this));

  traj_sub = _ros_node->create_subscription<rmf_fleet_msgs::msg::PathRequest>(
      "/robot_path_requests", 10, std::bind(&SlotcarPlugin::path_request_cb, this, std::placeholders::_1));

  mode_sub = _ros_node->create_subscription<rmf_fleet_msgs::msg::ModeRequest>(
      "/robot_mode_requests", 10, std::bind(&SlotcarPlugin::mode_request_cb, this, std::placeholders::_1));

  robot_state_pub = _ros_node->create_publisher<rmf_fleet_msgs::msg::RobotState>(
      "/robot_state", 10);

  // Subscription to /map
  auto qos_profile = rclcpp::QoS(10);
  qos_profile.transient_local();
  _building_map_sub =
    _ros_node->create_subscription<building_map_msgs::msg::BuildingMap>(
        "/map",
        qos_profile,
        std::bind(&SlotcarPlugin::map_cb, this, std::placeholders::_1));

  joints[0] = _model->GetJoint("joint_tire_left");
  if (!joints[0])
    RCLCPP_ERROR(logger(), "Could not find tire for [joint_tire_left]");

  joints[1] = _model->GetJoint("joint_tire_right");
  if (!joints[1])
    RCLCPP_ERROR(logger(), "Could not find tire for [joint_tire_right]");

  if (sdf->HasElement("nominal_drive_speed"))
    nominal_drive_speed = sdf->Get<double>("nominal_drive_speed");
  RCLCPP_INFO(logger(), "Setting nominal drive speed to: " + std::to_string(nominal_drive_speed));

  if (sdf->HasElement("nominal_drive_acceleration"))
    nominal_drive_acceleration = sdf->Get<double>("nominal_drive_acceleration");
  RCLCPP_INFO(logger(), "Setting nominal drive acceleration to: " + std::to_string(nominal_drive_acceleration));

  if (sdf->HasElement("max_drive_acceleration"))
    max_drive_acceleration = sdf->Get<double>("max_drive_acceleration");
  RCLCPP_INFO(logger(), "Setting max drive acceleration to: " + std::to_string(max_drive_acceleration));

  if (sdf->HasElement("nominal_turn_speed"))
    nominal_turn_speed = sdf->Get<double>("nominal_turn_speed");
  RCLCPP_INFO(logger(), "Setting nominal turn speed to:" + std::to_string(nominal_turn_speed));

  if (sdf->HasElement("nominal_turn_acceleration"))
    nominal_turn_acceleration = sdf->Get<double>("nominal_turn_acceleration");
  RCLCPP_INFO(logger(), "Setting nominal turn acceleration to:" + std::to_string(nominal_turn_acceleration));

  if (sdf->HasElement("max_turn_acceleration"))
    max_turn_acceleration = sdf->Get<double>("max_turn_acceleration");
  RCLCPP_INFO(logger(), "Setting max turn acceleration to:" + std::to_string(max_turn_acceleration));

  if (sdf->HasElement("stop_min_distance"))
    stop_min_distance = sdf->Get<double>("stop_min_distance");
  RCLCPP_INFO(logger(), "Setting stop min_distance to:" + std::to_string(stop_min_distance));

  if (sdf->HasElement("stop_radius"))
    stop_radius = sdf->Get<double>("stop_radius");
  RCLCPP_INFO(logger(), "Setting stop radius to:" + std::to_string(stop_radius));

  if (sdf->HasElement("tire_radius"))
    tire_radius = sdf->Get<double>("tire_radius");
  RCLCPP_INFO(logger(), "Setting tire radius to:" + std::to_string(tire_radius));

  if (sdf->HasElement("base_width"))
    base_width = sdf->Get<double>("base_width");
  RCLCPP_INFO(logger(), "Setting base width to:" + std::to_string(base_width));

  if (model->GetName().c_str())
    name = _model->GetName().c_str();
  RCLCPP_INFO(logger(), "Setting name to: " + name);
}

void SlotcarPlugin::OnUpdate()
{
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

      std::string name = m->GetName();
      std::for_each(name.begin(), name.end(), [](char& c){
        c = ::tolower(c);
      });

      if (name.find("door") != std::string::npos)
        infrastructure.insert(m.get());

      if (name.find("lift") != std::string::npos)
        infrastructure.insert(m.get());
    }
  }

  const double t = world->SimTime().Double();
  const double dt = t - last_update_time;
  last_update_time = t;

  ignition::math::Pose3d pose = _model->WorldPose();
  auto wp = _model->WorldPose();
  // RCLCPP_INFO(impl_->ros_node_->get_logger(), "world_pose: [ %.3f, %.3f, %.3f ]",
  //             wp.Pos().X(), wp.Pos().Y(), wp.Pos().Z());

  if (update_count % 10 == 0)
  {
    geometry_msgs::msg::TransformStamped tf_stamped;
    tf_stamped.header.stamp = rclcpp::Time(t);
    tf_stamped.header.frame_id = "world";
    tf_stamped.child_frame_id = _model->GetName() + "/base_link";
    tf_stamped.transform.translation.x = wp.Pos().X();
    tf_stamped.transform.translation.y = wp.Pos().Y();
    tf_stamped.transform.translation.z = wp.Pos().Z();
    tf_stamped.transform.rotation.x = wp.Rot().X();
    tf_stamped.transform.rotation.y = wp.Rot().Y();
    tf_stamped.transform.rotation.z = wp.Rot().Z();
    tf_stamped.transform.rotation.w = wp.Rot().W();
    tf2_broadcaster->sendTransform(tf_stamped);
  }

  const double time = world->SimTime().Double();
  const int32_t t_sec = static_cast<int32_t>(time);
  const uint32_t t_nsec =
      static_cast<uint32_t>((time-static_cast<double>(t_sec)) *1e9);
  const rclcpp::Time now{t_sec, t_nsec, RCL_ROS_TIME};

  if (time - _last_pub_time > 0.5) // publish at 2Hz
  {
    robot_state_msg.name = _model->GetName();

    robot_state_msg.location.x = wp.Pos().X();
    robot_state_msg.location.y = wp.Pos().Y();
    robot_state_msg.location.yaw = wp.Rot().Yaw();
    robot_state_msg.location.t = now;
    robot_state_msg.location.level_name = get_level_name(wp.Pos().Z());

    robot_state_msg.task_id = current_task_id;
    robot_state_msg.path = remaining_path;
    robot_state_msg.mode = current_mode;

    if (adapter_error)
    {
      robot_state_msg.mode.mode =
          rmf_fleet_msgs::msg::RobotMode::MODE_ADAPTER_ERROR;
    }

    robot_state_pub->publish(robot_state_msg);
    _last_pub_time = time;
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
      RCLCPP_INFO(logger(),
                  "%s reached waypoint %d/%d",
                  _model->GetName().c_str(),
                  traj_wp_idx,
                  (int)traj.size());
      if ((unsigned int)traj_wp_idx == traj.size())
      {
        RCLCPP_INFO(
            logger(),
            "%s reached goal -- rotating to face target",
            _model->GetName().c_str());
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
      wp.Pos() + stop_min_distance*current_heading;

  bool need_to_stop = false;
  const auto& all_models = world->Models();
  std::string avoiding_name;
  for (const auto& m : all_models)
  {
    if (m->IsStatic())
      continue;

    if (infrastructure.count(m.get()) > 0)
      continue;

    const auto p_obstacle = m->WorldPose().Pos();
    if ( (p_obstacle - stop_zone).Length() < stop_radius )
    {
      avoiding_name = m->GetName();
      need_to_stop = true;
      break;
    }
  }

  if (need_to_stop != emergency_stop)
  {
    emergency_stop = need_to_stop;
    if (need_to_stop)
      std::cout << "Stopping [" << name << "] to avoid a collision with [" << avoiding_name << "]" << std::endl;
    else
      std::cout << "No more obstacles; resuming course for [" << name << "]" << std::endl;
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
  if (msg->robot_name != name)
  {
    RCLCPP_INFO(
          logger(),
          "Ignoring path request for ["
          + msg->robot_name + "]");
    return;
  }

  if (msg->task_id == current_task_id)
  {
    RCLCPP_INFO(
          logger(),
          "Already received task [" + current_task_id
          + "] -- continuing as normal");
    return;
  }

  if (msg->path.size() == 0)
  {
    RCLCPP_WARN(logger(), "got a path with no waypoints");
    return;
  }

  RCLCPP_INFO(
      logger(),
      "greetings. got a path with %d waypoints",
      (int)msg->path.size());

  auto l_to_str = [](const ignition::math::Pose3d& p)
  {
    return std::to_string(p.Pos().X()) + ", "
        + std::to_string(p.Pos().Y()) + ", "
        + std::to_string(p.Rot().Yaw());
  };

  const auto initial_pose = _model->WorldPose();
  std::string path_str = "Path for [" + _model->GetName() + "] starting from ("
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
  adapter_error = false;

  std::cout << path_str << std::endl;

  const double initial_dist = compute_dpos(traj.front(), initial_pose).Length();
  if (initial_dist > 1.0)
  {
    RCLCPP_ERROR(
          _ros_node->get_logger(),
          "BIG ERROR IN INITIAL min_DISTANCE: " + std::to_string(initial_dist));
//    RCLCPP_ERROR(
//          logger(),
//          "Ignoring path request and stopping because it begins too far from "
//          "where we are: " + std::to_string(initial_dist));
    traj.clear();
    traj.push_back(initial_pose);
    start_time = _ros_node->now();
    adapter_error = true;
  }
}

void SlotcarPlugin::mode_request_cb(const rmf_fleet_msgs::msg::ModeRequest::SharedPtr msg)
{
  current_mode = msg->mode;
}

void SlotcarPlugin::map_cb(const building_map_msgs::msg::BuildingMap::SharedPtr msg)
{
  if (msg->levels.empty())
  {
    RCLCPP_ERROR(logger(), "Received empty building map");
    return;
  }

  RCLCPP_INFO(logger(),
    "Received building map with %d levels", msg->levels.size());

  for (const auto& level : msg->levels)
    _level_to_elevation.insert({level.name, level.elevation});

  _initialized_levels = true;
}

GZ_REGISTER_MODEL_PLUGIN(SlotcarPlugin)
