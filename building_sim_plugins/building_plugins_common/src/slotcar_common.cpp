/*
 * Copyright (C) 2020 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <memory>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rmf_fleet_msgs/msg/destination_request.hpp>
#include <rclcpp/logger.hpp>

#include <building_sim_common/utils.hpp>
#include <building_sim_common/slotcar_common.hpp>

using namespace building_sim_common;

static double compute_yaw(const Eigen::Isometry3d& pose)
{
  auto quat = Eigen::Quaterniond(pose.linear());
  // Taken from ignition math quaternion Euler()
  double yaw = std::atan2(2 * (quat.x()*quat.y() + quat.w()*quat.z()),
      (quat.w() * quat.w()) + (quat.x() * quat.x()) - (quat.y() * quat.y()) -
      (quat.z() * quat.z()));
  return yaw;
}

// Computes change in yaw angle from old_pose to pose
static double compute_yaw(const Eigen::Isometry3d& pose,
  const Eigen::Isometry3d& old_pose, int rot_dir)
{
  const double yaw = compute_yaw(pose);
  const double old_yaw = compute_yaw(old_pose);

  double disp = yaw - old_yaw;
  double eps = 0.01;
  // Account for edge cases where the robot rotates past -PI rad to PI rad, or vice versa
  if (rot_dir > 0 && yaw < (old_yaw - eps))
  {
    disp = (M_PI - old_yaw) + (yaw + M_PI);
  }
  else if (rot_dir < 0 && yaw > (old_yaw + eps))
  {
    disp = (M_PI - yaw) + (old_yaw + M_PI);
  }
  return disp;
}

static Eigen::Vector3d compute_heading(const Eigen::Isometry3d& pose)
{
  double yaw = compute_yaw(pose);
  return Eigen::Vector3d(std::cos(yaw), std::sin(yaw), 0.0);
}

inline static Eigen::Vector3d compute_dist(const Eigen::Isometry3d& old_pos,
  const Eigen::Isometry3d& new_pos)
{
  return new_pos.translation() - old_pos.translation();
}

inline static auto compute_dpos(const Eigen::Isometry3d& target,
  const Eigen::Isometry3d& actual)
{
  Eigen::Vector3d dpos(compute_dist(actual, target));
  dpos(2) = 0.0;
  return dpos;
}

double compute_friction_energy(
  const double f,
  const double m,
  const double v,
  const double dt)
{
  const double g = 9.81; // ms-1
  return f * m * g * v * dt;
}

SlotcarCommon::SlotcarCommon()
{
  // Make sure we initialize this message to TYPE_RESUME, or else the robot
  // might just sit and wait around unintentionally.
  pause_request = rmf_fleet_msgs::build<rmf_fleet_msgs::msg::PauseRequest>()
    .fleet_name("")
    .robot_name("")
    .mode_request_id(0)
    .type(rmf_fleet_msgs::msg::PauseRequest::TYPE_RESUME)
    .at_checkpoint(0);
}

rclcpp::Logger SlotcarCommon::logger() const
{
  return rclcpp::get_logger("slotcar_" + _model_name);
}

void SlotcarCommon::set_model_name(const std::string& model_name)
{
  _model_name = model_name;
}

std::string SlotcarCommon::model_name() const
{
  return _model_name;
}

void SlotcarCommon::init_ros_node(const rclcpp::Node::SharedPtr node)
{
  _current_mode.mode = rmf_fleet_msgs::msg::RobotMode::MODE_MOVING;
  _ros_node = std::move(node);

  _tf2_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(_ros_node);

  _robot_state_pub =
    _ros_node->create_publisher<rmf_fleet_msgs::msg::RobotState>(
    "/robot_state", 10);

  auto qos_profile = rclcpp::QoS(10);
  qos_profile.transient_local();
  _building_map_sub =
    _ros_node->create_subscription<building_map_msgs::msg::BuildingMap>(
    "/map",
    qos_profile,
    std::bind(&SlotcarCommon::map_cb, this, std::placeholders::_1));

  _traj_sub = _ros_node->create_subscription<rmf_fleet_msgs::msg::PathRequest>(
    "/robot_path_requests",
    10,
    std::bind(&SlotcarCommon::path_request_cb, this, std::placeholders::_1));

  using PauseRequest = rmf_fleet_msgs::msg::PauseRequest;
  _pause_sub = _ros_node->create_subscription<PauseRequest>(
    "/robot_pause_requests",
    10,
    std::bind(&SlotcarCommon::pause_request_cb, this, std::placeholders::_1));

  _mode_sub = _ros_node->create_subscription<rmf_fleet_msgs::msg::ModeRequest>(
    "/robot_mode_requests",
    10,
    std::bind(&SlotcarCommon::mode_request_cb, this, std::placeholders::_1));
}

bool SlotcarCommon::path_request_valid(
  const rmf_fleet_msgs::msg::PathRequest::SharedPtr msg)
{
  // Request is for another robot
  if (msg->robot_name != _model_name)
    return false;

  // Repeated task request
  if (msg->task_id == _current_task_id)
  {
    RCLCPP_INFO(
      logger(), "%s already received task [%s] -- continuing as normal",
      _current_task_id.c_str(), _model_name.c_str());
    return false;
  }

  // Empty task request
  if (msg->path.size() == 0)
  {
    RCLCPP_WARN(logger(), "%s received a path with no waypoints",
      _model_name.c_str());
    return false;
  }
  return true;
}

void SlotcarCommon::path_request_cb(
  const rmf_fleet_msgs::msg::PathRequest::SharedPtr msg)
{
  if (path_request_valid(msg) == false)
    return;

  const auto old_path = _remaining_path;

  RCLCPP_INFO(
    logger(),
    "%s received a path request with %d waypoints",
    _model_name.c_str(), (int)msg->path.size());

  std::lock_guard<std::mutex> lock(_mutex);
  // Reset this if we aren't at the final waypoint
  trajectory.resize(msg->path.size());
  _hold_times.resize(msg->path.size());
  for (size_t i = 0; i < msg->path.size(); ++i)
  {
    Eigen::Vector3d v3(
      msg->path[i].x,
      msg->path[i].y,
      0);

    Eigen::Vector3d yaw_euler(
      0,
      0,
      msg->path[i].yaw);

    Eigen::Quaterniond quat(
      Eigen::AngleAxisd(msg->path[i].yaw, Eigen::Vector3d::UnitZ()));
    trajectory.at(i).translation() = v3;
    trajectory.at(i).linear() = Eigen::Matrix3d(quat);

    _hold_times.at(i) = msg->path[i].t;
  }
  _remaining_path = msg->path;
  _traj_wp_idx = 0;

  _current_task_id = msg->task_id;
  _adapter_error = false;

  const double initial_dist = compute_dpos(trajectory.front(), _pose).norm();

  if (initial_dist > INITIAL_DISTANCE_THRESHOLD)
  {
    trajectory.clear();
    trajectory.push_back(_pose);

    _hold_times.clear();
    _hold_times.push_back(rclcpp::Time((int64_t)0, RCL_ROS_TIME));

    // We'll stick with the old path when an adapter error happens so that the
    // fleet adapter knows where the robot currently is along its previous path.
    _remaining_path = old_path;

    _adapter_error = true;
  }
  else
  {
    trajectory.erase(trajectory.begin());
    _hold_times.erase(_hold_times.begin());
    _remaining_path.erase(_remaining_path.begin());
  }
}

void SlotcarCommon::pause_request_cb(
  const rmf_fleet_msgs::msg::PauseRequest::SharedPtr msg)
{
  if (msg->robot_name != _model_name)
    return;

  std::lock_guard<std::mutex> lock(_mutex);
  pause_request = *msg;
}

std::array<double, 2> SlotcarCommon::calculate_control_signals(
  const std::array<double, 2>& curr_velocities,
  const std::pair<double, double>& velocities,
  const double dt) const
{
  const double v_robot = curr_velocities[0];
  const double w_robot = curr_velocities[1];

  const double v_target = compute_ds(velocities.first, v_robot,
      _nominal_drive_speed,
      _nominal_drive_acceleration, _max_drive_acceleration, dt);

  const double w_target = compute_ds(velocities.second, w_robot,
      _nominal_turn_speed,
      _nominal_turn_acceleration, _max_turn_acceleration, dt);

  return std::array<double, 2>{v_target, w_target};
}

std::array<double, 2> SlotcarCommon::calculate_model_control_signals(
  const std::array<double, 2>& curr_velocities,
  const std::pair<double, double>& velocities,
  const double dt) const
{
  return calculate_control_signals(curr_velocities, velocities, dt);
}

std::array<double, 2> SlotcarCommon::calculate_joint_control_signals(
  const std::array<double, 2>& w_tire,
  const std::pair<double, double>& velocities,
  const double dt) const
{
  std::array<double, 2> curr_velocities;
  curr_velocities[0] = (w_tire[0] + w_tire[1]) * _tire_radius / 2.0;
  curr_velocities[1] = (w_tire[1] - w_tire[0]) * _tire_radius / _base_width;

  std::array<double, 2> new_velocities = calculate_control_signals(
    curr_velocities, velocities, dt);

  std::array<double, 2> joint_signals;
  for (std::size_t i = 0; i < 2; ++i)
  {
    const double yaw_sign = i == 0 ? -1.0 : 1.0;
    joint_signals[i] = (new_velocities[0] / _tire_radius) + (yaw_sign *
      new_velocities[1] * _base_width / (2.0 * _tire_radius));
  }
  return joint_signals;
}

std::string to_str(uint32_t type)
{
  if (rmf_fleet_msgs::msg::PauseRequest::TYPE_RESUME == type)
    return "resume";
  else if (rmf_fleet_msgs::msg::PauseRequest::TYPE_PAUSE_IMMEDIATELY == type)
    return "pause immediately";
  else if (rmf_fleet_msgs::msg::PauseRequest::TYPE_PAUSE_AT_CHECKPOINT == type)
    return "pause at checkpoint";

  return "UNKNOWN: " + std::to_string(type) + "??";
}

// First value of par is x_target, second is yaw_target
std::pair<double, double> SlotcarCommon::update(const Eigen::Isometry3d& pose,
  const std::vector<Eigen::Vector3d>& obstacle_positions,
  const double time)
{
  std::lock_guard<std::mutex> lock(_mutex);
  std::pair<double, double> velocities;
  const int32_t t_sec = static_cast<int32_t>(time);
  const uint32_t t_nsec =
    static_cast<uint32_t>((time-static_cast<double>(t_sec)) *1e9);
  const rclcpp::Time now{t_sec, t_nsec, RCL_ROS_TIME};
  double dt = time - _last_update_time;
  _last_update_time = time;

  _pose = pose;
  publish_robot_state(time);

  // Update battery state of charge
  if (_initialized_pose)
  {
    const Eigen::Vector3d dist = compute_dpos(_old_pose, _pose); // Ignore movement along z-axis
    const Eigen::Vector3d lin_vel = dist / dt;
    double ang_disp = compute_yaw(_pose, _old_pose, _rot_dir);
    const double ang_vel = ang_disp / dt;

    // Try charging battery
    double eps = 0.01;
    bool stationary = lin_vel.norm() < eps && std::abs(ang_vel) < eps;
    bool in_charger_vicinity = near_charger(_pose);
    if (stationary && in_charger_vicinity)
    {
      if (_enable_instant_charge)
      {
        _soc = _soc_max;
      }
      else if (_enable_charge)
      {
        _soc += compute_charge(dt);
        _soc = std::min(_soc_max, _soc);
      }
    }
    // Discharge battery
    if (_enable_drain)
    {
      const Eigen::Vector3d lin_acc = (lin_vel - _old_lin_vel) / dt;
      const double ang_acc = (ang_vel - _old_ang_vel) / dt;
      _soc -= compute_discharge(lin_vel, ang_vel, lin_acc, ang_acc, dt);
      _soc = std::max(0.0, _soc);
    }
    // Update mode
    if (stationary && in_charger_vicinity &&
      (_enable_instant_charge || _enable_charge))
    {
      _current_mode.mode = rmf_fleet_msgs::msg::RobotMode::MODE_CHARGING;
    }
    else if (_docking)
    {
      _current_mode.mode = rmf_fleet_msgs::msg::RobotMode::MODE_DOCKING;
    }
    else if (stationary)
    {
      _current_mode.mode = rmf_fleet_msgs::msg::RobotMode::MODE_IDLE;
    }
    else
    {
      _current_mode.mode = rmf_fleet_msgs::msg::RobotMode::MODE_MOVING;
    }
    _old_lin_vel = lin_vel;
    _old_ang_vel = ang_vel;
  }
  _old_pose = _pose;
  _initialized_pose = true;

  if (trajectory.empty())
    return velocities;

  Eigen::Vector3d current_heading = compute_heading(_pose);

  if (_traj_wp_idx < trajectory.size())
  {
    const Eigen::Vector3d dpos = compute_dpos(
      trajectory.at(_traj_wp_idx), _pose);

    if (_hold_times.size() != trajectory.size())
    {
      throw std::runtime_error(
              "Mismatch between trajectory size ["
              + std::to_string(trajectory.size()) + "] and holding time size ["
              + std::to_string(_hold_times.size()) + "]");
    }

    auto dpos_mag = dpos.norm();
    // TODO(MXG): Some kind of crazy nonsense bug is somehow altering the
    // clock type value for the _hold_times. I don't know where this could
    // possibly be happening, but I suspect it must be caused by undefined
    // behavior. For now we deal with this by explicitly setting the clock type.
    const auto hold_time =
      rclcpp::Time(_hold_times.at(_traj_wp_idx), RCL_ROS_TIME);

    const bool close_enough = (dpos_mag < 0.02);

    const bool checkpoint_pause =
      pause_request.type == pause_request.TYPE_PAUSE_AT_CHECKPOINT
      && pause_request.at_checkpoint <= _remaining_path.front().index;
    const bool immediate_pause =
      pause_request.type == pause_request.TYPE_PAUSE_IMMEDIATELY;
    const bool pause = checkpoint_pause || immediate_pause;

    const bool hold = now < hold_time;

    const bool rotate_towards_next_target = close_enough && (hold || pause);

    if (rotate_towards_next_target)
    {
      if (_traj_wp_idx+1 < trajectory.size())
      {
        const auto dpos_next =
          compute_dpos(trajectory.at(_traj_wp_idx+1), _pose);

        const auto goal_heading =
          compute_heading(trajectory.at(_traj_wp_idx+1));

        double dir = 1.0;
        velocities.second = compute_change_in_rotation(
          current_heading, dpos_next, &goal_heading, &dir);

        if (dir < 0.0)
          current_heading *= -1.0;
      }
      else
      {
        const auto goal_heading = compute_heading(trajectory.at(_traj_wp_idx));
        velocities.second = compute_change_in_rotation(
          current_heading, goal_heading);
      }

      _current_mode.mode = rmf_fleet_msgs::msg::RobotMode::MODE_PAUSED;
    }
    else if (close_enough)
    {
      _traj_wp_idx++;
      if (_remaining_path.empty())
        return velocities;

      _remaining_path.erase(_remaining_path.begin());
      RCLCPP_INFO(logger(),
        "%s reached waypoint %d/%d",
        _model_name.c_str(),
        _traj_wp_idx,
        (int)trajectory.size());
      if (_traj_wp_idx == trajectory.size())
      {
        RCLCPP_INFO(
          logger(),
          "%s reached goal -- rotating to face target",
          _model_name.c_str());
      }
    }

    if (!rotate_towards_next_target && _traj_wp_idx < trajectory.size())
    {
      const double d_yaw_tolerance = 5.0 * M_PI / 180.0;
      auto goal_heading = compute_heading(trajectory.at(_traj_wp_idx));
      double dir = 1.0;
      velocities.second =
        compute_change_in_rotation(current_heading, dpos, &goal_heading, &dir);
      if (dir < 0.0)
        current_heading *= -1.0;

      // If d_yaw is less than a certain tolerance (i.e. we don't need to spin
      // too much), then we'll include the forward velocity. Otherwise, we will
      // only spin in place until we are oriented in the desired direction.
      velocities.first = std::abs(velocities.second) <
        d_yaw_tolerance ? dir * dpos_mag : 0.0;
    }
  }
  else
  {
    const auto goal_heading = compute_heading(trajectory.back());
    velocities.second = compute_change_in_rotation(
      current_heading,
      goal_heading);

    // Put in a deadzone if yaw is small enough. This essentially locks the
    // tires. COMMENTED OUT as it breaks rotations for some reason...
    // if(std::abs(velocities.second) < std::max(0.1*M_PI/180.00, goal_yaw_tolerance))
    // {
    //   velocities.second = 0.0;
    // }

    velocities.first = 0.0;
  }

  const bool immediate_pause =
    pause_request.type == pause_request.TYPE_PAUSE_IMMEDIATELY;

  const bool stop =
    immediate_pause || emergency_stop(obstacle_positions, current_heading);

  if (immediate_pause)
  {
    // If we are required to immediately pause, report that we are in paused
    // mode
    _current_mode.mode = _current_mode.MODE_PAUSED;
  }
  else if (stop)
  {
    // If we are not required to pause but we are being forced to emergency stop
    // because of an obstacle, report that we are in waiting mode.
    _current_mode.mode = _current_mode.MODE_WAITING;
  }

  if (stop)
  {
    // Allow spinning but not translating
    velocities.first = 0.0;
  }

  _rot_dir = velocities.second >= 0 ? 1 : -1;
  return velocities;
}

bool SlotcarCommon::emergency_stop(
  const std::vector<Eigen::Vector3d>& obstacle_positions,
  const Eigen::Vector3d& current_heading)
{
  const Eigen::Vector3d stop_zone =
    _pose.translation() + _stop_distance * current_heading;

  bool need_to_stop = false;
  for (const auto& obstacle_pos : obstacle_positions)
  {
    const double dist = (obstacle_pos - stop_zone).norm();
    if (dist < _stop_radius)
    {
      need_to_stop = true;
      break;
    }
  }

  if (need_to_stop != _emergency_stop)
  {
    _emergency_stop = need_to_stop;
    // TODO flush logger here
    // TODO get collision object name
    if (need_to_stop)
      RCLCPP_INFO_STREAM(logger(), "Stopping [" << _model_name <<
          "] to avoid a collision");
    else
      RCLCPP_INFO_STREAM(logger(), "No more obstacles; resuming course for [" <<
          _model_name << "]");
  }

  return _emergency_stop;
}

std::string SlotcarCommon::get_level_name(const double z) const
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

double SlotcarCommon::compute_change_in_rotation(
  const Eigen::Vector3d& heading_vec,
  const Eigen::Vector3d& dpos,
  const Eigen::Vector3d* traj_vec,
  double* const dir) const
{
  if (dpos.norm() < 1e-3)
  {
    // We're right next to the waypoint, so we don't really need any heading
    // to reach it.
    return 0.0;
  }

  Eigen::Vector3d target = dpos;
  // If a traj_vec is provided, of the two possible headings (dpos/-dpos),
  // choose the one closest to traj_vec
  if (traj_vec)
  {
    const double dot = traj_vec->dot(dpos);
    target = dot < 0 ? -dpos : dpos;
    // dir is negative if slotcar will need to reverse to go towards target
    if (dir)
    {
      *dir = dot < 0 ? -1.0 : 1.0;
    }
  }

  const auto cross = heading_vec.cross(target);
  const double direction = cross(2) < 0.0 ? -1.0 : 1.0;
  const double denom = heading_vec.norm() * target.norm();
  const double d_yaw = direction * std::asin(cross.norm() / denom);

  return d_yaw;
}

void SlotcarCommon::publish_robot_state(const double time)
{
  const int32_t t_sec = static_cast<int32_t>(time);
  const uint32_t t_nsec =
    static_cast<uint32_t>((time-static_cast<double>(t_sec)) *1e9);
  const rclcpp::Time ros_time{t_sec, t_nsec, RCL_ROS_TIME};
  if ((time - last_tf2_pub) > (1.0 / TF2_RATE))
  {
    // Publish tf2
    publish_tf2(ros_time);
    last_tf2_pub = time;
  }
  if ((time - last_topic_pub) > (1.0 / STATE_TOPIC_RATE))
  {
    // Publish state topic
    publish_state_topic(ros_time);
    last_topic_pub = time;
  }
}

void SlotcarCommon::publish_tf2(const rclcpp::Time& t)
{
  geometry_msgs::msg::TransformStamped tf_stamped;
  Eigen::Quaterniond quat(_pose.linear());
  tf_stamped.header.stamp = t;
  tf_stamped.header.frame_id = "world";
  tf_stamped.child_frame_id = _model_name + "/base_link";
  tf_stamped.transform.translation.x = _pose.translation()[0];
  tf_stamped.transform.translation.y = _pose.translation()[1];
  tf_stamped.transform.translation.z = _pose.translation()[2];
  tf_stamped.transform.rotation.x = quat.x();
  tf_stamped.transform.rotation.y = quat.y();
  tf_stamped.transform.rotation.z = quat.z();
  tf_stamped.transform.rotation.w = quat.w();
  _tf2_broadcaster->sendTransform(tf_stamped);
}

void SlotcarCommon::publish_state_topic(const rclcpp::Time& t)
{
  rmf_fleet_msgs::msg::RobotState robot_state_msg;
  robot_state_msg.name = _model_name;
  robot_state_msg.battery_percent = 100 * _soc;

  robot_state_msg.location.x = _pose.translation()[0];
  robot_state_msg.location.y = _pose.translation()[1];
  robot_state_msg.location.yaw = compute_yaw(_pose);
  robot_state_msg.location.t = t;
  robot_state_msg.location.level_name = get_level_name(_pose.translation()[2]);

  robot_state_msg.task_id = _current_task_id;
  robot_state_msg.path = _remaining_path;
  robot_state_msg.mode = _current_mode;
  robot_state_msg.mode.mode_request_id = pause_request.mode_request_id;

  if (_adapter_error)
  {
    robot_state_msg.mode.mode =
      rmf_fleet_msgs::msg::RobotMode::MODE_ADAPTER_ERROR;
  }

  robot_state_msg.seq = ++_sequence;
  _robot_state_pub->publish(robot_state_msg);
}

void SlotcarCommon::mode_request_cb(
  const rmf_fleet_msgs::msg::ModeRequest::SharedPtr msg)
{
  // Request is for another robot
  if (msg->robot_name != _model_name)
    return;

  _current_mode = msg->mode;
  if (msg->mode.mode == msg->mode.MODE_DOCKING)
    _docking = true;
  else
    _docking = false;
}

void SlotcarCommon::map_cb(
  const building_map_msgs::msg::BuildingMap::SharedPtr msg)
{
  if (msg->levels.empty())
  {
    RCLCPP_ERROR(logger(), "Received empty building map");
    return;
  }

  for (const auto& level : msg->levels)
  {
    _level_to_elevation.insert({level.name, level.elevation});
  }
  _initialized_levels = true;

}

// Enables/disables charge when called from Ignition/Gazebo plugin
void SlotcarCommon::charge_state_cb(
  const std::string& name, bool selected)
{
  if (name == _enable_charge_str)
  {
    _enable_charge = selected;
  }
  else if (name == _enable_instant_charge_str)
  {
    _enable_instant_charge = selected;
  }
  else if (name == _enable_drain_str)
  {
    _enable_drain = selected;
  }
  else
  {
    std::cerr << "Invalid button selected. " << std::endl;
  }
}

bool SlotcarCommon::near_charger(const Eigen::Isometry3d& pose) const
{
  std::string lvl_name = get_level_name(pose.translation()[2]);
  auto waypoints_it = _charger_waypoints.find(lvl_name);
  if (waypoints_it != _charger_waypoints.end())
  {
    const std::vector<ChargerWaypoint>& waypoints = waypoints_it->second;
    for (const ChargerWaypoint& waypoint : waypoints)
    {
      // Assumes it is on the same Z-plane
      double dist =
        sqrt(pow(waypoint.x - pose.translation()[0], 2)
          + pow(waypoint.y - pose.translation()[1], 2));
      if (dist < _charger_dist_thres)
      {
        return true;
      }
    }
  }
  return false;
}

double SlotcarCommon::compute_charge(const double run_time) const
{
  const double dQ = _params.charging_current * run_time; // Coulombs
  const double dSOC = dQ / (_params.nominal_capacity * 3600.0);
  return dSOC;
}

double SlotcarCommon::compute_discharge(
  const Eigen::Vector3d lin_vel, const double ang_vel,
  const Eigen::Vector3d lin_acc, const double ang_acc,
  const double run_time) const
{
  const double v = std::min(lin_vel.norm(), _nominal_drive_speed);
  const double w = std::min(std::abs(ang_vel), _nominal_turn_speed);
  const double a = std::min(lin_acc.norm(), _max_drive_acceleration);
  const double alpha = std::min(std::abs(ang_acc), _max_turn_acceleration);

  // Loss through acceleration
  const double EA = ((_params.mass * a * v) +
    (_params.inertia * alpha * w)) * run_time;

  // Loss through friction. Ignores friction due to rotation in place
  const double EF = compute_friction_energy(_params.friction_coefficient,
      _params.mass, v, run_time);

  // Change in energy as a result of motion
  const double dEM = EA + EF;
  // Change in energy due to any onboard device over the time period `run_time`
  const double dED = _params.nominal_power * run_time;

  // The charge consumed
  const double dQ = (dEM + dED) / _params.nominal_voltage;
  // The depleted state of charge as a fraction in range [0,1]
  double dSOC = dQ / (_params.nominal_capacity * 3600.0);

  return dSOC;
}
