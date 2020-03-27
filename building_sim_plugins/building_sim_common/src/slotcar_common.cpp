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

static Eigen::Vector3d compute_heading(const Eigen::Isometry3d& pose)
{
  double yaw = compute_yaw(pose);
  return Eigen::Vector3d(std::cos(yaw), std::sin(yaw), 0.0);
}

static auto compute_dpos(const Eigen::Isometry3d& target,
  const Eigen::Isometry3d& actual)
{
  Eigen::Vector3d dpos(target.translation() - actual.translation());
  dpos(2) = 0.0;
  return dpos;
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

double SlotcarCommon::stop_distance() const
{
  return _stop_distance;
}

double SlotcarCommon::stop_radius() const
{
  return _stop_radius;
}

void SlotcarCommon::init_ros_node(const rclcpp::Node::SharedPtr node)
{
  _current_mode.mode = rmf_fleet_msgs::msg::RobotMode::MODE_MOVING;
  _ros_node = std::move(node);

  _tf2_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(_ros_node);

  _robot_state_pub =
    _ros_node->create_publisher<rmf_fleet_msgs::msg::RobotState>(
      "/robot_state", 10);

  _traj_sub = _ros_node->create_subscription<rmf_fleet_msgs::msg::PathRequest>(
    "/robot_path_requests",
    10,
    std::bind(&SlotcarCommon::path_request_cb, this, std::placeholders::_1));

  _mode_sub = _ros_node->create_subscription<rmf_fleet_msgs::msg::ModeRequest>(
    "/robot_mode_requests",
    10,
    std::bind(&SlotcarCommon::mode_request_cb, this, std::placeholders::_1));

}

void SlotcarCommon::path_request_cb(
  const rmf_fleet_msgs::msg::PathRequest::SharedPtr msg)
{
  // TODO refactor checking in another function?
  if (msg->robot_name != _model_name)
  {
    RCLCPP_INFO(
      logger(),
      "Ignoring path request for ["
      + msg->robot_name + "]");
    return;
  }

  if (msg->task_id == _current_task_id)
  {
    RCLCPP_INFO(
      logger(),
      "Already received task [" + _current_task_id
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
    trajectory[i].translation() = v3;
    trajectory[i].linear() = Eigen::Matrix3d(quat);

    _hold_times[i] = msg->path[i].t;

  }
  _remaining_path = msg->path;
  _traj_wp_idx = 0;

  // TODO seems unused?
  /*
  if (!msg->path.empty())
    start_time = msg->path.front().t;

  _goal_yaw_tolerance = 0.1; // TODO: Clarify this placeholder tolerance
  */

  _current_task_id = msg->task_id;

  return;
}

std::array<double, 2> SlotcarCommon::calculate_control_signals(
  const std::array<double, 2>& w_tire_actual,
  const double x_target,
  const double yaw_target,
  const double dt) const
{
  std::array<double, 2> joint_signals;
  const double v_actual = (w_tire_actual[0] + w_tire_actual[1]) * _tire_radius /
    2.0;
  const double w_actual = (w_tire_actual[1] - w_tire_actual[0]) * _tire_radius /
    _base_width;

  const double v_target = compute_ds(x_target, v_actual, _nominal_drive_speed,
    _nominal_drive_acceleration, _max_drive_acceleration, dt);

  const double w_target = compute_ds(yaw_target, w_actual, _nominal_turn_speed,
    _nominal_turn_acceleration, _max_turn_acceleration, dt);
  for (std::size_t i = 0; i < 2; ++i)
  {
    const double yaw_sign = i == 0 ? -1.0 : 1.0;
    joint_signals[i] = v_target / _tire_radius + yaw_sign * w_target *
      _base_width / (2.0 * _tire_radius);
  }
  return joint_signals;
}

// TODO refactor, return instead of change reference parameters
bool SlotcarCommon::update(const Eigen::Isometry3d& pose, const double time,
  double& x_target, double& yaw_target)
{
  const int32_t t_sec = static_cast<int32_t>(time);
  const uint32_t t_nsec =
    static_cast<uint32_t>((time-static_cast<double>(t_sec)) *1e9);
  const rclcpp::Time now{t_sec, t_nsec, RCL_ROS_TIME};
  _last_update_time = time;

  publish_robot_state(pose, time);

  if (trajectory.empty())
    return false;

  Eigen::Vector3d current_heading = compute_heading(pose);

  if ((unsigned int)_traj_wp_idx < trajectory.size())
  {
    const Eigen::Vector3d dpos = compute_dpos(
      trajectory[_traj_wp_idx], pose);

    auto dpos_mag = dpos.norm();
    const auto hold_time = _hold_times[_traj_wp_idx];

    const bool close_enough = (dpos_mag < 0.02);
    const bool rotate_towards_next_target = close_enough && (now < hold_time);

    if (rotate_towards_next_target)
    {
      auto goal_heading = compute_heading(trajectory[_traj_wp_idx]);

      yaw_target = compute_change_in_rotation(
        current_heading, goal_heading);
    }
    else if (close_enough)
    {
      _traj_wp_idx++;
      if (_remaining_path.empty())
        return false;

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

    if (!rotate_towards_next_target)
    {
      const double d_yaw_tolerance = 5.0 * M_PI / 180.0;

      double dir = 1.0;
      yaw_target = compute_change_in_rotation(current_heading, dpos, &dir);
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
    const auto goal_heading = compute_heading(trajectory.back());
    yaw_target = compute_change_in_rotation(
      current_heading,
      goal_heading);

    // Put in a deadzone if yaw is small enough. This essentially locks the
    // tires. COMMENTED OUT as it breaks rotations for some reason...
    // if(std::abs(yaw_target) < std::max(0.1*M_PI/180.00, goal_yaw_tolerance))
    // {
    //   yaw_target = 0.0;
    // }

    x_target = 0.0;
  }

  return true;
}

double SlotcarCommon::compute_change_in_rotation(
  Eigen::Vector3d heading_vec,
  const Eigen::Vector3d& dpos,
  double* permissive)
{
  if (dpos.norm() < 1e-3)
  {
    // We're right next to the waypoint, so we don't really need any heading
    // to reach it.
    return 0.0;
  }

  // Flip the heading vector if the dot product is less than zero. That way,
  // the robot will turn towards the heading that's closer.
  const double dot = heading_vec.dot(dpos);
  if (permissive && dot < 0.0)
  {
    heading_vec = -1.0 * heading_vec;
    *permissive = -1.0;
  }
  else if (permissive)
  {
    *permissive = 1.0;
  }

  const auto cross = heading_vec.cross(dpos);
  const double direction = cross(2) < 0.0 ? -1.0 : 1.0;
  const double denom = heading_vec.norm() * dpos.norm();
  const double d_yaw = direction * std::asin(cross.norm() / denom);

  return d_yaw;
}

void SlotcarCommon::publish_robot_state(const Eigen::Isometry3d& pose,
  const double time)
{
  const int32_t t_sec = static_cast<int32_t>(time);
  const uint32_t t_nsec =
    static_cast<uint32_t>((time-static_cast<double>(t_sec)) *1e9);
  const rclcpp::Time ros_time{t_sec, t_nsec, RCL_ROS_TIME};
  if ((time - last_tf2_pub) > (1.0 / TF2_RATE))
  {
    // Publish tf2
    publish_tf2(pose, ros_time);
    last_tf2_pub = time;
  }
  if ((time - last_topic_pub) > (1.0 / STATE_TOPIC_RATE))
  {
    // Publish state topic
    publish_state_topic(pose, ros_time);
    last_topic_pub = time;
  }
}

void SlotcarCommon::publish_tf2(const Eigen::Isometry3d& pose,
  const rclcpp::Time& t)
{
  geometry_msgs::msg::TransformStamped tf_stamped;
  Eigen::Quaterniond quat(pose.linear());
  tf_stamped.header.stamp = t;
  tf_stamped.header.frame_id = "world";
  tf_stamped.child_frame_id = _model_name + "/base_link";
  tf_stamped.transform.translation.x = pose.translation()[0];
  tf_stamped.transform.translation.y = pose.translation()[1];
  tf_stamped.transform.translation.z = pose.translation()[2];
  tf_stamped.transform.rotation.x = quat.x();
  tf_stamped.transform.rotation.y = quat.y();
  tf_stamped.transform.rotation.z = quat.z();
  tf_stamped.transform.rotation.w = quat.w();
  _tf2_broadcaster->sendTransform(tf_stamped);
}

void SlotcarCommon::publish_state_topic(const Eigen::Isometry3d& pose,
  const rclcpp::Time& t)
{
  rmf_fleet_msgs::msg::RobotState robot_state_msg;
  robot_state_msg.name = _model_name;

  robot_state_msg.location.x = pose.translation()[0];
  robot_state_msg.location.y = pose.translation()[1];
  robot_state_msg.location.yaw = compute_yaw(pose);
  robot_state_msg.location.t = t;

  robot_state_msg.task_id = _current_task_id;
  robot_state_msg.path = _remaining_path;
  robot_state_msg.mode = _current_mode;

  _robot_state_pub->publish(robot_state_msg);
}

void SlotcarCommon::mode_request_cb(
  const rmf_fleet_msgs::msg::ModeRequest::SharedPtr msg)
{
  _current_mode = msg->mode;
}
