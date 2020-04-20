#include <memory>

#include <building_sim_common/utils.hpp>
#include <building_sim_common/door_common.hpp>

using namespace std::chrono_literals;

namespace building_sim_common {

rclcpp::Logger DoorCommon::logger() const
{
  return rclcpp::get_logger("door_" + _state.door_name);
}

DoorMode DoorCommon::requested_mode() const
{
  return _request.requested_mode;
}

std::string DoorCommon::left_door_joint_name() const
{
  return _left_door.first;
}

std::string DoorCommon::right_door_joint_name() const
{
  return _right_door.first;
}

MotionParams& DoorCommon::params()
{
  return _params;
}

bool DoorCommon::is_initialized() const
{
  return _initialized;
}
void DoorCommon::publish_state(const uint32_t door_value,
  const rclcpp::Time& time)
{
  if (!_initialized)
    return;

  _state.current_mode.value = door_value;
  _state.door_time = time;
  _door_state_pub->publish(_state);
}


DoorCommon::DoorCommon(const std::string& door_name,
  rclcpp::Node::SharedPtr node,
  const MotionParams& params,
  const std::string& left_door_joint_name,
  const std::string& right_door_joint_name)
: _ros_node(std::move(node)),
  _params(params)
{

  _right_door.first = std::move(right_door_joint_name);
  _right_door.second = nullptr;
  _left_door.first = std::move(left_door_joint_name);
  _left_door.second = nullptr;

  if (left_door_joint_name != "empty_joint")
    _doors.insert(std::make_pair(left_door_joint_name,
      nullptr));
  
  if (right_door_joint_name != "empty_joint")
    _doors.insert(std::make_pair(left_door_joint_name,
      nullptr));

  _state.door_name = door_name;
  _request.requested_mode.value = DoorMode::MODE_CLOSED;

  // TODO(YV) Consider putting these statements into a start() function
  // that can be called after checking if joint names exist in the model
  _door_state_pub = _ros_node->create_publisher<DoorState>(
    "/door_states", rclcpp::SystemDefaultsQoS());

  _door_request_sub = _ros_node->create_subscription<DoorRequest>(
    "/door_requests", rclcpp::SystemDefaultsQoS(),
    [&](DoorRequest::UniquePtr msg)
    {
      if (msg->door_name == _state.door_name)
        _request = *msg;
    });

  _initialized = true;
}

void DoorCommon::add_left_door(
  const double upper_limit,
  const double lower_limit)
{
  _left_door.second = std::make_shared<DoorCommon::DoorElement>(
      upper_limit, lower_limit);
}

void DoorCommon::add_right_door(
  const double upper_limit,
  const double lower_limit)
{
  _right_door.second = std::make_shared<DoorCommon::DoorElement>(
      upper_limit, lower_limit, true);
}

bool DoorCommon::all_doors_open()
{
  bool result = true;

  if (_left_door.second)
    result &= std::abs(_left_door.second->open_position
      - _left_door.second->current_position) <= _params.dx_min;

  if (_right_door.second)
    result &= std::abs(_right_door.second->open_position
      - _right_door.second->current_position) <= _params.dx_min;

  return result;
}

bool DoorCommon::all_doors_closed()
{
  bool result = true;

  if (_left_door.second)
    result &= std::abs(_left_door.second->closed_position
      - _left_door.second->current_position) <= _params.dx_min;
  if (_right_door.second)
    result &= std::abs(_right_door.second->closed_position
      - _right_door.second->current_position) <= _params.dx_min;
  
  return result;
}

double DoorCommon::calculate_target_velocity(
  const double target,
  const double current_position,
  const double current_velocity,
  const double dt)
{
  double dx = target - current_position;
  if (std::abs(dx) < _params.dx_min/2.0)
    dx = 0.0;

  double door_v = compute_desired_rate_of_change(
    dx, current_velocity, _params, dt);
  
  return door_v;
}

DoorCommon::DoorUpdateResult DoorCommon::update(
  const double time, const DoorCommon::DoorUpdateRequest& request)
{
  double dt = time - _last_update_time;
  _last_update_time = time;

  // Update state current position and velocity of each door
  if (_left_door.second)
  {
    if (request.left_position)
      _left_door.second->current_position = *request.left_position;
    if (request.left_velocity)
      _left_door.second->current_velocity = *request.left_velocity;
  }

  if (_right_door.second)
  {
    if (request.right_position)
      _right_door.second->current_position = *request.right_position;
    if (request.right_velocity)
      _right_door.second->current_velocity = *request.right_velocity;
  }

  // Calculate velocity to apply to door joints

  DoorCommon::DoorUpdateResult result;
  result.fmax = std::make_shared<double>(_params.f_max);
  if (requested_mode().value == DoorMode::MODE_OPEN)
  {
    if (_left_door.second)
    {
      result.left_velocity = std::make_shared<double>(
        calculate_target_velocity(
          _left_door.second->open_position,
          _left_door.second->current_position,
          _left_door.second->current_velocity,
          dt));
    }
    if (_right_door.second)
    {
      result.right_velocity = std::make_shared<double>(
        calculate_target_velocity(
          _right_door.second->open_position,
          _right_door.second->current_position,
          _right_door.second->current_velocity,
          dt));
    }
  }
  else
  {
    if (_left_door.second)
    {
      result.left_velocity = std::make_shared<double>(
        calculate_target_velocity(
          _left_door.second->closed_position,
          _left_door.second->current_position,
          _left_door.second->current_velocity,
          dt));
    }
    if (_right_door.second)
    {
      result.right_velocity = std::make_shared<double>(
        calculate_target_velocity(
          _right_door.second->closed_position,
          _right_door.second->current_position,
          _right_door.second->current_velocity,
          dt));
    }
  }
  
  // Publishing door states
  if (time - _last_pub_time >= 1.0)
  {
    _last_pub_time = time;
    const int32_t t_sec = static_cast<int32_t>(time);
    const uint32_t t_nsec =
      static_cast<uint32_t>((time-static_cast<double>(t_sec)) *1e9);
    const rclcpp::Time now{t_sec, t_nsec, RCL_ROS_TIME};

    if (all_doors_open())
    {
      publish_state(DoorMode::MODE_OPEN, now);
    }
    else if (all_doors_closed())
    {
      publish_state(DoorMode::MODE_CLOSED, now);
    }
    else
    {
      publish_state(DoorMode::MODE_MOVING, now);
    }
  }

  return result;
}


} // namespace building_sim_common
