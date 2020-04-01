#include <memory>

#include <building_sim_common/utils.hpp>
#include <building_sim_common/door_common.hpp>

using namespace std::chrono_literals;

namespace building_sim_common {

rclcpp::Logger DoorCommon::logger() const
{
  return rclcpp::get_logger("door_" + _state.door_name);
}

DoorState& DoorCommon::door_name(const std::string door_name)
{
  std::unique_lock<std::mutex> lock(_mutex);
  _state.door_name = door_name;
  return _state;
}

std::string DoorCommon::door_name() const
{
  return _state.door_name;
}


DoorState& DoorCommon::current_mode(const DoorMode door_mode)
{
  std::unique_lock<std::mutex> lock(_mutex);
  _state.current_mode = door_mode;
  return _state;
}

DoorMode DoorCommon::current_mode() const
{
  return _state.current_mode;
}

DoorMode DoorCommon::requested_mode() const
{
  return _request.requested_mode;
}

std::string DoorCommon::left_door_joint_name() const
{
  return _left_door_joint_name;
}

std::string DoorCommon::right_door_joint_name() const
{
  return _right_door_joint_name;
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
  const rclcpp::Time time)
{
  if (!_initialized)
    return;

  std::unique_lock<std::mutex> lock(_mutex);
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
  _params(params),
  _left_door_joint_name(std::move(left_door_joint_name)),
  _right_door_joint_name(std::move(right_door_joint_name))
{
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
      std::unique_lock<std::mutex> lock(_mutex);
      if (msg->door_name == _state.door_name)
        _request = *msg;
    });

  _initialized = true;
}


} // namespace building_sim_common
