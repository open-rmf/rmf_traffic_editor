#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>

#include <rmf_door_msgs/msg/door_mode.hpp>
#include <rmf_door_msgs/msg/door_state.hpp>
#include <rmf_door_msgs/msg/door_request.hpp>

#include "utils.hpp"

#include <mutex>
#include <vector>

namespace building_sim_common {

using DoorMode = rmf_door_msgs::msg::DoorMode;
using DoorState = rmf_door_msgs::msg::DoorState;
using DoorRequest = rmf_door_msgs::msg::DoorRequest;

//==============================================================================
class DoorCommon
{

public:

  template<typename SdfPtrT>
  static std::shared_ptr<DoorCommon> make(
    const std::string& door_name,
    rclcpp::Node::SharedPtr node,
    SdfPtrT& sdf);

  rclcpp::Logger logger() const;

  // Set the name of the door
  DoorState& door_name(const std::string door_name);
  // Get the name of the door
  std::string door_name() const;

  // Set the current mode of the door
  DoorState& current_mode(const DoorMode door_name);
  // Get the current mode of the door
  DoorMode current_mode() const;

  // Get the requested mode of the door
  DoorMode requested_mode() const;

  std::string left_door_joint_name() const;
  std::string right_door_joint_name() const;

  void publish_state(const uint32_t door_value, const rclcpp::Time time);

  bool is_initialized() const;

  MotionParams& params();

private:

  DoorCommon(const std::string& door_name,
    rclcpp::Node::SharedPtr node,
    const MotionParams& params,
    const std::string& left_door_joint_name,
    const std::string& right_door_joint_name);

  rclcpp::Node::SharedPtr _ros_node;
  rclcpp::Publisher<DoorState>::SharedPtr _door_state_pub;
  rclcpp::Subscription<DoorRequest>::SharedPtr _door_request_sub;

  DoorState _state;
  DoorRequest _request;

  MotionParams _params;

  double _last_update_time;
  double _last_pub_time;

  bool _initialized = false;
  bool _all_doors_open = false;
  bool _add_doors_close = true;

  std::string _left_door_joint_name;
  std::string _right_door_joint_name;

  std::mutex _mutex;
};

template<typename SdfPtrT>
std::shared_ptr<DoorCommon> DoorCommon::make(
  const std::string& door_name,
  rclcpp::Node::SharedPtr node,
  SdfPtrT& sdf)
{
  MotionParams params;
  get_sdf_param_if_available<double>(sdf, "v_max_door", params.v_max);
  get_sdf_param_if_available<double>(sdf, "a_max_door", params.a_max);
  get_sdf_param_if_available<double>(sdf, "a_nom_door", params.a_nom);
  get_sdf_param_if_available<double>(sdf, "dx_min_door", params.dx_min);
  get_sdf_param_if_available<double>(sdf, "f_max_door", params.f_max);

  SdfPtrT door_element;
  std::string left_door_joint_name;
  std::string right_door_joint_name;
  std::string door_type;

  if (!get_element_required(sdf, "door", door_element) ||
    !get_sdf_attribute_required<std::string>(
      door_element, "left_joint_name", left_door_joint_name) ||
    !get_sdf_attribute_required<std::string>(
      door_element, "right_joint_name", right_door_joint_name) ||
    !get_sdf_attribute_required<std::string>(
      door_element, "type", door_type))
  {
    RCLCPP_ERROR(
      node->get_logger(),
      " -- Missing required parameters for [%s] plugin",
      door_name.c_str());
    return nullptr;
  }

  if (left_door_joint_name == "empty_joint" &&
    right_door_joint_name == "empty_joint")
  {
    RCLCPP_ERROR(
      node->get_logger(),
      " -- Both door joint names are missing for [%s] plugin, at least one"
      " is required", door_name.c_str());
    return nullptr;
  }

  std::shared_ptr<DoorCommon> door_common(new DoorCommon(door_name,
    node, params, left_door_joint_name, right_door_joint_name));

  return door_common;

}

} // namespace building_sim_common
