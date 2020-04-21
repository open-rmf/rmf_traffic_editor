#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>

#include <rmf_door_msgs/msg/door_mode.hpp>
#include <rmf_door_msgs/msg/door_state.hpp>
#include <rmf_door_msgs/msg/door_request.hpp>

#include "utils.hpp"

#include <vector>
#include <unordered_map>

namespace building_sim_common {

using DoorMode = rmf_door_msgs::msg::DoorMode;
using DoorState = rmf_door_msgs::msg::DoorState;
using DoorRequest = rmf_door_msgs::msg::DoorRequest;

//==============================================================================
class DoorCommon
{

public:

  struct DoorUpdateRequest
  {
    std::string joint_name;
    double position;
    double velocity;
  };

  struct DoorUpdateResult
  {
    std::string joint_name;
    double velocity;
    double fmax;
  };

  template<typename SdfPtrT>
  static std::shared_ptr<DoorCommon> make(
    const std::string& door_name,
    rclcpp::Node::SharedPtr node,
    SdfPtrT& sdf);

  rclcpp::Logger logger() const;

  std::vector<std::string> joint_names() const;

  bool is_initialized() const;

  MotionParams& params();

  std::vector<DoorUpdateResult> update(const double time,
    const std::vector<DoorUpdateRequest>& request);

private:

  struct DoorElement
  {
    double closed_position;
    double open_position;
    double current_position;
    double current_velocity;

    DoorElement(
      const double lower_limit,
      const double upper_limit,
      const bool flip_direction = false)
    {
      if (flip_direction)
      {
        closed_position = lower_limit;
        open_position = upper_limit;
      }
      else
      {
        closed_position = upper_limit;
        open_position = lower_limit;
      }

      current_position = 0.0;
      current_velocity = 0.0;
    }
  };

  DoorMode requested_mode() const;

  void publish_state(const uint32_t door_value, const rclcpp::Time& time);

  double calculate_target_velocity(
    const double target,
    const double current_position,
    const double current_velocity,
    const double dt);

  DoorCommon(const std::string& door_name,
    rclcpp::Node::SharedPtr node,
    const MotionParams& params,
    const std::string& left_door_joint_name,
    const std::string& right_door_joint_name,
    const std::array<double, 2>& left_joint_limits,
    const std::array<double, 2>& right_joint_limits);

  bool all_doors_open();

  bool all_doors_closed();

  rclcpp::Node::SharedPtr _ros_node;
  rclcpp::Publisher<DoorState>::SharedPtr _door_state_pub;
  rclcpp::Subscription<DoorRequest>::SharedPtr _door_request_sub;

  DoorState _state;
  DoorRequest _request;

  MotionParams _params;

  double _last_update_time = 0.0;
  double _last_pub_time = 0.0;

  bool _initialized = false;

  // Map of joint_name and corresponding DoorElement
  std::unordered_map<std::string, std::shared_ptr<DoorElement>> _doors;
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

    // Get the joint names and door type
  if (!get_element_required(sdf, "door", door_element) ||
    !get_sdf_attribute_required<std::string>(
      door_element, "left_joint_name", left_door_joint_name) ||
    !get_sdf_attribute_required<std::string>(
      door_element, "right_joint_name", right_door_joint_name) ||
    !get_sdf_attribute_required<std::string>(
      door_element, "type", door_type))
  {
    RCLCPP_ERROR(node->get_logger(),
      " -- Missing required parameters for [%s] plugin",
      door_name.c_str());
    return nullptr;
  }

  if ((left_door_joint_name == "empty_joint" &&
      right_door_joint_name == "empty_joint") ||
      (left_door_joint_name.empty() && right_door_joint_name.empty()))
  {
    RCLCPP_ERROR(node->get_logger(),
      " -- Both door joint names are missing for [%s] plugin, at least one"
      " is required", door_name.c_str());
    return nullptr;
  }

    // Default values for joint limts
  double left_joint_lower_limit = -1.57;
  double left_joint_upper_limit = 0.0;
  double right_joint_lower_limit = 0.0;
  double right_joint_upper_limit = 1.57;

  auto extract_limits = [&](const SdfPtrT& joint_sdf)
  {
    std::string joint_name;
    get_sdf_attribute_required<std::string>(
      joint_sdf, "name", joint_name);
    SdfPtrT element;
    get_element_required(joint_sdf, "axis", element);
    get_element_required(element, "limit", element);
    if (joint_name == left_door_joint_name)
    {
      get_sdf_param_if_available<double>(element, "lower", left_joint_lower_limit);
      get_sdf_param_if_available<double>(element, "upper", left_joint_upper_limit);
      RCLCPP_INFO(node->get_logger(),
        "Joint [%s] lower [%f] upper [%f]",
        joint_name.c_str(),left_joint_lower_limit, left_joint_upper_limit);
    }
    else if (joint_name == right_door_joint_name)
    {
      get_sdf_param_if_available<double>(element, "lower", right_joint_lower_limit);
      get_sdf_param_if_available<double>(element, "upper", right_joint_upper_limit);
      RCLCPP_INFO(node->get_logger(),
        "Joint [%s] lower [%f] upper [%f]",
        joint_name.c_str(),left_joint_lower_limit, left_joint_upper_limit);
    }
  };

  // Get the joint limits from parent sdf
  auto parent = sdf->GetParent();

  // TODO Commented out as GetParent is returning nullptr with sdf used in ign
  // The default values will be used for ign sim
  // if (!parent)
  // {
  //   RCLCPP_ERROR(node->get_logger(),
  //     "Unable to access parent sdf to retrieve joint limits");
  //   return nullptr;
  // }
  if (parent)
  {
    auto joint_element = parent->GetElement("joint");
    if (!joint_element)
    {
      RCLCPP_ERROR(node->get_logger(),
        "Parent sdf missing required joint element");
      return nullptr;
    }

    extract_limits(joint_element);
    // Find next joint element if present
    joint_element = joint_element->GetNextElement("joint");
    if (joint_element)
      extract_limits(joint_element);
  }

  std::array<double,2> left_joint_limits ={
    left_joint_lower_limit, left_joint_upper_limit};
  std::array<double,2> right_joint_limits ={
    right_joint_lower_limit, right_joint_upper_limit};

  std::shared_ptr<DoorCommon> door_common(new DoorCommon(
      door_name,
      node,
      params,
      left_door_joint_name,
      right_door_joint_name,
      left_joint_limits,
      right_joint_limits));

  return door_common;

}

} // namespace building_sim_common
