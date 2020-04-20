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
    std::shared_ptr<double> left_position;
    std::shared_ptr<double> left_velocity;
    std::shared_ptr<double> right_position;
    std::shared_ptr<double> right_velocity;

    DoorUpdateRequest()
    {
      left_position = nullptr;
      left_velocity = nullptr;
      right_position = nullptr;
      right_velocity = nullptr;
    }
    ~DoorUpdateRequest()
    {}
  };

  struct DoorUpdateResult
  {
    std::shared_ptr<double> left_velocity;
    std::shared_ptr<double> right_velocity;
    std::shared_ptr<double> fmax;
    
    DoorUpdateResult()
    {
      left_velocity = nullptr;
      right_velocity = nullptr;
      fmax = nullptr;
    }
    ~DoorUpdateResult()
    {}
  };

  template<typename SdfPtrT>
  static std::shared_ptr<DoorCommon> make(
    const std::string& door_name,
    rclcpp::Node::SharedPtr node,
    SdfPtrT& sdf);

  rclcpp::Logger logger() const;

  std::string left_door_joint_name() const;
  std::string right_door_joint_name() const;

  void publish_state(const uint32_t door_value, const rclcpp::Time& time);

  bool is_initialized() const;

  MotionParams& params();

  void add_left_door(
    const double upper_limit,
    const double lower_limit);

  void add_right_door(
    const double upper_limit,
    const double lower_limit);

  DoorUpdateResult update(const double time,
    const DoorUpdateRequest& request);

private:

  struct DoorElement
  {
    double closed_position;
    double open_position;
    double current_position;
    double current_velocity;

    DoorElement(
      const double upper_limit,
      const double lower_limit,
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

  using Door = std::pair<std::string, std::shared_ptr<DoorElement>>;

    // Get the requested mode of the door
  DoorMode requested_mode() const;

  double calculate_target_velocity(
    const double target,
    const double current_position,
    const double current_velocity,
    const double dt);

  DoorCommon(const std::string& door_name,
    rclcpp::Node::SharedPtr node,
    const MotionParams& params,
    const std::string& left_door_joint_name,
    const std::string& right_door_joint_name);

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
  
  Door _left_door;
  Door _right_door;
  // Map of joint_name and DoorElement
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

  // TODO get joint limits from  door_element
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
