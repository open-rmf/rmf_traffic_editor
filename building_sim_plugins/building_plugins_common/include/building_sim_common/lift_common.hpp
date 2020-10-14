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

#ifndef BUILDING_SIM_COMMON__LIFT_COMMON_HPP
#define BUILDING_SIM_COMMON__LIFT_COMMON_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>

#include <rmf_lift_msgs/msg/lift_state.hpp>
#include <rmf_lift_msgs/msg/lift_request.hpp>
#include <rmf_door_msgs/msg/door_mode.hpp>
#include <rmf_door_msgs/msg/door_state.hpp>
#include <rmf_door_msgs/msg/door_request.hpp>

#include "utils.hpp"

#include <vector>
#include <utility>
#include <unordered_map>

namespace building_sim_common {

using LiftState = rmf_lift_msgs::msg::LiftState;
using LiftRequest = rmf_lift_msgs::msg::LiftRequest;
using DoorRequest = rmf_door_msgs::msg::DoorRequest;
using DoorState = rmf_door_msgs::msg::DoorState;
using DoorMode = rmf_door_msgs::msg::DoorMode;

//==============================================================================
class LiftCommon
{

public:

  struct LiftUpdateResult
  {
    double velocity;
    double fmax;
  };

  template<typename SdfPtrT>
  static std::unique_ptr<LiftCommon> make(
    const std::string& lift_name,
    rclcpp::Node::SharedPtr node,
    SdfPtrT& sdf);

  rclcpp::Logger logger() const;

  LiftUpdateResult update(const double time, const double position,
    const double velocity);

  std::string get_joint_name() const;

  double get_elevation() const;

  bool motion_state_changed();

private:

  rclcpp::Node::SharedPtr _ros_node;
  rclcpp::Publisher<LiftState>::SharedPtr _lift_state_pub;
  rclcpp::Publisher<DoorRequest>::SharedPtr _door_request_pub;
  rclcpp::Subscription<LiftRequest>::SharedPtr _lift_request_sub;
  rclcpp::Subscription<DoorState>::SharedPtr _door_state_sub;

  std::string _lift_name;
  std::string _cabin_joint_name;

  MotionParams _cabin_motion_params;
  LiftState::_motion_state_type _old_motion_state;

  std::vector<std::string> _floor_names;
  std::unordered_map<std::string, double> _floor_name_to_elevation;
  std::unordered_map<std::string,
    std::vector<std::string>> _floor_name_to_shaft_door_name;
  std::unordered_map<std::string,
    std::vector<std::string>> _floor_name_to_cabin_door_name;
  std::unordered_map<std::string, DoorState::SharedPtr> _shaft_door_states;
  std::unordered_map<std::string, DoorState::SharedPtr> _cabin_door_states;

  LiftState _lift_state;
  LiftRequest::UniquePtr _lift_request;

  double _last_update_time = 0.0;
  // random start time offset to prevent state message crossfire
  double _last_pub_time = ((double) std::rand()) / ((double) (RAND_MAX));

  void publish_door_request(const double time, std::string door_name,
    uint32_t door_state);

  LiftCommon(rclcpp::Node::SharedPtr node,
    const std::string& lift_name,
    const std::string& joint_name,
    const MotionParams& cabin_motion_params,
    const std::vector<std::string>& floor_names,
    const std::unordered_map<std::string, double>& floor_name_to_elevation,
    std::unordered_map<
      std::string, std::vector<std::string>> floor_name_to_shaft_door_name,
    std::unordered_map<
      std::string, std::vector<std::string>> floor_name_to_cabin_door_name,
    std::unordered_map<std::string, DoorState::SharedPtr> shaft_door_states,
    std::unordered_map<std::string, DoorState::SharedPtr> cabin_door_states,
    std::string initial_floor_name);

  double get_step_velocity(const double dt, const double position,
    const double velocity);

  void update_cabin_state(const double position, const double velocity);

  void move_doors(const double time, uint32_t door_mode);

  void open_doors(const double time);

  void close_doors(const double time);

  uint32_t get_door_state(
    const std::unordered_map<std::string,
    std::vector<std::string>>& floor_to_door_map,
    const std::unordered_map<std::string, DoorState::SharedPtr>& door_states);

  void pub_lift_state(const double time);

  void update_lift_door_state();

};

template<typename SdfPtrT>
std::unique_ptr<LiftCommon> LiftCommon::make(
  const std::string& lift_name,
  rclcpp::Node::SharedPtr node,
  SdfPtrT& sdf)
{
  MotionParams cabin_motion_params;
  std::string joint_name;
  std::vector<std::string> floor_names;
  std::unordered_map<std::string, double> floor_name_to_elevation;
  std::unordered_map<std::string,
    std::vector<std::string>> floor_name_to_shaft_door_name;
  std::unordered_map<std::string,
    std::vector<std::string>> floor_name_to_cabin_door_name;
  std::unordered_map<std::string, DoorState::SharedPtr> shaft_door_states;
  std::unordered_map<std::string, DoorState::SharedPtr> cabin_door_states;


  auto sdf_clone = sdf->Clone();

  // load lift cabin motion parameters
  get_sdf_param_if_available<double>(sdf_clone, "v_max_cabin",
    cabin_motion_params.v_max);
  get_sdf_param_if_available<double>(sdf_clone, "a_max_cabin",
    cabin_motion_params.a_max);
  get_sdf_param_if_available<double>(sdf_clone, "a_nom_cabin",
    cabin_motion_params.a_nom);
  get_sdf_param_if_available<double>(sdf_clone, "dx_min_cabin",
    cabin_motion_params.dx_min);
  get_sdf_param_if_available<double>(sdf_clone, "f_max_cabin",
    cabin_motion_params.f_max);
  if (!get_sdf_param_required(sdf_clone, "cabin_joint_name",
    joint_name))
    return nullptr;

  // load the floor name and elevation for each floor
  auto floor_element = sdf_clone;
  if (!get_element_required(sdf, "floor", floor_element))
  {
    RCLCPP_ERROR(node->get_logger(),
      " -- Missing required floor element for [%s] plugin",
      lift_name.c_str());
    return nullptr;
  }

  while (floor_element)
  {
    std::string floor_name;
    double floor_elevation;
    if (!get_sdf_attribute_required<std::string>(floor_element, "name",
      floor_name) ||
      !get_sdf_attribute_required<double>(floor_element, "elevation",
      floor_elevation))
    {
      RCLCPP_ERROR(
        node->get_logger(),
        " -- Missing required floor name or elevation attributes for [%s] plugin",
        lift_name.c_str());
      return nullptr;
    }
    floor_names.push_back(floor_name);
    floor_name_to_elevation.insert({floor_name, floor_elevation});

    auto door_pair_element = floor_element;
    if (get_element_required(floor_element, "door_pair", door_pair_element))
    {
      while (door_pair_element)
      {
        std::string shaft_door_name;
        std::string cabin_door_name;
        if (!get_sdf_attribute_required<std::string>(door_pair_element,
          "cabin_door", cabin_door_name) ||
          !get_sdf_attribute_required<std::string>(door_pair_element,
          "shaft_door", shaft_door_name))
        {
          RCLCPP_ERROR(node->get_logger(),
            " -- Missing required lift door attributes for [%s] plugin",
            lift_name.c_str());
          return nullptr;
        }
        floor_name_to_cabin_door_name[floor_name].push_back(cabin_door_name);
        floor_name_to_shaft_door_name[floor_name].push_back(shaft_door_name);
        shaft_door_states.insert({shaft_door_name, nullptr});
        cabin_door_states.insert({cabin_door_name, nullptr});

        door_pair_element = door_pair_element->GetNextElement("door_pair");
      }
    }
    floor_element = floor_element->GetNextElement("floor");
  }

  assert(!floor_names.empty());
  std::string initial_floor_name = floor_names[0];
  get_sdf_param_if_available<std::string>(sdf_clone, "initial_floor",
    initial_floor_name);

  if (std::find(floor_names.begin(), floor_names.end(), initial_floor_name) ==
    floor_names.end())
  {
    RCLCPP_WARN(
      node->get_logger(),
      "Initial floor [%s] is not available, changing to deafult",
      initial_floor_name.c_str());
    initial_floor_name = floor_names[0];
  }

  std::unique_ptr<LiftCommon> lift(new LiftCommon(
      node,
      lift_name,
      joint_name,
      cabin_motion_params,
      floor_names,
      floor_name_to_elevation,
      floor_name_to_shaft_door_name,
      floor_name_to_cabin_door_name,
      shaft_door_states,
      cabin_door_states,
      initial_floor_name));

  return lift;
}

} // namespace building_sim_common

#endif // BUILDING_SIM_COMMON__LIFT_COMMON_HPP
