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

#include <traffic_editor/crowd_sim/goal_set.h>

#include <iostream>

using namespace crowd_sim;

//==============================================
void GoalSet::add_goal_area(std::string area_name)
{
  if (area_name.empty())
  {
    std::cout << "Invalid area_name provided." << std::endl;
  }
  this->_goal_area_contained.insert(area_name);
}

//==============================================
YAML::Node GoalSet::get_goal_areas_to_yaml() const
{
  YAML::Node goal_area = YAML::Node(YAML::NodeType::Sequence);
  goal_area.SetStyle(YAML::EmitterStyle::Flow);
  for (auto area : get_goal_areas())
  {
    goal_area.push_back(area);
  }
  return goal_area;
}

//==============================================
YAML::Node GoalSet::to_yaml() const
{
  YAML::Node goalset_node(YAML::NodeType::Map);
  goalset_node.SetStyle(YAML::EmitterStyle::Flow);
  goalset_node["set_id"] = get_goal_set_id();
  goalset_node["capacity"] = get_capacity();
  goalset_node["set_area"] = get_goal_areas_to_yaml();
  return goalset_node;
}

//==============================================
void GoalSet::from_yaml(const YAML::Node& input)
{
  _set_goal_set_id(input["set_id"].as<size_t>());
  set_capacity(input["capacity"].as<size_t>());
  _goal_area_contained.clear();
  for (auto area : input["set_area"])
  {
    add_goal_area(area.as<std::string>());
  }
}