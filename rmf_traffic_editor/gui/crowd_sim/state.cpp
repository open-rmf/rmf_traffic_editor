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
#include <traffic_editor/crowd_sim/state.h>

using namespace crowd_sim;

//==============================================
bool State::is_valid() const
{
  if (_is_final_state && _name.size() > 0)
    return true;
  if (_name.size() > 0 && _navmesh_file_name.size() > 0 && _goal_set_id >= 0)
    return true;
  return false;
}

//==============================================
YAML::Node State::to_yaml() const
{
  YAML::Node state_node(YAML::NodeType::Map);
  state_node.SetStyle(YAML::EmitterStyle::Flow);
  state_node["name"] = get_name();
  state_node["goal_set"] = get_goal_set_id();
  state_node["navmesh_file_name"] = get_navmesh_file_name();
  state_node["final"] = get_final_state() ? 1 : 0;
  return state_node;
}

//==============================================
void State::from_yaml(const YAML::Node& input)
{
  set_name(input["name"].as<std::string>());
  set_navmesh_file_name(input["navmesh_file_name"].as<std::string>());
  set_final_state(input["final"].as<int>() == 0 ? false : true);
  set_goal_set_id(input["goal_set"].as<int>());
}