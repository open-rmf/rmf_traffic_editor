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

#include <traffic_editor/crowd_sim/agent_group.h>

using namespace crowd_sim;

//==============================================
YAML::Node AgentGroup::to_yaml() const
{
  YAML::Node group_node = YAML::Node(YAML::NodeType::Map);
  group_node.SetStyle(YAML::EmitterStyle::Flow);
  group_node["group_id"] = _group_id;
  group_node["profile_selector"] = _agent_profile;
  group_node["state_selector"] = _initial_state;
  group_node["agents_number"] = _spawn_number;
  group_node["agents_name"] = YAML::Node(YAML::NodeType::Sequence);
  for (auto name : _external_agent_name)
  {
    group_node["agents_name"].push_back(name);
  }
  group_node["x"] = _spawn_point_x;
  group_node["y"] = _spawn_point_y;
  return group_node;
}

//==============================================
void AgentGroup::from_yaml(const YAML::Node& input)
{
  _group_id = input["group_id"].as<size_t>();
  _spawn_point_x = input["x"].as<double>();
  _spawn_point_y = input["y"].as<double>();
  _spawn_number = input["agents_number"].as<int>();
  _agent_profile = input["profile_selector"].as<std::string>();
  _initial_state = input["state_selector"].as<std::string>();
  const YAML::Node& agent_name_node = input["agents_name"];
  for (YAML::const_iterator it = agent_name_node.begin();
    it != agent_name_node.end(); it++)
  {
    _external_agent_name.emplace_back( (*it).as<std::string>() );
  }
  if (_external_agent_name.size() > 0)
  {
    _is_external_group = true;
  }
}