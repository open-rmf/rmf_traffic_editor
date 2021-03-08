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
YAML::Node AgentGroup::to_yaml(YAML::Node& node) const
{
  node.SetStyle(YAML::EmitterStyle::Flow);
  node["profile_selector"] = _agent_profile;
  node["state_selector"] = _initial_state;
  node["x"] = _spawn_point_x;
  node["y"] = _spawn_point_y;
  return node;
}

//==============================================
YAML::Node AgentGroup::external_to_yaml() const
{
  YAML::Node group_node = YAML::Node(YAML::NodeType::Map);
  group_node["agents_number"] = _spawn_number;
  group_node["agents_name"] = YAML::Node(YAML::NodeType::Sequence);
  group_node["group_id"] = _group_id;
  for (auto name : _external_agent_names)
  {
    group_node["agents_name"].push_back(name);
  }
  return to_yaml(group_node);
}

//==============================================
std::vector<YAML::Node> AgentGroup::internal_to_yaml(int group_id) const
{
  std::vector<YAML::Node> nodes;
  //e.g. {agent_group_id: 1, model_name: OpenRobotics/MaleVisitorPhone, name: MaleVisitorPhone, profile_selector: human, state_selector: entry_lane, static: false, x: 1554.184, y: 434.535, yaw: 0, z: 0}
  for (size_t i = 0; i < _spawn_number; i++)
  {
    if (!is_human_valid())
      break;
    YAML::Node node = YAML::Node(YAML::NodeType::Map);
    node["agent_group_id"] = group_id; // use generated group_id, not the original _group_id to avoid skipped group id in case we have malformed agent groups and we decided to skip them
    node["model_name"] = _internal_agent_model_name;
    std::size_t slash_loc = _internal_agent_model_name.find("/");
    node["name"] = _internal_agent_model_name.substr(slash_loc+1);
    node["static"] = false;
    node["yaw"] = 0;
    node["z"] = 0;
    nodes.push_back(to_yaml(node));
  }
  return nodes;
}

//==============================================
void AgentGroup::external_from_yaml(const YAML::Node& input)
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
    _external_agent_names.emplace_back( (*it).as<std::string>() );
  }
}

//==============================================
void AgentGroup::internal_from_yaml(const YAML::Node& input)
{
  _group_id = input["agent_group_id"].as<size_t>();
  _spawn_number = 1;
  _spawn_point_x = input["x"].as<double>();
  _spawn_point_y = input["y"].as<double>();
  _agent_profile = input["profile_selector"].as<std::string>();
  _initial_state = input["state_selector"].as<std::string>();
  _internal_agent_model_name = input["model_name"].as<std::string>();
}