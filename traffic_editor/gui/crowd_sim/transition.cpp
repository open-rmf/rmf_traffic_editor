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

#include <traffic_editor/crowd_sim/transition.h>

using namespace crowd_sim;

//==============================================
YAML::Node Transition::to_yaml() const
{
  YAML::Node transition_node(YAML::NodeType::Map);
  transition_node.SetStyle(YAML::EmitterStyle::Flow);
  transition_node["from"] = _from_state_name;
  transition_node["to"] =
    _to_state_name.size() == 1 ? _to_state_name.begin()->first : "";
  transition_node["Condition"] = _condition->to_yaml();
  transition_node["Target"] = YAML::Node(YAML::NodeType::Sequence);
  for (auto to_state : _to_state_name)
  {
    YAML::Node target_node = YAML::Node(YAML::NodeType::Map);
    target_node["name"] = to_state.first;
    target_node["weight"] = to_state.second;
    transition_node["Target"].push_back(target_node);
  }
  return transition_node;
}

//==============================================
void Transition::from_yaml(const YAML::Node& input)
{
  set_from_state(input["from"].as<std::string>());
  if (input["to"] && input["to"].as<std::string>() != "")
  {
    add_to_state(input["to"].as<std::string>());
  }
  const YAML::Node& targets = input["Target"];
  for (YAML::const_iterator it = targets.begin(); it != targets.end(); it++)
  {
    if (!it->IsMap())
    {
      throw std::runtime_error("Single Target expects a map");
    }
    add_to_state((*it)["name"].as<std::string>(),
      (*it)["weight"].as<double>() );
  }

  //set condition from yaml
  ConditionPtr condition_ptr = std::make_shared<Condition>()->init_from_yaml(
    input["Condition"]);
  condition_ptr->from_yaml(input["Condition"]);
  set_condition(condition_ptr);
}