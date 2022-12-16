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

#include <traffic_editor/crowd_sim/crowd_sim_impl.h>

using namespace crowd_sim;

//=================================================
void CrowdSimImplementation::_initialize_state()
{
  if (_states.size() == 0)
    _states.emplace_back("external_static");
  else
    _states[0] = State("external_static");
}

//=================================================
void CrowdSimImplementation::_initialize_agent_profile()
{
  if (_agent_profiles.size() == 0)
    _agent_profiles.emplace_back("external_agent");
  else
    _agent_profiles[0] = AgentProfile("external_agent");
}

//=================================================
void CrowdSimImplementation::_initialize_agent_group()
{
  if (_agent_groups.size() == 0)
    _agent_groups.emplace_back(0, true);
  else
    _agent_groups[0] = AgentGroup(0, true);
  _agent_groups[0].set_agent_profile("external_agent");
  _agent_groups[0].set_initial_state("external_static");
}

//=================================================
void CrowdSimImplementation::_initialize_model_type()
{
  if (_model_types.size() == 0)
    _model_types.emplace_back("human", "walk");
  else
    _model_types[0] = ModelType("human", "walk");
  auto& default_type = _model_types.at(0);
  default_type.set_animation_speed(1.0);
  default_type.set_model_uri("model://MaleVisitorPhone");
  default_type.set_init_pose(
    {0, 0, 0, 0, 0, 0}
  );
}

//=================================================
YAML::Node CrowdSimImplementation::_output_obstacle_node() const
{
  // Currently only have one level navmesh
  YAML::Node obstacle_node = YAML::Node(YAML::NodeType::Map);
  obstacle_node.SetStyle(YAML::EmitterStyle::Flow);
  obstacle_node["class"] = 1;
  obstacle_node["type"] = "nav_mesh";
  if (!_navmesh_filename_list.empty())
    obstacle_node["file_name"] = this->_navmesh_filename_list[0];
  return obstacle_node;
}

//=================================================
YAML::Node CrowdSimImplementation::to_yaml()
{
  YAML::Node top_node = YAML::Node(YAML::NodeType::Map);
  top_node["enable"] = _enable_crowd_sim ? 1 : 0;
  top_node["update_time_step"] = _update_time_step;

  top_node["states"] = YAML::Node(YAML::NodeType::Sequence);
  for (auto state : _states)
  {
    if (!state.is_valid())
      continue;
    top_node["states"].push_back(state.to_yaml());
  }

  top_node["goal_sets"] = YAML::Node(YAML::NodeType::Sequence);
  for (auto goal_set : _goal_sets)
  {
    top_node["goal_sets"].push_back(goal_set.to_yaml());
  }
  if (!_goal_sets.size())
    top_node["goal_sets"].SetStyle(YAML::EmitterStyle::Flow);

  top_node["agent_profiles"] = YAML::Node(YAML::NodeType::Sequence);
  for (auto profile : _agent_profiles)
  {
    top_node["agent_profiles"].push_back(profile.to_yaml());
  }

  top_node["transitions"] = YAML::Node(YAML::NodeType::Sequence);
  for (auto transition : _transitions)
  {
    top_node["transitions"].push_back(transition.to_yaml());
  }
  if (!_transitions.size())
    top_node["transitions"].SetStyle(YAML::EmitterStyle::Flow);

  top_node["obstacle_set"] = _output_obstacle_node();

  top_node["agent_groups"] = YAML::Node(YAML::NodeType::Sequence);
  for (auto group : _agent_groups)
  {
    top_node["agent_groups"].push_back(group.to_yaml());
  }

  top_node["model_types"] = YAML::Node(YAML::NodeType::Sequence);
  for (auto model_type : _model_types)
  {
    top_node["model_types"].push_back(model_type.to_yaml());
  }
  if (!_model_types.size())
    top_node["model_types"].SetStyle(YAML::EmitterStyle::Flow);

  return top_node;
}

//=================================================
bool CrowdSimImplementation::from_yaml(const YAML::Node& input)
{
  if (!input["goal_sets"] || !input["goal_sets"].IsSequence())
  {
    printf("Error in load goal_sets\n");
    return false;
  }
  if (!input["states"] || !input["states"].IsSequence())
  {
    printf("Error in load states\n");
    return false;
  }
  if (!input["transitions"] || !input["transitions"].IsSequence())
  {
    printf("Error in load transitions\n");
    return false;
  }
  if (!input["agent_profiles"] || !input["agent_profiles"].IsSequence())
  {
    printf("Error in load agent_profiles\n");
    return false;
  }
  if (!input["agent_groups"] || !input["agent_groups"].IsSequence())
  {
    printf("Error in load agent_groups\n");
    return false;
  }
  if (!input["model_types"] || !input["model_types"].IsSequence())
  {
    printf("Error in load model_types\n");
    return false;
  }

  clear();

  if (input["update_time_step"] && input["update_time_step"].as<double>() > 0)
  {
    this->_update_time_step = input["update_time_step"].as<double>();
  }
  if (input["enable"] && input["enable"].as<int>() == 1)
  {
    this->_enable_crowd_sim = true;
  }
  else
  {
    // default disable crowd_simulation
    this->_enable_crowd_sim = false;
  }

  const YAML::Node& goal_set_node = input["goal_sets"];
  for (YAML::const_iterator it = goal_set_node.begin();
    it != goal_set_node.end(); it++)
  {
    GoalSet goalset_temp(*it);
    this->_goal_sets.emplace_back(goalset_temp);
  }
  printf("crowd_sim loaded %lu goal_sets\n", this->_goal_sets.size());

  const YAML::Node& state_node = input["states"];
  for (YAML::const_iterator it = state_node.begin();
    it != state_node.end();
    it++)
  {
    State state_temp(*it);
    this->_states.emplace_back(state_temp);
  }
  printf("crowd_sim loaded %lu states\n", this->_states.size());

  const YAML::Node& transition_node = input["transitions"];
  for (YAML::const_iterator it = transition_node.begin();
    it != transition_node.end(); it++)
  {
    Transition transition_temp(*it);
    this->_transitions.emplace_back(transition_temp);
  }
  printf("crowd_sim loaded %lu transitions\n", this->_transitions.size());

  const YAML::Node& agent_profile_node = input["agent_profiles"];
  for (YAML::const_iterator it = agent_profile_node.begin();
    it != agent_profile_node.end(); it++)
  {
    AgentProfile agent_profile_temp(*it);
    this->_agent_profiles.emplace_back(agent_profile_temp);
  }
  printf("crowd_sim loaded %lu agent_profiles\n", this->_agent_profiles.size());

  const YAML::Node& agent_group_node = input["agent_groups"];
  for (YAML::const_iterator it = agent_group_node.begin();
    it != agent_group_node.end(); it++)
  {
    AgentGroup agent_group_temp(*it);
    this->_agent_groups.emplace_back(agent_group_temp);
  }
  printf("crowd_sim loaded %lu agent_groups\n", this->_agent_profiles.size());

  const YAML::Node& model_type_node = input["model_types"];
  for (YAML::const_iterator it = model_type_node.begin();
    it != model_type_node.end(); it++)
  {
    ModelType model_type_temp(*it);
    this->_model_types.emplace_back(model_type_temp);
  }
  printf("crowd_sim loaded %lu model_types\n", this->_agent_profiles.size());

  init_default_configure();
  return true;
}

//=================================================
void CrowdSimImplementation::clear()
{
  _goal_areas.clear();
  _navmesh_filename_list.clear();

  _enable_crowd_sim = false;
  _update_time_step = 0.1;
  _states.clear();
  _goal_sets.clear();
  _transitions.clear();
  _agent_profiles.clear();
  _agent_groups.clear();
  _model_types.clear();
}

//=================================================
void CrowdSimImplementation::init_default_configure()
{
  _initialize_state();
  _initialize_agent_profile();
  _initialize_agent_group();
}

//=================================================
void CrowdSimImplementation::save_goal_sets(
  const std::vector<GoalSet>& goal_sets)
{
  _goal_sets.clear();
  _goal_sets = goal_sets;
}

//===================================================
void CrowdSimImplementation::save_states(const std::vector<State>& states)
{
  _states.clear();
  _states = states;
  _initialize_state();
}

//===================================================
void CrowdSimImplementation::save_transitions(
  const std::vector<Transition>& transitions)
{
  _transitions.clear();
  _transitions = transitions;
}

//=================================================
void CrowdSimImplementation::save_agent_profiles(
  const std::vector<AgentProfile>& agent_profiles)
{
  _agent_profiles.clear();
  _agent_profiles = agent_profiles;
  _initialize_agent_profile();
}

//=================================================
void CrowdSimImplementation::save_agent_groups(
  const std::vector<AgentGroup>& agent_groups)
{
  _agent_groups.clear();
  _agent_groups = agent_groups;
}

//=================================================
void CrowdSimImplementation::save_model_types(
  const std::vector<ModelType>& model_types)
{
  _model_types.clear();
  _model_types = model_types;
}
