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

#ifndef CROWD_SIM_IMPL__H
#define CROWD_SIM_IMPL__H

#include <string>
#include <set>
#include <memory>

#include <yaml-cpp/yaml.h>

#include <traffic_editor/crowd_sim/state.h>
#include <traffic_editor/crowd_sim/goal_set.h>
#include <traffic_editor/crowd_sim/transition.h>
#include <traffic_editor/crowd_sim/agent_profile.h>
#include <traffic_editor/crowd_sim/agent_group.h>
#include <traffic_editor/crowd_sim/model_type.h>

namespace crowd_sim {

class CrowdSimImplementation
{
public:
  CrowdSimImplementation()
  : _enable_crowd_sim(false),
    _update_time_step(0.1)
  {
    init_default_configure();
  }
  ~CrowdSimImplementation() {}


  YAML::Node to_yaml();
  bool from_yaml(const YAML::Node& input);
  void clear();
  void init_default_configure();

  void set_navmesh_file_name(std::vector<std::string> navmesh_filename)
  {
    _navmesh_filename_list = navmesh_filename;
  }
  std::vector<std::string> get_navmesh_file_name() const
  {
    return _navmesh_filename_list;
  }

  void set_enable_crowd_sim(bool is_enable) { _enable_crowd_sim = is_enable; }
  bool get_enable_crowd_sim() const { return _enable_crowd_sim; }

  void set_update_time_step(double update_time_step)
  {
    _update_time_step = update_time_step;
  }
  double get_update_time_step() const { return _update_time_step; }

  void set_goal_areas(std::set<std::string> goal_areas)
  {
    _goal_areas = goal_areas;
  }
  std::vector<std::string> get_goal_areas() const
  {
    return std::vector<std::string>(_goal_areas.begin(), _goal_areas.end());
  }

  void save_goal_sets(const std::vector<GoalSet>& goal_sets);
  std::vector<GoalSet> get_goal_sets() const { return _goal_sets; }

  void save_states(const std::vector<State>& states);
  std::vector<State> get_states() const { return _states; }

  void save_transitions(const std::vector<Transition>& transitions);
  std::vector<Transition> get_transitions() const { return _transitions; }

  void save_agent_profiles(const std::vector<AgentProfile>& agent_profiles);
  std::vector<AgentProfile> get_agent_profiles() const
  {
    return _agent_profiles;
  }

  void save_agent_groups(const std::vector<AgentGroup>& agent_groups);
  std::vector<AgentGroup> get_agent_groups() const { return _agent_groups; }

  void save_model_types(const std::vector<ModelType>& model_types);
  std::vector<ModelType> get_model_types() const { return _model_types; }

private:
  // update from project.building in crowd_sim_table
  std::set<std::string> _goal_areas;
  std::vector<std::string> _navmesh_filename_list;

  // real configurations
  bool _enable_crowd_sim;
  double _update_time_step;
  std::vector<State> _states;
  std::vector<GoalSet> _goal_sets;
  std::vector<Transition> _transitions;
  std::vector<AgentProfile> _agent_profiles;
  std::vector<AgentGroup> _agent_groups;
  std::vector<ModelType> _model_types;

  void _initialize_state();
  void _initialize_agent_profile();
  void _initialize_agent_group();
  void _initialize_model_type();

  YAML::Node _output_obstacle_node() const;
};

using CrowdSimImplPtr = std::shared_ptr<CrowdSimImplementation>;

} //namespace crowd_sim

#endif