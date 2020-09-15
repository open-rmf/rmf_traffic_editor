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

#ifndef CROWD_SIM_TRANSITION__H
#define CROWD_SIM_TRANSITION__H

#include <iostream>
#include <string>
#include <set>
#include <memory>

#include <yaml-cpp/yaml.h>

#include <traffic_editor/crowd_sim/condition.h>

namespace crowd_sim {

class Transition
{
public:
  using StateName = std::string;
  //store the to_state_name and relevant weight
  using ToStateType = std::map<StateName, double>;

  Transition(StateName from_state_name)
  : _from_state_name(from_state_name),
    _to_state_name({}),
    _condition(std::make_shared<Condition>())
  {}

  Transition(const YAML::Node& input)
  : _from_state_name("N.A."),
    _to_state_name({}),
    _condition(std::make_shared<Condition>())
  {
    from_yaml(input);
  }
  ~Transition() {}

  void set_from_state(StateName state_name)
  {
    this->_from_state_name = state_name;
  }
  std::string get_from_state() const { return this->_from_state_name; }

  void add_to_state(StateName state_name, double weight = 1.0)
  {
    this->_to_state_name.insert(std::make_pair(state_name, weight) );
  }
  void delete_to_state(StateName state_name)
  {
    this->_to_state_name.erase(state_name);
  }
  ToStateType get_to_state() const
  {
    return this->_to_state_name;
  }
  void clear_to_state()
  {
    this->_to_state_name.clear();
  }

  void set_condition(ConditionPtr condition) { this->_condition = condition; }
  ConditionPtr get_condition() const { return _condition; }

  bool is_valid()
  {
    if (_condition->is_valid() &&
      _to_state_name.size() > 0 &&
      _from_state_name.size() > 0)
      return true;
    std::cout << "Invalid transition" << std::endl;
    return false;
  }

  YAML::Node to_yaml() const;
  void from_yaml(const YAML::Node& input);

private:
  StateName _from_state_name;
  ToStateType _to_state_name;
  ConditionPtr _condition;
};

} //namespace crowd_sim

#endif