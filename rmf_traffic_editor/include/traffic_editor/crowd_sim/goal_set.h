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

#ifndef CROWD_SIM_GOAL_SET__H
#define CROWD_SIM_GOAL_SET__H

#include <string>
#include <set>
#include <memory>

#include <yaml-cpp/yaml.h>

namespace crowd_sim {

class GoalSet
{
public:
  GoalSet(size_t goal_id)
  : _id(goal_id),
    _capacity(1),
    _goal_area_contained({})
  {}

  GoalSet(const YAML::Node& input)
  : _id(65535), //initialize with invalid id
    _capacity(1),
    _goal_area_contained({})
  {
    from_yaml(input);
  }

  void add_goal_area(std::string goal_area_name);
  void set_capacity(size_t capacity) { this->_capacity = capacity; }

  std::set<std::string> get_goal_areas() const
  {
    return this->_goal_area_contained;
  }
  YAML::Node get_goal_areas_to_yaml() const;
  size_t get_goal_set_id() const { return this->_id; }
  size_t get_capacity() const {return this->_capacity; }

  YAML::Node to_yaml() const;
  void from_yaml(const YAML::Node& input);

private:
  size_t _id;
  size_t _capacity;
  std::set<std::string> _goal_area_contained;

  void _set_goal_set_id(size_t id_) { this->_id = id_; }
};

using GoalSetPtr = std::shared_ptr<GoalSet>;
} //namespace crowd_sim

#endif