/*
 * Copyright (C) 2019-2020 Open Source Robotics Foundation
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

#ifndef BEHAVIOR_H
#define BEHAVIOR_H

#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "behavior_node.h"
#include "behavior_schedule_item.h"

class Behavior
{
public:
  Behavior();
  ~Behavior();

  std::string name;

  std::vector<BehaviorNode> nodes;

  bool from_yaml(const std::string &_name, const YAML::Node& yaml_node);

  void print() const;
};

#endif
