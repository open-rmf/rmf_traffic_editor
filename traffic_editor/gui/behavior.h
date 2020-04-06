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

#include <memory>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "behavior_node.h"
#include "behavior_schedule_item.h"
#include "model_state.h"

class Behavior
{
public:
  Behavior();
  Behavior(const std::string& _name, const YAML::Node& yaml);
  Behavior(const Behavior& copy);
  ~Behavior();

  std::string name;

  std::vector<std::unique_ptr<BehaviorNode> > nodes;
  int active_node_idx = 0;

  void print() const;

  void tick(
      const double dt_seconds,
      ModelState& state,
      Building& building,
      const std::vector<std::unique_ptr<Model> >& active_models);

  // in the future, we'll probably have ways to parameterize this
  // for now it's basically a deep-copy
  std::unique_ptr<Behavior> instantiate() const;
};

#endif
