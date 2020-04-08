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

#ifndef BEHAVIOR_NODE_H
#define BEHAVIOR_NODE_H

#include <yaml-cpp/yaml.h>
#include <vector>

#include "model_state.h"
#include "planner_node.h"

class Building;
class Model;

class BehaviorNode
{
public:
  BehaviorNode();
  virtual ~BehaviorNode();

  virtual std::unique_ptr<BehaviorNode> clone() const = 0;

  virtual void print() const = 0;

  virtual void tick(
      const double dt_seconds,
      ModelState& state,
      Building& building,
      const std::vector<std::unique_ptr<Model> >& active_models) = 0;

  virtual bool is_complete() const = 0;

  bool populate_model_state_from_vertex_name(
      ModelState& state,
      const std::string vertex_name,
      Building& building);

  bool populate_planner_node_from_vertex_name(
      planner::Node& node,
      const std::string vertex_name,
      Building& building);

  double angle_difference(const double a, const double b) const;
  double angle_sum(const double a, const double b) const;
};

#endif
