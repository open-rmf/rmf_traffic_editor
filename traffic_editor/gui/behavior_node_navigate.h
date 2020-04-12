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

#ifndef BEHAVIOR_NODE_NAVIGATE_H
#define BEHAVIOR_NODE_NAVIGATE_H

#include <memory>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "behavior_node.h"
#include "model_state.h"
#include "planner_edge.h"

class BehaviorNodeNavigate : public BehaviorNode
{
public:
  std::string destination_name;
  bool destination_found = false;
  ModelState destination_state;
  std::vector<std::shared_ptr<planner::Node>> path;
  int nav_graph_idx;
  std::shared_ptr<planner::Node> previous_node;
  planner::Edge previous_edge;
  bool is_first_motion = true;

  enum class ControllerState
  {
    NAVIGATING,
    AWAITING_LANE
  };
  ControllerState controller_state = ControllerState::AWAITING_LANE;

  BehaviorNodeNavigate(const YAML::Node& yaml_node);
  ~BehaviorNodeNavigate();

  virtual std::unique_ptr<BehaviorNode> instantiate(
      const YAML::Node& params,
      const std::string& model_name) const override;

  virtual void print() const override;

  virtual void tick(
      const double dt_seconds,
      ModelState& model_state,
      Building& building,
      const std::vector<std::unique_ptr<Model> >& active_models,
      const std::vector<std::string>& inbound_signals,
      std::vector<std::string>& outbound_signals
  ) override;

  bool is_complete() const override;

private:
  double prev_error = 1e100;
};

#endif
