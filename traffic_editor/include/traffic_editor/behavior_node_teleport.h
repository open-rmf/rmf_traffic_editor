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

#ifndef BEHAVIOR_NODE_TELEPORT_H
#define BEHAVIOR_NODE_TELEPORT_H

#include <string>
#include <yaml-cpp/yaml.h>
#include "behavior_node.h"

class BehaviorNodeTeleport : public BehaviorNode
{
public:
  std::string model_to_teleport;
  std::string destination_name;
  bool destination_found = false;
  ModelState destination_state;

  std::string destination_yaw_str;
  double destination_yaw = 0.0;

  BehaviorNodeTeleport(const YAML::Node& y);
  ~BehaviorNodeTeleport();

  virtual void print() const override;

  virtual void tick(
      const double dt_seconds,
      ModelState& state,
      Building& building,
      const std::vector<std::string>& inbound_messages,
      std::vector<std::string>& outbound_messages
  ) override;

  bool is_complete() const override;
};

#endif
