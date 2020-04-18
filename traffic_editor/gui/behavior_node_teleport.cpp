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

#include "traffic_editor/behavior_node_teleport.h"
#include "traffic_editor/building.h"

using std::string;
using std::make_unique;


BehaviorNodeTeleport::BehaviorNodeTeleport(const YAML::Node& y)
: BehaviorNode()
{
  destination_name = y[1].as<string>();

  if (y.size() > 2)
    destination_yaw_str = y[2].as<string>();

  if (y.size() > 3)
    model_to_teleport = y[3].as<string>();
  else
    model_to_teleport = model_name;
}

BehaviorNodeTeleport::~BehaviorNodeTeleport()
{
}

void BehaviorNodeTeleport::print() const
{
  printf("      teleport: [%s]\n", destination_name.c_str());
}

void BehaviorNodeTeleport::tick(
    const double /*dt_seconds*/,
    ModelState& state,
    Building& building,
    const std::vector<std::string>& /*inbound_messages*/,
    std::vector<std::string>& /*outbound_messages*/)
{
  populate_model_state_from_vertex_name(
      destination_state,
      destination_name,
      building);
  destination_state.yaw = destination_yaw;

  if (model_to_teleport.empty())
    state = destination_state;  // teleport ourselves
  else
  {
    printf(
        "teleporting [%s] to [%s]\n",
        model_to_teleport.c_str(),
        destination_name.c_str());

    // dig around to find the model we're looking for
    for (auto& level : building.levels)
      for (auto& model : level->models)
        if (model->instance_name == model_to_teleport)
        {
          model->state = destination_state;
          return;
        }
  }
}

bool BehaviorNodeTeleport::is_complete() const
{
  return true;
}
