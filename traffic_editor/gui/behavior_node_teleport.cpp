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

#include "behavior_node_teleport.h"
#include "building.h"
using std::string;
using std::make_unique;

BehaviorNodeTeleport::BehaviorNodeTeleport(const YAML::Node& y)
: BehaviorNode()
{
  destination_name = y[1].as<string>();
  if (y.size() > 2)
    model_to_teleport = y[2].as<string>();
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

std::unique_ptr<BehaviorNode> BehaviorNodeTeleport::instantiate(
    const YAML::Node& params,
    const std::string& _model_name) const
{
  auto b = make_unique<BehaviorNodeTeleport>(*this);
  b->destination_name = interpolate_string_params(destination_name, params);
  b->model_name = _model_name;
  return b;
}

void BehaviorNodeTeleport::tick(
    const double /*dt_seconds*/,
    ModelState& state,
    Building& building,
    const std::vector<std::unique_ptr<Model>>& active_models,
    const std::vector<std::string>& /*inbound_signals*/,
    std::vector<std::string>& /*outbound_signals*/)
{
  if (model_to_teleport == model_name)
  {
    populate_model_state_from_vertex_name(
        destination_state,
        destination_name,
        building);
    state = destination_state;
  }
  else
  {
    // try to find the model we're looking for in the active_models vector
    for (auto& active_model : active_models)
      if (active_model->instance_name == model_to_teleport)
      {
        active_model->state = destination_state;
        break;
      }
  }
}

bool BehaviorNodeTeleport::is_complete() const
{
  return true;
}
