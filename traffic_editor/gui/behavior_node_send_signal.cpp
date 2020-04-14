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

#include "behavior_node_send_signal.h"

using std::make_unique;
using std::string;


BehaviorNodeSendSignal::BehaviorNodeSendSignal(const YAML::Node& y)
: BehaviorNode()
{
  signal_name = y[1].as<string>();
}

BehaviorNodeSendSignal::~BehaviorNodeSendSignal()
{
}

void BehaviorNodeSendSignal::print() const
{
  printf("      send_signal: %s\n", signal_name.c_str());
}

std::unique_ptr<BehaviorNode> BehaviorNodeSendSignal::instantiate(
    const YAML::Node& params,
    const std::string& _model_name) const
{
  auto b = make_unique<BehaviorNodeSendSignal>(*this);
  b->signal_name = interpolate_string_params(signal_name, params);
  b->model_name = _model_name;
  return b;
}

void BehaviorNodeSendSignal::tick(
    const double /*dt_seconds*/,
    ModelState& /*state*/,
    Building& /*building*/,
    const std::vector<std::unique_ptr<Model>>& /*active_models*/,
    const std::vector<std::string>& /*inbound_signals*/,
    std::vector<std::string>& outbound_signals)
{
  if (!signal_name.empty())
    outbound_signals.push_back(signal_name);
}

bool BehaviorNodeSendSignal::is_complete() const
{
  return true;
}
