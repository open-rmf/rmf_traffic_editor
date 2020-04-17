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

#include "traffic_editor/behavior_node_wait.h"
using std::string;

BehaviorNodeWait::BehaviorNodeWait(const YAML::Node& y)
: BehaviorNode()
{
  seconds = y[1].as<double>();
}

BehaviorNodeWait::~BehaviorNodeWait()
{
}

void BehaviorNodeWait::print() const
{
  printf("      wait: %.3f\n", seconds);
}

std::unique_ptr<BehaviorNode> BehaviorNodeWait::instantiate(
    const YAML::Node& /*params*/,
    const std::string& _model_name) const
{
  auto n = std::make_unique<BehaviorNodeWait>(*this);
  // copy constructor already copied the 'seconds' member variable for us.
  n->model_name = _model_name;
  return n;
}

void BehaviorNodeWait::tick(
    const double dt_seconds,
    ModelState& /*state*/,
    Building& /*building*/,
    const std::vector<std::unique_ptr<Model> >& /*active_models*/,
    const std::vector<std::string>& /*inbound_signals*/,
    std::vector<std::string>& /*outbound_signals*/)
{
  elapsed_seconds += dt_seconds;
}

bool BehaviorNodeWait::is_complete() const
{
  return elapsed_seconds > seconds;
}
