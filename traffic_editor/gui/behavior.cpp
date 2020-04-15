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

#include "behavior.h"
#include "behavior_node_teleport.h"
#include "behavior_node_wait.h"
#include "behavior_node_navigate.h"
#include "behavior_node_send_signal.h"
#include "behavior_node_await_signal.h"
#include "behavior_node_await_signals.h"
#include "building.h"

using std::string;
using std::unique_ptr;
using std::make_unique;

Behavior::Behavior()
{
}

Behavior::Behavior(const string &_name, const YAML::Node& yaml)
: name(_name)
{
  // printf("Behavior::from_yaml(%s)\n", name.c_str());

  if (!yaml["sequence"].IsSequence())
  {
    printf("ERROR: expected a map key 'sequence' !\n");
    return;
  }
  const YAML::Node &ys = yaml["sequence"];  // save some typing
  for (YAML::const_iterator it = ys.begin(); it != ys.end(); ++it)
  {
    if (!((*it).IsSequence()))
    {
      printf("ERROR: expected a sequence of YAML sequences...\n");
      return;
    }
    string type_name = (*it)[0].as<string>();
    // printf("node type name: [%s]\n", type_name.c_str());

    // i'm sure there is some hyper-elite C++ way to avoid this "if" tree...
    if (type_name == "teleport")
      nodes.push_back(make_unique<BehaviorNodeTeleport>(*it));
    else if (type_name == "wait")
      nodes.push_back(make_unique<BehaviorNodeWait>(*it));
    else if (type_name == "navigate")
      nodes.push_back(make_unique<BehaviorNodeNavigate>(*it));
    else if (type_name == "send_signal")
      nodes.push_back(make_unique<BehaviorNodeSendSignal>(*it));
    else if (type_name == "await_signal")
      nodes.push_back(make_unique<BehaviorNodeAwaitSignal>(*it));
    else if (type_name == "await_signals")
      nodes.push_back(make_unique<BehaviorNodeAwaitSignals>(*it));
    else
    {
      printf(
          "ERROR: unhandled behavior node type: [%s]\n",
          type_name.c_str());
    }
  }
}

Behavior::~Behavior()
{
}

void Behavior::print() const
{
  printf("    %s:\n", name.c_str());
  for (const auto& node : nodes)
    node->print();
}

void Behavior::tick(
      const double dt_seconds,
      ModelState &state,
      Building& building,
      const std::vector<std::unique_ptr<Model> >& active_models,
      const std::vector<std::string>& inbound_signals,
      std::vector<std::string>& outbound_signals)
{
  // printf("Behavior::tick() in behavior [%s]\n", name.c_str());
  if (active_node_idx >= static_cast<int>(nodes.size()))
    return;  // behavior is complete

  nodes[active_node_idx]->tick(
      dt_seconds,
      state,
      building,
      active_models,
      inbound_signals,
      outbound_signals);

  if (nodes[active_node_idx]->is_complete())
  {
    active_node_idx++;
    // if this is the last node in this behavior,
    // we can release all lane reservations
    if (active_node_idx >= static_cast<int>(nodes.size()))
      building.release_all_lane_edges_for_model(model_name);
  }
}

std::unique_ptr<Behavior> Behavior::instantiate(
    const YAML::Node& params,
    const std::string& _model_name) const
{
  std::unique_ptr<Behavior> instantiated = std::make_unique<Behavior>();
  instantiated->name = name;
  instantiated->model_name = _model_name;
  for (const auto& node : nodes)
    instantiated->nodes.push_back(node->instantiate(params, _model_name));
  return instantiated;
}

bool Behavior::is_completed()
{
  return active_node_idx >= static_cast<int>(nodes.size());
}
