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

#include "traffic_editor/behavior_node_send_messages.h"

using std::make_unique;
using std::string;
using std::vector;


BehaviorNodeSendMessages::BehaviorNodeSendMessages(
    const vector<string>& _messages)
: BehaviorNode()
{
  messages = _messages;
}

void BehaviorNodeSendMessages::print() const
{
  printf("      send_messages: FOO, BAR, BAZ  TODO FIXME\n");
}

void BehaviorNodeSendMessages::tick(
    const double /*dt_seconds*/,
    ModelState& /*state*/,
    Building& /*building*/,
    const std::vector<std::string>& /*inbound_messages*/,
    std::vector<std::string>& outbound_messages)
{
  for (const auto& message : messages)
    outbound_messages.push_back(message);
}

bool BehaviorNodeSendMessages::is_complete() const
{
  return true;
}
