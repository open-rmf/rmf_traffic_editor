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
using std::string;

Behavior::Behavior()
{
}

Behavior::~Behavior()
{
}

bool Behavior::from_yaml(const string &_name, const YAML::Node& y)
{
  name = _name;
  printf("Behavior::from_yaml(%s)\n", name.c_str());

  nodes.clear();

  if (!y["sequence"].IsSequence())
  {
    printf("ERROR: expected a map key 'sequence' !\n");
    return false;
  }
  const YAML::Node &ys = y["sequence"];  // save some typing
  for (YAML::const_iterator it = ys.begin(); it != ys.end(); ++it)
  {
    if (!((*it).IsSequence()))
    {
      printf("ERROR: expected a sequence of YAML sequences...\n");
      return false;
    }
    string type_name = (*it)[0].as<string>();
    printf("node type name: [%s]\n", type_name.c_str());

    // i'm sure there is some hyper-elite C++ way to avoid this "if" tree...
    if (type_name == "teleport")
    {
      BehaviorNodeTeleport node;
      if (node.from_yaml(*it))
        nodes.push_back(node);
    }
    else if (type_name == "wait")
    {
      BehaviorNodeWait node;
      if (node.from_yaml(*it))
        nodes.push_back(node);
    }
    else if (type_name == "navigate")
    {
      BehaviorNodeNavigate node;
      if (node.from_yaml(*it))
        nodes.push_back(node);
    }
    else
    {
      printf(
          "ERROR: unhandled behavior node type: [%s]\n",
          type_name.c_str());
    }
  }
  return true;
}

void Behavior::print() const
{
  printf("    %s:\n", name.c_str());
  for (const auto& node : nodes)
    node.print();
}
