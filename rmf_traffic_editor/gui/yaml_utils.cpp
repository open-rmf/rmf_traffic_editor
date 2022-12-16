/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include <algorithm>

#include "yaml_utils.h"

#include <string>
using std::string;


// Recursive function to write YAML ordered maps. Credit: Dave Hershberger
// posted to this GitHub issue: https://github.com/jbeder/yaml-cpp/issues/169
void yaml_utils::write_node(
  const YAML::Node& node,
  YAML::Emitter& emitter)
{
  switch (node.Style())
  {
    case YAML::EmitterStyle::Block:
      emitter << YAML::Block;
      break;
    case YAML::EmitterStyle::Flow:
      emitter << YAML::Flow;
      break;
    default:
      break;
  }

  switch (node.Type())
  {
    case YAML::NodeType::Sequence:
    {
      emitter << YAML::BeginSeq;
      for (std::size_t i = 0; i < node.size(); i++)
        write_node(node[i], emitter);
      emitter << YAML::EndSeq;
      break;
    }
    case YAML::NodeType::Map:
    {
      emitter << YAML::BeginMap;
      // the keys are stored in random order, so we need to collect and sort
      std::vector<string> keys;
      keys.reserve(node.size());
      for (YAML::const_iterator it = node.begin(); it != node.end(); ++it)
        keys.push_back(it->first.as<string>());
      std::sort(keys.begin(), keys.end());
      for (std::size_t i = 0; i < keys.size(); i++)
      {
        emitter << YAML::Key << keys[i] << YAML::Value;
        write_node(node[keys[i]], emitter);
      }
      emitter << YAML::EndMap;
      break;
    }
    default:
      emitter << node;
      break;
  }
}
