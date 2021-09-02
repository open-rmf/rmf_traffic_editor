/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include "graph.h"
using std::string;

Graph::Graph()
{
}

Graph::~Graph()
{
}

bool Graph::from_yaml(const int _idx, const YAML::Node& data)
{
  if (!data.IsMap())
    throw std::runtime_error("Graph::from_yaml() expected a map");
  idx = _idx;
  //idx = std::stoi(idx_str);

  if (data["name"])
    name = data["name"].as<string>();

  if (data["default_lane_width"])
    default_lane_width = data["default_lane_width"].as<double>();

  return true;
}

YAML::Node Graph::to_yaml() const
{
  YAML::Node data;
  data["name"] = name;
  data["default_lane_width"] = default_lane_width;
  return data;
}
