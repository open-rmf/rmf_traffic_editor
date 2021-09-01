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

#ifndef TRAFFIC_EDITOR_GRAPH_H
#define TRAFFIC_EDITOR_GRAPH_H

#include <string>

#include <yaml-cpp/yaml.h>

class Graph
{
public:
  Graph();
  ~Graph();

  int idx = 0;
  std::string name;
  double default_lane_width = 1.0;

  bool from_yaml(const int _idx, const YAML::Node& data);
  YAML::Node to_yaml() const;
};

#endif
