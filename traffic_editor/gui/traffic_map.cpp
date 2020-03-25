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

#include "traffic_map.h"

using std::string;

TrafficMap::TrafficMap()
{
}

TrafficMap::~TrafficMap()
{
}

bool TrafficMap::from_project_yaml(const YAML::Node& y)
{
  if (y["name"])
    name = y["name"].as<string>();

  if (y["filename"])
    filename = y["filename"].as<string>();

  if (y["x_offset"])
    x_offset = y["x_offset"].as<double>();

  if (y["y_offset"])
    y_offset = y["y_offset"].as<double>();

  // todo: open 'filename' and load its contents
}
