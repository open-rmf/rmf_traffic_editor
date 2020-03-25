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

bool TrafficMap::from_project_yaml(const string& _name, const YAML::Node& y)
{
  name = _name;

  if (y["offset"] && y["offset"].IsSequence())
  {
    x_offset = y["offset"][0].as<double>();
    y_offset = y["offset"][1].as<double>();
  }

  // todo: open 'filename' and load its contents
  if (y["filename"])
  {
    filename = y["filename"].as<string>();
    return load_file();
  }

  return true;
}

bool TrafficMap::load_file()
{
  return true;
}

YAML::Node TrafficMap::to_project_yaml() const
{
  YAML::Node y;
  y["filename"] = filename;
  y["offset"].push_back(x_offset);
  y["offset"].push_back(y_offset);
  y["offset"].SetStyle(YAML::EmitterStyle::Flow);
  return y;
}
