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

#ifndef TRAFFIC_MAP_H
#define TRAFFIC_MAP_H

#include <string>
#include <yaml-cpp/yaml.h>

class TrafficMap
{
public:
  std::string name;
  std::string filename;
  double x_offset = 0;
  double y_offset = 0;
  bool visible = true;

  /////////////////////////////////
  TrafficMap();
  ~TrafficMap();

  bool from_project_yaml(const std::string& name, const YAML::Node& data);
  YAML::Node to_project_yaml() const;

  bool load_file();
};

#endif
