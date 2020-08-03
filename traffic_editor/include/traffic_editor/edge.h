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

#ifndef EDGE_H
#define EDGE_H

#include <string>
#include <map>

#include <yaml-cpp/yaml.h>

#include "param.h"
#include <QString>


class Edge
{
public:
  int start_idx, end_idx;

  enum Type
  {
    UNDEFINED = 0,
    LANE,
    WALL,
    MEAS,
    DOOR,
    HUMAN_LANE,
  } type;

  bool selected;  // only for visualization, not saved to YAML

  Edge();
  Edge(const int _start_idx, const int _end_idx, const Type _type);
  ~Edge();

  std::map<std::string, Param> params;

  void from_yaml(const YAML::Node& data, const Type edge_type);
  YAML::Node to_yaml() const;

  void set_param(const std::string& name, const std::string& value);

  bool is_bidirectional() const;

  void create_required_parameters();

  template<typename T>
  void create_param_if_needed(
    const std::string& name,
    const Param::Type& param_type,
    const T& param_value);

  std::string type_to_string() const;
  QString type_to_qstring() const;
  void set_graph_idx(const int idx);
  int get_graph_idx() const;

  double get_width() const;
};

#endif
