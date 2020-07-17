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

#ifndef POLYGON_H
#define POLYGON_H

#include <map>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

#include <QPolygonF>

#include "param.h"


class Polygon
{
public:
  std::vector<int> vertices;
  bool selected = false;

  std::map<std::string, Param> params;

  enum Type
  {
    UNDEFINED = 0,
    FLOOR,
    ZONE,
    ROI,
    HOLE
  } type = UNDEFINED;

  Polygon();
  ~Polygon();

  void from_yaml(const YAML::Node& data, const Type polygon_type);
  YAML::Node to_yaml() const;

  void remove_vertex(const int vertex_idx);

  struct EdgeDragPolygon
  {
    QPolygonF polygon;
    int movable_vertex = -1;
  };

  void set_param(const std::string& name, const std::string& value);
  void create_required_parameters();

  template<typename T>
  void create_param_if_needed(
    const std::string& name,
    const Param::Type& param_type,
    const T& param_value);
};

#endif
