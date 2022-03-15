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

#ifndef VERTEX_H
#define VERTEX_H

#include <QUuid>
#include <map>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

#include <QColor>

#include "coordinate_system.h"
#include "param.h"

class QGraphicsScene;


class Vertex
{
public:
  double x;
  double y;
  std::string name;

  bool selected;

  QUuid uuid;
  std::map<std::string, Param> params;

  Vertex();
  Vertex(double _x, double _y, const std::string& _name = std::string());

  void from_yaml(
    const YAML::Node& data,
    const CoordinateSystem& coordinate_system);

  YAML::Node to_yaml(const CoordinateSystem& coordinate_system) const;

  void set_param(const std::string& name, const std::string& value);

  void draw(
    QGraphicsScene* scene,
    const double radius,
    const QFont& font,
    const CoordinateSystem& coordinate_system) const;

  bool is_parking_point() const;
  bool is_holding_point() const;
  bool is_cleaning_zone() const;
  bool is_charger() const;

  std::string dropoff_ingestor() const;
  std::string pickup_dispenser() const;
  std::string lift_cabin() const;


  ////////////////////////////////////////////////////////////
  static const std::vector<std::pair<std::string, Param::Type>> allowed_params;
};

#endif
