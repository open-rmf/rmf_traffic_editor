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

#ifndef TRAFFIC_EDITOR__FEATURE_HPP
#define TRAFFIC_EDITOR__FEATURE_HPP

#include <string>

#include <QPointF>
#include <QUuid>
#include <yaml-cpp/yaml.h>

#include "transform.hpp"

class QGraphicsScene;

//=============================================================================
/// A feature in an image, such as a corner of a room in an occupancy-grid map.
/// In the future, perhaps we can have more generic features, such as lines
/// or circles (pillars), etc. But for now, we'll use points.
class Feature
{
public:
  Feature();
  Feature(double x, double y);

  // "x" and "y" are pixels from the upper-left corner of the layer image
  double x() const { return _x; }
  void set_x(double x) { _x = x; }

  double y() const { return _y; }
  void set_y(double y) { _y = y; }

  QPointF transformed() { return _transformed; }

  std::string name() const { return _name; }

  QUuid const& id() const { return _id; }

  bool selected() const { return _selected; }

  void from_yaml(const YAML::Node& data);
  YAML::Node to_yaml() const;

  void draw(
    QGraphicsScene*,
    const Transform& transform,
    const QColor color,
    const double level_meters_per_pixel) const;

  static constexpr double radius_meters = 0.1;

private:
  double _x = 0.0;
  double _y = 0.0;
  QUuid _id;
  bool _selected = false;
  std::string _name;
  QPointF _transformed;
};

#endif  // TRAFFIC_EDITOR__FEATURE_HPP
