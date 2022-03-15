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

#ifndef TRAFFIC_EDITOR__TRANSFORM_HPP
#define TRAFFIC_EDITOR__TRANSFORM_HPP

#include <string>

#include <yaml-cpp/yaml.h>

#include <QPointF>

#include "coordinate_system.h"

//=============================================================================
/// A transform from one space to another. For now this will be linear,
/// but in the future we expect to use various types of nonlinear transforms.
class Transform
{
public:
  Transform();

  double _yaw = 0.0;
  double _scale = 1.0;
  QPointF _translation;

  double yaw() const { return _yaw; }
  void setYaw(const double next_yaw) { _yaw = next_yaw; }

  double scale() const { return _scale; }
  void setScale(const double next_scale) { _scale = next_scale; }

  QPointF translation() const { return _translation; }
  void setTranslation(const QPointF& next_translation)
  {
    _translation = next_translation;
  }

  QPointF forwards(const QPointF& p) const;
  QPointF backwards(const QPointF& p) const;

  bool from_yaml(
    const YAML::Node& data,
    const CoordinateSystem& coordinate_system);

  YAML::Node to_yaml(const CoordinateSystem& coordinate_system) const;

  Transform inverse() const;

  std::string to_string() const;
};

#endif  // TRAFFIC_EDITOR__TRANSFORM_HPP
