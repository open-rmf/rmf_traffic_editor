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

#include <cmath>

#include "transform.hpp"

using std::string;


Transform::Transform()
{
}

bool Transform::from_yaml(
  const YAML::Node& data,
  const CoordinateSystem& coordinate_system)
{
  if (!coordinate_system.is_global())
  {
    if (data["translation_x"])
      _translation.setX(data["translation_x"].as<double>());
    if (data["translation_y"])
      _translation.setY(data["translation_y"].as<double>());
  }
  else if (data["translation_lat"] && data["translation_lon"])
  {
    CoordinateSystem::WGS84Point wgs84_point;
    wgs84_point.lon = data["translation_lon"].as<double>();
    wgs84_point.lat = data["translation_lat"].as<double>();
    CoordinateSystem::ProjectedPoint p =
      coordinate_system.to_epsg3857(wgs84_point);
    _translation.setX(p.x);
    _translation.setY(p.y);
  }

  if (data["yaw"])
    _yaw = data["yaw"].as<double>();

  if (data["scale"])
    _scale = data["scale"].as<double>();

  return true;
}

YAML::Node Transform::to_yaml(const CoordinateSystem& coordinate_system) const
{
  YAML::Node y;
  y["yaw"] = _yaw;
  y["scale"] = _scale;

  if (!coordinate_system.is_global())
  {
    y["translation_x"] = _translation.x();
    y["translation_y"] = _translation.y();
  }
  else
  {
    const CoordinateSystem::WGS84Point p =
      coordinate_system.to_wgs84({_translation.x(), _translation.y()});
    y["translation_lat"] = p.lat;
    y["translation_lon"] = p.lon;
  }
  return y;
}

QPointF Transform::forwards(const QPointF& p) const
{
  const double qx =
    ( cos(_yaw) * p.x() + sin(_yaw) * p.y()) * _scale + _translation.x();

  const double qy =
    (-sin(_yaw) * p.x() + cos(_yaw) * p.y()) * _scale + _translation.y();

  return QPointF(qx, qy);
}

QPointF Transform::backwards(const QPointF& p) const
{
  // translate back and scale
  const double tsx = (p.x() - _translation.x()) / _scale;
  const double tsy = (p.y() - _translation.y()) / _scale;

  // rotate back
  const double rx = cos(-_yaw) * tsx + sin(-_yaw) * tsy;
  const double ry = -sin(-_yaw) * tsx + cos(-_yaw) * tsy;
  return QPointF(rx, ry);
}

Transform Transform::inverse() const
{
  Transform inv;
  inv.setYaw(-_yaw);
  inv.setScale(1.0 / _scale);
  inv.setTranslation(
    1.0 / _scale *
    QPointF(
      cos(-_yaw) * translation().x() - sin(-_yaw) * translation().y(),
      sin(-_yaw) * translation().x() + cos(-_yaw) * translation().y()));
  return inv;
}

string Transform::to_string() const
{
  // make a string the old fashioned way...
  char buf[1024] = {0};
  snprintf(
    buf,
    sizeof(buf),
    "rotation: %.5f\n"
    "scale: %.5f\n"
    "trans X: %.5f\n"
    "trans Y: %.5f",
    _yaw,
    _scale,
    _translation.x(),
    _translation.y());
  return string(buf);
}
