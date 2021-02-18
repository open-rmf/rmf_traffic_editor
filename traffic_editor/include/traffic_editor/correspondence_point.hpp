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

#ifndef TRAFFIC_EDITOR__CORRESPONDENCE_POINT_HPP
#define TRAFFIC_EDITOR__CORRESPONDENCE_POINT_HPP

#include <yaml-cpp/yaml.h>
#include <quuid.h>

class QGraphicsScene;

class CorrespondencePoint
{
public:
  CorrespondencePoint();
  CorrespondencePoint(double x, double y, int id);

  double x() const { return x_; }
  void set_x(double x) { x_ = x; }
  double y() const { return y_; }
  void set_y(double y) { y_ = y; }
  uint16_t id() const { return id_; }
  QUuid const& uuid() const { return uuid_; }
  bool selected() const { return selected_; }

  void from_yaml(const YAML::Node& data);
  YAML::Node to_yaml() const;

  void draw(QGraphicsScene*, double meters_per_pixel) const;

private:
  double x_ = 0.0;
  double y_ = 0.0;
  uint16_t id_ = 0;
  QUuid uuid_;
  bool selected_ = false;

  static uint16_t next_id_;
};

#endif  // TRAFFIC_EDITOR__CORRESPONDENCE_POINT_HPP
