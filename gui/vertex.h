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

#include <string>
#include <yaml-cpp/yaml.h>

class QGraphicsScene;


class Vertex
{
public:
  double x;
  double y;
  std::string name;

  bool selected;

  Vertex();
  Vertex(double _x, double _y, const std::string &_name = std::string());

  void from_yaml(const YAML::Node &data);
  YAML::Node to_yaml() const;

  void draw(QGraphicsScene *, const double meters_per_pixel) const;
};

#endif
