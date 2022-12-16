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

#ifndef FIDUCIAL_H
#define FIDUCIAL_H

#include <map>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <quuid.h>

class QGraphicsScene;


class Fiducial
{
public:
  double x = 0.0;
  double y = 0.0;
  std::string name;
  QUuid uuid;

  bool selected = false;

  Fiducial();
  Fiducial(double _x, double _y, const std::string& _name = std::string());

  void from_yaml(const YAML::Node& data);
  YAML::Node to_yaml() const;

  void draw(QGraphicsScene*, const double meters_per_pixel) const;

  double distance(const Fiducial& f);
};

#endif
