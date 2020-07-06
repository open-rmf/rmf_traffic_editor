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

#ifndef LAYER_H
#define LAYER_H

#include <string>

#include <QPixmap>

#include <yaml-cpp/yaml.h>


class Layer
{
public:
  Layer();
  ~Layer();

  std::string name;
  std::string filename;
  bool visible = true;

  double meters_per_pixel = 0.05;  // relative to the parent floorplan scale
  double translation_x = 0.0;
  double translation_y = 0.0;
  double rotation = 0.0;

  QPixmap pixmap;

  bool from_yaml(const std::string& name, const YAML::Node& data);
  YAML::Node to_yaml() const;
};

#endif
