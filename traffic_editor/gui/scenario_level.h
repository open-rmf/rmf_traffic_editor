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

#ifndef SCENARIO_LEVEL_H
#define SCENARIO_LEVEL_H

#include "traffic_editor/level.h"

#include <yaml-cpp/yaml.h>
#include <string>

class QGraphicsScene;


class ScenarioLevel : public Level
{
public:
  std::string name;

  ScenarioLevel();
  ~ScenarioLevel();

  bool from_yaml(const std::string& _name, const YAML::Node& yaml_node);
  YAML::Node to_yaml() const;

  bool delete_selected();

  void clear_selection();

  void draw(
    QGraphicsScene* scene,
    const double meters_per_pixel) const;

  void draw_polygons(QGraphicsScene* scene) const;
};

#endif
