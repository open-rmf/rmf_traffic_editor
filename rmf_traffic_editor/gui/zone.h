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

#ifndef ZONE_H
#define ZONE_H

class QGraphicsScene;

#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <cfloat>
#include "level.h"

class InternalVertex
{
public:

  std::string name;

  double x = 0.0;  // x location of vertex in meters
  double y = 0.0;  // y location of vertex in meters
  uint priority = 0; // Priority of vertex
  std::string group = "Default";  // Group label of vertex

  YAML::Node to_yaml() const;
  void from_yaml(const std::string& _name, const YAML::Node& data);
};

class ExternalVertex
{
public:

  std::string name;
  bool is_entry_point = false;
  bool is_exit_point = false;

  YAML::Node to_yaml() const;
  void from_yaml(const std::string& _name, const YAML::Node& data);
};

class Zone
{
public:
  std::string name;                           // Name of the zone
  std::string level;                          // Which level the zone is at
  std::string type;                           // What kind of zone it is

  double x = 0.0;                             // x coordinate of the zone in the global frame
  double y = 0.0;                             // y coordinate of the zone in the global frame
  double yaw = 0.0;                           // yaw of the zone in the global frame

  double width = 1.0;                         // Width of the zone in meters
  double depth = 1.0;                         // Depth of the zone in meters

  double elevation = DBL_MAX;                 // Zone elevation. Used to check which level the zone is at

  bool show_vertices = true;                  // Visual marker. Used to hide or show verticse in the zone
  bool show_zone = true;                      // Visual marker. Used to hide or show the zone
  bool show_lanes = true;                     // Visual marker. Used to hide or show the zone's transition lanes

  std::vector<InternalVertex> internal_vertices;
  std::vector<ExternalVertex> external_vertices;

  ////////////////////////////////////////////////////////////////////////

  Zone();

  YAML::Node to_yaml() const;

  void from_yaml(const std::string& _name, const YAML::Node& data,
    const std::vector<Level>& levels);

  void draw(
    QGraphicsScene* scene,
    const double meters_per_pixel,
    const std::string& level_name,
    const bool apply_transformation = true,
    const std::vector<Vertex>& level_vertices = {}) const;
};

#endif
