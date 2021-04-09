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

#ifndef LIFT_H
#define LIFT_H

class QGraphicsScene;
class QGraphicsView;

/*
 * This class represents a lift, including the shape of the lift cabin,
 * the location and names of its doors, and the floors at which it can stop.
 */

#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <cfloat>
#include "level.h"
#include "lift_door.h"


class Lift
{
public:
  std::string name;
  std::string reference_floor_name;
  std::string initial_floor_name;

  // (x, y, yaw) of the cabin center, relative to reference_floor_name origin
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;

  // for now, we will model all lift cabins as rectangles
  double width = 1.0;  // meters
  double depth = 1.0;  // meters

  std::string highest_floor;  // highest floor the lift can reach
  std::string lowest_floor;  // lowest floor the lift can reach
  double highest_elevation = DBL_MAX;
  double lowest_elevation = -DBL_MAX;

  std::vector<LiftDoor> doors;

  // Many lifts have multiple sets of doors which open depending on the
  // level the lift is visiting. In the most complex case, multiple
  // sets of doors can open on a level, so unfortunately, the data
  // structure needs to cater for this, even though it is not common.
  // If a (level name, door name) combination is not in this data structure,
  // that means that the door does not open on this level.
  // It is possible (and expected) that some lifts do not go to all levels,
  // and that some lifts skip some levels, so this data structure is
  // expected to be "sparse"
  typedef std::list<std::string> DoorNameList;
  typedef std::map<std::string, DoorNameList> LevelDoorMap;
  LevelDoorMap level_doors;

  // When this option is false, it indicates that a lift plugin and its lift
  // door plugins should not be included when building the simulation world.
  // This will help speed up the simulation as well as represent lifts that are
  // not accessible by AGVs.
  bool plugins = true;

  ////////////////////////////////////////////////////////////////////////

  Lift();

  YAML::Node to_yaml() const;
  void from_yaml(const std::string& _name, const YAML::Node& data,
    const std::vector<Level>& levels);

  void draw(
    QGraphicsScene* scene,
    const double meters_per_pixel,
    const std::string& level_name,
    const double elevation,
    const bool apply_transformation = true,
    const double scale = 1.0,
    const double translate_x = 0.0,
    const double translate_y = 0.0) const;

  bool level_door_opens(
    const std::string& level_name,
    const std::string& door_name,
    const std::vector<Level>& levels) const;
};

#endif
