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

/*
 * This class represents a lift, including the shape of the lift cabin,
 * the location and names of its doors, and the floors at which it can stop.
 */

#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include "lift_door.h"


class Lift
{
public:
  std::string name;
  std::string reference_floor_name;

  // (x, y, yaw) of the cabin center, relative to reference_floor_name origin
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;

  // for now, we will model all lift cabins as rectangles
  double width = 1.0;  // meters
  double depth = 1.0;  // meters

  std::vector<LiftDoor> doors;

  // Many lifts have multiple sets of doors which open depending on the
  // level the lift is visiting. This is captured in the level_door map.
  // If a level is not in this map, that means that this lift does not
  // stop at that level.
  std::map<std::string, std::string> level_door;

  ////////////////////////////////////////////////////////////////////////

  Lift();

  YAML::Node to_yaml() const;
  void from_yaml(const std::string& _name, const YAML::Node &data);
};

#endif
