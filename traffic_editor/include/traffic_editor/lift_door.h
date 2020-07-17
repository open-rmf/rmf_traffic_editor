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

#ifndef LIFT_DOOR_H
#define LIFT_DOOR_H

#include <yaml-cpp/yaml.h>


class LiftDoor
{
public:
  std::string name;

  double x = 0.0;  // center of door, relative to the cabin center
  double y = 0.5;  // center of door, relative to the cabin center

  double width = 1.0;  // doorway width in meters when fully opened

  // Motion axis is defined relative to the cabin orientation.
  // Typically it will be 0 or pi/2.
  double motion_axis_orientation = 0.0;

  enum DoorType
  {
    UNDEFINED = 0,
    SINGLE_SLIDING,
    DOUBLE_SLIDING,
    SINGLE_TELESCOPE,
    DOUBLE_TELESCOPE
  } door_type = DOUBLE_SLIDING;

  YAML::Node to_yaml() const;
  void from_yaml(const std::string& _name, const YAML::Node& data);
};

#endif
