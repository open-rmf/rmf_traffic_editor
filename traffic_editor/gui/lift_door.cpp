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

#include <cmath>

#include "traffic_editor/lift_door.h"

YAML::Node LiftDoor::to_yaml() const
{
  // This is in image space. I think it's safe to say nobody is clicking
  // with more than 1/1000 precision inside a single pixel.

  YAML::Node n;
  n["x"] = std::round(x * 1000.0) / 1000.0;
  n["y"] = std::round(y * 1000.0) / 1000.0;
  n["width"] = std::round(width * 1000.0) / 1000.0;
  n["door_type"] = static_cast<int>(door_type);
  // let's give yaw another decimal place because, I don't know, reasons (?)
  n["motion_axis_orientation"] =
    std::round(motion_axis_orientation * 10000.0) / 10000.0;
  return n;
}

void LiftDoor::from_yaml(const std::string& _name, const YAML::Node& data)
{
  if (!data.IsMap())
    throw std::runtime_error("LiftDoor::from_yaml() expected a map");
  x = data["x"].as<double>();
  y = data["y"].as<double>();
  width = data["width"].as<double>();
  door_type = static_cast<DoorType>(data["door_type"].as<int>());
  motion_axis_orientation = data["motion_axis_orientation"].as<double>();
  name = _name;
}
