/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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
#include <yaml-cpp/yaml.h>


class Lift
{
public:
  // (x, y, yaw) are defined on reference_floor_name
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  std::string name;
  std::string reference_floor_name;
  bool selected = false;  // only for visualization, not saved to YAML

  Lift();
  Lift(
      const double _x,
      const double _y,
      const double _yaw,
      const std::string &_name,
      const std::string &_reference_floor_name);

  YAML::Node to_yaml() const;
  void from_yaml(const std::string& _name, const YAML::Node &data);
};

#endif
