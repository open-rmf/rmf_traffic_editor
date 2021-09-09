/*
 * Copyright (C) 2019-2021 Open Source Robotics Foundation
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

#include "coordinate_system.h"

CoordinateSystem::CoordinateSystem()
: value(CoordinateSystem::Undefined)
{
}

CoordinateSystem::CoordinateSystem(const CoordinateSystem::Value& _value)
: value(_value)
{
}

CoordinateSystem::~CoordinateSystem()
{
}

std::string CoordinateSystem::to_string()
{
  switch (value)
  {
    case Legacy: return "legacy";
    case WebMercator: return "web_mercator";
    case CartesianMeters: return "cartesian_meters";

    case Undefined:
    default:
      return "undefined";
  }
}

CoordinateSystem CoordinateSystem::from_string(const std::string& s)
{
  CoordinateSystem cs;
  if (s == "legacy")
    cs.value = Legacy;
  else if (s == "web_mercator")
    cs.value = WebMercator;
  else if (s == "cartesian_meters")
    cs.value = CartesianMeters;
  else
    cs.value = Undefined;
  return cs;
}

bool CoordinateSystem::is_y_flipped()
{
  return cs.value == Legacy;
}
