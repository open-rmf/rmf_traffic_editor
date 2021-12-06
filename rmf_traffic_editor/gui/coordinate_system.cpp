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
    case ReferenceImage: return "reference_image";
    case WebMercator: return "web_mercator";
    case CartesianMeters: return "cartesian_meters";
    case WGS84: return "wgs84";

    case Undefined:
    default:
      return "undefined";
  }
}

CoordinateSystem CoordinateSystem::from_string(const std::string& s)
{
  CoordinateSystem cs;
  if (s == "reference_image")
    cs.value = ReferenceImage;
  else if (s == "web_mercator")
    cs.value = WebMercator;
  else if (s == "cartesian_meters")
    cs.value = CartesianMeters;
  else if (s == "wgs84")
    cs.value = WGS84;
  else
    cs.value = Undefined;
  return cs;
}

bool CoordinateSystem::is_y_flipped() const
{
  return value == ReferenceImage;
}

double CoordinateSystem::default_scale() const
{
  // Image-based maps are often somewhere around 5cm
  // pixel cell size, since that's the common discretization
  // of SLAM-based robot maps. Let's use that for the
  // default scale (unless computed otherwise; hopefully
  // the map provides a few proper measurements). However,
  // for other coordinate systems, we should assume a
  // scale of 1.0 so that the coordinates stay as specified
  // and the UI tools work as expected in, for example,
  // Cartesian-meters coordinate systems.

  if (value == CoordinateSystem::ReferenceImage)
    return 0.05;
  else
    return 1.0;
}

bool CoordinateSystem::has_tiles() const
{
  return (value == CoordinateSystem::WGS84);
}
