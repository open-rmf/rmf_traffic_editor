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
  printf("CoordinateSystem::CoordinateSystem()\n");
  throw;
  init_proj();
}

CoordinateSystem::CoordinateSystem(const CoordinateSystem::Value& _value)
: value(_value)
{
  printf("CoordinateSystem::CoordinateSystem(value)\n");
  init_proj();
}

void CoordinateSystem::init_proj()
{
  printf("CoordinateSystem::init_proj()\n");
  if (proj_context)
  {
    printf("CoordinateSystem::init_proj() called twice!\n");
    return;
  }
  proj_context = proj_context_create();
  epsg_3857_to_wgs84 = proj_create_crs_to_crs(
    proj_context,
    "EPSG:3857",
    "EPSG:4326",
    NULL);
}

CoordinateSystem::~CoordinateSystem()
{
  printf("CoordinateSystem::~CoordinateSystem()\n");
  if (epsg_3857_to_wgs84)
  {
    proj_destroy(epsg_3857_to_wgs84);
    epsg_3857_to_wgs84 = nullptr;
  }
  else
  {
    printf("no epsg_3857_to_wgs84 during CoordinateSystem destructor!\n");
  }

  if (proj_context)
  {
    proj_context_destroy(proj_context);
    proj_context = nullptr;
  }
  else
  {
    printf("no proj_context during CoordinateSystem destructor!\n");
  }

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

CoordinateSystem::Value CoordinateSystem::value_from_string(
  const std::string& s)
{
  if (s == "reference_image")
    return ReferenceImage;
  else if (s == "web_mercator")
    return WebMercator;
  else if (s == "cartesian_meters")
    return CartesianMeters;
  else if (s == "wgs84")
    return WGS84;
  else
    return Undefined;
}

bool CoordinateSystem::is_y_flipped() const
{
  return value == ReferenceImage;
}

bool CoordinateSystem::is_global() const
{
  return value == WGS84;
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
  return value == CoordinateSystem::WGS84;
}

CoordinateSystem::WGS84Point CoordinateSystem::to_wgs84(
  const ProjectedPoint& point) const
{
  if (value != ReferenceImage)
  {
    // todo: if CartesianMeters, use the CRS name provided somewhere
    const PJ_COORD c = proj_coord(point.x, point.y, 0, 0);
    const PJ_COORD wgs84_coord = proj_trans(
      epsg_3857_to_wgs84,
      PJ_FWD,
      c);
    return WGS84Point{wgs84_coord.v[0], wgs84_coord.v[1]};
  }
  else
  {
    return {0, 0};
  }
}

CoordinateSystem::ProjectedPoint CoordinateSystem::to_epsg3857(
  const WGS84Point& point) const
{
  if (value == WGS84)
  {
    const PJ_COORD p_in = proj_coord(point.lat, point.lon, 0, 0);
    const PJ_COORD p_out = proj_trans(
      epsg_3857_to_wgs84,
      PJ_INV,
      p_in);
    return ProjectedPoint({p_out.v[0], p_out.v[1]});
  }
  else
  {
    return {0, 0};
  }
}
