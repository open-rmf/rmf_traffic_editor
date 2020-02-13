/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include "polygon.h"
using std::vector;


Polygon::Polygon()
: selected(false), type(UNDEFINED)
{
}

Polygon::~Polygon()
{
}

void Polygon::from_yaml(const YAML::Node &data, const Type polygon_type)
{
  if (!data.IsMap())
    throw std::runtime_error("Polygon::from_yaml() expected a map");
  type = polygon_type;
  for (YAML::const_iterator it = data["vertices"].begin();
      it != data["vertices"].end(); ++it) {
    vertices.push_back(it->as<int>());
  }
}

YAML::Node Polygon::to_yaml() const
{
  YAML::Node y;
  for (const auto &vertex_idx : vertices)
    y["vertices"].push_back(vertex_idx);
  y["vertices"].SetStyle(YAML::EmitterStyle::Flow);
  return y;
}

void Polygon::remove_vertex(const int vertex_idx)
{
  if (vertex_idx < 0 || vertex_idx > static_cast<int>(vertices.size()))
    return;  // oh no
  vector<int> &v = vertices;  // save typing
  v.erase(std::remove(v.begin(), v.end(), vertex_idx), v.end());
}
