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

#include "polygon_add_vertex.h"

PolygonAddVertCommand::PolygonAddVertCommand(
  Polygon* polygon,
  int position,
  int vert_id)
{
  _polygon = polygon;
  _old_vertices = polygon->vertices;
  _position = position;
  _vert_id = vert_id;
}

PolygonAddVertCommand::~PolygonAddVertCommand()
{

}

void PolygonAddVertCommand::undo()
{
  _polygon->vertices.erase(_polygon->vertices.begin() + _position);
}

void PolygonAddVertCommand::redo()
{
  _polygon->vertices.insert(
    _polygon->vertices.begin() + _position,
    _vert_id);
}