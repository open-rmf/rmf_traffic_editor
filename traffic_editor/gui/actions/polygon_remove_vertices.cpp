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

#include "polygon_remove_vertices.h"
PolygonRemoveVertCommand::PolygonRemoveVertCommand(
  Polygon* polygon,
  int vert_id)
{
  _polygon = polygon;
  _vert_id = vert_id;
  _old_vertices = polygon->vertices;
}

PolygonRemoveVertCommand::~PolygonRemoveVertCommand()
{
}

void PolygonRemoveVertCommand::undo()
{
  _polygon->vertices = _old_vertices;
}

void PolygonRemoveVertCommand::redo()
{
  _polygon->remove_vertex(_vert_id);
}
