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
#include "add_polygon.h"

AddPolygonCommand::AddPolygonCommand(
  Building* building,
  Polygon polygon,
  int level_idx)
{
  // In this case to keep polygon undoing simple, we use simple snapshots.
  _building = building;
  _to_add = polygon;
  _level_idx = level_idx;
  _previous_polygons = _building->levels[level_idx].polygons;
}

AddPolygonCommand::~AddPolygonCommand()
{
}

void AddPolygonCommand::undo()
{
  _building->levels[_level_idx].polygons = _previous_polygons;
}

void AddPolygonCommand::redo()
{
  _building->levels[_level_idx].polygons.push_back(_to_add);
}
