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
#include "add_polygon.h"

AddPolygonCommand::AddPolygonCommand(
  Project* project,
  EditorModeId mode,
  Polygon polygon,
  int level_idx)
{
  //In this case to keep polygon undoing simple we use simple snapshots;
  _project = project;
  _mode = mode;
  _to_add = polygon;
  _level_idx = level_idx;
  if (_mode == MODE_BUILDING)
    _previous_polygons = _project->building.levels[level_idx].polygons;
  else if (mode == MODE_SCENARIO)
    _previous_polygons = _project->scenario_level(level_idx)->polygons;
}

AddPolygonCommand::~AddPolygonCommand()
{
}

void AddPolygonCommand::undo()
{
  if (_mode == MODE_BUILDING)
    _project->building.levels[_level_idx].polygons = _previous_polygons;
  else if (_mode == MODE_SCENARIO)
    _project->scenario_level(_level_idx)->polygons = _previous_polygons;
}

void AddPolygonCommand::redo()
{
  if (_mode == MODE_BUILDING)
    _project->building.levels[_level_idx].polygons.push_back(_to_add);
  else if (_mode == MODE_SCENARIO)
    _project->scenario_level(_level_idx)->polygons.push_back(_to_add);
}