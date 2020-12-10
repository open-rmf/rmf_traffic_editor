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

#include "add_vertex.h"

AddVertexCommand::AddVertexCommand(
  Project* project,
  EditorModeId mode,
  int level_idx,
  double x,
  double y)
{
  _mode = mode;
  _x = x;
  _y = y;
  _level_idx = level_idx; //TODO: Dependency on level_idx is dangerous.
  _project = project;
}

AddVertexCommand::~AddVertexCommand()
{
  //Empty to make linker happy
}

void AddVertexCommand::undo()
{
  if (_mode == MODE_BUILDING || _mode == MODE_TRAFFIC)
  {
    size_t length = _project->building.levels[_level_idx].vertices.size();
    //TODO: SLOW O(n) method... Need to rework datastructures.
    for (size_t i = 0; i < length; i++)
    {
      if (_project->building.levels[_level_idx].vertices[i].uuid == _vert_id)
      {
        _project->building.levels[_level_idx].vertices.erase(
          _project->building.levels[_level_idx].vertices.begin()+i);
        break;
      }
    }
  }
}

void AddVertexCommand::redo()
{
  if (_mode == MODE_BUILDING || _mode == MODE_TRAFFIC)
  {
    _project->building.add_vertex(_level_idx, _x, _y);
    size_t sz = _project->building.levels[_level_idx].vertices.size();
    _vert_id = _project->building.levels[_level_idx].vertices[sz-1].uuid;
  }
  else if (_mode == MODE_SCENARIO)
  {
    assert(_project->scenario_idx < 0);
    _project->add_scenario_vertex(_level_idx, _x, _y);
  }
}